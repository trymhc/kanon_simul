#!/usr/bin/env python3
"""
Godbit-kanon – simulator
========================

Simulerer hele kanonen på din egen maskin, uten deler:

  * en virtuell hund som hviler, rusler, går ut av rommet og løper til godbiter
  * et virtuelt kamera (Camera Module 3 Wide, 102° FOV) med støy og bom i deteksjonen
  * servoer med begrenset hastighet (MG996R pan/tilt, SG90 mater)
  * flywheel-motorer styrt med PWM, med enkel modell fra duty til utgangsfart
  * godbitbane med luftmotstand
  * tilstandsmaskinen fra tilstandsdiagrammet (Oppstart → Venter → ... → Nedkjøling)

All maskinvare ligger bak klassen `SimMaskinvare`. På Pi-en lager du en `PiMaskinvare`
med de samme metodene (set_pan, set_tilt, mat, set_motor, detect, selvtest, nodstopp),
og `Kontroller` kan brukes uendret.

Kjøring:
    python3 kanon_sim.py                 # 15 simulerte minutter, lagrer kanon_sim.png
    python3 kanon_sim.py --tof           # med VL53L1X-avstandssensor
    python3 kanon_sim.py --feil 300      # I²C-feil etter 300 s → Sikker stopp
    python3 kanon_sim.py --elev 40 --vis # annen utskytingsvinkel, vis plott i vindu

Krav: Python 3.9+, numpy, matplotlib
"""

from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass
from enum import Enum

G = 9.81


# ---------------------------------------------------------------------------
# Konfigurasjon – alle tall du kan justere ligger her
# ---------------------------------------------------------------------------
@dataclass
class Config:
    # Kamera og deteksjon
    fps: float = 10.0               # bilder per sekund YOLO klarer på Pi 5 uten AI HAT
    img_w: int = 640
    fov_h_deg: float = 102.0        # Camera Module 3 Wide
    det_miss: float = 0.08          # sannsynlighet for at YOLO bommer på et bilde
    det_px_noise: float = 4.0       # støy i bounding-box-senter (piksler)
    dist_noise_bbox: float = 0.10   # relativ feil i avstand estimert fra boksstørrelse
    dist_noise_tof: float = 0.02    # relativ feil med VL53L1X
    tof_fov_deg: float = 27.0
    use_tof: bool = False

    # Servoer
    servo_speed_dps: float = 300.0  # MG996R ca. 0,2 s per 60° under last
    elev_deg: float = 30.0          # utskytingsvinkel (tilt-servo)
    scan_speed_dps: float = 15.0    # sveip når ingen hund er sett

    # Utskyter
    mount_h: float = 0.25           # høyde over gulvet der godbiten forlater hjulene (m)
    wheel_d: float = 0.05           # flywheel-diameter (m)
    rpm_max: float = 10000.0        # 385-motor ubelastet ved 12 V
    eff: float = 0.75               # godbitfart / hjulets overflatefart
    duty_max: float = 0.90
    speed_sigma: float = 0.04       # skudd-til-skudd-variasjon i fart
    pan_sigma_deg: float = 0.7      # slark i pan

    # Godbit (tørrfôrkule)
    bit_d: float = 0.011
    bit_m: float = 0.0005
    cd: float = 0.47
    rho: float = 1.2

    # Logikk (fra tilstandsdiagrammet)
    confirm_frames: int = 5
    confirm_miss: int = 3
    lost_timeout: float = 2.0
    stable_deg: float = 2.0
    stable_frames: int = 5
    spinup_s: float = 0.5
    feed_s: float = 0.4
    shoot_s: float = 0.3
    cooldown_s: float = 30.0
    daily_max: int = 8
    aim_offset: float = 0.4         # sikt så mange meter foran hunden, aldri på den
    min_dist: float = 1.0
    kp: float = 0.5
    kd: float = 0.1

    # Rommet (kanonen står ved veggen i x = 0 og ser langs +x)
    room_x: tuple = (0.8, 5.0)
    room_y: tuple = (-2.0, 2.0)
    door: tuple = (5.0, 2.0)

    @property
    def focal_px(self) -> float:
        return (self.img_w / 2) / math.tan(math.radians(self.fov_h_deg / 2))


# ---------------------------------------------------------------------------
# Fysikk
# ---------------------------------------------------------------------------
class Ballistikk:
    """Godbitbane i 2D (horisontal avstand x, høyde z) med kvadratisk luftmotstand."""

    def __init__(self, cfg: Config):
        area = math.pi * (cfg.bit_d / 2) ** 2
        self.k = 0.5 * cfg.rho * cfg.cd * area / cfg.bit_m   # 1/m
        self.h0 = cfg.mount_h

    def bane(self, v, elev_deg, drag=True, dt=0.001, record=True):
        th = math.radians(elev_deg)
        k = self.k if drag else 0.0
        x, z = 0.0, self.h0
        vx, vz = v * math.cos(th), v * math.sin(th)
        xs, zs = [x], [z]
        t = 0.0
        while t < 5.0:
            s = math.hypot(vx, vz)
            vx += -k * s * vx * dt
            vz += (-G - k * s * vz) * dt
            nx, nz = x + vx * dt, z + vz * dt
            t += dt
            if nz <= 0.0:                       # interpoler treffpunktet på gulvet
                f = z / (z - nz)
                x += f * (nx - x)
                z = 0.0
                t += (f - 1) * dt
                if record:
                    xs.append(x)
                    zs.append(z)
                break
            x, z = nx, nz
            if record:
                xs.append(x)
                zs.append(z)
        if not record:
            xs, zs = [x], [z]
        return xs, zs, t

    def landing(self, v, elev_deg, drag=True):
        xs, _, t = self.bane(v, elev_deg, drag, record=False)
        return xs[-1], t

    def finn_fart(self, d, elev_deg, drag=True):
        """Fart som gir landing i avstand d (halveringsmetoden). None hvis utenfor rekkevidde."""
        lo, hi = 0.3, 30.0
        if self.landing(hi, elev_deg, drag)[0] < d:
            return None
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            if self.landing(mid, elev_deg, drag)[0] < d:
                lo = mid
            else:
                hi = mid
        return hi


class MotorModell:
    """Lineær modell: godbitfart = eff · π · D · rpm_max/60 · duty."""

    def __init__(self, cfg: Config):
        self.vmax = cfg.eff * math.pi * cfg.wheel_d * cfg.rpm_max / 60.0

    def duty(self, v):
        return v / self.vmax

    def fart(self, duty):
        return duty * self.vmax


# ---------------------------------------------------------------------------
# Den simulerte verden
# ---------------------------------------------------------------------------
class Hund:
    FART = {"går": 0.6, "går ut": 0.6, "løper": 2.0}

    def __init__(self, cfg: Config, rng: random.Random):
        self.cfg, self.rng = cfg, rng
        self.pos = [3.0, 1.0]
        self.tilstand = "hviler"
        self.timer = rng.uniform(3, 8)
        self.mal = None
        self.synlig = True

    def _tilfeldig_punkt(self):
        c = self.cfg
        return [self.rng.uniform(*c.room_x), self.rng.uniform(*c.room_y)]

    def step(self, dt, godbiter):
        rng = self.rng
        if self.tilstand in ("hviler", "går") and godbiter:
            self.mal = min(godbiter, key=lambda g: math.dist(g, self.pos))
            self.tilstand = "løper"

        if self.tilstand == "hviler":
            self.timer -= dt
            if self.timer <= 0:
                if rng.random() < 0.15:
                    self.tilstand, self.mal = "går ut", list(self.cfg.door)
                else:
                    self.tilstand, self.mal = "går", self._tilfeldig_punkt()

        elif self.tilstand in self.FART:
            dx, dy = self.mal[0] - self.pos[0], self.mal[1] - self.pos[1]
            dist = math.hypot(dx, dy)
            steg = self.FART[self.tilstand] * dt
            if dist <= steg:
                self.pos = list(self.mal)
                if self.tilstand == "går":
                    self.tilstand, self.timer = "hviler", rng.uniform(5, 30)
                elif self.tilstand == "går ut":
                    self.tilstand, self.timer, self.synlig = "borte", rng.uniform(20, 60), False
                else:  # løper → spiser
                    if self.mal in godbiter:
                        godbiter.remove(self.mal)
                    self.tilstand, self.timer = "spiser", 3.0
            else:
                self.pos[0] += dx / dist * steg
                self.pos[1] += dy / dist * steg

        elif self.tilstand == "borte":
            self.timer -= dt
            if self.timer <= 0:
                self.synlig, self.tilstand, self.timer = True, "hviler", 2.0

        elif self.tilstand == "spiser":
            self.timer -= dt
            if self.timer <= 0:
                self.tilstand, self.timer = "hviler", rng.uniform(5, 20)


class Verden:
    def __init__(self, cfg: Config, rng: random.Random):
        self.cfg = cfg
        self.hund = Hund(cfg, rng)
        self.godbiter = []      # landet og ikke spist
        self.i_lufta = []       # skudd som ikke har landet ennå
        self.skudd = []
        self.spor = []          # (t, x, y, synlig)

    def i_rommet(self, p):
        c = self.cfg
        return 0.0 <= p[0] <= c.room_x[1] and c.room_y[0] <= p[1] <= c.room_y[1]

    def step(self, t, dt):
        for s in list(self.i_lufta):
            if t >= s["t_land"]:
                self.i_lufta.remove(s)
                s["hund_ved_landing"] = list(self.hund.pos)
                s["hund_synlig_ved_landing"] = self.hund.synlig
                if self.i_rommet(s["landing"]):
                    self.godbiter.append(list(s["landing"]))
                else:
                    s["utenfor"] = True
        self.hund.step(dt, self.godbiter)
        self.spor.append((t, *self.hund.pos, self.hund.synlig))


# ---------------------------------------------------------------------------
# Simulert maskinvare – samme grensesnitt som den ekte skal få
# ---------------------------------------------------------------------------
class SimServo:
    def __init__(self, start, fart_dps):
        self.cmd = self.pos = float(start)
        self.fart = fart_dps

    def step(self, dt):
        maks = self.fart * dt
        self.pos += max(-maks, min(maks, self.cmd - self.pos))


class SimMaskinvare:
    def __init__(self, cfg: Config, verden: Verden, ball: Ballistikk, motor: MotorModell,
                 rng: random.Random, feil_tid=None):
        self.cfg, self.verden, self.ball, self.motor, self.rng = cfg, verden, ball, motor, rng
        self.pan = SimServo(90, cfg.servo_speed_dps)
        self.tilt = SimServo(cfg.elev_deg, cfg.servo_speed_dps)
        self.duty = 0.0
        self.t = 0.0
        self.feil_tid = feil_tid
        self._mating = None     # tidspunkt godbiten når hjulene
        self.siste_sikte = None

    # --- I²C (PCA9685) ---
    def _i2c(self):
        if self.feil_tid is not None and self.t >= self.feil_tid:
            raise OSError("I²C: PCA9685 svarer ikke på adresse 0x40")

    def selvtest(self):
        self._i2c()

    def set_pan(self, deg):
        self._i2c()
        self.pan.cmd = max(0.0, min(180.0, deg))

    def set_tilt(self, deg):
        self._i2c()
        self.tilt.cmd = max(10.0, min(60.0, deg))

    def mat(self, sikte):
        self._i2c()
        self._mating = self.t + self.cfg.feed_s
        self.siste_sikte = sikte

    # --- GPIO18 hardware-PWM (DRV8871) ---
    def set_motor(self, duty):
        self.duty = max(0.0, min(1.0, duty))

    def nodstopp(self):
        self.duty = 0.0
        self._mating = None

    # --- Kamera + YOLO ---
    def detect(self):
        """Returnerer (piksel_x, avstand_m) for «dog», eller None."""
        c, h = self.cfg, self.verden.hund
        if not h.synlig:
            return None
        rel = math.atan2(h.pos[1], h.pos[0]) - math.radians(self.pan.pos - 90)
        if abs(rel) > math.radians(c.fov_h_deg / 2) * 0.95:
            return None
        if self.rng.random() < c.det_miss:
            return None
        px = c.img_w / 2 - c.focal_px * math.tan(rel) + self.rng.gauss(0, c.det_px_noise)
        d = math.hypot(*h.pos)
        if c.use_tof and abs(rel) < math.radians(c.tof_fov_deg / 2):
            d *= 1 + self.rng.gauss(0, c.dist_noise_tof)
        else:
            d *= 1 + self.rng.gauss(0, c.dist_noise_bbox)
        return px, d

    # --- tid ---
    def step(self, dt):
        self.t += dt
        self.pan.step(dt)
        self.tilt.step(dt)
        if self._mating is not None and self.t >= self._mating:
            self._mating = None
            self._avfyr()

    def _avfyr(self):
        c = self.cfg
        if self.duty < 0.05:
            return          # hjulene står: godbiten blir liggende i løpet
        v = self.motor.fart(self.duty) * (1 + self.rng.gauss(0, c.speed_sigma))
        retning = math.radians(self.pan.pos - 90) + math.radians(self.rng.gauss(0, c.pan_sigma_deg))
        xs, zs, tof = self.ball.bane(v, self.tilt.pos)
        lengde = xs[-1]
        s = dict(self.siste_sikte)
        s.update(
            nr=len(self.verden.skudd) + 1,
            t=self.t, v=v, tof=tof, elev=self.tilt.pos,
            landing=[lengde * math.cos(retning), lengde * math.sin(retning)],
            bane=(xs, zs), t_land=self.t + tof,
            hund_ved_skudd=list(self.verden.hund.pos), utenfor=False,
        )
        self.verden.skudd.append(s)
        self.verden.i_lufta.append(s)


# ---------------------------------------------------------------------------
# Tilstandsmaskinen – denne skal kjøre uendret på Pi-en
# ---------------------------------------------------------------------------
class Tilstand(Enum):
    OPPSTART = "Oppstart"
    VENTER = "Venter"
    BEKREFTER = "Bekrefter"
    SIKTER = "Sikter"
    SPINNER_OPP = "Spinner opp"
    MATER = "Mater"
    SKYTER = "Skyter"
    NEDKJOLING = "Nedkjøling"
    PAUSE = "Pause"
    SIKKER_STOPP = "Sikker stopp"


class Kontroller:
    def __init__(self, cfg: Config, hw, ball: Ballistikk, motor: MotorModell, logg=print):
        self.cfg, self.hw, self.ball, self.motor, self.logg = cfg, hw, ball, motor, logg
        self.tilstand = Tilstand.OPPSTART
        self.t_inn = 0.0
        self.t = 0.0
        self.skudd_i_dag = 0
        self.pan_cmd = 90.0
        self.scan_retning = 1
        self.historikk = [(0.0, self.tilstand)]
        self._nullstill()

    def _nullstill(self):
        self.treff = self.bom = self.stabil = 0
        self.forrige_feil = None
        self.avstander = []
        self.sist_sett = self.t
        self.varslet = False

    def _ga(self, ny: Tilstand, grunn: str):
        m, s = divmod(self.t, 60)
        self.logg(f"[{int(m):02d}:{s:04.1f}] {self.tilstand.value:>12} → {ny.value:<12} {grunn}")
        self.tilstand, self.t_inn = ny, self.t
        self.historikk.append((self.t, ny))
        self._nullstill()

    def step(self, t, dt):
        self.t = t
        try:
            self._step(dt)
        except OSError as e:
            self.hw.nodstopp()
            self._ga(Tilstand.SIKKER_STOPP, f"feil: {e}")

    def _step(self, dt):
        c, hw, T = self.cfg, self.hw, Tilstand
        tid_i_tilstand = self.t - self.t_inn

        if self.tilstand == T.OPPSTART:
            hw.selvtest()
            hw.set_pan(90)
            hw.set_tilt(c.elev_deg)
            hw.set_motor(0)
            if tid_i_tilstand >= 1.0:
                self._ga(T.VENTER, "kamera og servoer OK")
            return

        if self.tilstand in (T.PAUSE, T.SIKKER_STOPP):
            return

        det = hw.detect()

        if self.tilstand == T.VENTER:
            if det:
                self._ga(T.BEKREFTER, "«dog» funnet")
                self.treff = 1
            elif tid_i_tilstand > 3.0:               # sveip sakte for å se hele rommet
                self.pan_cmd += self.scan_retning * c.scan_speed_dps * dt
                if not 45 <= self.pan_cmd <= 135:
                    self.scan_retning *= -1
                    self.pan_cmd = max(45, min(135, self.pan_cmd))
                hw.set_pan(self.pan_cmd)

        elif self.tilstand == T.BEKREFTER:
            if det:
                self.treff += 1
                self.bom = 0
                if self.treff >= c.confirm_frames:
                    self._ga(T.SIKTER, f"sett i {c.confirm_frames} bilder på rad")
            else:
                self.bom += 1
                if self.bom >= c.confirm_miss:
                    self._ga(T.VENTER, "forsvant")

        elif self.tilstand == T.SIKTER:
            self._sikt(det, dt)

        elif self.tilstand == T.SPINNER_OPP:
            if tid_i_tilstand >= c.spinup_s:
                hw.mat(self.sikte)
                self._ga(T.MATER, "SG90 slipper én godbit")

        elif self.tilstand == T.MATER:
            if tid_i_tilstand >= c.feed_s:
                self._ga(T.SKYTER, "godbiten er i hjulene")

        elif self.tilstand == T.SKYTER:
            if tid_i_tilstand >= c.shoot_s:
                hw.set_motor(0)
                self.skudd_i_dag += 1
                self._ga(T.NEDKJOLING, f"skudd {self.skudd_i_dag} av {c.daily_max}, flywheels av")

        elif self.tilstand == T.NEDKJOLING:
            if self.skudd_i_dag >= c.daily_max:
                self._ga(T.PAUSE, "dagskvote nådd")
            elif tid_i_tilstand >= c.cooldown_s:
                self._ga(T.VENTER, f"{c.cooldown_s:.0f} s cooldown ferdig")

    def _sikt(self, det, dt):
        c, hw = self.cfg, self.hw
        if det is None:
            if self.t - self.sist_sett > c.lost_timeout:
                self._ga(Tilstand.VENTER, f"hund borte i over {c.lost_timeout:.0f} s")
            return

        px, d = det
        self.sist_sett = self.t
        feil = math.degrees(math.atan((c.img_w / 2 - px) / c.focal_px))   # + = hunden til venstre
        dfeil = 0.0 if self.forrige_feil is None else feil - self.forrige_feil
        self.forrige_feil = feil
        self.pan_cmd = max(0.0, min(180.0, self.pan_cmd + c.kp * feil + c.kd * dfeil))
        hw.set_pan(self.pan_cmd)

        self.avstander = (self.avstander + [d])[-5:]
        self.stabil = self.stabil + 1 if abs(feil) < c.stable_deg else 0
        if self.stabil < c.stable_frames or len(self.avstander) < 5:
            return

        d_hund = sorted(self.avstander)[2]          # median av siste 5
        d_mal = d_hund - c.aim_offset
        if d_mal < c.min_dist:
            if not self.varslet:
                self.logg(f"           hunden er for nær ({d_hund:.2f} m), skyter ikke")
                self.varslet = True
            return
        v = self.ball.finn_fart(d_mal, c.elev_deg)
        duty = None if v is None else self.motor.duty(v)
        if duty is None or duty > c.duty_max:
            if not self.varslet:
                self.logg(f"           hunden er utenfor rekkevidde ({d_hund:.2f} m)")
                self.varslet = True
            return

        retning = math.radians(self.pan_cmd - 90)
        self.sikte = dict(d_est=d_hund, d_mal=d_mal, v_cmd=v, duty=duty,
                          mal=[d_mal * math.cos(retning), d_mal * math.sin(retning)])
        hw.set_tilt(c.elev_deg)
        hw.set_motor(duty)
        self._ga(Tilstand.SPINNER_OPP,
                 f"d≈{d_hund:.2f} m, sikter på {d_mal:.2f} m, v={v:.2f} m/s, PWM {duty * 100:.0f} %")


# ---------------------------------------------------------------------------
# Kjøring og plott
# ---------------------------------------------------------------------------
def kjor(cfg: Config, minutter: float, seed: int, feil_tid=None, stille=False):
    rng = random.Random(seed)
    ball, motor = Ballistikk(cfg), MotorModell(cfg)
    verden = Verden(cfg, rng)
    hw = SimMaskinvare(cfg, verden, ball, motor, rng, feil_tid)
    logg = (lambda *_: None) if stille else print
    kontroller = Kontroller(cfg, hw, ball, motor, logg)

    dt = 1.0 / cfg.fps
    t = 0.0
    while t < minutter * 60:
        verden.step(t, dt)
        hw.step(dt)
        kontroller.step(t, dt)
        t += dt
    for _ in range(int(3 / dt)):                 # la siste skudd lande
        verden.step(t, dt)
        hw.step(dt)
        t += dt
    return verden, kontroller, ball, motor


def oppsummer(verden: Verden):
    skudd = [s for s in verden.skudd if "hund_ved_landing" in s]
    print("\n── Oppsummering " + "─" * 50)
    print(f"Skudd: {len(skudd)}")
    if not skudd:
        return
    print(f"{'nr':>3} {'d hund':>7} {'mål':>6} {'v':>6} {'PWM':>5} {'tid i lufta':>11} "
          f"{'avvik fra mål':>14} {'til hunden':>11}")
    avvik, til_hund = [], []
    for s in skudd:
        a = math.dist(s["landing"], s["mal"])
        h = math.dist(s["landing"], s["hund_ved_landing"])
        avvik.append(a)
        til_hund.append(h)
        merk = "  utenfor rommet" if s["utenfor"] else ""
        print(f"{s['nr']:>3} {math.hypot(*s['hund_ved_skudd']):>6.2f}m {s['d_mal']:>5.2f}m "
              f"{s['v']:>5.2f} {s['duty'] * 100:>4.0f}% {s['tof']:>10.2f}s "
              f"{a:>13.2f}m {h:>10.2f}m{merk}")
    n = len(skudd)
    print(f"\nSnitt avvik fra siktepunkt: {sum(avvik) / n:.2f} m")
    print(f"Snitt avstand fra hunden ved landing: {sum(til_hund) / n:.2f} m "
          f"(ønsket ≈ {Config.aim_offset:.1f} m)")
    print(f"Landet innen 0,75 m fra hunden: {sum(h < 0.75 for h in til_hund)} av {n}")


def plott(verden, kontroller, ball, motor, cfg: Config, fil, vis=False):
    import matplotlib
    if not vis:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    farge_hund, farge_skudd, farge_mal = "#8a8f98", "#d9631e", "#2f6fb0"
    fig, ax = plt.subplots(2, 2, figsize=(14, 10.5))
    fig.suptitle("Godbit-kanon – simulering", fontsize=15, fontweight="bold")

    # 1. Rommet ovenfra
    a = ax[0, 0]
    c = cfg
    a.add_patch(plt.Rectangle((0, c.room_y[0]), c.room_x[1], c.room_y[1] - c.room_y[0],
                              fill=False, ec="#444", lw=1.2))
    spor = np.array([(x, y) for _, x, y, synlig in verden.spor if synlig])
    if len(spor):
        a.plot(spor[:, 0], spor[:, 1], ".", ms=1, color=farge_hund, alpha=0.35, label="Hundens bevegelser")
    a.plot(0, 0, "s", ms=11, color="#222", label="Kanon")
    a.plot(*c.door, marker=(3, 0, 0), ms=11, color="#666", ls="none", label="Dør")
    for s in verden.skudd:
        if "hund_ved_landing" not in s:
            continue
        a.plot([0, s["landing"][0]], [0, s["landing"][1]], color=farge_skudd, lw=0.6, alpha=0.4)
        a.plot(*s["mal"], "x", color=farge_mal, ms=8, mew=2)
        a.plot(*s["landing"], "o", color=farge_skudd, ms=7)
        a.plot(*s["hund_ved_landing"], "o", mfc="none", mec="#222", ms=11)
        a.annotate(str(s["nr"]), s["landing"], xytext=(5, 5), textcoords="offset points", fontsize=9)
    a.plot([], [], "x", color=farge_mal, mew=2, label="Siktepunkt")
    a.plot([], [], "o", color=farge_skudd, label="Landing")
    a.plot([], [], "o", mfc="none", mec="#222", label="Hunden ved landing")
    a.set_aspect("equal")
    a.set_xlim(-0.3, c.room_x[1] + 0.3)
    a.set_ylim(c.room_y[0] - 0.3, c.room_y[1] + 0.3)
    a.set_xlabel("x (m)")
    a.set_ylabel("y (m)")
    a.set_title("Rommet ovenfra")
    a.legend(loc="lower left", fontsize=8, framealpha=0.9)

    # 2. Banene fra siden
    a = ax[0, 1]
    for s in verden.skudd:
        xs, zs = s["bane"]
        a.plot(xs, zs, color=farge_skudd, lw=1.2, alpha=0.8)
    if verden.skudd:
        s0 = verden.skudd[0]
        xs, zs, _ = ball.bane(s0["v"], s0["elev"], drag=False)
        a.plot(xs, zs, "--", color="#222", lw=1, label=f"Skudd 1 uten luftmotstand")
        a.plot([], [], color=farge_skudd, label="Med luftmotstand")
        a.legend(fontsize=8)
    a.axhline(0, color="#444", lw=1)
    a.set_xlabel("Horisontal avstand (m)")
    a.set_ylabel("Høyde (m)")
    a.set_title(f"Godbitbaner fra siden (utskytingsvinkel {cfg.elev_deg:.0f}°)")
    a.grid(alpha=0.25)

    # 3. Kalibreringskurve
    a = ax[1, 0]
    ds = np.linspace(cfg.min_dist, 5.0, 25)
    for elev, stil in ((cfg.elev_deg, "-"), (45.0, ":")):
        for drag, farge in ((True, farge_skudd), (False, "#222")):
            farter = [ball.finn_fart(d, elev, drag) for d in ds]
            duty = [np.nan if v is None else motor.duty(v) * 100 for v in farter]
            navn = f"{elev:.0f}°, {'med' if drag else 'uten'} luftmotstand"
            a.plot(ds, duty, stil, color=farge, lw=1.6, label=navn)
    a.axhline(cfg.duty_max * 100, color="#b33", lw=1, ls="--", label="Maks PWM")
    a.set_xlabel("Landingsavstand (m)")
    a.set_ylabel("PWM duty (%)")
    a.set_title("Kalibrering: avstand → PWM")
    a.grid(alpha=0.25)
    a.legend(fontsize=8)

    # 4. Tidslinje over tilstander
    a = ax[1, 1]
    rekkefolge = list(Tilstand)
    hist = kontroller.historikk + [(kontroller.t, kontroller.tilstand)]
    ts = [h[0] / 60 for h in hist]
    ys = [rekkefolge.index(h[1]) for h in hist]
    a.step(ts, ys, where="post", color=farge_mal, lw=1.2)
    for s in verden.skudd:
        a.axvline(s["t"] / 60, color=farge_skudd, lw=0.8, alpha=0.5)
    a.set_yticks(range(len(rekkefolge)))
    a.set_yticklabels([s.value for s in rekkefolge])
    a.invert_yaxis()
    a.set_xlabel("Tid (minutter)")
    a.set_title("Tilstandsmaskinen over tid (oransje = skudd)")
    a.grid(alpha=0.25, axis="x")

    fig.tight_layout()
    fig.savefig(fil, dpi=130)
    print(f"\nPlott lagret i {fil}")
    if vis:
        plt.show()


def main():
    p = argparse.ArgumentParser(description="Simulator for godbit-kanonen")
    p.add_argument("--minutter", type=float, default=15, help="simulert tid (standard 15)")
    p.add_argument("--seed", type=int, default=1, help="tilfeldig frø, gir samme forløp hver gang")
    p.add_argument("--tof", action="store_true", help="bruk VL53L1X i stedet for avstand fra bildet")
    p.add_argument("--elev", type=float, default=30, help="utskytingsvinkel i grader (standard 30)")
    p.add_argument("--kvote", type=int, default=8, help="maks skudd per dag (standard 8)")
    p.add_argument("--cooldown", type=float, default=30, help="sekunder mellom skudd (standard 30)")
    p.add_argument("--feil", type=float, default=None, metavar="SEK", help="simuler I²C-feil etter SEK sekunder")
    p.add_argument("--stille", action="store_true", help="ikke skriv tilstandsoverganger")
    p.add_argument("--ut", default="kanon_sim.png", help="filnavn for plott")
    p.add_argument("--vis", action="store_true", help="vis plott i et vindu")
    a = p.parse_args()

    cfg = Config(use_tof=a.tof, elev_deg=a.elev, daily_max=a.kvote, cooldown_s=a.cooldown)
    verden, kontroller, ball, motor = kjor(cfg, a.minutter, a.seed, a.feil, a.stille)
    oppsummer(verden)
    plott(verden, kontroller, ball, motor, cfg, a.ut, a.vis)


if __name__ == "__main__":
    main()
