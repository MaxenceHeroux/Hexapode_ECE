"""Figures (sans ROS, testees seules) : coucou, danse, cabrer, coeur.

Chaque figure decrit, en fonction du temps :
  - la POSE DU CORPS (translation x, y, z et roulis / tangage / lacet) par rapport aux
    pieds poses au sol ;
  - les PATTES LEVEES et la trajectoire de leur bout de patte.
Entree et sortie en douceur (`BLEND` s) depuis / vers la position debout. Les pieds qui
restent au sol ne bougent pas dans le monde : c'est le corps qui se deplace au-dessus.
Pour lever une patte, le corps se decale d'abord a l'oppose (centre de gravite au-dessus
des 5 autres pattes).

Pattes : 1 avant gauche, 2 avant droite, 3 milieu droite, 4 arriere droite,
5 arriere gauche, 6 milieu gauche. Repere : x avant, y gauche, z haut.
"""
import math

BLEND = 1.2       # s d'entree / de sortie


def _smooth(s):
    s = min(1.0, max(0.0, s))
    return s * s * (3 - 2 * s)


def _rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def _world_to_body(pose, p):
    """Point p (repere sol, origine = corps au repos) -> repere du corps a la pose `pose`."""
    x, y, z, r, pi, ya = pose
    R = _rot(r, pi, ya)
    d = (p[0] - x, p[1] - y, p[2] - z)
    return tuple(R[0][i] * d[0] + R[1][i] * d[1] + R[2][i] * d[2] for i in range(3))   # R^T d


# --------------------------------------------------------------------------- figures
# Chaque figure : duree (s), fonction(t, ctx) -> (pose du corps, {patte: point sol}) avec
# t de 0 a duree (hors entree / sortie). ctx : pieds au repos et geometrie.

def _lift_point(ctx, n, out=0.0, up=0.0, side=0.0):
    """Point au-dessus du pied au repos de la patte n : `out` vers l'exterieur, `side` de cote
    (sens trigo autour de la hanche), `up` vers le haut (m)."""
    lg = ctx["legs"][n]
    fx, fy, fz = ctx["feet"][n]
    a = lg.myaw
    return (fx + out * math.cos(a) - side * math.sin(a), fy + out * math.sin(a) + side * math.cos(a), fz + up)


def coucou(t, ctx):
    """Patte avant gauche levee, qui fait coucou (3 allers-retours)."""
    pose = (-0.03, -0.03, 0.005, math.radians(-4), 0.0, 0.0)          # corps vers l'arriere-droite
    lift = _smooth(t / 0.8) * _smooth((ctx["dur"] - t) / 0.8)
    wave = math.sin(2 * math.pi * 1.2 * max(0.0, t - 0.8)) if 0.8 < t < ctx["dur"] - 0.8 else 0.0
    p = _lift_point(ctx, 1, out=0.03 * lift, up=0.10 * lift, side=0.05 * wave * lift)
    return pose, {1: p}


def danse(t, ctx):
    """Le corps se balance en rythme, pieds au sol."""
    w = 2 * math.pi
    a = min(1.0, t / 1.0) * min(1.0, (ctx["dur"] - t) / 1.0)
    # amplitudes reduites apres l'essai du 26/11 (pics de charge des servos a 90 %)
    pose = (0.015 * math.sin(w * 0.5 * t) * a, 0.015 * math.cos(w * 0.5 * t) * a,
            0.008 * math.sin(w * 1.0 * t) * a,
            math.radians(4) * math.sin(w * 0.5 * t) * a, math.radians(3) * math.sin(w * 0.5 * t + 1.6) * a,
            math.radians(8) * math.sin(w * 0.25 * t) * a)
    return pose, {}


def cabrer(t, ctx):
    """Se cabre : corps vers l'arriere et nez en haut, pattes avant en l'air qui battent."""
    up = _smooth(t / 1.2) * _smooth((ctx["dur"] - t) / 1.2)
    beat = math.sin(2 * math.pi * 1.5 * t) * up
    # allege apres l'essai du 26/11 (pattes arriere a 93 % de charge, 57 unites de retard)
    pose = (-0.025 * up, 0.0, 0.0, 0.0, math.radians(-8) * up, 0.0)
    feet = {1: _lift_point(ctx, 1, out=0.01 * up, up=(0.06 + 0.015 * beat) * up),
            2: _lift_point(ctx, 2, out=0.01 * up, up=(0.06 - 0.015 * beat) * up)}
    return pose, feet


def coeur(t, ctx):
    """Dessine un coeur en l'air avec la patte avant gauche (2 fois)."""
    pose = (-0.03, -0.03, 0.005, math.radians(-4), 0.0, 0.0)
    lift = _smooth(t / 0.8) * _smooth((ctx["dur"] - t) / 0.8)
    if 0.8 < t < ctx["dur"] - 0.8:
        s = 2 * math.pi * 2 * (t - 0.8) / (ctx["dur"] - 1.6)          # 2 coeurs
        hx = 16 * math.sin(s) ** 3
        hy = 13 * math.cos(s) - 5 * math.cos(2 * s) - 2 * math.cos(3 * s) - math.cos(4 * s)
    else:
        hx, hy = 0.0, 5.0
    k = 0.0022 * lift                                                   # coeur de ~7 cm
    p = _lift_point(ctx, 1, out=0.03 * lift, up=0.09 * lift + k * (hy - 5.0), side=k * hx)
    return pose, {1: p}


FIGURES = {"coucou": (coucou, 6.0), "danse": (danse, 10.0), "cabrer": (cabrer, 7.0), "coeur": (coeur, 9.0)}


class ActionPlayer:
    def __init__(self, legs):
        self.legs = {lg.n: lg for lg in legs}
        self.name = None
        self.t = 0.0
        self.ctx = None
        self.stopping = False

    @property
    def active(self):
        return self.name is not None

    def start(self, name, feet_rest):
        if name not in FIGURES:
            raise ValueError(f"figure inconnue : {name} ({', '.join(FIGURES)})")
        self.name, self.t, self.stopping = name, 0.0, False
        self.ctx = {"legs": self.legs, "feet": {n: tuple(p) for n, p in feet_rest.items()},
                    "dur": FIGURES[name][1]}

    def stop(self):
        """Arret en douceur : la figure se fige et revient a la position debout en BLEND s."""
        if self.active and not self.stopping:
            self.stopping = True
            self.stop_env = self._env()
            self.stop_tf = min(FIGURES[self.name][1], max(0.0, self.t - BLEND))
            self.stop_t = 0.0

    def _env(self):
        total = FIGURES[self.name][1] + 2 * BLEND
        return _smooth(self.t / BLEND) * _smooth((total - self.t) / BLEND)

    def step(self, dt):
        """Avance de dt ; renvoie ({patte: bout de patte, repere corps}, pattes levees) ou
        None quand la figure est finie."""
        if not self.active:
            return None
        fn, dur = FIGURES[self.name]
        total = dur + 2 * BLEND
        if self.stopping:
            self.stop_t += dt
            if self.stop_t >= BLEND:
                self.name = None
                return None
            e = self.stop_env * (1 - _smooth(self.stop_t / BLEND))
            tf = self.stop_tf
        else:
            self.t += dt
            if self.t >= total:
                self.name = None
                return None
            e = self._env()
            tf = min(dur, max(0.0, self.t - BLEND))
        pose, lifted = fn(tf, self.ctx)
        pose = tuple(v * e for v in pose)
        feet = {}
        for n, rest in self.ctx["feet"].items():
            g = lifted.get(n, rest)
            g = tuple(r + (a - r) * e for r, a in zip(rest, g))
            feet[n] = _world_to_body(pose, g)
        return feet, set(lifted)
