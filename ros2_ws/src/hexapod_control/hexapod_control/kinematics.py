"""Cinematique des pattes, en metres et radians, dans le repere base_link.

Memes formules et conventions que Dyna_test/gui/hexapod_model.py (et que l'URDF) :
  - repere de la patte : x vers l'exterieur, z vers le haut, origine sur l'axe de la hanche ;
  - coxa autour de z (0 = dans l'axe du montage), femur et tibia autour de -y
    (angle > 0 = vers le haut) ;
  - bout de patte a (foot_x, 0, foot_z) dans le repere du genou (tibia cintre).
Les parametres viennent de hexapod_description/config/legs.yaml.
"""
import math
import os

import yaml

JOINTS = ("coxa", "femur", "tibia")


def default_legs_yaml():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("hexapod_description"), "config", "legs.yaml")


class Leg:
    def __init__(self, n, g, d):
        self.n = n
        self.name = d["name"]
        self.mx, self.my, self.myaw = (float(v) for v in d["mount"])
        self.lc, self.lf = float(g["coxa"]), float(g["femur"])
        self.vx, self.vz = float(g["foot_x"]), float(g["foot_z"])
        self.le = math.hypot(self.vx, self.vz)
        self.phi = math.atan2(self.vz, self.vx)
        self.limits = [tuple(float(v) for v in d["limits"][j]) for j in JOINTS]
        self.joint_names = [f"leg_{n}_{j}_joint" for j in JOINTS]

    def neutral(self, radius, z):
        """Bout de patte dans l'axe du montage, a `radius` de l'axe de la hanche."""
        return (self.mx + radius * math.cos(self.myaw), self.my + radius * math.sin(self.myaw), z)

    def ik(self, x, y, z):
        """Bout de patte (base_link) -> (coxa, femur, tibia) en radians, ou None."""
        dx, dy = x - self.mx, y - self.my
        c = math.atan2(dy, dx) - self.myaw
        c = math.atan2(math.sin(c), math.cos(c))
        px, pz = math.hypot(dx, dy) - self.lc, z
        d = math.hypot(px, pz)
        if d < 1e-9 or d > self.lf + self.le or d < abs(self.lf - self.le):
            return None
        cos_a = (self.lf ** 2 + d * d - self.le ** 2) / (2 * self.lf * d)
        a = math.acos(max(-1.0, min(1.0, cos_a)))
        f = math.atan2(pz, px) + a                       # genou vers le haut
        kx, kz = self.lf * math.cos(f), self.lf * math.sin(f)
        t = math.atan2(pz - kz, px - kx) - self.phi - f
        t = math.atan2(math.sin(t), math.cos(t))
        return (c, f, t)

    def fk(self, q):
        c, f, t = q
        r = self.lc + self.lf * math.cos(f) + self.vx * math.cos(f + t) - self.vz * math.sin(f + t)
        z = self.lf * math.sin(f) + self.vx * math.sin(f + t) + self.vz * math.cos(f + t)
        a = self.myaw + c
        return (self.mx + r * math.cos(a), self.my + r * math.sin(a), z)

    def in_limits(self, q, margin=0.0):
        return all(lo + margin <= v <= hi - margin for v, (lo, hi) in zip(q, self.limits))


def load_legs(path=None):
    with open(path or default_legs_yaml(), encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    g = cfg["geometry"]
    return [Leg(n, g, cfg["legs"][f"leg_{n}"]) for n in range(1, 7)]
