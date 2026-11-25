"""Modele de l'hexapode partage par leg_gui.py et test_torque.py.

- configuration (6 pattes : IDs, sens, offsets, position sur le corps) dans
  leg_gui_config.json ;
- conversions position servo (0..1023) <-> angle d'articulation (deg) ;
- cinematique directe et inverse d'une patte (coxa / femur / tibia cintre).

Conventions
-----------
Repere du corps : x vers l'avant, y vers la gauche, z vers le haut (mm).
Repere d'une patte : origine sur l'axe de la coxa ; l'angle de montage `mount.angle`
(deg, 0 = vers l'avant, 90 = vers la gauche) donne la direction de la patte quand la
coxa est a 0 deg. Dans le plan de la patte : r = distance a l'axe de la coxa, z = hauteur.
Angles d'articulation (deg) : coxa + = sens trigo vu de dessus ; femur + = vers le haut ;
tibia = angle de l'axe du genou par rapport au femur (+ = vers le haut).
Angle articulation = angle servo (512 = 0, sens selon `inverted`) + offset.
"""

import json
import math
import os

DEG_PER_UNIT = 300.0 / 1024.0          # AX : 0..1023 <-> 0..300 deg, 512 = milieu
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "leg_gui_config.json")
JOINT_NAMES = ("Coxa (hanche)", "Femur", "Tibia")

# Disposition par defaut : IDs par triplets en tournant autour du corps (a verifier /
# corriger dans l'interface, cadre "Configuration des pattes"). Les pattes du cote du
# trio 12/13/14 (deja calibre) reprennent son sens, celles d'en face le sens oppose.
_SIDE_A = {"inverted": [True, False, False]}          # sens calibre sur la patte 12/13/14
_SIDE_B = {"inverted": [False, True, True]}           # miroir (a verifier)
DEFAULT_LEGS = [
    {"name": "Avant gauche",   "ids": [0, 1, 2],    **_SIDE_A, "mount": {"x": 80,  "y": 60,  "angle": 45}},
    {"name": "Avant droite",   "ids": [3, 4, 5],    **_SIDE_B, "mount": {"x": 80,  "y": -60, "angle": -45}},
    {"name": "Milieu droite",  "ids": [6, 7, 8],    **_SIDE_B, "mount": {"x": 0,   "y": -80, "angle": -90}},
    {"name": "Arriere droite", "ids": [9, 10, 11],  **_SIDE_B, "mount": {"x": -80, "y": -60, "angle": -135}},
    {"name": "Arriere gauche", "ids": [12, 13, 14], **_SIDE_A, "mount": {"x": -80, "y": 60,  "angle": 135}},
    {"name": "Milieu gauche",  "ids": [15, 16, 17], **_SIDE_A, "mount": {"x": 0,   "y": 80,  "angle": 90}},
]
for _leg in DEFAULT_LEGS:
    _leg["offset_deg"] = [0.0, 0.0, 0.0]

DEFAULT_CONFIG = {
    "host": "192.168.137.10",
    "port": 5555,
    "lengths_mm": {"coxa": 50, "femur": 75, "tibia": 125},   # tibia = genou -> bout de patte
    # Courbure du tibia : angle (deg) entre l'axe du servo du genou et la droite
    # genou -> bout de patte. Le tibia du modele est cintre vers le corps.
    "tibia_bend_deg": -30,
    # Decalage du bout de patte (mm) dans le repere du tibia : "along" = dans l'axe du
    # genou (+ = plus loin), "perp" = perpendiculaire (+ = vers le haut patte horizontale).
    "foot_offset_mm": {"along": 0, "perp": 0},
    "show_limits": True,      # lignes rouges des butees CW / CCW sur le dessin
    "legs": DEFAULT_LEGS,
    "selected_leg": 4,        # patte affichee dans les panneaux (12/13/14)
    # Mouvement de pas de l'interface : ECARTS (deg) coxa, femur, tibia autour de la pose
    # de depart.
    "gait": [[-15, 25, 20], [15, 25, 20], [15, 0, 0], [-15, 0, 0]],
}


def _copy(o):
    return json.loads(json.dumps(o))


def load_config(path=CONFIG_FILE):
    """Config par defaut + fichier utilisateur. Migre l'ancien format (une seule patte,
    cle "joints") vers "legs"."""
    cfg = _copy(DEFAULT_CONFIG)
    user = {}
    try:
        with open(path, encoding="utf-8") as f:
            user = json.load(f)
    except (OSError, ValueError):
        pass
    cfg.update({k: v for k, v in user.items() if k in cfg})
    if "legs" not in user and "joints" in user:            # ancien format
        old = user["joints"]
        ids = [j["id"] for j in old]
        for leg in cfg["legs"]:
            if leg["ids"] == ids:
                leg["inverted"] = [bool(j["inverted"]) for j in old]
                leg["offset_deg"] = [float(j["offset_deg"]) for j in old]
    for leg in cfg["legs"]:                                 # champs manquants
        leg.setdefault("name", "Patte")
        leg.setdefault("inverted", [False, False, False])
        leg.setdefault("offset_deg", [0.0, 0.0, 0.0])
        leg.setdefault("mount", {"x": 0, "y": 0, "angle": 0})
    cfg["selected_leg"] = max(0, min(len(cfg["legs"]) - 1, int(cfg.get("selected_leg", 0))))
    return cfg


def save_config(cfg, path=CONFIG_FILE):
    data = {k: v for k, v in cfg.items() if k != "joints"}   # "joints" = vue de l'interface
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
    except OSError:
        pass


def all_ids(cfg):
    out = []
    for leg in cfg["legs"]:
        for i in leg["ids"]:
            if i not in out:
                out.append(i)
    return out


# --------------------------------------------------------------------------- conversions
def raw_to_deg(raw, inverted):
    d = (raw - 512) * DEG_PER_UNIT
    return -d if inverted else d


def deg_to_raw(deg, inverted):
    d = -deg if inverted else deg
    return int(round(512 + d / DEG_PER_UNIT))


def raw_to_joint(leg, k, raw):
    """Position servo -> angle de l'articulation k (0 coxa, 1 femur, 2 tibia)."""
    return raw_to_deg(raw, leg["inverted"][k]) + float(leg["offset_deg"][k])


def joint_to_raw(leg, k, deg):
    return deg_to_raw(deg - float(leg["offset_deg"][k]), leg["inverted"][k])


# --------------------------------------------------------------------------- cinematique
def _tibia_vector(cfg):
    """Vecteur genou -> bout de patte dans le repere de l'axe du genou (x = axe)."""
    lt = cfg["lengths_mm"]["tibia"]
    b = math.radians(cfg.get("tibia_bend_deg", 0))
    fo = cfg.get("foot_offset_mm", {})
    return (lt * math.cos(b) + float(fo.get("along", 0)),
            lt * math.sin(b) + float(fo.get("perp", 0)))


def leg_fk(cfg, angles):
    """Cinematique directe dans le plan de la patte. angles = (coxa, femur, tibia) en deg.
    Renvoie knee, foot, ctrl (point de controle de la courbe du tibia) en (r, z) mm."""
    L = cfg["lengths_mm"]
    lc, lf, lt = L["coxa"], L["femur"], L["tibia"]
    _, f, t = (math.radians(a) for a in angles)
    axis = f + t                                            # axe du servo du genou
    vx, vz = _tibia_vector(cfg)
    knee = (lc + lf * math.cos(f), lf * math.sin(f))
    foot = (knee[0] + vx * math.cos(axis) - vz * math.sin(axis),
            knee[1] + vx * math.sin(axis) + vz * math.cos(axis))
    ctrl = (knee[0] + 0.55 * lt * math.cos(axis), knee[1] + 0.55 * lt * math.sin(axis))
    return {"knee": knee, "foot": foot, "ctrl": ctrl}


def leg_ik(cfg, coxa_deg, r, z, knee_up=True):
    """Cinematique inverse dans le plan de la patte : bout de patte a (r, z) mm avec la
    coxa a coxa_deg. Renvoie (coxa, femur, tibia) en deg, ou None si hors d'atteinte."""
    L = cfg["lengths_mm"]
    lc, lf = L["coxa"], L["femur"]
    vx, vz = _tibia_vector(cfg)
    le = math.hypot(vx, vz)                  # longueur effective genou -> bout de patte
    phi = math.atan2(vz, vx)                 # angle bout de patte / axe du genou
    px, pz = r - lc, z
    d = math.hypot(px, pz)
    if d < 1e-6 or d > lf + le or d < abs(lf - le):
        return None
    cos_a = (lf * lf + d * d - le * le) / (2 * lf * d)
    a = math.acos(max(-1.0, min(1.0, cos_a)))
    f = math.atan2(pz, px) + (a if knee_up else -a)
    kx, kz = lf * math.cos(f), lf * math.sin(f)
    chord = math.atan2(pz - kz, px - kx)
    t = chord - phi - f
    t = math.atan2(math.sin(t), math.cos(t))  # ramene dans -180..180
    return (coxa_deg, math.degrees(f), math.degrees(t))


def body_to_leg(leg, x, y, z):
    """Bout de patte (x, y, z) dans le repere du corps -> (coxa_deg, r, z) de la patte."""
    m = leg["mount"]
    dx, dy = x - m["x"], y - m["y"]
    a = math.degrees(math.atan2(dy, dx)) - m["angle"]
    a = (a + 180) % 360 - 180
    return a, math.hypot(dx, dy), z


def leg_to_body(cfg, leg, angles):
    """Angles d'une patte -> positions (x, y) corps de la coxa, du genou et du bout + z."""
    m = leg["mount"]
    g = leg_fk(cfg, angles)
    A = math.radians(m["angle"] + angles[0])
    pts = {}
    for name, r in (("hip", 0.0), ("coxa_end", cfg["lengths_mm"]["coxa"]),
                    ("knee", g["knee"][0]), ("foot", g["foot"][0])):
        pts[name] = (m["x"] + r * math.cos(A), m["y"] + r * math.sin(A))
    pts["foot_z"] = g["foot"][1]
    return pts


def body_ik(cfg, leg, x, y, z):
    """Bout de patte vise dans le repere du corps -> angles (deg) ou None."""
    c, r, zz = body_to_leg(leg, x, y, z)
    return leg_ik(cfg, c, r, zz)
