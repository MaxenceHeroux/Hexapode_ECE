#!/usr/bin/env python3
"""Genere config/legs.yaml (parametres de l'URDF) a partir de la calibration des pattes.

Sources :
  - Dyna_test/gui/leg_gui_config.json : longueurs, tibia cintre, decalage du pied,
    montage des pattes, IDs, sens, offsets (calibration faite dans leg_gui) ;
  - butees EEPROM des servos (CW / CCW), lues sur leg_server si --host est donne,
    sinon reprises du legs.yaml existant.

Les angles d'articulation sont ceux de hexapod_model.py (0 = servo a 512 + offset), en
radians ; l'URDF les utilise tels quels, donc ROS et hexapod_model ont la meme convention.

    python3 gen_legs_yaml.py --host 192.168.137.10      # relit aussi les butees
    python3 gen_legs_yaml.py                            # garde les butees du yaml
"""
import argparse
import json
import math
import os
import socket
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(PKG, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "Dyna_test", "gui"))
import hexapod_model as hm  # noqa: E402

YAML = os.path.join(PKG, "config", "legs.yaml")
JOINTS = ("coxa", "femur", "tibia")

# Masses estimees (kg), a remplacer par les pesees / proprietes de masse SolidWorks.
# AX-12A = 54.6 g (doc ROBOTIS). Une patte = 3 servos + supports imprimes + pied.
MASSES = {"body": 0.60, "coxa": 0.075, "femur": 0.080, "tibia": 0.045}


def read_limits(host, port):
    s = socket.create_connection((host, port), timeout=5)
    f = s.makefile()
    s.sendall(b"STATE\n")
    line = f.readline()
    s.close()
    data = json.loads(line[line.index("{"):])
    return {v["id"]: (v["cw_limit"], v["ccw_limit"]) for v in data["servos"] if v.get("ok")}


def old_limits():
    """Butees brutes deja dans legs.yaml (commentaire 'raw')."""
    out = {}
    if not os.path.exists(YAML):
        return out
    for line in open(YAML, encoding="utf-8"):
        if "# raw" in line and "id " in line:
            # ex : "      coxa: [-1.23, 1.23]   # raw id 3 : 224..800"
            part = line.split("# raw")[1]
            i = int(part.split("id")[1].split(":")[0])
            a, b = part.split(":")[1].split("..")
            out[i] = (int(a), int(b))
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", help="leg_server a interroger pour les butees")
    p.add_argument("--port", type=int, default=5555)
    p.add_argument("--config", default=hm.CONFIG_FILE, help="leg_gui_config.json")
    a = p.parse_args()

    if not os.path.exists(a.config):
        sys.exit(f"Calibration absente : {a.config} (voir Dyna_test/test_torque.md section 1)")
    cfg = hm.load_config(a.config)
    limits = read_limits(a.host, a.port) if a.host else old_limits()

    L = cfg["lengths_mm"]
    fo = cfg.get("foot_offset_mm", {})
    vx, vz = hm._tibia_vector(cfg)
    out = []
    w = out.append
    w("# Parametres de l'URDF de l'hexapode. GENERE par scripts/gen_legs_yaml.py depuis")
    w("# Dyna_test/gui/leg_gui_config.json (+ butees EEPROM des servos). Ne pas editer a la main.")
    w("# Unites : metres, radians. Angles = convention de hexapod_model.py (0 = servo 512 + offset).")
    w("geometry:")
    w(f"  coxa: {L['coxa'] / 1000:.5f}       # hanche -> axe du femur")
    w(f"  femur: {L['femur'] / 1000:.5f}      # axe du femur -> axe du genou")
    w(f"  tibia: {L['tibia'] / 1000:.5f}      # genou -> bout de patte (avant decalage)")
    w(f"  tibia_bend: {math.radians(cfg.get('tibia_bend_deg', 0)):.6f}   # {cfg.get('tibia_bend_deg', 0)} deg")
    w(f"  foot_along: {float(fo.get('along', 0)) / 1000:.4f}")
    w(f"  foot_perp: {float(fo.get('perp', 0)) / 1000:.4f}")
    w(f"  foot_x: {vx / 1000:.6f}    # genou -> bout de patte, repere du tibia (calcule)")
    w(f"  foot_z: {vz / 1000:.6f}")
    w("masses:   # ESTIMATIONS (kg) - a remplacer par les pesees")
    for k, m in MASSES.items():
        w(f"  {k}: {m}")
    w("legs:")
    missing = []
    for n, leg in enumerate(cfg["legs"], start=1):
        m = leg["mount"]
        w(f"  leg_{n}:")
        w(f"    name: \"{leg['name']}\"")
        w(f"    ids: {leg['ids']}")
        w(f"    inverted: {[bool(x) for x in leg['inverted']]}")
        w(f"    offset_deg: {[float(x) for x in leg['offset_deg']]}")
        w(f"    side: {'right' if m['y'] < 0 else 'left'}   # maillages meshes/leg_<side>_*.dae")
        w(f"    mount: [{m['x'] / 1000:.4f}, {m['y'] / 1000:.4f}, {math.radians(m['angle']):.6f}]   # x, y, lacet ({m['angle']} deg)")
        w("    limits:")
        for k, j in enumerate(JOINTS):
            sid = leg["ids"][k]
            if sid in limits:
                cw, ccw = limits[sid]
                q = sorted(math.radians(hm.raw_to_joint(leg, k, r)) for r in (cw, ccw))
                w(f"      {j}: [{q[0]:.4f}, {q[1]:.4f}]   # raw id {sid} : {cw}..{ccw}")
            else:
                missing.append(sid)
                q = sorted(math.radians(hm.raw_to_joint(leg, k, r)) for r in (0, 1023))
                w(f"      {j}: [{q[0]:.4f}, {q[1]:.4f}]   # butee inconnue (id {sid}) : course complete")
    open(YAML, "w", encoding="utf-8").write("\n".join(out) + "\n")
    print(f"{YAML} ecrit" + (f" ; butees inconnues pour {missing}" if missing else ""))


if __name__ == "__main__":
    main()
