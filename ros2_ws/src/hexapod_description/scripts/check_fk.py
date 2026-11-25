#!/usr/bin/env python3
"""Verifie l'URDF : position des bouts de patte calculee par l'URDF (chaine des joints)
et par hexapod_model.py, pour des poses tirees au hasard dans les butees.
Critere P1 (tps_archi.md) : ecart < 2 mm.

    python3 check_fk.py                 # genere l'URDF avec xacro (paquet compile ou source)
    python3 check_fk.py --urdf robot.urdf --n 5000
"""
import argparse
import math
import os
import random
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(PKG, "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "Dyna_test", "gui"))
import hexapod_model as hm  # noqa: E402


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K


def load_joints(urdf_xml):
    root = ET.fromstring(urdf_xml)
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        ax = j.find("axis")
        lim = j.find("limit")
        joints[j.find("child").get("link")] = {
            "name": j.get("name"), "type": j.get("type"), "parent": j.find("parent").get("link"),
            "xyz": np.array(xyz), "R": rpy_matrix(*rpy),
            "axis": [float(v) for v in ax.get("xyz").split()] if ax is not None else None,
            "limit": (float(lim.get("lower")), float(lim.get("upper"))) if lim is not None else None,
        }
    return joints


def link_pose(joints, link, q):
    """Pose (R, p) de `link` dans base_link."""
    chain = []
    while link in joints and link != "base_link":
        chain.append(joints[link])
        link = joints[link]["parent"]
    R, p = np.eye(3), np.zeros(3)
    for j in reversed(chain):
        p = p + R @ j["xyz"]
        R = R @ j["R"]
        if j["type"] == "revolute":
            R = R @ axis_angle(j["axis"], q.get(j["name"], 0.0))
    return R, p


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--urdf", help="URDF deja genere (sinon : xacro)")
    a.add_argument("--n", type=int, default=2000, help="poses par patte")
    args = a.parse_args()

    if args.urdf:
        xml = open(args.urdf, encoding="utf-8").read()
    else:
        xml = subprocess.check_output([
            "xacro", os.path.join(PKG, "urdf", "hexapod.urdf.xacro"),
            "legs_yaml:=" + os.path.join(PKG, "config", "legs.yaml")], text=True)
    joints = load_joints(xml)
    cfg = hm.load_config()
    rnd = random.Random(1)
    worst = 0.0
    for n, leg in enumerate(cfg["legs"], start=1):
        names = [f"leg_{n}_{j}_joint" for j in ("coxa", "femur", "tibia")]
        lims = [next(v["limit"] for v in joints.values() if v["name"] == nm) for nm in names]
        err_leg = 0.0
        for _ in range(args.n):
            q = {nm: rnd.uniform(*lim) for nm, lim in zip(names, lims)}
            _, p = link_pose(joints, f"leg_{n}_foot_link", q)
            ang = [math.degrees(q[nm]) for nm in names]
            ref = hm.leg_to_body(cfg, leg, ang)
            ref_p = np.array([ref["foot"][0], ref["foot"][1], ref["foot_z"]]) / 1000.0
            err_leg = max(err_leg, float(np.linalg.norm(p - ref_p)) * 1000)
        worst = max(worst, err_leg)
        print(f"patte {n} ({leg['name']:15s}) : ecart max {err_leg:.4f} mm sur {args.n} poses")
    ok = worst < 2.0
    print(f"{'OK' if ok else 'ECHEC'} : ecart max {worst:.4f} mm (critere < 2 mm)")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
