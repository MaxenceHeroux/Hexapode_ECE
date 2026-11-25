"""Tests hors ROS : cinematique (meme resultat que l'URDF / hexapod_model) et marche
(toutes les poses atteignables et dans les butees, pieds fixes au sol en appui)."""
import math
import os
import random

import pytest

from hexapod_control.gait import TripodGait
from hexapod_control.kinematics import load_legs

YAML = os.path.join(os.path.dirname(__file__), "..", "..", "hexapod_description", "config", "legs.yaml")


@pytest.fixture(scope="module")
def legs():
    return load_legs(YAML)


def test_ik_fk_roundtrip(legs):
    rnd = random.Random(0)
    for lg in legs:
        for _ in range(500):
            q = [rnd.uniform(lo, hi) for lo, hi in lg.limits]
            p = lg.fk(q)
            q2 = lg.ik(*p)
            if q2 is None:
                continue
            assert math.dist(lg.fk(q2), p) < 1e-6


def run(legs, cmd, seconds, height=0.09, new_height=None):
    g = TripodGait(legs, body_height=height)
    g.set_cmd(*cmd)
    dt, worst = 0.02, []
    x = y = yaw = 0.0
    planted = {}
    for k in range(int(seconds / dt)):
        if new_height and k == int(seconds / dt / 2):
            g.set_height(new_height)
        vx, vy, wz = g.step(dt)
        c, s = math.cos(yaw), math.sin(yaw)
        for lg in legs:
            p = g.feet[lg.n]
            q = lg.ik(*p)
            assert q is not None, f"patte {lg.n} hors d'atteinte {p}"
            assert lg.in_limits(q), f"patte {lg.n} hors butees {[round(math.degrees(a)) for a in q]}"
            if g.in_stance(lg.n) and g.walking:                  # pied fixe dans le monde
                w = (x + p[0] * c - p[1] * s, y + p[0] * s + p[1] * c)
                if lg.n in planted:
                    worst.append(math.dist(w, planted[lg.n]))
                planted[lg.n] = w
            else:
                planted.pop(lg.n, None)
        x += (vx * c - vy * s) * dt
        y += (vx * s + vy * c) * dt
        yaw += wz * dt
    return g, (x, y, yaw), max(worst) if worst else 0.0


@pytest.mark.parametrize("cmd", [(0.10, 0, 0), (-0.10, 0, 0), (0, 0.08, 0), (0, 0, 0.6), (0.07, 0.0, 0.4)])
def test_walk(legs, cmd):
    g, pose, slip = run(legs, cmd, 6.0)
    assert slip < 0.004          # glissement d'un pied en appui < 4 mm par pas de temps (rampes)
    if cmd[0]:
        assert pose[0] * cmd[0] > 0


def test_height_change_while_walking(legs):
    g, _, _ = run(legs, (0.08, 0, 0), 6.0, height=0.08, new_height=0.12)
    assert abs(g.height - 0.12) < 1e-3


def test_stops_at_neutral(legs):
    g = TripodGait(legs)
    g.set_cmd(0.1, 0, 0)
    for _ in range(150):
        g.step(0.02)
    g.set_cmd(0, 0, 0)
    for _ in range(300):
        g.step(0.02)
    assert not g.walking
    for lg in legs:
        assert math.dist(g.feet[lg.n][:2], g.neutral(lg)[:2]) < 0.005
