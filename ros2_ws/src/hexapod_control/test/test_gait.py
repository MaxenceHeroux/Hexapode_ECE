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


def run(legs, cmd, seconds, height=0.12, new_height=None, gait="tripod"):
    g = TripodGait(legs, body_height=height, stance_radius=0.12, cycle_time=0.8,
                   max_linear=0.20, max_angular=0.8, gait=gait)
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


@pytest.mark.parametrize("cmd", [(0.10, 0, 0), (-0.10, 0, 0), (0, 0.08, 0), (0, 0, 0.6), (0.07, 0.0, 0.4),
                                 (0.20, 0, 0), (-0.20, 0, 0), (0, 0.20, 0), (0, 0, 0.8), (0.15, 0.0, 0.6)])
@pytest.mark.parametrize("gait", ["tripod", "tetrapod", "wave"])
def test_walk(legs, cmd, gait):
    g, pose, slip = run(legs, cmd, 6.0, gait=gait)
    assert slip < 0.004          # glissement d'un pied en appui < 4 mm par pas de temps (rampes)
    if cmd[0]:
        assert pose[0] * cmd[0] > 0


def test_height_change_while_walking(legs):
    g, _, _ = run(legs, (0.08, 0, 0), 6.0, height=0.08, new_height=0.12)
    assert abs(g.height - 0.12) < 1e-3


def test_stops_without_extra_steps(legs):
    """Consigne relachee : le robot repose les pattes en l'air puis s'arrete (au plus un
    demi-cycle), sans pas supplementaires pour revenir au neutre."""
    g = TripodGait(legs)
    g.set_cmd(0.1, 0, 0)
    for _ in range(150):
        g.step(0.02)
    g.set_cmd(0, 0, 0)
    for k in range(200):
        g.step(0.02)
        if not g.walking:
            break
    assert not g.walking
    assert k * 0.02 <= g.cycle_time * 0.5 + 0.6     # freinage (0,5 s) + fin du demi-pas
    for lg in legs:
        assert abs(g.feet[lg.n][2] + g.height) < 1e-9          # tous les pieds au sol


def _all_ok(g, legs):
    for lg in legs:
        q = lg.ik(*g.feet[lg.n])
        assert q is not None and lg.in_limits(q), \
            f"patte {lg.n} hors butees a h={g.height:.3f} r={g.radius:.3f}"


@pytest.mark.parametrize("cmd", [(0, 0, 0), (0.08, 0, 0), (0, 0, 0.5)])
def test_height_sweep_adapts_radius(legs, cmd):
    """Hauteur de haut en bas et retour, avec les pattes serrees voulues (0.12) :
    le rayon s'adapte et aucune pose ne sort des butees."""
    g = TripodGait(legs, stance_radius=0.12, body_height=0.12, height_limits=(0.05, 0.15))   # bornes du noeud
    g.set_cmd(*cmd)
    for target in (g.h_min, g.h_max, 0.08, 0.12):
        g.set_height(target)
        for _ in range(int(8 / 0.02)):
            g.step(0.02)
            _all_ok(g, legs)
        assert abs(g.height - g.height_target) < 1e-3, (target, g.height)


def test_cadence_follows_speed(legs):
    """Lent : pas lents (2 x cycle_time) ; vite : cycle_time ; le pas ne depasse jamais 2 x reach."""
    g = TripodGait(legs, cycle_time=0.8, max_linear=0.20)
    g.set_cmd(0.02, 0, 0)
    g.vel = g.cmd
    assert g.cycle_period() == pytest.approx(1.6)
    g.set_cmd(0.20, 0, 0)
    g.vel = g.cmd
    assert g.cycle_period() == pytest.approx(0.8)
    for v in (0.03, 0.08, 0.13, 0.2):
        g.set_cmd(v, 0, 0)
        g.vel = g.cmd
        assert g.foot_speed(g.vel) * g.duty * g.cycle_period() <= 2 * g.reach + 1e-9


@pytest.mark.parametrize("gait,min_feet", [("tripod", 3), ("tetrapod", 4), ("wave", 5)])
def test_feet_on_ground(legs, gait, min_feet):
    """Nombre de pieds au sol a tout instant, et recouvrement (plus de 3 / 4 / 5 par moments)."""
    g = TripodGait(legs, gait=gait)
    g.set_cmd(0.1, 0, 0)
    counts = []
    for _ in range(400):
        g.step(0.02)
        if g.walking:
            counts.append(sum(g.in_stance(lg.n) for lg in legs))
    assert min(counts) >= min_feet and max(counts) > min_feet


def test_pattern_change_waits_for_stop(legs):
    g = TripodGait(legs)
    g.set_cmd(0.1, 0, 0)
    for _ in range(50):
        g.step(0.02)
    g.set_pattern("wave")
    assert g.pattern == "tripod"
    g.set_cmd(0, 0, 0)
    for _ in range(200):
        g.step(0.02)
    assert g.pattern == "wave" and not g.walking
