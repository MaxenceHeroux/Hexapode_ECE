"""Figures : toutes les poses atteignables et dans les butees, retour a la position debout."""
import math
import os

import pytest

from hexapod_control.actions import FIGURES, ActionPlayer
from hexapod_control.gait import TripodGait
from hexapod_control.kinematics import load_legs

YAML = os.path.join(os.path.dirname(__file__), "..", "..", "hexapod_description", "config", "legs.yaml")


@pytest.fixture(scope="module")
def legs():
    return load_legs(YAML)


@pytest.mark.parametrize("name", list(FIGURES))
@pytest.mark.parametrize("height,radius", [(0.12, 0.12), (0.10, 0.16)])
def test_figure_reachable(legs, name, height, radius):
    g = TripodGait(legs, stance_radius=radius, body_height=height, height_limits=(0.05, 0.15))
    rest = {n: tuple(p) for n, p in g.feet.items()}
    pl = ActionPlayer(legs)
    pl.start(name, rest)
    lifted_seen, k = set(), 0
    while True:
        r = pl.step(0.02)
        if r is None:
            break
        feet, lifted = r
        lifted_seen |= lifted
        for lg in legs:
            q = lg.ik(*feet[lg.n])
            assert q is not None and lg.in_limits(q), (name, round(k * 0.02, 2), lg.n,
                                                       [round(math.degrees(a)) for a in q] if q else None)
        k += 1
    assert k * 0.02 > FIGURES[name][1]


@pytest.mark.parametrize("name", list(FIGURES))
def test_stop_returns_smoothly(legs, name):
    g = TripodGait(legs, stance_radius=0.12, body_height=0.12, height_limits=(0.05, 0.15))
    rest = {n: tuple(p) for n, p in g.feet.items()}
    pl = ActionPlayer(legs)
    pl.start(name, rest)
    prev = None
    for k in range(400):
        if k == 150:
            pl.stop()
        r = pl.step(0.02)
        if r is None:
            break
        feet = r[0]
        if prev:                                   # pas de saut : < 2 cm par pas de 20 ms
            assert max(math.dist(feet[n], prev[n]) for n in feet) < 0.02
        prev = feet
    assert r is None and k < 220                   # fini peu apres l'arret
    for n in rest:                                 # revenu exactement debout
        assert math.dist(prev[n], rest[n]) < 0.003
