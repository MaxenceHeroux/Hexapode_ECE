"""Stabilisation IMU : mise a niveau sur sol incline / mobile, tenue de cap, chute."""
import math
import os

from hexapod_control.gait import TripodGait
from hexapod_control.kinematics import load_legs
from hexapod_control.stabilizer import Stabilizer, rot_rp, rot_rp_inv

YAML = os.path.join(os.path.dirname(__file__), "..", "..", "hexapod_description", "config", "legs.yaml")


def quat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                              math.cos(y / 2), math.sin(y / 2))
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def test_rotations_inverse():
    p = (0.12, -0.05, -0.1)
    q = rot_rp_inv(0.1, -0.2, rot_rp(0.1, -0.2, p))
    assert all(abs(a - b) < 1e-12 for a, b in zip(p, q))


def run_level(ground, seconds=4.0, dt=0.02):
    s = Stabilizer()
    for k in range(int(seconds / dt)):
        g = ground(k * dt)
        s.imu(*quat(g[0] + s.tilt[0], g[1] + s.tilt[1], 0.0))   # corps = sol + inclinaison appliquee
        s.update(dt)
    return s


def test_level_on_slope():
    s = run_level(lambda t: (math.radians(6), math.radians(-4)))
    assert abs(s.tilt[0] + math.radians(6)) < math.radians(0.3)
    assert abs(s.tilt[1] - math.radians(4)) < math.radians(0.3)


def test_level_moving_ground():
    """Sol qui oscille lentement (+-5 deg, 0.15 Hz) : le corps reste a moins de 2 deg."""
    s = Stabilizer()
    worst, dt = 0.0, 0.02
    for k in range(int(12 / dt)):
        t = k * dt
        g = (math.radians(5) * math.sin(2 * math.pi * 0.15 * t), math.radians(5) * math.cos(2 * math.pi * 0.15 * t))
        body = (g[0] + s.tilt[0], g[1] + s.tilt[1])
        s.imu(*quat(*body, 0.0))
        s.update(dt)
        if t > 4:
            worst = max(worst, abs(body[0]), abs(body[1]))
    assert worst < math.radians(2.0), math.degrees(worst)


def test_level_feet_reachable():
    """Corps incline de 8 deg (dans les 2 axes) a la pose par defaut : pieds atteignables."""
    legs = load_legs(YAML)
    g = TripodGait(legs, stance_radius=0.12, body_height=0.12)
    s = Stabilizer()
    for tilt in ((math.radians(8), 0), (0, math.radians(8)), (math.radians(-6), math.radians(6))):
        feet = s.transform_feet(g.feet, {lg.n: True for lg in legs}, False, tilt)
        for lg in legs:
            q = lg.ik(*feet[lg.n])
            assert q is not None and lg.in_limits(q), (tilt, lg.n)


def test_heading_hold():
    """Le robot derive de 0.05 rad/s (pattes inegales) : le cap reste a moins de 2 deg."""
    s = Stabilizer()
    yaw, dt, worst = 0.0, 0.02, 0.0
    for k in range(int(20 / dt)):
        s.imu(*quat(0, 0, yaw))
        corr = s.heading_correction((0.1, 0.0, 0.0), True, dt)
        yaw += (corr + 0.05) * dt
        if k * dt > 3:
            worst = max(worst, abs(yaw))
    assert worst < math.radians(2.0), math.degrees(worst)


def test_heading_released_when_turning():
    s = Stabilizer()
    s.imu(*quat(0, 0, 0.3))
    assert s.heading_correction((0.1, 0.0, 0.4), True, 0.02) == 0.0 and s.heading_ref is None


def test_fall_and_no_imu():
    s = Stabilizer()
    s.imu(*quat(math.radians(40), 0, 0))
    s.update(0.02)
    assert s.fallen
    s2 = Stabilizer()
    s2.tilt = [0.1, -0.1]
    for _ in range(100):
        s2.update(0.02)                     # jamais d'IMU : retour doux a plat
    assert abs(s2.tilt[0]) < 1e-9 and abs(s2.tilt[1]) < 1e-9


def test_sag_comp_only_stance_during_swing():
    s = Stabilizer(sag_comp=0.003)
    feet = {1: (0.2, 0.1, -0.12), 2: (0.2, -0.1, -0.09)}
    out = s.transform_feet(feet, {1: True, 2: False}, True, (0.0, 0.0))
    assert abs(out[1][2] + 0.123) < 1e-9 and abs(out[2][2] + 0.09) < 1e-9


def test_turn_assist_compensates_slip():
    """Les pieds patinent : le robot ne tourne qu'a 40 % de la commande. Avec l'aide du
    gyroscope, la rotation reelle rejoint la rotation demandee."""
    s = Stabilizer()
    yaw, rate, dt = 0.0, 0.0, 0.02
    for k in range(int(6 / dt)):
        s.imu(*quat(0, 0, yaw), gyro_z=rate)
        cmd = s.turn_command(0.4, True, dt, 1.2)
        rate = 0.4 * cmd                         # 60 % de patinage
        yaw += rate * dt
    assert abs(s.gyro_z - 0.4) < 0.03, s.gyro_z
