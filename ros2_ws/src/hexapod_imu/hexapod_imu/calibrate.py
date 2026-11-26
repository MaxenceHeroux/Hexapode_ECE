"""Calibration de l'IMU (a lancer SUR LA PI, robot pose a plat et immobile).

    ros2 run hexapod_imu calibrate_imu                 # biais gyro + echelle + niveau
    ros2 run hexapod_imu calibrate_imu --axes          # + sens de montage : soulever l'avant

Etapes :
  1. attend 10 s d'immobilite (fenetres de 0,5 s : ecart-type gyro < 1,5 deg/s) ;
     biais du gyro = moyenne ; echelle accel = g / |accel moyenne| ;
  2. (--axes) demande de soulever l'avant du robot : l'axe du capteur qui recoit la
     gravite donne le sens de montage (lacet 0, 90, 180 ou -90 deg) ;
  3. correction de niveau : roulis / tangage residuels a plat ramenes a 0.
Ecrit config/imu_calib.yaml dans les sources du paquet (a commiter).
"""
import argparse
import math
import os
import statistics as st
import time

from .calibration import Calibration, apply, level_from_gravity, rpy_matrix
from .lsm6dsv16x import G, LSM6DSV16X

SRC_CALIB = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "config", "imu_calib.yaml")


def windows(imu, seconds=0.5):
    w, t0 = [], time.time()
    while time.time() - t0 < seconds:
        w.append(imu.read())
        time.sleep(0.004)
    return w


def wait_still(imu, n_samples=1200, timeout=90):
    still, t_end = [], time.time() + timeout
    while time.time() < t_end and len(still) < n_samples:
        w = windows(imu)
        sd = max(math.degrees(st.pstdev(x[1][i] for x in w)) for i in range(3))
        mn = max(abs(math.degrees(st.mean(x[1][i] for x in w))) for i in range(3))
        if sd < 1.5 and mn < 2.0:
            still += w
            print(f"\r  immobile : {len(still) / max(1, len(w)) * 0.5:4.1f} s", end="", flush=True)
        else:
            still = []
            print("\r  ca bouge, on recommence...   ", end="", flush=True)
    print()
    if len(still) < n_samples:
        raise SystemExit("robot jamais immobile assez longtemps : le poser et ne plus le toucher")
    return still


def find_mount_yaw(imu, base_acc):
    print("  >>> SOULEVE L'AVANT du robot (20-30 deg) et garde-le 3 s <<<")
    t_end = time.time() + 30
    while time.time() < t_end:
        w = windows(imu, 0.3)
        a = [st.mean(x[0][i] for x in w) for i in range(3)]
        d = [a[i] - base_acc[i] for i in range(2)]
        if math.hypot(*d) > 0.25 * G:                       # ~15 deg d'inclinaison
            i = 0 if abs(d[0]) > abs(d[1]) else 1
            # avant leve : la gravite (mesuree vers le haut) passe sur +avant
            fwd = (1 if d[i] > 0 else -1, i)
            yaw = {(1, 0): 0.0, (-1, 0): math.pi, (1, 1): -math.pi / 2, (-1, 1): math.pi / 2}[fwd]
            print(f"  avant du robot = axe {'+' if fwd[0] > 0 else '-'}{'XY'[i]} du capteur "
                  f"-> lacet de montage {math.degrees(yaw):.0f} deg")
            return yaw
    raise SystemExit("pas d'inclinaison vue en 30 s")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--axes", action="store_true", help="retrouver le sens de montage (soulever l'avant)")
    p.add_argument("--file", default=SRC_CALIB)
    p.add_argument("--address", type=lambda s: int(s, 0), default=0x6A)
    a = p.parse_args()

    old = Calibration.load(a.file) if os.path.exists(a.file) else Calibration()
    imu = LSM6DSV16X(address=a.address)
    print(f"IMU LSM6DSV16X ok (0x{a.address:02x}), {imu.temperature():.1f} degC")
    print("1. Robot pose A PLAT, ne plus le toucher (10 s)")
    still = wait_still(imu)
    acc = [st.mean(x[0][i] for x in still) for i in range(3)]
    gyr = [st.mean(x[1][i] for x in still) for i in range(3)]
    norm = math.sqrt(sum(v * v for v in acc))
    mount = list(old.mount_rpy)
    if a.axes:
        print("2. Sens de montage")
        mount[2] = find_mount_yaw(imu, acc)
    acc_body = apply(rpy_matrix(*mount), [v * G / norm for v in acc])
    cal = Calibration(gyr, G / norm, mount, level_from_gravity(acc_body))
    cal.save(a.file, f"{time.strftime('%Y-%m-%d %H:%M')}, {imu.temperature():.1f} degC, {len(still)} mesures\n"
                     f"gravite mesuree {norm:.3f} m/s2 ; biais gyro (deg/s) "
                     + " ".join(f"{math.degrees(v):+.3f}" for v in gyr))
    print(f"biais gyro (deg/s) : {' '.join(f'{math.degrees(v):+.3f}' for v in gyr)}")
    print(f"echelle accel : {G / norm:.4f}  |  niveau : roulis {math.degrees(cal.level_rpy[0]):+.2f}, "
          f"tangage {math.degrees(cal.level_rpy[1]):+.2f} deg  |  montage lacet {math.degrees(mount[2]):.0f} deg")
    print(f"ecrit : {a.file}")


if __name__ == "__main__":
    main()
