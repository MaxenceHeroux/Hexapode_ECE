"""Calibration et mise dans le repere du robot (sans ROS).

Le fichier de calibration (config/imu_calib.yaml) contient :
  gyro_bias   [x, y, z] rad/s, repere du capteur (moyenne a l'arret)
  acc_scale   facteur : |acceleration| a l'arret ramenee a g
  mount_rpy   [roulis, tangage, lacet] rad : capteur -> base_link (montage de la carte)
  level_rpy   [roulis, tangage] rad : petite correction pour que le robot pose a plat
              donne roulis = tangage = 0 (carte pas parfaitement a plat)
"""
import math

import yaml

G = 9.80665


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def apply(m, v):
    return tuple(m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2] for i in range(3))


class Calibration:
    def __init__(self, gyro_bias=(0.0, 0.0, 0.0), acc_scale=1.0, mount_rpy=(0.0, 0.0, 0.0),
                 level_rpy=(0.0, 0.0)):
        self.gyro_bias = tuple(gyro_bias)
        self.acc_scale = float(acc_scale)
        self.mount_rpy = tuple(mount_rpy)
        self.level_rpy = tuple(level_rpy)
        # base_link <- capteur : d'abord le montage, puis la correction de niveau
        self.R = matmul(rpy_matrix(self.level_rpy[0], self.level_rpy[1], 0.0), rpy_matrix(*self.mount_rpy))

    @classmethod
    def load(cls, path):
        with open(path, encoding="utf-8") as f:
            d = yaml.safe_load(f) or {}
        return cls(d.get("gyro_bias", (0, 0, 0)), d.get("acc_scale", 1.0),
                   d.get("mount_rpy", (0, 0, 0)), d.get("level_rpy", (0, 0)))

    def save(self, path, comment=""):
        lines = ["# Calibration de l'IMU (LSM6DSV16X). GENERE par : ros2 run hexapod_imu calibrate_imu",
                 *(f"# {c}" for c in comment.splitlines() if c)]
        lines += [f"gyro_bias: [{', '.join(f'{v:.6f}' for v in self.gyro_bias)}]   # rad/s, repere capteur",
                  f"acc_scale: {self.acc_scale:.6f}",
                  f"mount_rpy: [{', '.join(f'{v:.6f}' for v in self.mount_rpy)}]   # rad, capteur -> base_link",
                  f"level_rpy: [{', '.join(f'{v:.6f}' for v in self.level_rpy)}]   # rad, correction de niveau"]
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")

    def correct(self, acc, gyr):
        """Mesures brutes (repere capteur) -> (acc m/s2, gyr rad/s) corriges, repere base_link."""
        a = tuple(v * self.acc_scale for v in acc)
        g = tuple(v - b for v, b in zip(gyr, self.gyro_bias))
        return apply(self.R, a), apply(self.R, g)


def level_from_gravity(acc_body):
    """Correction (roulis, tangage) qui ramene la gravite mesuree (deja dans les axes du robot)
    sur +Z : a appliquer apres le montage."""
    ax, ay, az = acc_body
    # R = Ry(tangage) . Rx(roulis) : Rx annule la composante y, puis Ry la composante x
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.hypot(ay, az))
    return (roll, pitch)


class MahonyFilter:
    """Filtre de Mahony 6 axes (gyro + accel) : roulis / tangage recales sur la gravite,
    lacet integre (pas de magnetometre : il derive lentement)."""

    def __init__(self, kp=1.0, ki=0.0):
        self.q = [1.0, 0.0, 0.0, 0.0]          # w, x, y, z (base_link dans le monde)
        self.kp, self.ki = kp, ki
        self.e_int = [0.0, 0.0, 0.0]
        self.initialized = False

    def init_from_acc(self, acc):
        ax, ay, az = acc
        roll, pitch = math.atan2(ay, az), math.atan2(-ax, math.hypot(ay, az))
        cr, sr, cp, sp = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2)
        self.q = [cr * cp, sr * cp, cr * sp, -sr * sp]
        self.initialized = True

    def update(self, acc, gyr, dt):
        if not self.initialized:
            self.init_from_acc(acc)
            return self.q
        w, x, y, z = self.q
        gx, gy, gz = gyr
        n = math.sqrt(sum(v * v for v in acc))
        if 0.5 * G < n < 1.5 * G:                      # recalage seulement si pas de gros choc
            ax, ay, az = (v / n for v in acc)
            vx, vy, vz = 2 * (x * z - w * y), 2 * (w * x + y * z), w * w - x * x - y * y + z * z
            ex, ey, ez = ay * vz - az * vy, az * vx - ax * vz, ax * vy - ay * vx
            if self.ki > 0:
                self.e_int = [i + e * dt for i, e in zip(self.e_int, (ex, ey, ez))]
            gx += self.kp * ex + self.ki * self.e_int[0]
            gy += self.kp * ey + self.ki * self.e_int[1]
            gz += self.kp * ez + self.ki * self.e_int[2]
        hd = 0.5 * dt
        q = [w - (x * gx + y * gy + z * gz) * hd, x + (w * gx + y * gz - z * gy) * hd,
             y + (w * gy - x * gz + z * gx) * hd, z + (w * gz + x * gy - y * gx) * hd]
        n = math.sqrt(sum(v * v for v in q))
        self.q = [v / n for v in q]
        return self.q

    def rpy(self):
        w, x, y, z = self.q
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return roll, pitch, yaw
