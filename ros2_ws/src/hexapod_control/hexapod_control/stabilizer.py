"""Stabilisation par l'IMU (sans ROS, teste seul).

Trois corrections, appliquees par gait_node entre le generateur de pas et la
cinematique inverse :

1. TENUE DE CAP : quand on ne demande pas de tourner, le cap au moment ou l'on part
   tout droit est memorise ; l'ecart (lacet IMU) est corrige par une petite vitesse de
   rotation (PI) -> le robot avance droit malgre les glissements / pattes inegales.
2. MISE A NIVEAU (compensation active) : le corps est incline par rapport a la pose des
   pieds de l'angle inverse du roulis / tangage mesure (PI) -> si le sol penche ou bouge,
   le haut du robot reste horizontal. Les pieds sont tournes autour du centre du corps.
3. ANTI-AFFAISSEMENT : pendant qu'un trepied est en l'air, les pattes au sol poussent
   de `sag_comp` m de plus vers le bas (les servos plient un peu sous la charge) -> le
   corps ne descend plus a chaque pas.
Et une SECURITE : au-dela de `fall_deg` d'inclinaison, la marche s'arrete.
"""
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_rpy(w, x, y, z):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def rot_rp(roll, pitch, p):
    """Rotation R = Ry(pitch) . Rx(roll) appliquee au point p."""
    x, y, z = p
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    y, z = cr * y - sr * z, sr * y + cr * z
    x, z = cp * x + sp * z, -sp * x + cp * z
    return (x, y, z)


def rot_rp_inv(roll, pitch, p):
    x, y, z = p
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    x, z = cp * x - sp * z, sp * x + cp * z
    y, z = cr * y + sr * z, -sr * y + cr * z
    return (x, y, z)


class Stabilizer:
    def __init__(self, heading_kp=1.2, heading_ki=0.2, heading_max=0.3,
                 level_kp=0.3, level_ki=3.0, level_max=math.radians(10), level_rate=math.radians(40),
                 sag_comp=0.003, fall=math.radians(35), imu_timeout=0.5):
        self.heading_kp, self.heading_ki, self.heading_max = heading_kp, heading_ki, heading_max
        self.level_kp, self.level_ki, self.level_max, self.level_rate = level_kp, level_ki, level_max, level_rate
        self.sag_comp, self.fall, self.imu_timeout = sag_comp, fall, imu_timeout
        self.enable_heading = self.enable_level = True
        self.rpy = None                   # attitude mesuree (base_link dans le monde)
        self.imu_age = 1e9
        self.heading_ref = None
        self.heading_int = 0.0
        self.level_int = [0.0, 0.0]       # integrale du roulis / tangage
        self.tilt = [0.0, 0.0]            # inclinaison appliquee au corps (roulis, tangage)
        self.fallen = False
        self.gyro_z = 0.0                 # vitesse de lacet mesuree (filtree), rad/s
        self.turn_kp, self.turn_ki, self.turn_boost = 0.5, 1.5, 1.5
        self.enable_turn = True
        self.turn_int = 0.0

    # ------------------------------------------------------------------ entrees
    def imu(self, w, x, y, z, gyro_z=None):
        self.rpy = quat_to_rpy(w, x, y, z)
        self.imu_age = 0.0
        if gyro_z is not None:
            self.gyro_z += 0.15 * (gyro_z - self.gyro_z)       # passe-bas (bruit ~0.5 deg/s)

    @property
    def imu_ok(self):
        return self.rpy is not None and self.imu_age < self.imu_timeout

    # ------------------------------------------------------------------ cap
    def heading_correction(self, cmd, walking, dt):
        """Vitesse de rotation a ajouter a la consigne pour garder le cap."""
        vx, vy, wz = cmd
        straight = walking and abs(wz) < 1e-3 and math.hypot(vx, vy) > 1e-3
        if not (self.enable_heading and self.imu_ok and straight):
            self.heading_ref, self.heading_int = None, 0.0
            return 0.0
        yaw = self.rpy[2]
        if self.heading_ref is None:
            self.heading_ref = yaw
            return 0.0
        e = wrap(self.heading_ref - yaw)
        self.heading_int = max(-0.5, min(0.5, self.heading_int + e * dt))
        return max(-self.heading_max, min(self.heading_max, self.heading_kp * e + self.heading_ki * self.heading_int))

    # ------------------------------------------------------------------ rotation
    def turn_command(self, wz, walking, dt, wz_max):
        """Rotation demandee -> rotation a commander pour que la rotation MESUREE suive
        (les pieds patinent en tournant : sans ca le robot tourne 2 a 3 fois moins)."""
        if not (self.enable_turn and self.imu_ok and walking and abs(wz) > 1e-3):
            self.turn_int = 0.0
            return wz
        e = wz - self.gyro_z
        lim = self.turn_boost * abs(wz)
        self.turn_int = max(-lim, min(lim, self.turn_int + self.turn_ki * e * dt))
        out = wz + self.turn_kp * e + self.turn_int
        return max(-wz_max, min(wz_max, out))

    # ------------------------------------------------------------------ niveau
    def update(self, dt):
        """Met a jour l'inclinaison a donner au corps ; renvoie (roulis, tangage) appliques."""
        self.imu_age += dt
        if self.imu_ok:
            r, p, _ = self.rpy
            self.fallen = max(abs(r), abs(p)) > self.fall
        if not (self.enable_level and self.imu_ok):
            target = [0.0, 0.0]                       # pas d'IMU : retour doux a plat
            self.level_int = [0.0, 0.0]
        else:
            # l'attitude mesuree inclut deja la correction appliquee : on vise 0
            meas = self.rpy[:2]
            for i in range(2):
                self.level_int[i] = max(-self.level_max, min(self.level_max, self.level_int[i] - self.level_ki * meas[i] * dt))
            target = [max(-self.level_max, min(self.level_max, self.level_int[i] - self.level_kp * meas[i]))
                      for i in range(2)]
        step = self.level_rate * dt
        for i in range(2):
            self.tilt[i] += max(-step, min(step, target[i] - self.tilt[i]))
        return tuple(self.tilt)

    def transform_feet(self, feet, stance, any_swing, tilt=None):
        """Pieds de la marche (repere corps a plat) -> pieds a commander (corps incline de
        `tilt`, pattes d'appui un peu plus bas si un trepied est en l'air)."""
        r, p = self.tilt if tilt is None else tilt
        out = {}
        for n, (x, y, z) in feet.items():
            if any_swing and stance.get(n):
                z -= self.sag_comp
            # corps tourne de (r, p) par rapport au sol : les pieds, fixes au sol, sont
            # vus tournes de l'inverse dans le repere du corps
            out[n] = rot_rp_inv(r, p, (x, y, z))
        return out
