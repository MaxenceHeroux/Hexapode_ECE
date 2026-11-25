"""Generateur de marche en trepied, independant de ROS (teste seul).

Chaque patte alterne :
  - APPUI (stance) : le bout de patte reste fixe au sol dans le monde, donc dans le
    repere du corps il recule a la vitesse -(v + w x p) ;
  - VOL (swing) : il se leve de `step_height`, et va de sa position de decollage au
    point d'atterrissage (en avant de la position neutre de la moitie du pas), sur
    une trajectoire en cloche.
Trepied A = pattes 1, 3, 5 (avant gauche, milieu droite, arriere gauche), trepied B =
2, 4, 6, en opposition de phase. Sans consigne, le robot finit son cycle et s'arrete
pattes en position neutre.
"""
import math

TRIPOD_OFFSET = {1: 0.0, 3: 0.0, 5: 0.0, 2: 0.5, 4: 0.5, 6: 0.5}
DUTY = 0.5                     # fraction du cycle en appui


def _smoothstep(s):
    s = min(1.0, max(0.0, s))
    return s * s * (3 - 2 * s)


class TripodGait:
    def __init__(self, legs, stance_radius=0.17, body_height=0.09, step_height=0.03,
                 cycle_time=1.0, height_rate=0.03, accel=0.2, ang_accel=1.0):
        self.legs = legs
        self.stance_radius = stance_radius
        self.height = body_height          # hauteur actuelle (axes des hanches au-dessus du sol)
        self.height_target = body_height
        self.step_height = step_height
        self.cycle_time = cycle_time
        self.height_rate = height_rate
        self.accel, self.ang_accel = accel, ang_accel
        self.cmd = (0.0, 0.0, 0.0)          # consigne (vx, vy, wz)
        self.vel = (0.0, 0.0, 0.0)          # vitesse appliquee (rampe)
        self.phase = 0.0
        self.walking = False
        self.feet = {lg.n: list(lg.neutral(stance_radius, -body_height)) for lg in legs}
        self.liftoff = {lg.n: None for lg in legs}

    # ------------------------------------------------------------------ consignes
    def set_cmd(self, vx, vy, wz):
        self.cmd = (vx, vy, wz)

    def set_height(self, h):
        self.height_target = h

    # ------------------------------------------------------------------ outils
    def neutral(self, leg):
        return leg.neutral(self.stance_radius, -self.height)

    @staticmethod
    def _foot_vel(vel, x, y):
        """Vitesse du sol vue du corps au point (x, y) : -(v + w x p)."""
        vx, vy, wz = vel
        return (-(vx - wz * y), -(vy + wz * x))

    def leg_phase(self, n):
        return (self.phase + TRIPOD_OFFSET[n]) % 1.0

    def in_stance(self, n):
        return self.leg_phase(n) < DUTY

    # ------------------------------------------------------------------ pas de temps
    def step(self, dt):
        # rampes : vitesse et hauteur
        v = list(self.vel)
        for i, (lim) in enumerate((self.accel, self.accel, self.ang_accel)):
            dv = self.cmd[i] - v[i]
            v[i] += max(-lim * dt, min(lim * dt, dv))
        self.vel = tuple(v)
        dh = self.height_target - self.height
        self.height += max(-self.height_rate * dt, min(self.height_rate * dt, dh))

        moving = any(abs(c) > 1e-4 for c in self.cmd) or any(abs(c) > 1e-4 for c in self.vel)
        off_neutral = max(math.dist(self.feet[lg.n][:2], self.neutral(lg)[:2]) for lg in self.legs)
        if not self.walking and (moving or off_neutral > 0.004):
            self.walking = True
        if self.walking and not moving and off_neutral < 0.004 and self._phase_at_switch(dt):
            self.walking = False

        if self.walking:
            self.phase = (self.phase + dt / self.cycle_time) % 1.0

        t_stance = DUTY * self.cycle_time
        for lg in self.legs:
            n, p = lg.n, self.feet[lg.n]
            if not self.walking:
                p[2] = -self.height
                self.liftoff[n] = None
                continue
            ph = self.leg_phase(n)
            if ph < DUTY:                                    # appui
                self.liftoff[n] = None
                fx, fy = self._foot_vel(self.vel, p[0], p[1])
                p[0] += fx * dt
                p[1] += fy * dt
                p[2] = -self.height
            else:                                            # vol
                if self.liftoff[n] is None:
                    self.liftoff[n] = (p[0], p[1])
                s = (ph - DUTY) / (1 - DUTY)
                nx, ny, _ = self.neutral(lg)
                fx, fy = self._foot_vel(self.vel, nx, ny)
                tx, ty = nx - fx * t_stance / 2, ny - fy * t_stance / 2   # atterrit en avant
                k = _smoothstep(s)
                x0, y0 = self.liftoff[n]
                p[0] = x0 + (tx - x0) * k
                p[1] = y0 + (ty - y0) * k
                p[2] = -self.height + self.step_height * math.sin(math.pi * s)
        return self.vel

    def _phase_at_switch(self, dt):
        """Vrai pres d'un changement de trepied (toutes les pattes au sol)."""
        step = dt / self.cycle_time
        return min(self.phase % DUTY, DUTY - self.phase % DUTY) < step * 1.5
