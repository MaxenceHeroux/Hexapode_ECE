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


class FeasibilityMap:
    """Couples (rayon, hauteur) ou la marche reste atteignable et dans les butees.

    Un couple est bon si, pour les 6 pattes, le pied neutre ET les points a `reach`
    autour de lui (au sol et en haut du pas) passent la cinematique inverse dans les
    butees. Grille de 5 mm ; un point entre deux cases n'est bon que si les 4 cases
    voisines le sont (prudent).
    """

    def __init__(self, legs, step_height, reach=0.025, r_range=(0.08, 0.22), h_range=(0.03, 0.17), d=0.005):
        self.d, self.r0, self.h0 = d, r_range[0], h_range[0]
        self.nr = int(round((r_range[1] - r_range[0]) / d)) + 1
        self.nh = int(round((h_range[1] - h_range[0]) / d)) + 1
        ring = [(reach * math.cos(a), reach * math.sin(a)) for a in [k * math.pi / 4 for k in range(8)]]
        self.ok = [[False] * self.nh for _ in range(self.nr)]
        for i in range(self.nr):
            r = self.r0 + i * d
            for j in range(self.nh):
                h = self.h0 + j * d
                good = True
                for lg in legs:
                    nx, ny, _ = lg.neutral(r, -h)
                    pts = [(nx + dx, ny + dy, -h) for dx, dy in ring + [(0.0, 0.0)]]
                    pts += [(nx + dx / 2, ny + dy / 2, -h + step_height) for dx, dy in ring + [(0.0, 0.0)]]
                    for p in pts:
                        q = lg.ik(*p)
                        if q is None or not lg.in_limits(q):
                            good = False
                            break
                    if not good:
                        break
                self.ok[i][j] = good

    def feasible(self, r, h):
        fi, fj = (r - self.r0) / self.d, (h - self.h0) / self.d
        fi = round(fi) if abs(fi - round(fi)) < 1e-6 else fi        # sur une case : pas de voisine
        fj = round(fj) if abs(fj - round(fj)) < 1e-6 else fj
        i0, j0 = math.floor(fi), math.floor(fj)
        for i in (i0, min(i0 + 1, self.nr - 1)) if fi != i0 else (i0,):
            for j in (j0, min(j0 + 1, self.nh - 1)) if fj != j0 else (j0,):
                if not (0 <= i < self.nr and 0 <= j < self.nh and self.ok[i][j]):
                    return False
        return True

    def radius_for(self, r_wanted, h):
        """Rayon faisable a la hauteur h le plus proche de r_wanted (None si aucun)."""
        cands = [self.r0 + i * self.d for i in range(self.nr) if self.feasible(self.r0 + i * self.d, h)]
        if not cands:
            return None
        if cands[0] <= r_wanted <= cands[-1] and self.feasible(r_wanted, h):
            return r_wanted
        return min(cands, key=lambda r: abs(r - r_wanted))

    def height_range(self):
        hs = [self.h0 + j * self.d for j in range(self.nh) if any(self.ok[i][j] for i in range(self.nr))]
        return (hs[0], hs[-1]) if hs else (None, None)


def _smoothstep(s):
    s = min(1.0, max(0.0, s))
    return s * s * (3 - 2 * s)


class TripodGait:
    def __init__(self, legs, stance_radius=0.17, body_height=0.09, step_height=0.03,
                 cycle_time=1.0, height_rate=0.03, accel=0.2, ang_accel=1.0,
                 max_linear=0.10, max_angular=0.6, reach=0.045, height_limits=(0.0, 1.0)):
        self.legs = legs
        self.reach = reach                  # demi-pas max (m) : taille de pas maximale = 2 x reach
        self.fmap = FeasibilityMap(legs, step_height, reach)
        lo, hi = self.fmap.height_range()
        self.h_min, self.h_max = max(lo, height_limits[0]), min(hi, height_limits[1])
        self.stance_radius = stance_radius  # rayon voulu ; le rayon applique s'adapte a la hauteur
        body_height = min(self.h_max, max(self.h_min, body_height))
        self.radius = self.fmap.radius_for(stance_radius, body_height)
        self.height = body_height          # hauteur actuelle (axes des hanches au-dessus du sol)
        self.height_target = body_height
        self.max_linear, self.max_angular = max_linear, max_angular
        self.step_height = step_height
        self.cycle_time = cycle_time
        self.height_rate = height_rate
        self.accel, self.ang_accel = accel, ang_accel
        self.cmd = (0.0, 0.0, 0.0)          # consigne (vx, vy, wz)
        self.vel = (0.0, 0.0, 0.0)          # vitesse appliquee (rampe)
        self.phase = 0.0
        self.walking = False
        self.reposition = False             # pieds a deplacer apres un changement de hauteur / rayon
        self.feet = {lg.n: list(lg.neutral(self.radius, -body_height)) for lg in legs}
        self.liftoff = {lg.n: None for lg in legs}

    # ------------------------------------------------------------------ consignes
    def set_cmd(self, vx, vy, wz):
        """Consigne ; reduite (en gardant la direction) si le pas depasserait 2 x reach."""
        t_st = DUTY * self.cycle_time
        worst = max(math.hypot(*self._foot_vel((vx, vy, wz), *lg.neutral(self.radius, 0)[:2]))
                    for lg in self.legs) * t_st / 2
        k = min(1.0, self.reach / worst) if worst > 1e-9 else 1.0
        self.cmd = (vx * k, vy * k, wz * k)
        return k

    def set_height(self, h):
        """Hauteur voulue, bornee a ce que les pattes permettent. Le rayon des pieds
        s'adapte (le robot fait des pas pour les deplacer) ; renvoie la hauteur retenue."""
        self.height_target = min(self.h_max, max(self.h_min, h))
        self._update_radius()
        return self.height_target

    def set_radius(self, r):
        self.stance_radius = r
        self._update_radius()
        return self.radius

    def _update_radius(self):
        r = self.fmap.radius_for(self.stance_radius, self.height_target)
        if r is not None:
            if abs(r - self.radius) > 1e-6:
                self.reposition = True
            self.radius = r

    # ------------------------------------------------------------------ outils
    def radius_now(self):
        """Rayon vise MAINTENANT : celui voulu pour la hauteur cible, ramene dans ce qui
        est faisable a la hauteur actuelle (en changeant de hauteur, pieds et corps
        avancent par etapes sans jamais sortir des butees)."""
        key = round(self.height, 3), self.radius
        if key != getattr(self, "_rn_key", None):
            self._rn_key = key
            self._rn = self.fmap.radius_for(self.radius, self.height) or self.radius
        return self._rn

    def neutral(self, leg):
        return leg.neutral(self.radius_now(), -self.height)

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
        h_prev = self.height
        dh = self.height_target - self.height
        h_next = self.height + max(-self.height_rate * dt, min(self.height_rate * dt, dh))
        if h_next != self.height and self._feet_ok_at(h_next):
            self.height = h_next          # sinon on attend que les pieds aient ete deplaces

        moving = any(abs(c) > 1e-4 for c in self.cmd) or any(abs(c) > 1e-4 for c in self.vel)
        off_neutral = max(math.dist(self.feet[lg.n][:2], self.neutral(lg)[:2]) for lg in self.legs)
        height_blocked = abs(self.height_target - self.height) > 1e-4 and self.height == h_prev
        if self.reposition and off_neutral < 0.004 and not height_blocked:
            self.reposition = False
        if height_blocked:
            self.reposition = True          # le corps attend : faire des pas pour replacer les pieds
        # Pas a l'arret : seulement pour deplacer les pieds apres un changement de hauteur /
        # rayon. Quand la consigne s'arrete, le robot pose ses pattes en l'air et s'arrete
        # la, sans pas supplementaire pour revenir au neutre.
        if not self.walking and (moving or self.reposition):
            self.walking = True
        if self.walking and not moving and not self.reposition and self._phase_at_switch(dt):
            self.walking = False

        if self.walking:
            self.phase = (self.phase + dt / self.cycle_period()) % 1.0

        t_stance = DUTY * self.cycle_period()
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
                nxt = (x0 + (tx - x0) * k, y0 + (ty - y0) * k)
                # levee reduite si la trajectoire passe hors butees (pied pres du corps et
                # bas) ; en dernier recours le pied attend sur place
                for lift in (1.0, 0.5, 0.25, 0.0):
                    cand = (nxt[0], nxt[1], -self.height + lift * self.step_height * math.sin(math.pi * s))
                    if self._reachable(lg, cand):
                        p[0], p[1], p[2] = cand
                        break
                else:
                    if self._reachable(lg, (p[0], p[1], -self.height)):
                        p[2] = -self.height
        return self.vel

    @staticmethod
    def _reachable(lg, pt):
        q = lg.ik(*pt)
        return q is not None and lg.in_limits(q)

    def cycle_period(self):
        """Cadence suivant la vitesse : `cycle_time` a vitesse max, jusqu'a 2x plus lent
        a mi-vitesse et en dessous (pas plus courts ET moins frequents)."""
        if not any(abs(c) > 1e-4 for c in self.cmd):
            return self.cycle_time            # arret / replacement : finir le pas sans trainer
        vx, vy, wz = self.vel
        frac = max(math.hypot(vx, vy) / self.max_linear, abs(wz) / self.max_angular)
        return self.cycle_time / max(0.5, min(1.0, frac))

    def _feet_ok_at(self, h):
        """Les pieds (au sol ou en l'air) restent-ils atteignables si le corps passe a h ?"""
        for lg in self.legs:
            x, y, z = self.feet[lg.n]
            q = lg.ik(x, y, z - (h - self.height))
            if q is None or not lg.in_limits(q):
                return False
        return True

    def _phase_at_switch(self, dt):
        """Vrai pres d'un changement de trepied (toutes les pattes au sol)."""
        step = dt / self.cycle_period()
        return min(self.phase % DUTY, DUTY - self.phase % DUTY) < step * 1.5
