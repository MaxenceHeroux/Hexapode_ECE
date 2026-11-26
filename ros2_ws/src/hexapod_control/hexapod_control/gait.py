"""Generateur de marche en trepied, independant de ROS (teste seul).

Chaque patte alterne :
  - APPUI (stance) : le bout de patte reste fixe au sol dans le monde, donc dans le
    repere du corps il recule a la vitesse -(v + w x p) ;
  - VOL (swing) : il se leve de `step_height`, et va de sa position de decollage au
    point d'atterrissage (en avant de la position neutre de la moitie du pas), sur
    une trajectoire en cloche.
Allures (pattes : 1 avant gauche, 2 avant droite, 3 milieu droite, 4 arriere droite,
5 arriere gauche, 6 milieu gauche) :
  - tripod   : 2 trepieds (1,3,5) / (2,4,6), 3 pieds au sol (rapide) ;
  - tetrapod : 3 paires (2,5) (3,6) (1,4), 4 pieds au sol (plus d'adherence) ;
  - wave     : une patte a la fois (4,3,2,5,6,1), 5 pieds au sol (lent, tres stable).
`duty` = fraction du cycle au sol ; au-dessus du minimum de l'allure, les groupes se
recouvrent : la patte qui se pose est chargee AVANT que la suivante decolle (moins de
glissement).
"""
import math

GAITS = {   # nom -> (decalage de phase de chaque patte, duty minimal, duty par defaut)
    "tripod": ({1: 0.0, 3: 0.0, 5: 0.0, 2: 0.5, 4: 0.5, 6: 0.5}, 0.5, 0.6),
    "tetrapod": ({2: 0.0, 5: 0.0, 3: 1 / 3, 6: 1 / 3, 1: 2 / 3, 4: 2 / 3}, 2 / 3, 0.72),
    "wave": ({4: 0.0, 3: 1 / 6, 2: 2 / 6, 5: 3 / 6, 6: 4 / 6, 1: 5 / 6}, 5 / 6, 0.86),
}
TRIPOD_OFFSET, _, DUTY = GAITS["tripod"]


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
                 max_linear=0.10, max_angular=0.6, reach=0.045, height_limits=(0.0, 1.0),
                 gait="tripod", duty=0.0):
        self.legs = legs
        self.offsets, self.duty = GAITS["tripod"][0], 0.5
        self.pattern = None
        self.pending = None
        self.set_pattern(gait, duty)
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
    STRIDE_MARGIN = 0.8            # pas vise = 80 % du pas maximal (2 x reach) : marge pour les transitions

    def foot_speed(self, vel):
        """Vitesse max d'un pied au sol (m/s) pour la vitesse du corps `vel`."""
        return max(math.hypot(*self._foot_vel(vel, *lg.neutral(self.radius, 0)[:2])) for lg in self.legs)

    def set_cmd(self, vx, vy, wz):
        """Consigne ; reduite (en gardant la direction) si, meme a la cadence la plus rapide
        (`cycle_time`), le pas depasserait 2 x reach."""
        fs = self.foot_speed((vx, vy, wz))
        stride = fs * self.duty * self.cycle_time
        k = min(1.0, 2 * self.reach * self.STRIDE_MARGIN / stride) if stride > 1e-9 else 1.0
        self.cmd = (vx * k, vy * k, wz * k)
        return k

    def set_pattern(self, name, duty=0.0):
        """Change d'allure (appliquee a l'arret si le robot marche). duty 0 = defaut."""
        if name not in GAITS:
            raise ValueError(f"allure inconnue : {name} ({', '.join(GAITS)})")
        offsets, dmin, ddef = GAITS[name]
        d = ddef if not duty else max(dmin, min(0.95, duty))
        if getattr(self, "walking", False):
            self.pending = (name, offsets, d)
        else:
            self.pattern, self.offsets, self.duty = name, offsets, d
        return d

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
        return (self.phase + self.offsets[n]) % 1.0

    def in_stance(self, n):
        return self.leg_phase(n) < self.duty

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
        if not self.walking and self.pending:                       # nouvelle allure : a l'arret
            self.pattern, self.offsets, self.duty = self.pending
            self.pending = None

        if self.walking:
            self.phase = (self.phase + dt / self.cycle_period()) % 1.0

        t_stance = self.duty * self.cycle_period()
        for lg in self.legs:
            n, p = lg.n, self.feet[lg.n]
            if not self.walking:
                p[2] = -self.height
                self.liftoff[n] = None
                continue
            ph = self.leg_phase(n)
            if ph < self.duty:                               # appui
                self.liftoff[n] = None
                fx, fy = self._foot_vel(self.vel, p[0], p[1])
                p[0] += fx * dt
                p[1] += fy * dt
                p[2] = -self.height
            else:                                            # vol
                if self.liftoff[n] is None:
                    self.liftoff[n] = (p[0], p[1])
                s = (ph - self.duty) / (1 - self.duty)
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
        """Cadence suivant la vitesse : le pas fait ~90 % de la longueur permise ; la periode
        va de `cycle_time` (vite) a 2 x `cycle_time` (lent : pas plus courts et plus lents)."""
        if not any(abs(c) > 1e-4 for c in self.cmd):
            return self.cycle_time            # arret / replacement : finir le pas sans trainer
        fs = self.foot_speed(self.vel)
        if fs < 1e-6:
            return 2 * self.cycle_time
        margin = self.STRIDE_MARGIN
        if self.reposition or abs(self.height_target - self.height) > 1e-3:
            margin *= 0.5                 # changement de hauteur en marchant : pas plus courts
        period = 2 * self.reach * margin / (fs * self.duty)
        return max(self.cycle_time, min(2 * self.cycle_time, period))

    def _feet_ok_at(self, h):
        """Les pieds (au sol ou en l'air) restent-ils atteignables si le corps passe a h ?"""
        for lg in self.legs:
            x, y, z = self.feet[lg.n]
            q = lg.ik(x, y, z - (h - self.height))
            if q is None or not lg.in_limits(q):
                return False
        return True

    def _phase_at_switch(self, dt):
        """Vrai quand toutes les pattes sont au sol (ou a un pas de temps de l'etre)."""
        step = 1.5 * dt / self.cycle_period()
        for n in self.offsets:
            ph = self.leg_phase(n)
            if ph >= self.duty and min(ph - self.duty, 1.0 - ph) > step:
                return False
        return True
