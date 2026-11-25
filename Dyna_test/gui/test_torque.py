"""test_torque - fait se lever tout l'hexapode (6 pattes) et mesure l'effort des servos.

Utilise la configuration des pattes de l'interface (leg_gui_config.json : IDs, sens,
offsets, montage, geometrie) et la cinematique inverse de hexapod_model.py.

Deroule (robot pose au sol, corps sur le sol) :
  0. VERIFS   : 18 servos presents, aucun en alarme, temperature < --temp-max ;
                TOUTES les poses du plan sont calculees et verifiees (atteignables, dans
                les butees) avant le moindre mouvement
  1. REGLAGES : femurs / tibias pente 8, punch 100 (dyna.md 2.4), vitesse --speed ;
                consigne = position actuelle puis couple ON (aucun saut)
  2. RAPPROCHER (si --radius) : par trepieds, chaque patte se leve de --clear mm, va a
                --radius mm de l'axe de sa hanche et se repose a la hauteur de depart.
                Bras de levier plus court = moins de couple pour se lever.
  3. LEVER    : tous les bouts de patte descendent par pas de --step mm jusqu'a --stand-z
                (hauteur du bout de patte sous l'axe des hanches). A CHAQUE PAS on attend
                que les 18 servos soient arrives (ecart <= --tol) : pas de passage au pas
                suivant sur delai. Si un pas n'est pas atteint en --settle-max s, la montee
                s'arrete (echec note) et le robot redescend.
  4. TENIR    : --hold s debout (mesures)
  5. (--full) DEPLACER le corps et TREPIEDS, comme avant
  6. REDESCENDRE a la hauteur de depart

Mesures (pour graphiques), dans gui/logs/ :
  test_torque_<date>_echantillons.csv : chaque lecture (t, phase, pas, z vise, et pour les
                                        18 servos position, consigne, charge, temperature)
  test_torque_<date>_pas.csv          : un resume par pas (z, atteint, temps, charges
                                        max / moyennes femurs / tibias, bras de levier)
  test_torque_<date>.json             : parametres + resume
  essais.csv                          : une ligne par essai (cumul de tous les essais)

Securite : arret immediat (consignes figees, couple garde pour ne pas lacher le robot)
si un servo passe en alarme ou depasse --temp-max ; Ctrl+C = meme arret.

    python test_torque.py --dry-run                  # lit le robot, calcule, ne bouge rien
    python test_torque.py --radius 170 --stand-z -90 # essai reel (confirmation demandee)
"""

import argparse
import csv
import json
import math
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hexapod_model as hm  # noqa: E402

SAFETY_BITS = {0x01: "tension", 0x04: "surchauffe", 0x20: "surcharge", 0x08: "hors plage"}
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
AX12_STALL_NM = 1.5          # couple de blocage AX-12A a 12 V (doc ROBOTIS)


class Abort(Exception):
    pass


class Robot:
    def __init__(self, host, port, dry):
        self.dry = dry
        self.sock = None
        if host:
            try:
                self.sock = socket.create_connection((host, port), timeout=5)
                self.sock.settimeout(5)
                self.f = self.sock.makefile("r", encoding="utf-8")
            except OSError as e:
                if not dry:
                    raise Abort(f"connexion impossible a {host}:{port} ({e}) : leg_server lance sur la Pi ?")
                print(f"(simulation sans serveur : {e})")

    def rpc(self, line, force=False):
        """Envoie une commande. En simulation, seules les lectures partent au serveur."""
        if self.sock is None or (self.dry and not force):
            return "OK (simulation)"
        try:
            self.sock.sendall((line + "\n").encode())
            rep = self.f.readline()
        except (socket.timeout, TimeoutError):
            raise Abort(f"pas de reponse de leg_server a '{line.split()[0]}' en 5 s : serveur occupe "
                        "par un autre client (ancienne version : un seul client a la fois, fermer "
                        "l'interface) ou bloque")
        except OSError as e:
            raise Abort(f"liaison avec leg_server perdue ({e})")
        if not rep:
            raise Abort("serveur deconnecte")
        if rep.startswith("ERR trop de clients"):
            raise Abort("leg_server : trop de clients connectes (fermer une interface)")
        return rep.strip()

    def state(self):
        if self.sock is None:
            return {}
        rep = self.rpc("STATE", force=True)
        if not rep.startswith("OK "):
            raise Abort("STATE : " + rep)
        return {s["id"]: s for s in json.loads(rep[3:])["servos"]}


class TorqueTest:
    def __init__(self, args):
        self.a = args
        self.cfg = hm.load_config()
        self.legs = self.cfg["legs"]
        self.ids = hm.all_ids(self.cfg)
        self.r = Robot(None if args.offline else args.host, args.port, args.dry_run)
        self.st = {}
        self.goals = {}
        self.peak = {}
        self.t0 = time.time()
        self.tag = time.strftime("%Y%m%d_%H%M%S")
        self.phase, self.step_no, self.z_target = "", 0, None
        self.steps = []                    # resume par pas
        self.samples_f = None
        self.max_lift = 0.0
        self.result = "non lance"

    # ------------------------------------------------------------------ mesures
    def open_logs(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, f"test_torque_{self.tag}_echantillons.csv")
        self.samples_f = open(path, "w", newline="", encoding="utf-8")
        self.samples = csv.writer(self.samples_f, delimiter=";")
        head = ["t_s", "phase", "pas", "z_vise_mm"]
        for sid in self.ids:
            head += [f"pos_{sid}", f"cons_{sid}", f"charge_{sid}", f"temp_{sid}"]
        head += ["tension_V"]
        self.samples.writerow(head)
        return path

    def log_sample(self):
        if not self.samples_f or not self.st:
            return
        row = [round(time.time() - self.t0, 3), self.phase, self.step_no,
               "" if self.z_target is None else round(self.z_target, 1)]
        volts = []
        for sid in self.ids:
            s = self.st.get(sid, {})
            if s.get("ok"):
                load = s["load"] if s.get("torque") else ""
                row += [s["pos"], self.goals.get(sid, s["goal"]), load, s["temp"]]
                volts.append(s["volt"])
            else:
                row += ["", "", "", ""]
        row.append(round(min(volts) / 10, 1) if volts else "")
        self.samples.writerow(row)

    # ------------------------------------------------------------------ geometrie
    def leg_angles_now(self, leg):
        out = []
        for k, sid in enumerate(leg["ids"]):
            s = self.st.get(sid)
            if not s or not s.get("ok"):
                return None
            out.append(hm.raw_to_joint(leg, k, s["pos"]))
        return out

    def feet_now(self):
        feet = []
        for leg in self.legs:
            ang = self.leg_angles_now(leg) or [0.0, 0.0, -60.0]   # simulation : pose nominale
            p = hm.leg_to_body(self.cfg, leg, ang)
            feet.append((p["foot"][0], p["foot"][1], p["foot_z"]))
        return feet

    def at_radius(self, leg, foot, radius):
        """Meme direction depuis la hanche, a `radius` mm de l'axe de la hanche."""
        m = leg["mount"]
        dx, dy = foot[0] - m["x"], foot[1] - m["y"]
        d = math.hypot(dx, dy) or 1.0
        return (m["x"] + dx / d * radius, m["y"] + dy / d * radius, foot[2])

    def levers(self, feet):
        """Bras de levier horizontaux moyens (mm) : femur (axe femur -> bout) et tibia
        (genou -> bout), dans le plan de chaque patte."""
        lf, lt = [], []
        for leg, (x, y, z) in zip(self.legs, feet):
            sol = hm.body_ik(self.cfg, leg, x, y, z)
            if sol is None:
                continue
            g = hm.leg_fk(self.cfg, sol)
            lf.append(g["foot"][0] - self.cfg["lengths_mm"]["coxa"])
            lt.append(g["foot"][0] - g["knee"][0])
        return (sum(lf) / len(lf) if lf else 0), (sum(lt) / len(lt) if lt else 0)

    def pose_goals(self, feet):
        """Bouts de patte (repere corps) -> {id: raw}. Abort si hors d'atteinte / butees."""
        goals = {}
        for leg, (x, y, z) in zip(self.legs, feet):
            sol = hm.body_ik(self.cfg, leg, x, y, z)
            if sol is None:
                raise Abort(f"pose hors d'atteinte pour la patte {leg['name']} "
                            f"(bout de patte {x:.0f}, {y:.0f}, {z:.0f} mm)")
            for k, sid in enumerate(leg["ids"]):
                raw = hm.joint_to_raw(leg, k, sol[k])
                s = self.st.get(sid, {})
                lo, hi = (s.get("cw_limit", 0), s.get("ccw_limit", 1023)) if s.get("ok") else (0, 1023)
                if not lo <= raw <= hi:
                    raise Abort(f"patte {leg['name']}, ID {sid} : consigne {raw} hors butees {lo}..{hi} "
                                f"(articulation {sol[k]:+.0f} deg, bout de patte z {z:.0f} mm)")
                goals[sid] = raw
        return goals

    # ------------------------------------------------------------------ surveillance
    def check_safety(self):
        for sid in self.ids:
            s = self.st.get(sid)
            if not s or not s.get("ok"):
                if self.r.sock is not None:
                    raise Abort(f"ID {sid} ne repond plus")
                continue
            bits = [t for b, t in SAFETY_BITS.items() if s.get("err", 0) & b]
            if bits or s.get("tlimit") == 0:
                raise Abort(f"ID {sid} en securite ({', '.join(bits) or 'limite de couple 0'})")
            if s["temp"] >= self.a.temp_max:
                raise Abort(f"ID {sid} a {s['temp']} degC (max {self.a.temp_max})")
            if s.get("torque"):            # couple OFF : charge figee, sans signification
                self.peak[sid] = max(self.peak.get(sid, 0), abs(s["load"]) * 100 / 1023)

    def stats(self):
        """charge max (%, id), charge moyenne femurs / tibias (%), ecart max, id de l'ecart."""
        worst, wid, fem, tib, err, eid = 0, None, [], [], 0, None
        for leg in self.legs:
            for k, sid in enumerate(leg["ids"]):
                s = self.st.get(sid)
                if not s or not s.get("ok"):
                    continue
                if s.get("torque"):
                    l = abs(s["load"]) * 100 / 1023
                    if l > worst:
                        worst, wid = l, sid
                    (fem if k == 1 else tib if k == 2 else []).append(l)
                if sid in self.goals and abs(s["pos"] - self.goals[sid]) > err:
                    err, eid = abs(s["pos"] - self.goals[sid]), sid
        mean = lambda v: sum(v) / len(v) if v else 0
        return worst, wid, mean(fem), mean(tib), err, eid

    def move(self, feet):
        """Envoie une pose et attend que TOUS les servos soient arrives (ecart <= tol).
        Renvoie (atteint, duree)."""
        self.goals = self.pose_goals(feet)
        self.r.rpc("GOALSID " + " ".join(f"{i} {v}" for i, v in self.goals.items()))
        t0 = time.time()
        if self.r.sock is None or self.a.dry_run:
            return True, 0.0
        stable = 0
        while True:
            time.sleep(0.06)
            self.st = self.r.state()
            self.check_safety()
            self.log_sample()
            *_, err, _eid = self.stats()
            stable = stable + 1 if err <= self.a.tol else 0
            if stable >= 3:                         # 3 lectures de suite dans la tolerance
                return True, time.time() - t0
            if time.time() - t0 > self.a.settle_max:
                return False, time.time() - t0

    def record_step(self, label, feet, ok, dt):
        worst, wid, fem, tib, err, eid = self.stats() if self.st else (0, None, 0, 0, 0, None)
        lf, lt = self.levers(feet)
        temps = [s["temp"] for s in self.st.values() if s.get("ok")]
        volts = [s["volt"] for s in self.st.values() if s.get("ok")]
        row = dict(t_s=round(time.time() - self.t0, 2), phase=self.phase, pas=self.step_no, etape=label,
                   z_vise_mm=round(feet[0][2], 1), atteint=int(ok), duree_s=round(dt, 2),
                   charge_max_pct=round(worst), id_charge_max=wid, femurs_moy_pct=round(fem),
                   tibias_moy_pct=round(tib), ecart_max=err, id_ecart_max=eid,
                   levier_femur_mm=round(lf), levier_tibia_mm=round(lt),
                   temp_max=max(temps) if temps else "", tension_V=round(min(volts) / 10, 1) if volts else "")
        self.steps.append(row)
        self.say(f"  {label:22s} z {row['z_vise_mm']:+6.1f} | {'OK ' if ok else 'NON'} {dt:4.1f}s | "
                 f"charge max {worst:3.0f}% (ID {wid}) femurs {fem:3.0f}% tibias {tib:3.0f}% | "
                 f"ecart {err} (ID {eid}) | levier femur {lf:.0f} mm | T {row['temp_max']} | {row['tension_V']} V")

    def say(self, msg):
        print(msg, flush=True)

    # ------------------------------------------------------------------ plan
    def plan(self):
        """Liste de (phase, label, pose). Pose = 6 bouts de patte (x, y, z) repere corps."""
        a = self.a
        start = self.start
        # hauteur de depart commune des bouts de patte : fixe (--start-z), pour ne pas
        # dependre d'une patte laissee en l'air ; defaut -10 mm (sol vers -37 mm le 25/11)
        z0 = a.start_z if a.start_z is not None else max(z for _, _, z in start)
        base = [(x, y, z0) for (x, y, _) in start]
        out = []
        if a.radius:
            order = sorted(range(6), key=lambda k: self.legs[k]["mount"]["angle"])
            cur = list(base)
            for grp in (order[0::2], order[1::2]):
                up = [(x, y, z + a.clear) if k in grp else (x, y, z) for k, (x, y, z) in enumerate(cur)]
                out.append(("rapprocher", "lever trepied", up))
                moved = [self.at_radius(self.legs[k], p, a.radius) if k in grp else p
                         for k, p in enumerate(up)]
                out.append(("rapprocher", "deplacer trepied", moved))
                cur = [(x, y, z0) if k in grp else (x, y, z) for k, (x, y, z) in enumerate(moved)]
                out.append(("rapprocher", "poser trepied", cur))
            base = cur
        n = max(1, int(math.ceil((z0 - a.stand_z) / a.step)))
        for j in range(1, n + 1):
            z = z0 + (a.stand_z - z0) * j / n
            out.append(("lever", f"pas {j}/{n}", [(x, y, z) for (x, y, _) in base]))
        stand = [(x, y, a.stand_z) for (x, y, _) in base]
        out.append(("tenir", "debout", stand))
        if a.full:
            for name, (dx, dy) in (("corps avant", (-a.shift, 0)), ("corps arriere", (a.shift, 0)),
                                   ("corps gauche", (0, -a.shift)), ("corps droite", (0, a.shift))):
                out.append(("deplacer", name, [(x + dx, y + dy, z) for (x, y, z) in stand]))
                out.append(("deplacer", "centre", stand))
            order = sorted(range(6), key=lambda k: self.legs[k]["mount"]["angle"])
            for name, grp in (("trepied A", order[0::2]), ("trepied B", order[1::2])):
                out.append(("trepied", name + " leve",
                            [(x, y, z + a.clear) if k in grp else (x, y, z) for k, (x, y, z) in enumerate(stand)]))
                out.append(("trepied", name + " pose", stand))
        for j in range(n - 1, -1, -1):
            z = z0 + (a.stand_z - z0) * j / n
            out.append(("redescendre", f"pas {n - j}/{n}", [(x, y, z) for (x, y, _) in base]))
        self.z0 = z0
        return out

    # ------------------------------------------------------------------ deroulement
    def run(self):
        a = self.a
        self.say(f"=== test_torque {self.tag} {'(SIMULATION, rien ne bouge)' if a.dry_run else ''}")
        if self.r.sock is not None:
            self.say("IDS -> " + self.r.rpc("IDS " + " ".join(map(str, self.ids)), force=True))
            self.st = self.r.state()
            missing = [i for i in self.ids if not self.st.get(i, {}).get("ok")]
            if missing:
                raise Abort(f"servos absents : {missing}")
            self.check_safety()
        self.start = self.feet_now()
        self.say("Pose de depart (bouts de patte, repere corps, mm) :")
        for leg, (x, y, z) in zip(self.legs, self.start):
            r = math.hypot(x - leg["mount"]["x"], y - leg["mount"]["y"])
            self.say(f"  {leg['name']:15s} IDs {leg['ids']}  x {x:5.0f}  y {y:5.0f}  z {z:5.0f}  (a {r:3.0f} mm de la hanche)")
        plan = self.plan()
        for _, _, feet in plan:
            self.pose_goals(feet)
        lf0, _ = self.levers(plan[0][2])
        lf1, _ = self.levers(plan[-1 - (len([p for p in plan if p[0] == 'redescendre']))][2])
        self.say(f"Plan verifie : {len(plan)} poses, toutes atteignables et dans les butees. "
                 f"z depart {self.z0:.0f} mm -> debout {a.stand_z:.0f} mm ; bras de levier femur "
                 f"moyen {lf0:.0f} -> {lf1:.0f} mm.")
        if a.dry_run:
            for ph, label, feet in plan:
                g = self.pose_goals(feet)
                self.say(f"  {ph:11s} {label:18s} z {feet[0][2]:+6.1f} " + " ".join(f"{i}:{g[i]}" for i in sorted(g)))
            self.result = "simulation"
            return
        if not a.yes:
            input("\nLe robot va BOUGER. Robot pose au sol, toi a cote de l'alimentation ?\n"
                  "Entree pour continuer, Ctrl+C pour annuler... ")
        self.say("Mesures : " + self.open_logs())

        # 1. reglages et couple ON sans saut
        for leg in self.legs:
            for k, sid in enumerate(leg["ids"]):
                cmds = [f"SPEED {sid} {a.speed}", f"TLIMIT {sid} 1023"]
                if k > 0:
                    cmds += [f"SLOPE {sid} 8 8", f"PUNCH {sid} 100", f"MARGIN {sid} 1 1"]
                for c in cmds:
                    rep = self.r.rpc(c)
                    if not rep.startswith("OK"):
                        self.say(f"  ! {c} -> {rep}")
        self.st = self.r.state()
        hold = {sid: s["pos"] for sid, s in self.st.items() if s.get("ok")}
        self.r.rpc("GOALSID " + " ".join(f"{i} {v}" for i, v in hold.items()))
        for sid in hold:
            self.r.rpc(f"TORQUE {sid} 1")
        time.sleep(0.5)

        # 2..6
        self.result = "debout"
        failed = False
        cur_z = self.z0
        for ph, label, feet in plan:
            if failed and ph in ("lever", "tenir", "deplacer", "trepied"):
                continue
            # apres un echec, ne jamais "redescendre" vers une pose plus basse que l'actuelle
            if ph == "redescendre" and feet[0][2] < cur_z - 0.01:
                continue
            self.phase, self.z_target = ph, feet[0][2]
            self.step_no += 1
            ok, dt = self.move(feet)
            if ph in ("lever", "redescendre", "tenir"):
                cur_z = feet[0][2]
            self.record_step(f"{ph} {label}", feet, ok, dt)
            if ph == "lever" and ok:
                self.max_lift = self.z0 - feet[0][2]
            if not ok and ph in ("lever", "rapprocher"):
                self.say(f"-> pas non atteint en {a.settle_max} s : arret de la montee, on redescend")
                self.result = f"echec ({ph} {label})"
                failed = True
                if ph == "rapprocher":
                    break
            if ph == "tenir" and ok:
                t_end = time.time() + a.hold
                while time.time() < t_end:
                    time.sleep(0.1)
                    self.st = self.r.state()
                    self.check_safety()
                    self.log_sample()
                self.record_step("tenir fin", feet, True, a.hold)
        self.say(f"Termine : {self.result}. Hauteur levee max {self.max_lift:.0f} mm. "
                 "Couple toujours actif (robot repose).")

    def save(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        if self.samples_f:
            self.samples_f.close()
        base = os.path.join(LOG_DIR, f"test_torque_{self.tag}")
        if self.steps:
            with open(base + "_pas.csv", "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=list(self.steps[0].keys()), delimiter=";")
                w.writeheader()
                w.writerows(self.steps)
        summary = {"date": self.tag, "parametres": vars(self.a), "resultat": self.result,
                   "hauteur_levee_max_mm": round(self.max_lift, 1),
                   "pics_charge_pct": {str(k): round(v) for k, v in self.peak.items()},
                   "config_pattes": self.legs, "geometrie": {k: self.cfg[k] for k in
                                                            ("lengths_mm", "tibia_bend_deg", "foot_offset_mm")}}
        with open(base + ".json", "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=1, ensure_ascii=False)
        # cumul des essais
        if not self.a.dry_run:
            idx = os.path.join(LOG_DIR, "essais.csv")
            new = not os.path.exists(idx)
            top = max(self.peak.items(), key=lambda kv: kv[1]) if self.peak else (None, 0)
            temps = [r["temp_max"] for r in self.steps if r["temp_max"] != ""]
            volts = [r["tension_V"] for r in self.steps if r["tension_V"] != ""]
            with open(idx, "a", newline="", encoding="utf-8") as f:
                w = csv.writer(f, delimiter=";")
                if new:
                    w.writerow(["date", "resultat", "rayon_mm", "z_debout_mm", "pas_mm", "vitesse",
                                "hauteur_levee_mm", "charge_max_pct", "id_charge_max", "temp_max",
                                "tension_min_V", "duree_s", "fichier"])
                w.writerow([self.tag, self.result, self.a.radius or "", self.a.stand_z, self.a.step,
                            self.a.speed, round(self.max_lift, 1), round(top[1]), top[0],
                            max(temps) if temps else "", min(volts) if volts else "",
                            round(time.time() - self.t0), os.path.basename(base)])
        return base


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default=None, help="IP du leg_server (defaut : celle de l'interface)")
    p.add_argument("--port", type=int, default=None)
    p.add_argument("--dry-run", action="store_true", help="calcule et affiche, ne bouge rien")
    p.add_argument("--offline", action="store_true", help="simulation sans se connecter au serveur")
    p.add_argument("--yes", action="store_true", help="pas de confirmation avant de bouger")
    p.add_argument("--radius", type=float, default=None,
                   help="distance bout de patte - axe de hanche pendant le lever (mm) ; defaut : inchangee")
    p.add_argument("--stand-z", type=float, default=-90, help="hauteur du bout de patte debout (mm, <0)")
    p.add_argument("--start-z", type=float, default=-10,
                   help="hauteur de depart commune des bouts de patte, corps pose (mm) ; "
                        "sol mesure vers -37 mm")
    p.add_argument("--step", type=float, default=4, help="pas de descente des bouts de patte (mm)")
    p.add_argument("--clear", type=float, default=15, help="levee des pattes pour les deplacer (mm)")
    p.add_argument("--tol", type=int, default=6, help="ecart toléré position / consigne (unites servo)")
    p.add_argument("--settle-max", type=float, default=8, help="attente max d'un pas avant echec (s)")
    p.add_argument("--hold", type=float, default=5, help="duree debout (s)")
    p.add_argument("--speed", type=int, default=80, help="vitesse des servos (1..1023)")
    p.add_argument("--temp-max", type=int, default=62, help="temperature d'arret (degC)")
    p.add_argument("--full", action="store_true", help="ajoute DEPLACER le corps et TREPIEDS")
    p.add_argument("--shift", type=float, default=20, help="deplacement du corps (mm), avec --full")
    args = p.parse_args()
    cfg = hm.load_config()
    args.host = args.host or cfg["host"]
    args.port = args.port or cfg["port"]

    t = TorqueTest(args)
    try:
        t.run()
    except Abort as e:
        t.result = f"arret : {e}"
        print(f"\n*** ARRET : {e}\n    Consignes figees, couple garde (le robot n'est pas lache).")
    except KeyboardInterrupt:
        t.result = "interrompu"
        print("\n*** Interrompu (Ctrl+C) : consignes figees, couple garde.")
    finally:
        if not args.dry_run:                  # les simulations n'ecrivent rien dans logs/
            print("Fichiers :", t.save() + "_*")
        if t.peak:
            top = sorted(t.peak.items(), key=lambda kv: -kv[1])[:6]
            print("Charges max :", ", ".join(f"ID {i} {v:.0f}%" for i, v in top))


if __name__ == "__main__":
    main()
