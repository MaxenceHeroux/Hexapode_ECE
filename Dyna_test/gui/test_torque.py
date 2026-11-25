"""test_torque - fait bouger les 6 pattes pour soulever puis deplacer le robot, en
mesurant l'effort de chaque servo (charge, temperature, alarmes).

Utilise la configuration des pattes de l'interface (leg_gui_config.json : IDs, sens,
offsets, montage, geometrie) et la cinematique inverse de hexapod_model.py.

Phases :
  0. verifications : 18 servos presents, aucun en alarme, temperatures correctes
  1. reglages : femur / tibia pente 8, punch 100 (cf. README 14.5), vitesse moderee ;
     consigne = position actuelle puis couple ON (aucun saut)
  2. SOULEVER : tous les bouts de patte descendent par pas de --step mm jusqu'a --lift mm
     (le corps monte) ; arret de la montee si un servo depasse --max-load %
  3. TENIR : maintien --hold s a la hauteur atteinte
  4. DEPLACER : le corps se deplace de --shift mm avant / arriere / gauche / droite
     (tous les bouts de patte glissent ensemble dans l'autre sens)
  5. TREPIED : 3 pattes sur 2 se levent de --tripod mm, les 3 autres portent le robot
  6. REPOSER : retour progressif a la pose de depart

Securite : arret immediat (consignes figees, couple garde pour ne pas lacher le robot)
si un servo passe en alarme, depasse --temp-max degC, ou si une pose est hors d'atteinte
ou hors butees. Ctrl+C = meme arret.

    python test_torque.py --dry-run          # calcule et affiche tout, ne bouge rien
    python test_torque.py                    # essai reel (demande confirmation)
    python test_torque.py --lift 20 --skip-move --skip-tripod
"""

import argparse
import json
import math
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hexapod_model as hm  # noqa: E402

SAFETY_BITS = {0x01: "tension", 0x04: "surchauffe", 0x20: "surcharge", 0x08: "hors plage"}


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
                    raise
                print(f"(simulation sans serveur : {e})")
                self.sock = None

    def rpc(self, line, force=False):
        """Envoie une commande. En simulation, seules les lectures partent au serveur."""
        if self.sock is None or (self.dry and not force):
            return "OK (simulation)"
        self.sock.sendall((line + "\n").encode())
        rep = self.f.readline()
        if not rep:
            raise Abort("serveur deconnecte")
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
        self.log = []                      # mesures pour le fichier de resultats
        self.st = {}
        self.start_feet = None             # positions corps des bouts de patte au depart
        self.goals = {}                    # derniere consigne envoyee {id: raw}
        self.peak = {}                     # charge max par servo (%)

    # ------------------------------------------------------------------ utilitaires
    def say(self, msg):
        print(msg, flush=True)

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
            ang = self.leg_angles_now(leg) or [0.0, 0.0, -60.0]     # simulation : pose nominale
            p = hm.leg_to_body(self.cfg, leg, ang)
            feet.append((p["foot"][0], p["foot"][1], p["foot_z"]))
        return feet

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
                                f"(articulation {sol[k]:+.0f} deg)")
                goals[sid] = raw
        return goals

    def check_safety(self):
        hot = []
        for sid in self.ids:
            s = self.st.get(sid)
            if not s or not s.get("ok"):
                if self.r.sock is not None and not self.a.allow_missing:
                    raise Abort(f"ID {sid} ne repond plus")
                continue
            bits = [t for b, t in SAFETY_BITS.items() if s.get("err", 0) & b]
            if bits or s.get("tlimit") == 0:
                raise Abort(f"ID {sid} en securite ({', '.join(bits) or 'limite de couple 0'})")
            if s["temp"] >= self.a.temp_max:
                raise Abort(f"ID {sid} a {s['temp']} degC (max {self.a.temp_max})")
            # couple OFF : le registre de charge garde une valeur figee (souvent +-1023)
            if s.get("torque"):
                load = abs(s["load"]) * 100 / 1023
                self.peak[sid] = max(self.peak.get(sid, 0), load)
            if s["temp"] >= self.a.temp_max - 5:
                hot.append(sid)
        return hot

    def loads(self):
        """(charge max %, id), charge moyenne femur %, ecart de position max."""
        worst, wid, fem, n, err = 0, None, 0, 0, 0
        for leg in self.legs:
            for k, sid in enumerate(leg["ids"]):
                s = self.st.get(sid)
                if not s or not s.get("ok") or not s.get("torque"):
                    continue                    # couple OFF : charge figee, sans signification
                l = abs(s["load"]) * 100 / 1023
                if l > worst:
                    worst, wid = l, sid
                if k == 1:
                    fem += l
                    n += 1
                if sid in self.goals:
                    err = max(err, abs(s["pos"] - self.goals[sid]))
        return worst, wid, (fem / n if n else 0), err

    def move(self, feet, label, settle=None):
        """Envoie une pose et attend qu'elle soit atteinte en surveillant les servos."""
        goals = self.pose_goals(feet)
        self.goals = goals
        self.r.rpc("GOALSID " + " ".join(f"{i} {v}" for i, v in goals.items()))
        t0 = time.time()
        settle = self.a.settle if settle is None else settle
        over_since = None
        while True:
            if self.r.sock is None or self.a.dry_run:
                break
            time.sleep(0.08)
            self.st = self.r.state()
            self.check_safety()
            worst, wid, fem, err = self.loads()
            if worst > self.a.max_load:
                over_since = over_since or time.time()
                if time.time() - over_since > 0.6:
                    self.record(label, "charge max depassee")
                    return False
            else:
                over_since = None
            if err <= 6 and time.time() - t0 > 0.3:
                break
            if time.time() - t0 > settle:
                break
        self.record(label)
        return True

    def record(self, label, note=""):
        worst, wid, fem, err = self.loads() if self.st else (0, None, 0, 0)
        temps = [s["temp"] for s in self.st.values() if s.get("ok")]
        row = dict(t=round(time.time(), 2), etape=label, charge_max=round(worst), id_max=wid,
                   femur_moy=round(fem), ecart_max=err, temp_max=max(temps) if temps else None, note=note)
        self.log.append(row)
        self.say(f"  {label:28s} charge max {worst:3.0f}% (ID {wid})  femurs {fem:3.0f}%  "
                 f"ecart {err:3d}  T {row['temp_max']}  {note}")

    def path(self, a, b, step):
        """Liste de poses intermediaires (interpolation lineaire, pas <= step mm)."""
        d = max(math.dist(p, q) for p, q in zip(a, b))
        n = max(1, int(math.ceil(d / step)))
        return [[tuple(p[i] + (q[i] - p[i]) * j / n for i in range(3)) for p, q in zip(a, b)]
                for j in range(1, n + 1)]

    def go(self, target, label, step=None):
        cur = self.current
        for j, pose in enumerate(self.path(cur, target, step or self.a.step), 1):
            ok = self.move(pose, f"{label} {j}")
            self.current = pose
            if not ok:
                return False
        return True

    # ------------------------------------------------------------------ deroulement
    def run(self):
        a = self.a
        self.say(f"=== test_torque {'(SIMULATION, rien ne bouge)' if a.dry_run else ''}")
        # 0. verifications
        if self.r.sock is not None:
            self.say("IDS -> " + self.r.rpc("IDS " + " ".join(map(str, self.ids)), force=True))
            self.st = self.r.state()
            missing = [i for i in self.ids if not self.st.get(i, {}).get("ok")]
            if missing:
                msg = f"servos absents : {missing}"
                if not a.allow_missing:
                    raise Abort(msg + " (configurer les pattes dans l'interface, ou --allow-missing)")
                self.say("ATTENTION " + msg)
            self.check_safety()
        self.start_feet = self.feet_now()
        self.current = list(self.start_feet)
        self.say("Pose de depart (bouts de patte, repere corps, mm) :")
        for leg, (x, y, z) in zip(self.legs, self.start_feet):
            self.say(f"  {leg['name']:16s} IDs {leg['ids']}  x {x:6.0f}  y {y:6.0f}  z {z:6.0f}")

        # verification a sec de toutes les poses avant de bouger
        plan = self.plan()
        for label, feet in plan:
            self.pose_goals(feet)
        self.say(f"Plan verifie : {len(plan)} poses cles atteignables et dans les butees.")
        if a.dry_run:
            for label, feet in plan:
                g = self.pose_goals(feet)
                self.say(f"  {label:24s} " + " ".join(f"{i}:{g[i]}" for i in sorted(g)))
            return
        if not a.yes:
            input("\nLe robot va BOUGER. Robot pose au sol, toi a cote de l'alimentation ?\n"
                  "Entree pour continuer, Ctrl+C pour annuler... ")

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
        self.st = self.r.state()
        self.record("couple ON (depart)")

        # 2..6
        for label, feet in plan:
            if not self.go(feet, label):
                self.say(f"-> {label} : arret de la phase (charge > {a.max_load}%), retour a la pose de depart")
                break
            if label.startswith("tenir"):
                t_end = time.time() + a.hold
                while time.time() < t_end:
                    time.sleep(0.25)
                    self.st = self.r.state()
                    self.check_safety()
                self.record("fin du maintien")
        self.go(self.start_feet, "reposer")
        self.say("Termine : robot revenu a la pose de depart (couple toujours actif).")

    def plan(self):
        """Poses cles (repere corps) des phases 2 a 6."""
        a = self.a
        base = self.start_feet
        lifted = [(x, y, z - a.lift) for (x, y, z) in base]
        steps = [("soulever", lifted), ("tenir", lifted)]
        if not a.skip_move:
            for name, (dx, dy) in (("corps avant", (-a.shift, 0)), ("corps arriere", (a.shift, 0)),
                                   ("corps gauche", (0, -a.shift)), ("corps droite", (0, a.shift))):
                steps.append((name, [(x + dx, y + dy, z) for (x, y, z) in lifted]))
                steps.append(("centre", lifted))
        if not a.skip_tripod:
            order = sorted(range(len(self.legs)), key=lambda k: self.legs[k]["mount"]["angle"])
            for name, group in (("trepied A", order[0::2]), ("trepied B", order[1::2])):
                up = [(x, y, z + a.tripod) if k in group else (x, y, z)
                      for k, (x, y, z) in enumerate(lifted)]
                steps.append((name + " leve", up))
                steps.append((name + " pose", lifted))
        return steps

    def save(self):
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        os.makedirs(out, exist_ok=True)
        path = os.path.join(out, time.strftime("test_torque_%Y%m%d_%H%M%S.json"))
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"args": vars(self.a), "pics_charge_pct": self.peak, "mesures": self.log}, f,
                      indent=1, ensure_ascii=False)
        return path


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default=None, help="IP du leg_server (defaut : celle de l'interface)")
    p.add_argument("--port", type=int, default=None)
    p.add_argument("--dry-run", action="store_true", help="calcule et affiche, ne bouge rien")
    p.add_argument("--offline", action="store_true", help="simulation sans se connecter au serveur")
    p.add_argument("--yes", action="store_true", help="pas de confirmation avant de bouger")
    p.add_argument("--lift", type=float, default=30, help="hauteur de soulevement (mm), defaut 30")
    p.add_argument("--step", type=float, default=5, help="pas de deplacement (mm), defaut 5")
    p.add_argument("--shift", type=float, default=20, help="deplacement du corps (mm), defaut 20")
    p.add_argument("--tripod", type=float, default=20, help="levee des pattes en trepied (mm)")
    p.add_argument("--hold", type=float, default=3, help="duree de maintien en haut (s)")
    p.add_argument("--speed", type=int, default=100, help="vitesse des servos (1..1023)")
    p.add_argument("--settle", type=float, default=1.5, help="attente max par pas (s)")
    p.add_argument("--max-load", type=float, default=85, help="charge max avant arret de phase (%%)")
    p.add_argument("--temp-max", type=int, default=62, help="temperature d'arret (degC)")
    p.add_argument("--allow-missing", action="store_true", help="continuer meme si des servos manquent")
    p.add_argument("--skip-move", action="store_true", help="sans la phase DEPLACER")
    p.add_argument("--skip-tripod", action="store_true", help="sans la phase TREPIED")
    args = p.parse_args()
    cfg = hm.load_config()
    args.host = args.host or cfg["host"]
    args.port = args.port or cfg["port"]

    t = TorqueTest(args)
    try:
        t.run()
    except Abort as e:
        print(f"\n*** ARRET : {e}\n    Consignes figees, couple garde (le robot n'est pas lache).\n"
              "    Remettre en etat avec l'interface (Reset securite / reposer le robot).")
    except KeyboardInterrupt:
        print("\n*** Interrompu (Ctrl+C) : consignes figees, couple garde.")
    finally:
        if t.log:
            print("Mesures :", t.save())
        if t.peak:
            top = sorted(t.peak.items(), key=lambda kv: -kv[1])[:6]
            print("Charges max :", ", ".join(f"ID {i} {v:.0f}%" for i, v in top))


if __name__ == "__main__":
    main()
