"""Pont provisoire ROS 2 -> leg_server (en attendant hexapod_hardware, phase P2).

Lit /joint_states (radians, convention hexapod_model), convertit en positions servo
(sens, offsets de legs.yaml) et envoie GOALSID au leg_server (TCP 5555) de la Pi.

Securite :
  - send:=false (defaut) : SIMULATION, rien n'est envoye aux servos ; affiche l'ecart
    entre la pose demandee et la pose reelle ;
  - au demarrage : 18 servos presents, aucun en alarme ; reglages (vitesse, compliance
    8/100 fémurs/tibias) ; consigne = position actuelle PUIS couple ON (aucun saut) ;
  - rampe : la pose passe de la position reelle a la pose de la marche en `ramp_time` s ;
  - consignes hors butees -> gel ; servo absent, en alarme, trop chaud, ou charge >
    `max_load` % pendant plus de `load_time` s -> GEL : on n'envoie plus rien, le couple
    reste actif (le robot n'est pas lache). Relancer le noeud pour repartir.
  - Ctrl+C : gel (couple garde). Couper le couple : `nc 192.168.137.10 5555` puis ESTOP.
"""
import json
import math
import socket
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int8MultiArray
from std_srvs.srv import SetBool, Trigger

from .kinematics import load_legs

DEG_PER_UNIT = 300.0 / 1024.0
SAFETY_BITS = {0x01: "tension", 0x04: "surchauffe", 0x20: "surcharge", 0x08: "hors plage"}


class LegServer:
    """Client TCP du leg_server. Sur une liaison qui perd des paquets, une reponse qui
    tarde fait rouvrir la connexion (la reponse en retard ne doit pas etre lue a la
    place de la suivante) ; l'appelant decide de reessayer ou non."""

    def __init__(self, host, port, timeout=1.5):
        self.host, self.port, self.timeout = host, port, timeout
        self.sock = None
        self.connect()

    def connect(self):
        self.close()
        self.sock = socket.create_connection((self.host, self.port), timeout=5)
        self.sock.settimeout(self.timeout)
        self.f = self.sock.makefile("r", encoding="utf-8")

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def rpc(self, line):
        try:
            if self.sock is None:
                self.connect()
            self.sock.sendall((line + "\n").encode())
            rep = self.f.readline()
        except (OSError, ValueError) as e:              # timeout, connexion coupee
            self.close()
            raise ConnectionError(f"{type(e).__name__} : {e}")
        if not rep:
            self.close()
            raise ConnectionError("leg_server deconnecte")
        return rep.strip()

    def state(self):
        rep = self.rpc("STATE")
        if not rep.startswith("OK "):
            raise RuntimeError("STATE : " + rep)
        return {s["id"]: s for s in json.loads(rep[3:])["servos"]}


class Bridge(Node):
    def __init__(self):
        super().__init__("leg_server_bridge")
        p = self.declare_parameter
        p("host", "192.168.137.10")
        p("port", 5555)
        p("legs_yaml", "")
        p("send", False)
        p("rate", 25.0)
        p("speed", 100)            # vitesse des servos (1..1023), limite les a-coups
        p("ramp_time", 5.0)
        p("max_load", 85.0)
        p("load_time", 0.6)
        p("temp_max", 62)
        # Mouvement fluide : vitesse de chaque servo = celle qu'il faut pour arriver pile a la
        # consigne suivante (GOALSPD du leg_server), x speed_gain, bornee.
        p("speed_gain", 1.3)
        p("min_speed", 20)
        # Compliance selon la phase de la patte (/leg_phases de gait_node), par articulation
        # [coxa, femur, tibia] : au sol (porte le robot) femur / tibia raides ; en l'air tout
        # souple et sans gros punch (sinon les servos du haut de la patte tremblent).
        p("stance_slope", [8, 8, 8])
        p("stance_punch", [100, 100, 100])
        p("swing_slope", [64, 128, 64])
        p("swing_punch", [16, 16, 24])
        # Robot PORTE en l'air (aucune patte au sol ne porte de poids) : tout souple, sans
        # punch -> mouvements fluides, pas d'a-coups. Detection par la charge des femurs /
        # tibias des pattes censees etre au sol (avec hysteresis).
        # Detection automatique DESACTIVEE par defaut : les engrenages des AX-12A tiennent le
        # poids sans effort moteur, la "charge" lue est ~0 % robot debout (mesure du 26/11).
        # Mode porte a la main : service /set_lifted (touche p de la fenetre Actions).
        p("auto_air", False)
        p("air_load", 6.0)         # % : charge moyenne des femurs au sol en dessous -> en l'air...
        p("air_time", 0.6)         # s : ... depuis plus longtemps
        p("ground_load", 12.0)     # % : charge moyenne au-dessus -> de nouveau au sol (debout : ~20 %)
        p("air_slope", [64, 128, 128])
        p("air_punch", [0, 0, 0])
        # Auto-reparation au lieu du gel : un servo en alarme (surcharge...) ou bloque loin de
        # sa consigne (> stuck_err unites pendant stuck_time s) est remis en route (RESET) ;
        # au-dela de max_resets en reset_window s, ou en surchauffe, ou absent plus de
        # absent_time s, le pont gele.
        p("stuck_err", 60)
        p("stuck_time", 1.0)
        p("max_resets", 6)
        p("reset_window", 30.0)
        p("absent_time", 2.0)
        p("reset_ramp_time", 3.0)  # s : rampe apres /reset_servos (position reelle -> pose de la marche)
        p("link_timeout", 4.0)     # s sans reponse du leg_server (liaison qui perd des paquets) avant gel
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.send = bool(g("send"))
        self.legs = load_legs(g("legs_yaml") or None)
        import yaml
        from .kinematics import default_legs_yaml
        raw = yaml.safe_load(open(g("legs_yaml") or default_legs_yaml(), encoding="utf-8"))
        self.servo = {}            # nom d'articulation -> (id, inverse, offset_deg, cw, ccw)
        for lg in self.legs:
            d = raw["legs"][f"leg_{lg.n}"]
            for k, nm in enumerate(lg.joint_names):
                self.servo[nm] = (d["ids"][k], bool(d["inverted"][k]), float(d["offset_deg"][k]))

        self.ls = LegServer(g("host"), g("port"))
        st = self.ls.state()
        self.limits = {sid: (s["cw_limit"], s["ccw_limit"]) for sid, s in st.items() if s.get("ok")}
        missing = [sid for sid, *_ in self.servo.values() if sid not in self.limits]
        if missing:
            raise RuntimeError(f"servos absents : {missing}")
        self.check(st, startup=True)
        # Depart = position reelle, ramenee dans les butees : robot affaisse (couple coupe),
        # un servo peut etre 1 ou 2 unites au-dela ; il y reviendra pendant la rampe.
        self.start_raw = {sid: min(self.limits[sid][1], max(self.limits[sid][0], st[sid]["pos"]))
                          for sid in self.limits}
        out = {sid: st[sid]["pos"] for sid in self.limits if st[sid]["pos"] != self.start_raw[sid]}
        if out:
            self.get_logger().info(f"positions hors butees au depart (ramenees dans les butees) : {out}")
        self.target = None
        self.frozen = None
        self.t_ramp = None
        self.over = {}
        self.stuck = {}
        self.absent_since = {}
        self.resets = []
        self.link_lost = None
        self.cycle = 0
        self.last_goal = dict(self.start_raw)
        self.leg_ids = {lg.n: [self.servo[nm][0] for nm in lg.joint_names] for lg in self.legs}
        self.phases = {lg.n: 1 for lg in self.legs}        # au demarrage : toutes au sol
        self.applied = {}
        self.lifted = False
        self.low_since = None
        self.pub_lifted = self.create_publisher(Bool, "robot_lifted", 10)
        self.has_goalspd = self.ls.rpc("GOALSPD") == "ERR aucune consigne"
        self.has_syncw = self.ls.rpc("SYNCW") == "ERR addr len manquants"
        if not (self.has_goalspd and self.has_syncw):
            self.get_logger().warn(
                "leg_server ancien (sans GOALSPD / SYNCW) : vitesse fixe et compliance fixe. "
                "Mettre a jour Dyna_test/src/leg_server.cpp sur la Pi et le relancer.")
        if self.send:
            for nm, (sid, _, _) in self.servo.items():
                cmds = [f"SPEED {sid} {g('speed')}", f"TLIMIT {sid} 1023"]
                if not nm.endswith("coxa_joint"):
                    cmds += [f"SLOPE {sid} 8 8", f"PUNCH {sid} 100", f"MARGIN {sid} 1 1"]
                for c in cmds:
                    rep = self.ls.rpc(c)
                    if not rep.startswith("OK"):
                        self.get_logger().warn(f"{c} -> {rep}")
            self.ls.rpc("GOALSID " + " ".join(f"{i} {v}" for i, v in self.start_raw.items()))
            for sid in self.start_raw:
                self.ls.rpc(f"TORQUE {sid} 1")
            self.get_logger().warn("ENVOI ACTIF : le robot va bouger (rampe de "
                                   f"{g('ramp_time'):.0f} s vers la pose de la marche)")
        else:
            self.get_logger().info("SIMULATION (send:=false) : rien n'est envoye aux servos")
        self.create_subscription(JointState, "joint_states", self.on_js, 10)
        self.create_subscription(Int8MultiArray, "leg_phases", self.on_phases, 10)
        self.figure_until = 0.0              # pas de detection "porte en l'air" pendant une figure
        self.create_subscription(Bool, "figure_active", self.on_figure, 10)
        self.ramp_dur = g("ramp_time")
        self.create_service(Trigger, "reset_servos", self.on_reset)
        self.create_service(SetBool, "set_lifted", self.on_set_lifted)
        self.create_timer(1.0 / g("rate"), self.tick)

    # ------------------------------------------------------------------ conversions
    def to_raw(self, name, rad):
        sid, inv, off = self.servo[name]
        d = math.degrees(rad) - off
        if inv:
            d = -d
        return sid, int(round(512 + d / DEG_PER_UNIT))

    def on_js(self, m):
        goals = {}
        for nm, q in zip(m.name, m.position):
            if nm in self.servo:
                sid, raw = self.to_raw(nm, q)
                goals[sid] = raw
        if len(goals) == 18:
            self.target = goals

    def on_phases(self, m):
        for n, v in enumerate(m.data, start=1):
            if n in self.phases:
                self.phases[n] = int(v)

    def apply_compliance(self):
        """Envoie pente / punch des pattes dont la phase a change (2 sync write au plus)."""
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        slope, punch = [], []
        for n, ph in self.phases.items():
            if self.lifted:
                sl, pu = g("air_slope"), g("air_punch")
            else:
                sl, pu = (g("stance_slope"), g("stance_punch")) if ph else (g("swing_slope"), g("swing_punch"))
            want = (tuple(sl), tuple(pu))
            if self.applied.get(n) == want:
                continue
            self.applied[n] = want
            for k, sid in enumerate(self.leg_ids[n]):        # k : 0 coxa, 1 femur, 2 tibia
                slope.append(f"{sid} {sl[k] | (sl[k] << 8)}")       # pentes CW et CCW (reg. 28-29)
                punch.append(f"{sid} {pu[k]}")
        if slope:
            self.ls.rpc("SYNCW 28 2 " + " ".join(slope))
            self.ls.rpc("SYNCW 48 2 " + " ".join(punch))

    def on_set_lifted(self, req, resp):
        """Mode "porte en l'air" a la main : pattes souples sans punch, stabilisation en pause."""
        self.lifted, self.low_since = bool(req.data), None
        self.applied.clear()
        self.pub_lifted.publish(Bool(data=self.lifted))
        resp.success = True
        resp.message = "porte en l'air : pattes souples, sans punch" if self.lifted else "au sol : compliance normale"
        self.get_logger().info(resp.message)
        return resp

    def on_figure(self, m):
        if m.data:
            self.figure_until = time.time() + 3.0     # + 3 s apres la fin de la figure

    def update_lifted(self, st):
        if not self.get_parameter("auto_air").value:
            self.pub_lifted.publish(Bool(data=self.lifted))
            return
        self._auto_lifted(st)

    def _auto_lifted(self, st):
        """Robot porte en l'air ? Charge moyenne des FEMURS des pattes au sol (ce sont eux qui
        portent le poids). Seulement apres la rampe de lever."""
        if self.t_ramp is None or time.time() - self.t_ramp < self.ramp_dur:
            return
        if time.time() < self.figure_until:          # figure : pattes levees expres, charges faussees
            self.low_since = None
            return
        loads = []
        for n, ph in self.phases.items():
            if not ph:
                continue
            for sid in self.leg_ids[n][1:2]:
                s = st.get(sid)
                if s and s.get("ok") and s.get("torque"):
                    loads.append(abs(s["load"]) * 100 / 1023)
        if not loads:
            return
        mean = sum(loads) / len(loads)
        now = time.time()
        if not self.lifted:
            if mean < self.get_parameter("air_load").value:
                self.low_since = self.low_since or now
                if now - self.low_since > self.get_parameter("air_time").value:
                    self.lifted = True
                    self.get_logger().info(f"robot porte en l'air (charge {mean:.0f} %) : pattes souples, sans punch")
            else:
                self.low_since = None
        elif mean > self.get_parameter("ground_load").value:
            self.lifted, self.low_since = False, None
            self.get_logger().info(f"robot de nouveau au sol (charge {mean:.0f} %) : compliance normale")
        self.pub_lifted.publish(Bool(data=self.lifted))

    def on_reset(self, req, resp):
        """Remise en route de tous les servos (bloques, en securite, couple coupe, pont gele) :
        RESET ALL, puis rampe douce de la position reelle vers la pose de la marche."""
        try:
            rep = self.ls.rpc("RESET ALL")
            st = self.ls.state()
        except (OSError, RuntimeError, ConnectionError) as e:
            resp.success, resp.message = False, f"leg_server injoignable : {e}"
            return resp
        absent = [sid for sid in self.limits if not st.get(sid, {}).get("ok")]
        self.start_raw = {sid: min(self.limits[sid][1], max(self.limits[sid][0], st[sid]["pos"]))
                          for sid in self.limits if st.get(sid, {}).get("ok")}
        for sid in absent:
            self.start_raw[sid] = self.last_goal.get(sid, 512)
        self.ramp_dur = self.get_parameter("reset_ramp_time").value
        self.t_ramp = None
        self.frozen = None
        self.resets, self.stuck, self.over, self.absent_since = [], {}, {}, {}
        self.applied.clear()
        self.link_lost = None
        msg = f"servos remis en route ({rep}), rampe de {self.ramp_dur:.0f} s" + \
              (f" ; ABSENTS : {absent}" if absent else "")
        self.get_logger().warn(msg)
        resp.success, resp.message = not absent, msg
        return resp

    # ------------------------------------------------------------------ securite
    def check(self, st, startup=False):
        for sid in self.limits if not startup else st:
            s = st.get(sid)
            if not s or not s.get("ok"):
                return f"ID {sid} ne repond plus"
            bits = [t for b, t in SAFETY_BITS.items() if s.get("err", 0) & b]
            if bits or s.get("tlimit") == 0:
                return f"ID {sid} en securite ({', '.join(bits) or 'limite de couple 0'})"
            if s["temp"] >= self.get_parameter("temp_max").value:
                return f"ID {sid} a {s['temp']} degC"
        if startup:
            return None
        now = time.time()
        for sid, s in st.items():
            if not s.get("torque"):
                continue
            load = abs(s["load"]) * 100 / 1023
            if load > self.get_parameter("max_load").value:
                self.over.setdefault(sid, now)
                if now - self.over[sid] > self.get_parameter("load_time").value:
                    return f"ID {sid} a {load:.0f} % de charge depuis plus de " \
                           f"{self.get_parameter('load_time').value} s"
            else:
                self.over.pop(sid, None)
        return None

    def supervise(self, st, goals):
        """Surveillance en marche : repare ce qui peut l'etre (RESET d'un servo en alarme ou
        bloque), gele seulement si c'est grave ou si les reparations s'enchainent."""
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        now = time.time()
        to_reset = {}
        for sid in self.limits:
            s = st.get(sid)
            if not s or not s.get("ok"):                     # USB qui decroche : on attend un peu
                t0 = self.absent_since.setdefault(sid, now)
                if now - t0 > g("absent_time"):
                    return f"ID {sid} ne repond plus depuis {g('absent_time')} s"
                continue
            self.absent_since.pop(sid, None)
            if s["temp"] >= g("temp_max"):
                return f"ID {sid} a {s['temp']} degC"
            bits = [t for b, t in SAFETY_BITS.items() if s.get("err", 0) & b]
            if bits or s.get("tlimit") == 0 or not s.get("torque"):
                to_reset[sid] = ", ".join(bits) or ("limite de couple 0" if s.get("tlimit") == 0 else "couple coupe")
                continue
            if sid in goals and abs(s["pos"] - goals[sid]) > g("stuck_err"):
                t0 = self.stuck.setdefault(sid, now)
                if now - t0 > g("stuck_time"):
                    to_reset[sid] = f"bloque a {s['pos']} pour {goals[sid]}"
            else:
                self.stuck.pop(sid, None)
            load = abs(s["load"]) * 100 / 1023
            if load > g("max_load"):
                self.get_logger().warn(f"ID {sid} a {load:.0f} % de charge", throttle_duration_sec=2.0)
        if to_reset:
            self.resets = [t for t in self.resets if now - t < g("reset_window")] + [now] * len(to_reset)
            if len(self.resets) > g("max_resets"):
                return f"trop de remises en route ({len(self.resets)} en {g('reset_window'):.0f} s) : {to_reset}"
            for sid, why in to_reset.items():
                rep = self.ls.rpc(f"RESET {sid}")
                self.stuck.pop(sid, None)
                self.applied.clear()                          # RESET remet la compliance a reappliquer
                self.get_logger().warn(f"ID {sid} remis en route ({why}) -> {rep}")
        return None

    def freeze(self, why):
        if self.frozen is None:
            self.frozen = why
            self.get_logger().error(f"GEL : {why}. Plus aucune consigne envoyee, couple garde.")

    # ------------------------------------------------------------------ boucle
    def tick(self):
        if self.frozen or self.target is None:
            return
        if self.t_ramp is None:
            self.t_ramp = time.time()
        k = min(1.0, (time.time() - self.t_ramp) / self.ramp_dur)
        goals = {sid: int(round(self.start_raw[sid] + (self.target[sid] - self.start_raw[sid]) * k))
                 for sid in self.target}
        # Seule la pose demandee par la marche peut etre hors butees (la rampe part de
        # positions deja ramenees dans les butees) : on la verifie elle.
        bad = [sid for sid, v in self.target.items() if not self.limits[sid][0] <= v <= self.limits[sid][1]]
        if bad:
            self.freeze(f"consignes hors butees pour {sorted(bad)}")
            return
        try:
            if self.send:
                if self.has_goalspd:
                    dt = 1.0 / self.get_parameter("rate").value
                    gain, vmin = self.get_parameter("speed_gain").value, self.get_parameter("min_speed").value
                    parts = []
                    for i, v in goals.items():
                        deg_s = abs(v - self.last_goal.get(i, v)) * DEG_PER_UNIT / dt
                        spd = int(max(vmin, min(1023, deg_s / 0.666 * gain)))   # AX : 0.111 tr/min = 0.666 deg/s
                        parts.append(f"{i} {v} {spd}")
                    self.ls.rpc("GOALSPD " + " ".join(parts))
                else:
                    self.ls.rpc("GOALSID " + " ".join(f"{i} {v}" for i, v in goals.items()))
                if self.has_syncw:
                    self.apply_compliance()
            self.last_goal = goals
            self.cycle += 1
            if self.cycle % 5 == 0:                     # STATE ~40 ms : 5 Hz suffit
                st = self.ls.state()
                why = self.supervise(st, goals) if self.send else self.check(st)
                self.update_lifted(st)
                if why:
                    self.freeze(why)
                    return
                if self.cycle % 50 == 0:
                    err = max((abs(st[i]["pos"] - goals[i]) for i in goals if st.get(i, {}).get("ok")), default=0)
                    load = max((abs(st[i]["load"]) * 100 / 1023 for i in goals
                                if st.get(i, {}).get("ok") and st[i].get("torque")), default=0)
                    self.get_logger().info(
                        f"{'rampe %.0f %%' % (k * 100) if k < 1 else 'suivi'} | ecart max "
                        f"consigne/position {err} | charge max {load:.0f} % | "
                        f"{min((s['volt'] for s in st.values() if s.get('ok')), default=0) / 10:.1f} V")
            if self.link_lost is not None:
                self.get_logger().info(f"liaison leg_server retablie apres {time.time() - self.link_lost:.1f} s")
                self.link_lost = None
        except (ValueError, KeyError, TypeError) as e:          # reponse incomplete (USB qui decroche...)
            self.get_logger().warn(f"etat des servos incomplet ({type(e).__name__} : {e}) -> on continue",
                                   throttle_duration_sec=1.0)
        except (OSError, RuntimeError, ConnectionError) as e:
            # Liaison qui perd des paquets / U2D2 qui se reconnecte : on reessaie au tick
            # suivant (connexion rouverte) ; gel seulement si ca dure plus de link_timeout.
            now = time.time()
            if self.link_lost is None:
                self.link_lost = now
                self.get_logger().warn(f"liaison leg_server : {e} -> nouvel essai")
            elif now - self.link_lost > self.get_parameter("link_timeout").value:
                self.freeze(f"liaison leg_server perdue depuis {now - self.link_lost:.1f} s ({e})")


def main():
    rclpy.init()
    node = None
    try:
        node = Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError) as e:
        print(f"leg_server_bridge : {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
