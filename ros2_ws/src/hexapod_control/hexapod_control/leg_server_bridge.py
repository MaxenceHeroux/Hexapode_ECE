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

from .kinematics import load_legs

DEG_PER_UNIT = 300.0 / 1024.0
SAFETY_BITS = {0x01: "tension", 0x04: "surchauffe", 0x20: "surcharge", 0x08: "hors plage"}


class LegServer:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.f = self.sock.makefile("r", encoding="utf-8")

    def rpc(self, line):
        self.sock.sendall((line + "\n").encode())
        rep = self.f.readline()
        if not rep:
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
        self.start_raw = {sid: st[sid]["pos"] for sid in self.limits}
        self.target = None
        self.frozen = None
        self.t_ramp = None
        self.over = {}
        self.cycle = 0
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
        k = min(1.0, (time.time() - self.t_ramp) / self.get_parameter("ramp_time").value)
        goals = {sid: int(round(self.start_raw[sid] + (self.target[sid] - self.start_raw[sid]) * k))
                 for sid in self.target}
        bad = [sid for sid, v in goals.items() if not self.limits[sid][0] <= v <= self.limits[sid][1]]
        if bad:
            self.freeze(f"consignes hors butees pour {sorted(bad)}")
            return
        try:
            if self.send:
                self.ls.rpc("GOALSID " + " ".join(f"{i} {v}" for i, v in goals.items()))
            self.cycle += 1
            if self.cycle % 5 == 0:                     # STATE ~40 ms : 5 Hz suffit
                st = self.ls.state()
                why = self.check(st)
                if why:
                    self.freeze(why)
                    return
                if self.cycle % 50 == 0:
                    err = max(abs(st[i]["pos"] - goals[i]) for i in goals)
                    load = max(abs(st[i]["load"]) * 100 / 1023 for i in goals if st[i].get("torque")) \
                        if any(st[i].get("torque") for i in goals) else 0
                    self.get_logger().info(
                        f"{'rampe %.0f %%' % (k * 100) if k < 1 else 'suivi'} | ecart max "
                        f"consigne/position {err} | charge max {load:.0f} % | "
                        f"{min(s['volt'] for s in st.values() if s.get('ok')) / 10:.1f} V")
        except (OSError, RuntimeError, ConnectionError) as e:
            self.freeze(f"liaison leg_server : {e}")


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
