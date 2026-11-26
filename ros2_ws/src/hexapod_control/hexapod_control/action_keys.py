"""Clavier des actions de l'hexapode (fenetre "Actions hexapode", a cote du Teleop).

    ros2 run hexapod_control action_keys

  1 coucou   2 danse   3 cabrer   4 coeur   0 / espace : arreter la figure
  r          remise en route des moteurs (servos bloques / en securite / pont gele)
  p          mode porte en l'air (pattes souples) / retour au sol
  q          quitter
Publie /action (std_msgs/String) et appelle le service /reset_servos du pont.
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

KEYS = {"1": "coucou", "2": "danse", "3": "cabrer", "4": "coeur", "0": "stop", " ": "stop"}
MENU = """
=== Actions de l'hexapode ===
  1 coucou   2 danse   3 cabrer   4 coeur      0 / espace : arreter
  r : REMISE EN ROUTE DES MOTEURS (quand ca bloque)
  p : mode PORTE EN L'AIR (pattes souples, fluides) / retour AU SOL
  q : quitter
(cette fenetre doit avoir le focus ; marcher : fenetre Teleop)
"""


class ActionKeys(Node):
    def __init__(self):
        super().__init__("action_keys")
        self.pub = self.create_publisher(String, "action", 10)
        self.reset = self.create_client(Trigger, "reset_servos")
        self.lift = self.create_client(SetBool, "set_lifted")
        self.lifted = False

    def send(self, name):
        self.pub.publish(String(data=name))
        print(f"\r-> {name}                    ")

    def toggle_lifted(self):
        if not self.lift.wait_for_service(timeout_sec=1.0):
            print("\r-> pont vers le robot absent (simulation ?)")
            return
        fut = self.lift.call_async(SetBool.Request(data=not self.lifted))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        if r and r.success:
            self.lifted = not self.lifted
        print(f"\r-> {r.message if r else 'pas de reponse'}")

    def do_reset(self):
        if not self.reset.wait_for_service(timeout_sec=1.0):
            print("\r-> pont vers le robot absent (simulation ?) : pas de remise en route")
            return
        fut = self.reset.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        r = fut.result()
        print(f"\r-> remise en route : {r.message if r else 'pas de reponse'}")


def main():
    rclpy.init()
    node = ActionKeys()
    old = termios.tcgetattr(sys.stdin) if sys.stdin.isatty() else None
    print(MENU)
    try:
        if old:
            tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if select.select([sys.stdin], [], [], 0.1)[0]:
                k = sys.stdin.read(1)
                if k == "q":
                    break
                if k in ("r", "R"):
                    node.do_reset()
                elif k in ("p", "P"):
                    node.toggle_lifted()
                elif k in KEYS:
                    node.send(KEYS[k])
    except KeyboardInterrupt:
        pass
    finally:
        if old:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
