"""Noeud de marche : /cmd_vel -> bouts de patte -> cinematique inverse -> /joint_states.

Entrees :
  /cmd_vel      geometry_msgs/Twist   vitesse (linear.x, linear.y, angular.z)
  /body_height  std_msgs/Float64      hauteur du corps (m, axes des hanches au-dessus du sol)
  parametres    body_height, stance_radius, step_height, cycle_time, max_linear,
                max_angular (modifiables en marche : ros2 param set / Foxglove)
Sorties :
  /joint_states             sensor_msgs/JointState  18 articulations (robot_state_publisher)
  /leg_phases               std_msgs/Int8MultiArray pattes 1..6 : 1 au sol (appui), 0 en l'air
  /foot_targets             geometry_msgs/PoseArray bouts de patte vises, repere odom
  /odom                     nav_msgs/Odometry       odometrie (integration de la consigne)
  TF odom -> base_footprint (deplacement), base_footprint -> base_link (hauteur),
     odom -> leg_<n>_foot_target (bouts de patte vises, dans le repere general)
"""
import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64, Int8MultiArray, String
from tf2_ros import TransformBroadcaster

from .actions import FIGURES, ActionPlayer
from .gait import TripodGait
from .stabilizer import Stabilizer
from .kinematics import load_legs


def _quat_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class GaitNode(Node):
    def __init__(self):
        super().__init__("gait_node")
        p = self.declare_parameter
        p("legs_yaml", "")
        p("rate", 50.0)
        p("body_height", 0.09)
        p("min_height", 0.05)
        p("max_height", 0.13)
        p("stance_radius", 0.17)
        p("step_height", 0.03)
        p("cycle_time", 1.0)
        p("max_linear", 0.10)
        p("max_angular", 0.6)
        p("cmd_timeout", 0.5)
        p("accel", 0.6)            # m/s2 : rampe de vitesse (plus = repond plus vite au clavier)
        p("ang_accel", 2.5)        # rad/s2
        p("height_gain", 0.04)     # linear.z (t / b) -> vitesse de la hauteur : 0.10 m/s -> 4 mm/s
        p("height_rate", 0.01)     # m/s : rampe de hauteur (douce)
        p("step_reach", 0.045)
        p("gait", "tripod")        # tripod (3 pieds au sol) | tetrapod (4) | wave (5) : plus = moins de glissement
        p("duty", 0.0)             # fraction du cycle au sol (0 = defaut de l'allure, avec recouvrement)
        # Stabilisation par l'IMU (/imu/data, hexapod_imu sur la Pi) : voir stabilizer.py
        p("heading_hold", True)    # avancer droit : garde le cap quand on ne tourne pas
        p("heading_kp", 1.2)
        p("heading_ki", 0.2)
        p("heading_max", 0.3)      # rad/s de correction max
        p("turn_assist", True)     # rotation asservie au gyroscope (compense le patinage)
        p("turn_kp", 0.5)
        p("turn_ki", 1.5)
        p("turn_boost", 1.5)       # commande jusqu'a (1 + 1.5) x la rotation demandee
        p("level", True)           # compensation active : corps horizontal si le sol penche
        p("level_kp", 0.3)
        p("level_ki", 3.0)
        p("level_max_deg", 10.0)
        p("sag_comp", 0.003)       # m : pattes au sol plus basses pendant qu'un trepied est en l'air
        p("fall_deg", 35.0)        # inclinaison -> arret de la marche
        # Derive laterale : le robot glisse de cote en avancant (coxas pas parfaitement
        # centrees, pieds qui glissent). Vitesse laterale ajoutee = drift_comp x avance :
        # s'il part de D cm a GAUCHE pour 1 m d'avance, mettre drift_comp = -D/100.
        p("drift_comp", 0.0)     # demi-pas max (m) : pas jusqu'a 9 cm ; plus grand = pieds plus ecartes     # linear.z (t / b) -> vitesse de la hauteur : 0.05 m/s -> 7.5 mm/s
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.legs = load_legs(g("legs_yaml") or None)
        self.gait = TripodGait(self.legs, stance_radius=g("stance_radius"), body_height=g("body_height"),
                               step_height=g("step_height"), cycle_time=g("cycle_time"),
                               max_linear=g("max_linear"), max_angular=g("max_angular"), reach=g("step_reach"),
                               height_rate=g("height_rate"), gait=g("gait"), duty=g("duty"),
                               accel=g("accel"), ang_accel=g("ang_accel"),
                               height_limits=(g("min_height"), g("max_height")))
        self.q = {lg.n: lg.ik(*self.gait.feet[lg.n]) or (0.0, 0.0, 0.0) for lg in self.legs}
        self.x = self.y = self.yaw = 0.0
        self.last_cmd = self.get_clock().now()
        self.vz = 0.0
        self.last_speed = None
        self.user_cmd = (0.0, 0.0, 0.0)
        self.stab = Stabilizer(heading_kp=g("heading_kp"), heading_ki=g("heading_ki"), heading_max=g("heading_max"),
                               level_kp=g("level_kp"), level_ki=g("level_ki"),
                               level_max=math.radians(g("level_max_deg")), sag_comp=g("sag_comp"),
                               fall=math.radians(g("fall_deg")))
        self.stab.enable_heading, self.stab.enable_level = g("heading_hold"), g("level")
        self.stab.enable_turn = g("turn_assist")
        self.stab.turn_kp, self.stab.turn_ki, self.stab.turn_boost = g("turn_kp"), g("turn_ki"), g("turn_boost")
        self.yaw0 = None
        self.imu_was_ok = False

        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.pub_feet = self.create_publisher(PoseArray, "foot_targets", 10)
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_phase = self.create_publisher(Int8MultiArray, "leg_phases", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_subscription(Float64, "body_height", self.on_height, 10)
        self.create_subscription(Imu, "imu/data", self.on_imu, 20)
        self.lifted = False
        self.create_subscription(Bool, "robot_lifted", self.on_lifted, 10)
        # Figures (coucou, danse, cabrer, coeur) : /action std_msgs/String ("stop" pour arreter)
        self.player = ActionPlayer(self.legs)
        self.pending_action = None
        self.lifted_legs = set()
        self.create_subscription(String, "action", self.on_action, 10)
        self.pub_busy = self.create_publisher(Bool, "figure_active", 10)
        self.add_on_set_parameters_callback(self.on_params)
        self.dt = 1.0 / g("rate")
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f"Marche en trepied prete : hauteur {self.gait.height * 1000:.0f} mm "
            f"(possible {self.gait.h_min * 1000:.0f}-{self.gait.h_max * 1000:.0f}), rayon des pieds "
            f"{self.gait.radius * 1000:.0f} mm (voulu {self.gait.stance_radius * 1000:.0f}, "
            f"allure {self.gait.pattern}, au sol {self.gait.duty * 100:.0f} %, "
            f"vitesse max reelle {self.max_speed()[0]:.2f} m/s / {self.max_speed()[1]:.2f} rad/s, "
            f"adapte a la hauteur), cycle {self.gait.cycle_time:.2f} s")

    # ------------------------------------------------------------------ entrees
    def _set_height(self, h):
        got = self.gait.set_height(h)
        msg = f"hauteur -> {got * 1000:.0f} mm, rayon des pieds {self.gait.radius * 1000:.0f} mm"
        if abs(got - h) > 1e-4:
            msg += f" (demande {h * 1000:.0f} mm hors de {self.gait.h_min * 1000:.0f}-{self.gait.h_max * 1000:.0f})"
        self.get_logger().info(msg)

    def max_speed(self):
        """Vitesses max atteignables (avance, rotation) : limitees par la longueur de pas
        (step_reach) et la cadence la plus rapide (cycle_time)."""
        g = self.gait
        vx = min(g.max_linear, g.max_linear * g.set_cmd(g.max_linear, 0.0, 0.0))
        wz = min(g.max_angular, g.max_angular * g.set_cmd(0.0, 0.0, g.max_angular))
        g.set_cmd(0.0, 0.0, 0.0)
        return vx, wz

    def on_cmd(self, m):
        vl, va = self.get_parameter("max_linear").value, self.get_parameter("max_angular").value
        clip = lambda v, lim: max(-lim, min(lim, v))  # noqa: E731
        vx, vy = m.linear.x, m.linear.y
        n = math.hypot(vx, vy)
        if n > vl:
            vx, vy = vx * vl / n, vy * vl / n
        if self.player.active and (abs(m.linear.x) + abs(m.linear.y) + abs(m.angular.z)) > 1e-3:
            self.player.stop()                     # on veut marcher : la figure s'arrete d'abord
            self.pending_action = None
        if self.player.active:
            return
        k = self.gait.set_cmd(vx, vy, clip(m.angular.z, va))
        self.user_cmd = self.gait.cmd
        self.vz = m.linear.z                   # t / b du clavier : monter / descendre le corps
        self.last_cmd = self.get_clock().now()
        speed = (round(math.hypot(m.linear.x, m.linear.y), 3), round(abs(m.angular.z), 3))
        if speed != self.last_speed and any(speed):
            self.last_speed = speed
            lim = " (plafonnee)" if math.hypot(m.linear.x, m.linear.y) > vl or abs(m.angular.z) > va or k < 1 else ""
            self.get_logger().info(f"consigne : {math.hypot(vx, vy) * k:.3f} m/s, {abs(clip(m.angular.z, va)) * k:.2f} "
                                   f"rad/s{lim} (max {vl:.2f} m/s, {va:.2f} rad/s)", throttle_duration_sec=1.0)

    def on_lifted(self, m):
        """Robot porte (pont leg_server) : pas de mise a niveau ni de tenue de cap, sinon les
        pattes bougent toutes seules quand on incline le robot a la main."""
        if m.data != self.lifted:
            self.lifted = m.data
            self.stab.enable_level = self.get_parameter("level").value and not m.data
            self.stab.enable_heading = self.get_parameter("heading_hold").value and not m.data
            self.stab.enable_turn = self.get_parameter("turn_assist").value and not m.data
            self.get_logger().info("porte en l'air : stabilisation en pause" if m.data else "au sol : stabilisation reprise")

    def on_action(self, m):
        name = m.data.strip().lower()
        if name in ("stop", ""):
            self.pending_action = None
            if self.player.active:
                self.player.stop()
                self.get_logger().info("figure : arret (retour debout)")
            return
        if name not in FIGURES:
            self.get_logger().warn(f"figure inconnue : {name} ({', '.join(FIGURES)}, stop)")
            return
        if self.player.active:
            self.get_logger().info(f"figure {self.player.name} en cours : {name} ignoree")
            return
        self.pending_action = name
        self.get_logger().info(f"figure : {name}" + (" (apres l'arret de la marche)" if self.gait.walking else ""))

    def on_imu(self, m):
        o = m.orientation
        self.stab.imu(o.w, o.x, o.y, o.z, m.angular_velocity.z)

    def on_height(self, m):
        self._set_height(m.data)

    def on_params(self, params):
        for prm in params:
            if prm.name == "body_height":
                self._set_height(prm.value)
            elif prm.name == "stance_radius":
                r = self.gait.set_radius(prm.value)
                self.get_logger().info(f"rayon des pieds -> {r * 1000:.0f} mm (voulu {prm.value * 1000:.0f})")
            elif prm.name == "step_height":
                self.gait.step_height = prm.value
            elif prm.name == "cycle_time" and prm.value > 0.2:
                self.gait.cycle_time = prm.value
            elif prm.name == "heading_hold":
                self.stab.enable_heading = bool(prm.value)
            elif prm.name == "level":
                self.stab.enable_level = bool(prm.value)
            elif prm.name == "sag_comp":
                self.stab.sag_comp = prm.value
            elif prm.name in ("gait", "duty"):
                name = prm.value if prm.name == "gait" else self.get_parameter("gait").value
                duty = prm.value if prm.name == "duty" else self.get_parameter("duty").value
                try:
                    d = self.gait.set_pattern(name, duty)
                except ValueError as e:
                    return SetParametersResult(successful=False, reason=str(e))
                self.get_logger().info(f"allure -> {name} (au sol {d * 100:.0f} % du cycle)"
                                       + (" : appliquee a l'arret" if self.gait.walking else ""))
            elif prm.name == "turn_assist":
                self.stab.enable_turn = bool(prm.value)
            elif prm.name in ("heading_kp", "heading_ki", "heading_max", "level_kp", "level_ki",
                              "turn_kp", "turn_ki", "turn_boost"):
                setattr(self.stab, prm.name, prm.value)
            elif prm.name == "level_max_deg":
                self.stab.level_max = math.radians(prm.value)
            elif prm.name == "height_rate" and prm.value > 0:
                self.gait.height_rate = prm.value
            elif prm.name == "accel" and prm.value > 0:
                self.gait.accel = prm.value
            elif prm.name == "ang_accel" and prm.value > 0:
                self.gait.ang_accel = prm.value
            elif prm.name == "max_linear" and prm.value > 0:
                self.gait.max_linear = prm.value
            elif prm.name == "max_angular" and prm.value > 0:
                self.gait.max_angular = prm.value
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ boucle
    def tick(self):
        now = self.get_clock().now()
        if (now - self.last_cmd).nanoseconds * 1e-9 > self.get_parameter("cmd_timeout").value:
            self.gait.set_cmd(0.0, 0.0, 0.0)          # plus de consigne : on s'arrete
            self.user_cmd = (0.0, 0.0, 0.0)
            self.vz = 0.0
        # IMU : securite chute, tenue de cap (avancer droit)
        if self.stab.imu_ok != self.imu_was_ok:
            self.imu_was_ok = self.stab.imu_ok
            self.get_logger().info("IMU recue : tenue de cap et mise a niveau actives" if self.imu_was_ok
                                   else "plus d'IMU : marche sans stabilisation")
        if self.stab.fallen:
            self.gait.set_cmd(0.0, 0.0, 0.0)
            self.user_cmd = (0.0, 0.0, 0.0)
            self.get_logger().error("inclinaison > fall_deg : marche arretee", throttle_duration_sec=2.0)
        else:
            corr = self.stab.heading_correction(self.user_cmd, self.gait.walking, self.dt)
            vx_u, vy_u, wz_u = self.user_cmd
            # derive laterale (glissement en crabe) : invisible pour l'IMU, compensee par
            # une vitesse laterale proportionnelle a l'avance
            vy_u += self.get_parameter("drift_comp").value * vx_u
            wz_c = self.stab.turn_command(wz_u, self.gait.walking, self.dt, self.get_parameter("max_angular").value)
            self.gait.set_cmd(vx_u, vy_u, wz_c + corr)     # (reduite si le pas depasse 2 x reach)
            if abs(wz_u) > 1e-3 and self.stab.imu_ok:
                self.get_logger().info(f"rotation : demandee {wz_u:+.2f}, mesuree {self.stab.gyro_z:+.2f}, "
                                       f"commandee {wz_c:+.2f} rad/s", throttle_duration_sec=1.0)
            if self.stab.heading_ref is not None:
                e = math.degrees(math.atan2(math.sin(self.stab.heading_ref - self.stab.rpy[2]),
                                            math.cos(self.stab.heading_ref - self.stab.rpy[2])))
                self.get_logger().info(f"tout droit : ecart de cap {e:+.1f} deg, correction {corr:+.2f} rad/s",
                                       throttle_duration_sec=2.0)
        if abs(self.vz) > 1e-4:                      # t / b : la hauteur suit linear.z (m/s)
            before = self.gait.height_target
            got = self.gait.set_height(before + self.vz * self.get_parameter("height_gain").value * self.dt)
            if abs(got - before) < 1e-9:
                self.get_logger().info(f"hauteur en butee : {got * 1000:.0f} mm "
                                       f"(possible {self.gait.h_min * 1000:.0f}-{self.gait.h_max * 1000:.0f})",
                                       throttle_duration_sec=2.0)
            else:
                self.get_logger().info(f"hauteur -> {got * 1000:.0f} mm, rayon des pieds "
                                       f"{self.gait.radius * 1000:.0f} mm", throttle_duration_sec=0.5)
        vx, vy, wz = self.gait.step(self.dt)
        # demarrer une figure quand le robot est arrete, pieds au sol
        if self.pending_action and not self.gait.walking and not self.player.active:
            self.player.start(self.pending_action, self.gait.feet)
            self.pending_action = None

        # odometrie (integration de la vitesse appliquee)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (vx * c - vy * s) * self.dt
        self.y += (vx * s + vy * c) * self.dt
        self.yaw += wz * self.dt
        if self.stab.imu_ok:                         # cap mesure par l'IMU (plus juste que la consigne)
            if self.yaw0 is None:
                self.yaw0 = self.stab.rpy[2] - self.yaw
            self.yaw = self.stab.rpy[2] - self.yaw0

        # stabilisation : corps incline pour rester horizontal, anti-affaissement
        tilt = self.stab.update(self.dt)
        if self.player.active:                       # figure : pas de mise a niveau (le corps bouge expres)
            self.stab.tilt = [0.0, 0.0]
            self.stab.level_int = [0.0, 0.0]
            fig = self.player.step(self.dt)
            if fig is None:
                self.get_logger().info("figure terminee")
                self.lifted_legs = set()
            else:
                feet_fig, self.lifted_legs = fig
                bad = []
                for lg in self.legs:
                    q = lg.ik(*feet_fig[lg.n])
                    if q is None or not lg.in_limits(q):
                        bad.append(lg.n)
                        continue
                    self.q[lg.n] = q
                if bad:
                    self.get_logger().warn(f"figure : pattes {bad} hors d'atteinte, angles figes",
                                           throttle_duration_sec=1.0)
                self.publish_state(now, vx, vy, wz)
                return
        stance = {lg.n: (not self.gait.walking) or self.gait.in_stance(lg.n) for lg in self.legs}
        any_swing = not all(stance.values())
        feet_cmd = None
        for k in (1.0, 0.5, 0.0):                    # si l'inclinaison sort des butees : moins
            cand = self.stab.transform_feet(self.gait.feet, stance, any_swing, (tilt[0] * k, tilt[1] * k))
            if all((q := lg.ik(*cand[lg.n])) is not None and lg.in_limits(q) for lg in self.legs):
                feet_cmd = cand
                break
        if feet_cmd is None:
            feet_cmd = self.gait.feet

        # cinematique inverse
        bad = []
        for lg in self.legs:
            q = lg.ik(*feet_cmd[lg.n])
            if q is None or not lg.in_limits(q):
                bad.append(lg.n)
                continue
            self.q[lg.n] = q
        if bad:
            self.get_logger().warn(f"pattes {bad} hors d'atteinte / hors butees : angles figes",
                                   throttle_duration_sec=1.0)

        self.publish_state(now, vx, vy, wz)

    def publish_state(self, now, vx, vy, wz):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        stamp = now.to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.name = [nm for lg in self.legs for nm in lg.joint_names]
        js.position = [float(v) for lg in self.legs for v in self.q[lg.n]]
        self.pub_js.publish(js)
        ph = Int8MultiArray()           # pattes 1..6 : 1 = au sol (porte le robot), 0 = en l'air
        ph.data = [0 if lg.n in self.lifted_legs else
                   (1 if (not self.gait.walking or self.gait.in_stance(lg.n)) else 0) for lg in self.legs]
        self.pub_phase.publish(ph)
        self.pub_busy.publish(Bool(data=self.player.active))

        h = self.gait.height
        tfs = [self._tf(stamp, "odom", "base_footprint", self.x, self.y, 0.0, self.yaw),
               self._tf(stamp, "base_footprint", "base_link", 0.0, 0.0, h, 0.0)]
        feet = PoseArray()
        feet.header.stamp, feet.header.frame_id = stamp, "odom"
        for lg in self.legs:
            fx, fy, fz = self.gait.feet[lg.n]
            wx, wy, wz_ = self.x + fx * c - fy * s, self.y + fx * s + fy * c, fz + h
            tfs.append(self._tf(stamp, "odom", f"leg_{lg.n}_foot_target", wx, wy, wz_, 0.0))
            ps = Pose()
            ps.position.x, ps.position.y, ps.position.z = wx, wy, wz_
            ps.orientation.w = 1.0
            feet.poses.append(ps)
        self.tf.sendTransform(tfs)
        self.pub_feet.publish(feet)

        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, "odom", "base_footprint"
        od.pose.pose.position.x, od.pose.pose.position.y = self.x, self.y
        qx, qy, qz, qw = _quat_yaw(self.yaw)
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.angular.z = vx, vy, wz
        self.pub_odom.publish(od)

    @staticmethod
    def _tf(stamp, parent, child, x, y, z, yaw):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        qx, qy, qz, qw = _quat_yaw(yaw)
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        return t


def main():
    rclpy.init()
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
