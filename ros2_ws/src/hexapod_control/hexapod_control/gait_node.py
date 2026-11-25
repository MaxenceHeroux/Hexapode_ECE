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
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Int8MultiArray
from tf2_ros import TransformBroadcaster

from .gait import TripodGait
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
        p("height_gain", 0.04)     # linear.z (t / b) -> vitesse de la hauteur : 0.10 m/s -> 4 mm/s
        p("height_rate", 0.01)     # m/s : rampe de hauteur (douce)
        p("step_reach", 0.045)     # demi-pas max (m) : pas jusqu'a 9 cm ; plus grand = pieds plus ecartes     # linear.z (t / b) -> vitesse de la hauteur : 0.05 m/s -> 7.5 mm/s
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.legs = load_legs(g("legs_yaml") or None)
        self.gait = TripodGait(self.legs, stance_radius=g("stance_radius"), body_height=g("body_height"),
                               step_height=g("step_height"), cycle_time=g("cycle_time"),
                               max_linear=g("max_linear"), max_angular=g("max_angular"), reach=g("step_reach"),
                               height_rate=g("height_rate"),
                               height_limits=(g("min_height"), g("max_height")))
        self.q = {lg.n: lg.ik(*self.gait.feet[lg.n]) or (0.0, 0.0, 0.0) for lg in self.legs}
        self.x = self.y = self.yaw = 0.0
        self.last_cmd = self.get_clock().now()
        self.vz = 0.0
        self.last_speed = None

        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.pub_feet = self.create_publisher(PoseArray, "foot_targets", 10)
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_phase = self.create_publisher(Int8MultiArray, "leg_phases", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_subscription(Float64, "body_height", self.on_height, 10)
        self.add_on_set_parameters_callback(self.on_params)
        self.dt = 1.0 / g("rate")
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f"Marche en trepied prete : hauteur {self.gait.height * 1000:.0f} mm "
            f"(possible {self.gait.h_min * 1000:.0f}-{self.gait.h_max * 1000:.0f}), rayon des pieds "
            f"{self.gait.radius * 1000:.0f} mm (voulu {self.gait.stance_radius * 1000:.0f}, "
            f"adapte a la hauteur), cycle {self.gait.cycle_time:.2f} s")

    # ------------------------------------------------------------------ entrees
    def _set_height(self, h):
        got = self.gait.set_height(h)
        msg = f"hauteur -> {got * 1000:.0f} mm, rayon des pieds {self.gait.radius * 1000:.0f} mm"
        if abs(got - h) > 1e-4:
            msg += f" (demande {h * 1000:.0f} mm hors de {self.gait.h_min * 1000:.0f}-{self.gait.h_max * 1000:.0f})"
        self.get_logger().info(msg)

    def on_cmd(self, m):
        vl, va = self.get_parameter("max_linear").value, self.get_parameter("max_angular").value
        clip = lambda v, lim: max(-lim, min(lim, v))  # noqa: E731
        vx, vy = m.linear.x, m.linear.y
        n = math.hypot(vx, vy)
        if n > vl:
            vx, vy = vx * vl / n, vy * vl / n
        k = self.gait.set_cmd(vx, vy, clip(m.angular.z, va))
        self.vz = m.linear.z                   # t / b du clavier : monter / descendre le corps
        self.last_cmd = self.get_clock().now()
        speed = (round(math.hypot(m.linear.x, m.linear.y), 3), round(abs(m.angular.z), 3))
        if speed != self.last_speed and any(speed):
            self.last_speed = speed
            lim = " (plafonnee)" if math.hypot(m.linear.x, m.linear.y) > vl or abs(m.angular.z) > va or k < 1 else ""
            self.get_logger().info(f"consigne : {math.hypot(vx, vy) * k:.3f} m/s, {abs(clip(m.angular.z, va)) * k:.2f} "
                                   f"rad/s{lim} (max {vl:.2f} m/s, {va:.2f} rad/s)")

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
            elif prm.name == "height_rate" and prm.value > 0:
                self.gait.height_rate = prm.value
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
            self.vz = 0.0
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

        # odometrie (integration de la vitesse appliquee)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (vx * c - vy * s) * self.dt
        self.y += (vx * s + vy * c) * self.dt
        self.yaw += wz * self.dt

        # cinematique inverse
        bad = []
        for lg in self.legs:
            q = lg.ik(*self.gait.feet[lg.n])
            if q is None or not lg.in_limits(q):
                bad.append(lg.n)
                continue
            self.q[lg.n] = q
        if bad:
            self.get_logger().warn(f"pattes {bad} hors d'atteinte / hors butees : angles figes",
                                   throttle_duration_sec=1.0)

        stamp = now.to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.name = [nm for lg in self.legs for nm in lg.joint_names]
        js.position = [float(v) for lg in self.legs for v in self.q[lg.n]]
        self.pub_js.publish(js)
        ph = Int8MultiArray()           # pattes 1..6 : 1 = au sol (porte le robot), 0 = en l'air
        ph.data = [1 if (not self.gait.walking or self.gait.in_stance(lg.n)) else 0 for lg in self.legs]
        self.pub_phase.publish(ph)

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
