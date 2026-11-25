"""Noeud de marche : /cmd_vel -> bouts de patte -> cinematique inverse -> /joint_states.

Entrees :
  /cmd_vel      geometry_msgs/Twist   vitesse (linear.x, linear.y, angular.z)
  /body_height  std_msgs/Float64      hauteur du corps (m, axes des hanches au-dessus du sol)
  parametres    body_height, stance_radius, step_height, cycle_time, max_linear,
                max_angular (modifiables en marche : ros2 param set / Foxglove)
Sorties :
  /joint_states             sensor_msgs/JointState  18 articulations (robot_state_publisher)
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
from std_msgs.msg import Float64
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
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.legs = load_legs(g("legs_yaml") or None)
        self.gait = TripodGait(self.legs, stance_radius=g("stance_radius"),
                               body_height=self._clamp_h(g("body_height")),
                               step_height=g("step_height"), cycle_time=g("cycle_time"))
        self.q = {lg.n: lg.ik(*self.gait.feet[lg.n]) or (0.0, 0.0, 0.0) for lg in self.legs}
        self.x = self.y = self.yaw = 0.0
        self.last_cmd = self.get_clock().now()

        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.pub_feet = self.create_publisher(PoseArray, "foot_targets", 10)
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_subscription(Float64, "body_height", self.on_height, 10)
        self.add_on_set_parameters_callback(self.on_params)
        self.dt = 1.0 / g("rate")
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f"Marche en trepied prete : hauteur {self.gait.height * 1000:.0f} mm, "
            f"rayon {self.gait.stance_radius * 1000:.0f} mm, cycle {self.gait.cycle_time:.2f} s")

    # ------------------------------------------------------------------ entrees
    def _clamp_h(self, h):
        return max(self.get_parameter("min_height").value, min(self.get_parameter("max_height").value, h))

    def on_cmd(self, m):
        vl, va = self.get_parameter("max_linear").value, self.get_parameter("max_angular").value
        clip = lambda v, lim: max(-lim, min(lim, v))  # noqa: E731
        vx, vy = m.linear.x, m.linear.y
        n = math.hypot(vx, vy)
        if n > vl:
            vx, vy = vx * vl / n, vy * vl / n
        self.gait.set_cmd(vx, vy, clip(m.angular.z, va))
        self.last_cmd = self.get_clock().now()

    def on_height(self, m):
        self.gait.set_height(self._clamp_h(m.data))

    def on_params(self, params):
        for prm in params:
            if prm.name == "body_height":
                self.gait.set_height(self._clamp_h(prm.value))
            elif prm.name == "stance_radius":
                self.gait.stance_radius = prm.value
            elif prm.name == "step_height":
                self.gait.step_height = prm.value
            elif prm.name == "cycle_time" and prm.value > 0.2:
                self.gait.cycle_time = prm.value
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ boucle
    def tick(self):
        now = self.get_clock().now()
        if (now - self.last_cmd).nanoseconds * 1e-9 > self.get_parameter("cmd_timeout").value:
            self.gait.set_cmd(0.0, 0.0, 0.0)          # plus de consigne : on s'arrete
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
