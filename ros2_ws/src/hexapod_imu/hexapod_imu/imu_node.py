"""Noeud IMU (sur la Pi) : LSM6DSV16X en I2C -> ROS.

Publie (repere imu_link = axes du robot : x avant, y gauche, z haut) :
  /imu/data_raw     sensor_msgs/Imu          acceleration + vitesse angulaire calibrees
  /imu/data         sensor_msgs/Imu          + orientation (filtre de Mahony 6 axes)
  /imu/temperature  sensor_msgs/Temperature  1 Hz
Le lacet (cap) n'a pas de reference (pas de magnetometre) : il derive lentement.

    ros2 launch hexapod_imu imu.launch.py
"""
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature

from .calibration import Calibration, MahonyFilter
from .lsm6dsv16x import LSM6DSV16X


def diag(a, b, c):
    return [float(a), 0.0, 0.0, 0.0, float(b), 0.0, 0.0, 0.0, float(c)]


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        p = self.declare_parameter
        p("bus", 1)
        p("address", 0x6A)
        p("rate", 100.0)
        p("frame_id", "imu_link")
        p("calib_file", "")
        p("odr", 240)
        p("acc_fs", 4)
        p("gyr_fs", 1000)
        p("filter_kp", 1.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        path = g("calib_file") or os.path.join(get_package_share_directory("hexapod_imu"), "config", "imu_calib.yaml")
        self.cal = Calibration.load(path) if os.path.exists(path) else Calibration()
        if not os.path.exists(path):
            self.get_logger().warn(f"pas de calibration ({path}) : ros2 run hexapod_imu calibrate_imu")
        self.imu = LSM6DSV16X(bus=g("bus"), address=g("address"), odr=g("odr"),
                              acc_fs=g("acc_fs"), gyr_fs=g("gyr_fs"))
        self.filter = MahonyFilter(kp=g("filter_kp"))
        self.frame = g("frame_id")
        self.pub_raw = self.create_publisher(Imu, "imu/data_raw", 20)
        self.pub = self.create_publisher(Imu, "imu/data", 20)
        self.pub_t = self.create_publisher(Temperature, "imu/temperature", 5)
        self.dt = 1.0 / g("rate")
        self.last = None
        self.errors = 0
        self.create_timer(self.dt, self.tick)
        self.create_timer(1.0, self.tick_temp)
        self.get_logger().info(
            f"IMU LSM6DSV16X (0x{g('address'):02x}) a {g('rate'):.0f} Hz, {self.imu.temperature():.1f} degC, "
            f"calibration {path}")

    def tick(self):
        try:
            acc, gyr = self.cal.correct(*self.imu.read())
        except OSError as e:
            self.errors += 1
            self.get_logger().warn(f"lecture I2C : {e}", throttle_duration_sec=2.0)
            return
        now = self.get_clock().now()
        dt = self.dt if self.last is None else max(1e-4, (now - self.last).nanoseconds * 1e-9)
        self.last = now
        q = self.filter.update(acc, gyr, dt)

        m = Imu()
        m.header.stamp, m.header.frame_id = now.to_msg(), self.frame
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = acc
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = gyr
        m.linear_acceleration_covariance = diag(0.03 ** 2, 0.03 ** 2, 0.03 ** 2)
        m.angular_velocity_covariance = diag(0.012 ** 2, 0.012 ** 2, 0.012 ** 2)
        m.orientation_covariance[0] = -1.0                  # data_raw : pas d'orientation
        self.pub_raw.publish(m)
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = q
        m.orientation_covariance = diag(math.radians(1) ** 2, math.radians(1) ** 2,
                                        math.radians(10) ** 2)      # lacet sans reference
        self.pub.publish(m)

    def tick_temp(self):
        try:
            t = Temperature()
            t.header.stamp, t.header.frame_id = self.get_clock().now().to_msg(), self.frame
            t.temperature, t.variance = self.imu.temperature(), 0.25
            self.pub_t.publish(t)
        except OSError:
            pass


def main():
    rclpy.init()
    node = None
    try:
        node = ImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError) as e:
        print(f"imu_node : {e}")
    finally:
        if node:
            node.imu.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
