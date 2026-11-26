"""IMU de l'hexapode (a lancer SUR LA PI, carte Neuro IMU en I2C).

    ros2 launch hexapod_imu imu.launch.py
    ros2 launch hexapod_imu imu.launch.py rate:=200
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rate", default_value="100.0"),
        DeclareLaunchArgument("calib_file", default_value=""),
        Node(package="hexapod_imu", executable="imu_node", name="imu_node", output="screen",
             parameters=[{"rate": LaunchConfiguration("rate"),
                          "calib_file": LaunchConfiguration("calib_file")}]),
    ])
