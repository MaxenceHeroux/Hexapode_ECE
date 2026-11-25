"""Affiche l'hexapode dans RViz avec des curseurs pour les 18 articulations.

    ros2 launch hexapod_description display.launch.py
    ros2 launch hexapod_description display.launch.py gui:=false use_imu:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("hexapod_description")
    args = [
        DeclareLaunchArgument("gui", default_value="true", description="curseurs d'articulations"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_imu", default_value="false"),
        DeclareLaunchArgument("use_lidar", default_value="false"),
        DeclareLaunchArgument("use_camera", default_value="false"),
    ]
    robot_description = ParameterValue(Command([
        "xacro ", PathJoinSubstitution([pkg, "urdf", "hexapod.urdf.xacro"]),
        " use_imu:=", LaunchConfiguration("use_imu"),
        " use_lidar:=", LaunchConfiguration("use_lidar"),
        " use_camera:=", LaunchConfiguration("use_camera"),
    ]), value_type=str)

    return LaunchDescription(args + [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             condition=IfCondition(LaunchConfiguration("gui"))),
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             condition=UnlessCondition(LaunchConfiguration("gui"))),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", PathJoinSubstitution([pkg, "rviz", "display.rviz"])],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
