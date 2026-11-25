"""Marche de l'hexapode : modele (URDF), noeud de marche, pont Foxglove, RViz (option).

    ros2 launch hexapod_control gait.launch.py                 # + foxglove_bridge s'il est installe
    ros2 launch hexapod_control gait.launch.py rviz:=true
    ros2 launch hexapod_control gait.launch.py foxglove:=false

Piloter : /cmd_vel (Teleop de Foxglove, ou teleop_twist_keyboard), hauteur : /body_height
(std_msgs/Float64) ou parametre body_height de /gait_node.
Foxglove : connexion "Foxglove WebSocket" a ws://<machine>:8765.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc = FindPackageShare("hexapod_description")
    ctrl = get_package_share_directory("hexapod_control")
    try:
        get_package_share_directory("foxglove_bridge")
        has_foxglove = "true"
    except PackageNotFoundError:
        has_foxglove = "false"

    robot_description = ParameterValue(Command([
        "xacro ", PathJoinSubstitution([desc, "urdf", "hexapod.urdf.xacro"]), " footprint:=false",
    ]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("foxglove", default_value=has_foxglove),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="hexapod_control", executable="gait_node", name="gait_node", output="screen",
             parameters=[os.path.join(ctrl, "config", "gait.yaml")]),
        Node(package="foxglove_bridge", executable="foxglove_bridge",
             parameters=[{"port": LaunchConfiguration("foxglove_port"),
                          "asset_uri_allowlist": ["^package://.*"]}],
             condition=IfCondition(LaunchConfiguration("foxglove"))),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(ctrl, "config", "gait.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
