"""Lancement complet : simulation de la marche, et / ou pont vers le vrai robot.

    ros2 launch hexapod_control hexapod.launch.py                    # simulation (Foxglove)
    ros2 launch hexapod_control hexapod.launch.py rviz:=true         # simulation + RViz
    ros2 launch hexapod_control hexapod.launch.py robot:=dry         # + pont en SIMULATION (verifie, n'envoie rien)
    ros2 launch hexapod_control hexapod.launch.py robot:=true        # + pont : LE ROBOT BOUGE
    ros2 launch hexapod_control hexapod.launch.py teleop:=true       # + terminal de pilotage clavier
    ros2 launch hexapod_control hexapod.launch.py app:=true          # + application Foxglove connectee

Tout d'un coup : ros2_ws/hexapode.sh (simulation | dry | robot).
Arguments : robot (false | dry | true), rviz, foxglove, app, teleop, host (IP du leg_server).
Voir docs/ du paquet (gait_node.md, leg_server_bridge.md, lancement.md).
"""
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("hexapod_control")
    local_app = os.path.expanduser("~/Applications/foxglove/root/opt/Foxglove/foxglove-studio")
    foxglove_app = shutil.which("foxglove-studio") or (local_app if os.path.exists(local_app) else "")
    robot = LaunchConfiguration("robot")
    is_robot = PythonExpression(["'", robot, "' in ('true', 'dry')"])
    send = PythonExpression(["'", robot, "' == 'true'"])

    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="false",
                              description="false : simulation seule ; dry : pont sans envoi ; true : le robot bouge"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("teleop", default_value="false",
                              description="ouvre un terminal teleop_twist_keyboard sur /cmd_vel"),
        DeclareLaunchArgument("host", default_value="192.168.137.10", description="IP du leg_server"),
        DeclareLaunchArgument("app", default_value="false",
                              description="ouvre l'application Foxglove connectee a ws://localhost:8765"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "gait.launch.py")),
            launch_arguments={"rviz": LaunchConfiguration("rviz"),
                              "foxglove": LaunchConfiguration("foxglove")}.items()),

        LogInfo(msg="*** robot:=true : LE ROBOT VA SE LEVER (rampe) PUIS SUIVRE LA MARCHE ***",
                condition=IfCondition(send)),
        Node(package="hexapod_control", executable="leg_server_bridge", name="leg_server_bridge",
             output="screen", condition=IfCondition(is_robot),
             parameters=[os.path.join(share, "config", "bridge.yaml"),
                         {"send": send, "host": LaunchConfiguration("host")}]),

        ExecuteProcess(cmd=[foxglove_app, "--no-sandbox",
                            "foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:8765"],
                       output="log", condition=IfCondition(PythonExpression(
                           ["'", LaunchConfiguration("app"), "' == 'true' and '", foxglove_app, "' != ''"]))),
        LogInfo(msg="Application Foxglove introuvable (~/Applications/foxglove ou foxglove-studio) : "
                    "ouvrir Foxglove a la main sur ws://localhost:8765",
                condition=IfCondition(PythonExpression(
                    ["'", LaunchConfiguration("app"), "' == 'true' and '", foxglove_app, "' == ''"]))),

        Node(package="hexapod_control", executable="action_keys", name="action_keys",
             prefix="gnome-terminal --title='Actions hexapode' --wait --",
             condition=IfCondition(LaunchConfiguration("teleop"))),

        Node(package="teleop_twist_keyboard", executable="teleop_twist_keyboard", name="teleop",
             prefix="gnome-terminal --title='Teleop hexapode' --wait --",
             parameters=[{"speed": 0.10, "turn": 0.4}],
             condition=IfCondition(LaunchConfiguration("teleop"))),
    ])
