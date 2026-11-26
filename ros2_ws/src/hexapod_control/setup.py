from glob import glob
from setuptools import setup

package_name = "hexapod_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml") + glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hexapode ECE",
    maintainer_email="noreply@example.com",
    description="Locomotion de l'hexapode (marche en trepied sur /cmd_vel).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gait_node = hexapod_control.gait_node:main",
            "leg_server_bridge = hexapod_control.leg_server_bridge:main",
            "action_keys = hexapod_control.action_keys:main",
        ],
    },
)
