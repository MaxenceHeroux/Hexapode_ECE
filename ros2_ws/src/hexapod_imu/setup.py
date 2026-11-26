from glob import glob
from setuptools import setup

package_name = "hexapod_imu"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hexapode ECE",
    maintainer_email="noreply@example.com",
    description="IMU LSM6DSV16X de l'hexapode",
    license="MIT",
    entry_points={
        "console_scripts": [
            "imu_node = hexapod_imu.imu_node:main",
            "calibrate_imu = hexapod_imu.calibrate:main",
        ],
    },
)
