from setuptools import find_packages, setup
from glob import glob
import os

package_name = "rover_core"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob("urdf/*"),
        ),
    ],

    install_requires=["setuptools"],
    zip_safe=True,

    maintainer="udithpulijala17",
    maintainer_email="udithpulijala17@gmail.com",

    description="Core ROS 2 package for the 5G autonomous rover",
    license="Apache-2.0",

    tests_require=["pytest"],

    entry_points={
        "console_scripts": [
            "heartbeat_node = rover_core.heartbeat_node:main",
            "wt901_imu_node = rover_core.wt901_imu_node:main",
        ],
    },
)
