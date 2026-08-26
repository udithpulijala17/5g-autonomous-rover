from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_share = get_package_share_directory("rover_core")

    xacro_file = os.path.join(
        package_share,
        "urdf",
        "rover.urdf.xacro",
    )

    robot_description = ParameterValue(
        Command([
            "xacro",
            " ",
            xacro_file,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
            }
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
    ])
