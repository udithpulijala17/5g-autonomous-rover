from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    robot_description = ParameterValue(
        Command([
            "xacro",
            " ",
            "/workspace/ros_ws/src/rover_core/urdf/rover.urdf.xacro",
        ]),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_description}
            ],
        )
    ])
