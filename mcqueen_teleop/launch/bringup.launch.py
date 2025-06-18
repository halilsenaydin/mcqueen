from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "mcqueen_teleop"

    return LaunchDescription(
        [
            Node(
                package=package_name,
                executable="web_server",
                name="web_server",
                output="screen",
            )
        ]
    )
