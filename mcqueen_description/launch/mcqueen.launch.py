from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import xacro


def generate_launch_description():
    package_name = "mcqueen_description"
    pkg_share = FindPackageShare(package_name).find(package_name)
    xacro_path = os.path.join(pkg_share, "urdf", "mcqueen.urdf.xacro")

    # Compile xacro file
    robot_description_config = xacro.process_file(xacro_path)
    robot_description = {"robot_description": robot_description_config.toxml()}

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
        ]
    )
