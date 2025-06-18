import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    package_name_hw = "mcqueen_hw"
    package_name_teleop = "mcqueen_teleop"

    pkg_share_hw = FindPackageShare(package_name_hw).find(package_name_hw)
    pkg_share_teleop = FindPackageShare(package_name_teleop).find(package_name_teleop)

    controller_launch = os.path.join(
        pkg_share_hw, "launch", "control_manager.launch.py"
    )
    teleop_launch = os.path.join(pkg_share_teleop, "launch", "bringup.launch.py")
    sensor_param_yaml = os.path.join(
        pkg_share_hw, "config", "mcqueen_sensor_param.yaml"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(controller_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(teleop_launch)),
            Node(
                package=package_name_hw,
                executable="distance_publisher",
                name="distance_publisher",
                parameters=[sensor_param_yaml],
                output="screen",
            ),
            Node(
                package=package_name_hw,
                executable="buzzer_service",
                name="buzzer_service",
                parameters=[sensor_param_yaml],
                output="screen",
            ),
            Node(
                package=package_name_hw,
                executable="screen_service",
                name="screen_service",
                parameters=[sensor_param_yaml],
                output="screen",
            ),
        ]
    )
