from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
import os


def generate_launch_description():
    package_name_desc = "mcqueen_description"
    package_name_hw = "mcqueen_hw"

    pkg_share_desc = FindPackageShare(package_name_desc).find(package_name_desc)
    pkg_share_hw = FindPackageShare(package_name_hw).find(package_name_hw)

    desc_launch = os.path.join(pkg_share_desc, "launch", "mcqueen.launch.py")
    rsp = IncludeLaunchDescription(PythonLaunchDescriptionSource(desc_launch))

    robot_description = Command(
        ["ros2 param get --hide-type /robot_state_publisher robot_description"]
    )
    controller_yaml = os.path.join(pkg_share_hw, "config", "mcqueen_controller.yaml")
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controller_yaml],
    )
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    mcqueen_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_cont"],
    )
    delayed_mcqueen_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[mcqueen_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    return LaunchDescription(
        [
            rsp,
            delayed_controller_manager,
            delayed_mcqueen_drive_spawner,
            delayed_joint_broad_spawner,
        ]
    )
