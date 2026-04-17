from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    repo_root = Path(__file__).resolve().parents[3]
    binary_path = repo_root / "build" / "bin" / "remote_control_node"
    default_config = repo_root / "third_party" / "remote_control" / "config" / "xbox_like.yaml"
    joy_share = Path(get_package_share_directory("joy"))
    default_joy_params = joy_share / "config" / "joy-params.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=str(default_config)),
            DeclareLaunchArgument("joy_params_file", default_value=str(default_joy_params)),
            DeclareLaunchArgument("joy_device_id", default_value="0"),
            DeclareLaunchArgument("start_joy_node", default_value="true"),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("joy_params_file"),
                    {"device_id": LaunchConfiguration("joy_device_id")},
                ],
                condition=IfCondition(LaunchConfiguration("start_joy_node")),
            ),
            ExecuteProcess(
                cmd=[
                    str(binary_path),
                    "--ros-args",
                    "--params-file",
                    LaunchConfiguration("config_file"),
                ],
                output="screen",
            ),
        ]
    )
