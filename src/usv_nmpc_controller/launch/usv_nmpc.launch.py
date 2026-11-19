"""使用可指定的参数文件启动 NMPC 跟踪节点."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """返回读取指定参数文件的 NMPC 节点启动描述."""
    package_name = "usv_nmpc_controller"
    default_config = str(
        Path(get_package_share_directory(package_name))
        / "config"
        / "usv_nmpc.yaml"
    )

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="USV NMPC YAML configuration file",
        ),
        Node(
            package=package_name,
            executable="usv_nmpc_node",
            name="usv_nmpc_node",
            output="screen",
            output_format="{line}",
            parameters=[config_file],
            emulate_tty=True,
        ),
    ])
