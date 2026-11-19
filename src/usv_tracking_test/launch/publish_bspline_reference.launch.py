"""启动往复式路径 B 样条参考发布器."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """返回 B 样条测试参考发布器的启动描述."""
    return LaunchDescription([
        DeclareLaunchArgument("output", default_value="screen",
                              description="Node output: screen or log"),
        Node(
            package="usv_tracking_test",
            executable="publish_bspline_reference.py",
            name="bspline_reference_publisher",
            output=LaunchConfiguration("output"),
        ),
    ])
