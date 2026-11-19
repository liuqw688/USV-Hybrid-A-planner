"""在 RViz2 中设计泊位并保存到 XML 海图."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """返回泊位编辑模式的启动描述."""
    shared_launch = Path(get_package_share_directory("usv_map_tools")) / (
        "launch/area_editor.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("input_chart", default_value=""),
        DeclareLaunchArgument("output_chart", default_value=""),
        DeclareLaunchArgument("rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(shared_launch)),
            launch_arguments={
                "editor": "berth",
                "input_chart": LaunchConfiguration("input_chart"),
                "output_chart": LaunchConfiguration("output_chart"),
                "rviz": LaunchConfiguration("rviz"),
            }.items(),
        ),
    ])
