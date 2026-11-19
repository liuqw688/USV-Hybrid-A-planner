"""启动无人船动力学仿真状态源."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """返回船体动力学仿真节点及可选键盘控制的启动描述."""
    package_share = FindPackageShare("usv_model_sim")
    config = PathJoinSubstitution([package_share, "config", "usv_model.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("output", default_value="screen",
                              description="Node output: screen or log"),
        DeclareLaunchArgument(
            "model_config", default_value=config, description="USV model parameters"
        ),
        Node(
            package="usv_model_sim",
            executable="usv_model_node",
            name="usv_model_node",
            output=LaunchConfiguration("output"),
            parameters=[LaunchConfiguration("model_config")],
        ),
    ])
