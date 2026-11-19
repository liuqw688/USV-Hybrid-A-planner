"""启动海图和仿真环境，可选启用控制器及简易参考发布器."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package, filename, arguments=None, condition=None):
    """
    创建指定 ROS 包中启动文件的包含动作.

    :param package: ROS 包名。
    :param filename: 包内 launch 目录下的启动文件名。
    :param arguments: 可选启动参数字典。
    :param condition: 可选执行条件。

    :return: IncludeLaunchDescription 动作。
    """
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), "launch", filename,
        ])),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    """返回测试环境启动描述，控制器与参考发布器默认独立启动."""
    default_config = PathJoinSubstitution([
        FindPackageShare("usv_nmpc_controller"), "config", "usv_nmpc.yaml",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz2"),
        DeclareLaunchArgument("publish_reference", default_value="false",
                              description="Publish the built-in B-spline test route"),
        DeclareLaunchArgument("start_controller", default_value="false",
                              description="Start NMPC; disable when launching it separately"),
        DeclareLaunchArgument("simulate_perception", default_value="true",
                              description="Start simulated obstacle perception"),
        DeclareLaunchArgument("config_file", default_value=default_config,
                              description="NMPC parameter file"),
        include("usv_map", "usv_map.launch.py", {
            "rviz": LaunchConfiguration("rviz"),
            "simulate_perception": LaunchConfiguration("simulate_perception"),
            "output": "log",
        }),
        include("usv_model_sim", "usv_model.launch.py", {"output": "log"}),
        include("usv_tracking_test", "publish_bspline_reference.launch.py",
                {"output": "log"}, IfCondition(LaunchConfiguration("publish_reference"))),
        include("usv_nmpc_controller", "usv_nmpc.launch.py", {
            "config_file": LaunchConfiguration("config_file"),
        }, IfCondition(LaunchConfiguration("start_controller"))),
    ])
