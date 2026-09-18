"""一键启动实际河道、C++ 航道策略、Hybrid A*、DWA 和无人船仿真。"""

import fcntl
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# launch 模块在 ros2 launch 进程存活期间持有该文件锁。重复执行一键启动时，
# 第二套规划器/仿真器会在创建 ROS 节点前退出，避免两个 odom->base_link 发布者
# 让 RViz 中的本船在两组位置间跳变。进程退出后内核会自动释放锁。
_launch_lock = None


# [功能与联系] 持有进程级文件锁拒绝重复一键启动；防止多个本船仿真器发布相同base_link。
def _acquire_single_instance_lock():
    global _launch_lock
    lock_path = f"/tmp/river_navigation_{os.getuid()}.lock"
    lock_file = open(lock_path, "w", encoding="utf-8")
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        lock_file.close()
        raise RuntimeError(
            "river_navigation 已经在运行，请先 Ctrl-C 结束旧实例，"
            "不要同时启动两套 boat_simulator/base_link。"
        ) from error
    lock_file.write(str(os.getpid()))
    lock_file.flush()
    _launch_lock = lock_file


# [功能与联系] 组装节点、参数与条件启动动作；launch只负责接线，不直接执行规划或控制。
def generate_launch_description():
    _acquire_single_instance_lock()
    manager_share = get_package_share_directory("channel_navigation_manager")
    chart_share = get_package_share_directory("river_chart")
    local_share = get_package_share_directory("local_costmap_generator")
    hybrid_share = get_package_share_directory("hybrid_a_star_planner")
    astar_share = get_package_share_directory("channel_astar_global_planner")
    dwa_share = get_package_share_directory("dwa_deep")
    target_share = get_package_share_directory("virtual_boat_simulator")

    use_rviz = LaunchConfiguration("use_rviz")
    use_virtual_boat = LaunchConfiguration("use_virtual_boat")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # 河道测试虚拟船默认置于本船前方；RViz目标仍由用户自行点击。
        DeclareLaunchArgument("target_x", default_value="-310.0"),
        DeclareLaunchArgument("target_y", default_value="-210.0"),
        DeclareLaunchArgument("target_yaw", default_value="-0.99483767"),
        DeclareLaunchArgument(
            "use_virtual_boat",
            default_value="false",
            description="保留 COLREG 测试入口；河道测试默认不发布虚拟船",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(chart_share, "launch", "river_chart.launch.py")
            ),
            # 海图和规划、仿真使用同一数值坐标；map->odom 静态 TF 供 RViz 使用。
            launch_arguments={"rviz": "false", "frame_id": "odom"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(local_share, "launch", "local_costmap.launch.py")
            )
        ),
        Node(
            package="channel_navigation_manager",
            executable="channel_navigation_node",
            name="channel_navigation_node",
            output="screen",
            parameters=[os.path.join(manager_share, "config", "channel_navigation.yaml")],
        ),
        Node(
            package="dwa_deep",
            executable="boat_simulator",
            name="boat_simulator",
            output="screen",
            parameters=[os.path.join(dwa_share, "config", "sim_params.yaml")],
        ),
        Node(
            package="channel_astar_global_planner",
            executable="channel_astar_node",
            name="channel_astar_node",
            output="screen",
            parameters=[os.path.join(astar_share, "config", "channel_astar.yaml")],
        ),
        Node(
            package="hybrid_a_star_planner",
            executable="hybrid_a_star_node",
            name="hybrid_a_star_node",
            output="screen",
            parameters=[os.path.join(hybrid_share, "config", "hybrid_a_star_params.yaml"),
                        {"global_route_topic": "/astar_channel_waypoints"}],
        ),
        Node(
            package="dwa_deep",
            executable="dwa_planner",
            name="dwa_planner",
            output="screen",
            parameters=[os.path.join(dwa_share, "config", "dwa_params.yaml")],
        ),
        Node(
            package="virtual_boat_simulator",
            executable="virtual_boat_node",
            name="virtual_boat_node",
            output="screen",
            parameters=[os.path.join(target_share, "config", "virtual_boat.yaml"), {
                "initial_x": LaunchConfiguration("target_x"),
                "initial_y": LaunchConfiguration("target_y"),
                "initial_yaw": LaunchConfiguration("target_yaw"),
            }],
            condition=IfCondition(use_virtual_boat),
        ),
        # 不发布固定目标；RViz 的 2D Goal Pose 可随时重设 /goal_pose。
        Node(
            package="rviz2",
            executable="rviz2",
            name="river_navigation_rviz",
            output="screen",
            arguments=["-d", os.path.join(manager_share, "rviz", "river_navigation.rviz")],
            condition=IfCondition(use_rviz),
        ),
    ])
