import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
# [功能与联系] 组装节点、参数与条件启动动作；launch只负责接线，不直接执行规划或控制。
def generate_launch_description():
    """
    启动无人船仿真器和 DWA 规划器。
    所有节点都会加载参数文件，不再自动启动 RViz2。
    用户可在终端手动运行 rviz2 并加载本包提供的默认配置。
    """
    pkg_dir = get_package_share_directory('dwa_deep')
    config_file = os.path.join(pkg_dir, 'config', 'dwa_params.yaml')
    config_file_s = os.path.join(pkg_dir, 'config', 'sim_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'sim_param.rviz')
    return LaunchDescription([
        # 无人船仿真器
        Node(
            package='dwa_deep',
            executable='boat_simulator',
            name='boat_simulator',
            output='screen',
            parameters=[config_file_s]
        ),
        # DWA 局部规划器
        Node(
            package='dwa_deep',
            executable='dwa_planner',
            name='dwa_planner',
            output='screen',
            parameters=[config_file]
        ),
        # 不再自动启动 RViz2，用户可按需手动启动：
        # ros2 run rviz2 rviz2 -d <path_to_dwa_deep/rviz/dwa_display.rviz>
        
        # ExecuteProcess(
        #     cmd=['rviz2', '-d', rviz_config],
        #     output='screen'
        # )
    ])
