import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# [功能与联系] 组装节点、参数与条件启动动作；launch只负责接线，不直接执行规划或控制。
def generate_launch_description():
    # 获取包的路径
    pkg_dir = get_package_share_directory('hybrid_a_star_planner')
    
    # 加载 YAML 参数文件
    params_file = os.path.join(pkg_dir, 'config', 'hybrid_a_star_params.yaml')

    return LaunchDescription([
        Node(
            package='hybrid_a_star_planner',
            executable='hybrid_a_star_node',
            name='hybrid_a_star_node',
            output='screen',
            parameters=[params_file],
            remappings=[
                # 如果需要重映射话题，可以在这里添加
                # ('/local_costmap/costmap', '/my_custom_costmap')
            ]
        )
    ])
