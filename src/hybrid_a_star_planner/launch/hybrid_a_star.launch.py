import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

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
            parameters=[params_file]
        )
    ])
