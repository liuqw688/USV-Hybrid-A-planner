import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# [功能与联系] 组装节点、参数与条件启动动作；launch只负责接线，不直接执行规划或控制。
def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('virtual_boat_simulator'),
        'config',
        'virtual_boat.yaml'
    )

    return LaunchDescription([
        # 发布 map -> odom 静态 TF，方便 RViz 全局显示
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # 虚拟船仿真节点
        Node(
            package='virtual_boat_simulator',
            executable='virtual_boat_node',
            name='virtual_boat_node',
            output='screen',
            parameters=[config]
        )
    ])
