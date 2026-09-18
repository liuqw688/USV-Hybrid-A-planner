import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# [功能与联系] 组装节点、参数与条件启动动作；launch只负责接线，不直接执行规划或控制。
def generate_launch_description():
    package_share = get_package_share_directory("local_costmap_generator")
    params_file = os.path.join(package_share,"config","local_costmap.yaml")

    return LaunchDescription([
        Node(
            package="local_costmap_generator",
            executable="local_costmap_node",
            name="local_costmap_node",
            output="screen",
            parameters=[params_file],
        )
    ])
