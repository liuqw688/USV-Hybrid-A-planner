import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# [功能与联系] 组装节点、参数与条件启动动作；launch只负责接线，不直接执行规划或控制。
def generate_launch_description():
    share = get_package_share_directory('channel_astar_global_planner')
    return LaunchDescription([
      Node(package='channel_astar_global_planner', executable='channel_astar_node',
           name='channel_astar_node',
           parameters=[os.path.join(share, 'config', 'channel_astar.yaml')])
    ])
