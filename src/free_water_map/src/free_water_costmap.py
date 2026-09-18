#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid


class FreeWaterCostmap(Node):
    """Publish a bounded free-water map for the self-contained COLREGs demo."""
    # [功能与联系] 读取本节点参数并创建ROS输入输出/调度；后续回调与主循环关系见包内功能说明。
    def __init__(self):
        super().__init__('free_water_costmap')
        self.declare_parameter('topic', '/local_costmap')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('resolution', 1.0)
        self.declare_parameter('width', 220)
        self.declare_parameter('height', 180)
        self.declare_parameter('origin_x', -50.0)
        self.declare_parameter('origin_y', -90.0)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('topic').value, qos)
        self.timer = self.create_timer(1.0, self.publish_map)
        self.publish_map()

    # [功能与联系] 按参数构建全0有界开阔水域图并周期发布；仅用于演示，边界之外仍不可规划。
    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.info.resolution = float(self.get_parameter('resolution').value)
        msg.info.width = int(self.get_parameter('width').value)
        msg.info.height = int(self.get_parameter('height').value)
        msg.info.origin.position.x = float(self.get_parameter('origin_x').value)
        msg.info.origin.position.y = float(self.get_parameter('origin_y').value)
        msg.info.origin.orientation.w = 1.0
        msg.data = [0] * (msg.info.width * msg.info.height)
        self.pub.publish(msg)


# [功能与联系] 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。
def main():
    rclpy.init()
    node = FreeWaterCostmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
