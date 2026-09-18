#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


class DemoGoalPublisher(Node):
    # [功能与联系] 读取本节点参数并创建ROS输入输出/调度；后续回调与主循环关系见包内功能说明。
    def __init__(self):
        super().__init__('demo_goal_publisher')
        self.declare_parameter('topic', '/goal_pose')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('x', 55.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('publish_initial_goal', True)
        # The initial goal is transient-local and published only once.  The old
        # periodic publisher overwrote every goal selected with RViz.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            PoseStamped, self.get_parameter('topic').value, qos)
        self.timer = self.create_timer(0.8, self.publish_goal_once)

    # [功能与联系] 启动后取消自己的timer，可选发布一次演示目标；避免周期性覆盖用户RViz目标。
    def publish_goal_once(self):
        self.timer.cancel()
        if not bool(self.get_parameter('publish_initial_goal').value):
            return
        yaw = float(self.get_parameter('yaw').value)
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_parameter('frame_id').value
        message.pose.position.x = float(self.get_parameter('x').value)
        message.pose.position.y = float(self.get_parameter('y').value)
        message.pose.orientation.z = math.sin(0.5 * yaw)
        message.pose.orientation.w = math.cos(0.5 * yaw)
        self.publisher.publish(message)
        self.get_logger().info(
            'Initial goal published once; RViz 2D Goal Pose can now replace it.')


# [功能与联系] 初始化ROS上下文、创建本文件节点并进入回调调度；退出时释放节点与通信资源，是launch启动可执行程序后的入口。
def main():
    rclpy.init()
    node = DemoGoalPublisher()
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
