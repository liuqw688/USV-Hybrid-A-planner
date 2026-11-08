#!/usr/bin/env python3
"""A stopped vessel outside strict tolerance must not be reported as success."""
import importlib.util
import math
from pathlib import Path
import tempfile
import time

import rclpy
from nav_msgs.msg import Odometry


def main():
    source=Path(__file__).resolve().parents[1]/'scripts/ins_replay_node.py'
    spec=importlib.util.spec_from_file_location('replay_node',source)
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    with tempfile.TemporaryDirectory() as output:
        rclpy.init(args=['--ros-args','-p',f'output_dir:={output}'])
        node=module.InsReplay()
        try:
            node.index=29; node.running=True
            target=node.route_data.points[-1]; yaw=node.route_data.headings[-1]
            msg=Odometry(); msg.header.frame_id='odom'; msg.header.stamp=node.get_clock().now().to_msg()
            msg.pose.pose.position.x=target[0]+0.713
            msg.pose.pose.position.y=target[1]
            msg.pose.pose.orientation.z=math.sin(yaw/2); msg.pose.pose.orientation.w=math.cos(yaw/2)
            node.odom=msg; node.terminal_settle_hold=time.monotonic()-node.started_at-3.
            node.tick()
            assert node.terminal_settled and not node.completed
            node.publish_references()  # Also validates the final RViz boundary marker message.
            msg.pose.pose.position.x=target[0]
            msg.header.stamp=node.get_clock().now().to_msg()
            node.final_hold=time.monotonic()-node.started_at-3.
            node.tick()
            assert node.completed
            print('PASS: outside 0.6m -> settled but failed; exact target -> strict completion; marker publishes')
        finally:
            node.close(); node.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
