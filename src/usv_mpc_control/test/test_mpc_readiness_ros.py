#!/usr/bin/env python3
"""隔离ROS域：缺TF制动，TF迟到自动恢复，planning/blocked空路径仍制动。"""
import os
import subprocess
import time
os.environ['ROS_DOMAIN_ID']='177'
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['ROS_LOG_DIR']='/tmp/usv_mpc_readiness_logs'
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped, Wrench, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

def main():
    rclpy.init()
    node=rclpy.create_node('mpc_readiness_fixture')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    grid_pub=node.create_publisher(OccupancyGrid,'/readiness/map',qos)
    path_pub=node.create_publisher(Path,'/readiness/path',qos)
    odom_pub=node.create_publisher(Odometry,'/readiness/odom',20)
    status_pub=node.create_publisher(String,'/hybrid_a_star/planning_status',qos)
    profile_pub=node.create_publisher(TwistStamped,'/hybrid_a_star/speed_limit',qos)
    commands=[]
    node.create_subscription(Wrench,'/readiness/wrench',lambda m: commands.append(m.force.x),20)
    log_path='/tmp/usv_mpc_readiness.log'
    log=open(log_path,'w')
    binary=get_package_prefix('usv_mpc_control')+'/lib/usv_mpc_control/mpc_controller'
    proc=subprocess.Popen([binary,'--ros-args','-p','channel_enabled:=false','-p','require_speed_profile:=true',
        '-p','path_topic:=/readiness/path','-p','costmap_topic:=/readiness/map',
        '-p','odom_topic:=/readiness/odom','-p','effort_topic:=/readiness/wrench'],stdout=log,stderr=log)
    path=Path();path.header.frame_id='odom'
    for x in range(31):
        p=PoseStamped();p.header.frame_id='odom';p.pose.position.x=float(x);p.pose.orientation.w=1.
        path.poses.append(p)
    active=path
    def spin(seconds,send_profile=True):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            odom=Odometry();odom.header.frame_id='odom';odom.header.stamp=node.get_clock().now().to_msg()
            odom.pose.pose.orientation.w=1.;odom.twist.twist.linear.x=1.
            odom_pub.publish(odom)
            active.header.stamp=odom.header.stamp;path_pub.publish(active)
            if send_profile:
                profile=TwistStamped();profile.header=active.header;profile.twist.linear.x=4.
                profile_pub.publish(profile)
            rclpy.spin_once(node,timeout_sec=.025)
    try:
        grid=OccupancyGrid();grid.header.frame_id='map'
        grid.info.resolution=1.;grid.info.width=grid.info.height=100
        grid.info.origin.position.x=grid.info.origin.position.y=-40.
        grid.info.origin.orientation.w=1.;grid.data=[0]*10000;grid_pub.publish(grid)
        # 制动警告有3s节流；等待跨过节流窗，日志断言不能依赖恰好一次启动调度。
        spin(3.5)
        assert commands and commands[-1]<0,'Missing TF did not brake'
        tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='map'
        tf.transform.rotation.w=1.
        broadcaster=StaticTransformBroadcaster(node);broadcaster.sendTransform(tf)
        commands.clear();spin(2.)
        assert commands and commands[-1]>0,'Late TF did not restore tracking'
        path.poses[-1].pose.position.x=31.
        commands.clear();spin(1.,False)
        assert commands and commands[-1]<0,'New geometry accepted mismatched old speed profile'
        commands.clear();spin(1.)
        assert commands and commands[-1]>0,'Matching speed profile did not restore tracking'
        active=Path();active.header.frame_id='odom'
        status=String();status.data='planning';status_pub.publish(status)
        commands.clear();spin(1.)
        assert commands and commands[-1]<0,'Planning empty path did not brake'
        active=path;commands.clear();spin(1.)
        assert commands and commands[-1]>0,'Valid path did not restore tracking'
        active=Path();active.header.frame_id='odom';status.data='blocked';status_pub.publish(status)
        commands.clear();spin(1.)
        assert commands and commands[-1]<0,'Blocked path did not brake'
        log.flush()
        with open(log_path) as report: output=report.read()
        assert 'TF unavailable/invalid' in output
        assert 'MPC waiting safely:' in output
        assert 'waiting for speed profile matching new path' in output
        assert 'planner withdrew path (blocked)' in output
        assert 'waiting for a same-frame path' not in output
        print('PASS: missing TF brakes; late TF restores; empty planning/blocked paths brake; valid path resumes; detailed diagnostics')
    finally:
        proc.terminate()
        try: proc.wait(timeout=5)
        except subprocess.TimeoutExpired: proc.kill();proc.wait()
        log.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__': main()
