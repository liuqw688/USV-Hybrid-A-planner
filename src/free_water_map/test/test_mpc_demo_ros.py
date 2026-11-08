#!/usr/bin/env python3
"""隔离域191：开阔水域演示启动MPC链路，检查消息配对、执行器与真实运动。"""
import os
os.environ['ROS_DOMAIN_ID']='191';os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['ROS_LOG_DIR']='/tmp/usv_cleanup_demo_logs'
import argparse
import math
import signal
import subprocess
import time
import rclpy
from geometry_msgs.msg import Wrench, TwistStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--scenario',choices=['open_water','head_on'],default='open_water')
    args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('mpc_demo_probe')
    paths=[];profiles=[];states=[];commands=[];statuses=[]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Path,'/hybrid_a_star_trajectory',paths.append,qos)
    node.create_subscription(TwistStamped,'/hybrid_a_star/speed_limit',profiles.append,qos)
    node.create_subscription(Odometry,'/odom',states.append,20)
    node.create_subscription(Wrench,'/cmd_wrench',commands.append,20)
    node.create_subscription(String,'/hybrid_a_star/planning_status',lambda m:statuses.append(m.data),qos)
    log=open('/tmp/usv_cleanup_mpc_demo.log','w')
    process=subprocess.Popen(['ros2','launch','free_water_map','colregs_demo.launch.py',
      'scenario:='+args.scenario,'use_rviz:=false'],stdout=log,stderr=log,start_new_session=True)
    try:
        begin=time.monotonic()
        while time.monotonic()-begin<12:
            assert process.poll() is None,'Demo launch exited'
            rclpy.spin_once(node,timeout_sec=.025)
        assert states and commands,'MPC startup did not produce state/control messages'
        first,last=states[0].pose.pose.position,states[-1].pose.pose.position
        distance=math.hypot(last.x-first.x,last.y-first.y)
        blocked = args.scenario=='head_on' and statuses and statuses[-1]=='blocked'
        if blocked:
            # This is a startup/fail-safe check, not successful collision avoidance.
            assert all(c.force.x<=0 for c in commands[-5:]),'Blocked path did not brake'
            assert states[-1].twist.twist.linear.x<.15,'Blocked vessel still accelerating'
        else:
            path=next((p for p in reversed(paths) if len(p.poses)>2),None)
            assert path is not None,'No safe trajectory was published'
            profile=next((p for p in profiles if p.header==path.header),None)
            assert profile is not None,'Path speed profile did not match'
            assert any(c.force.x>0 for c in commands),'MPC did not drive vessel'
            assert distance>5,('No real motion',distance)
        assert all(0<=m.twist.twist.linear.x<=4.001 for m in states)
        assert all(abs(c.force.x)<=10.001 and abs(c.torque.z)<=5.001 for c in commands)
        names=node.get_node_names()
        assert 'mpc_controller' in names and 'boat_simulator' in names and 'dwa_planner' not in names
        assert not any(t in {'/cmd_vel','/candidate_trajectories','/optimal_trajectory','/scan'}
                       for t,_ in node.get_topic_names_and_types())
        outcome='blocked braking' if blocked else 'paired Path/profile and actual motion'
        print(f'PASS: MPC-only {args.scenario} startup; {outcome}; motion={distance:.2f}m; actuator limits; last_status={statuses[-1] if statuses else None}')
    finally:
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=4)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
