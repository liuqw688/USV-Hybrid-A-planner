#!/usr/bin/env python3
"""隔离域190：实际仿真器仅接收力命令；缺失/超时制动，旧速度接口不可绕过。"""
import os
os.environ['ROS_DOMAIN_ID']='190'
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['ROS_LOG_DIR']='/tmp/usv_cleanup_timeout_logs'
import math
import subprocess
import time
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Wrench, Twist
from nav_msgs.msg import Odometry


def main():
    rclpy.init();node=rclpy.create_node('simulator_timeout_fixture');states=[]
    node.create_subscription(Odometry,'/odom',states.append,20)
    effort=node.create_publisher(Wrench,'/cmd_wrench',10)
    obsolete=node.create_publisher(Twist,'/cmd_vel',10)
    prefix=get_package_prefix('usv_mpc_control')
    log=open('/tmp/usv_cleanup_simulator_timeout.log','w')
    process=subprocess.Popen([prefix+'/lib/usv_mpc_control/boat_simulator','--ros-args',
      '-p','initial_speed:=0.0','-p','max_linear_speed:=4.0','-p','max_angular_speed:=0.45',
      '-p','effort_timeout:=0.3'],stdout=log,stderr=log)
    def spin(seconds,command=None,velocity=None):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if command is not None:effort.publish(command)
            if velocity is not None:obsolete.publish(velocity)
            rclpy.spin_once(node,timeout_sec=.02)
    try:
        spin(1.);assert states and states[-1].twist.twist.linear.x==0
        old=Twist();old.linear.x=4.;old.angular.z=.4
        spin(.5,velocity=old);assert states[-1].twist.twist.linear.x==0
        assert obsolete.get_subscription_count()==0,'Removed velocity interface is still subscribed'
        command=Wrench();command.force.x=4.;command.torque.z=.2
        spin(2.,command=command);speed=states[-1].twist.twist.linear.x
        assert speed>.5,('Effort command did not move vessel',speed)
        spin(2.,velocity=old)
        assert states[-1].twist.twist.linear.x<.1,'Timed-out command was held'
        assert all(0<=m.twist.twist.linear.x<=4.001 for m in states)
        assert all(math.isfinite(m.pose.pose.position.x) for m in states)
        assert not any(t=='/scan' for t,_ in node.get_topic_names_and_types()),'Empty sensor output remains'
        print('PASS: MPC effort motion, no legacy velocity subscriber, missing/stale effort braking, no fake scan')
    finally:
        process.terminate()
        try:process.wait(timeout=3)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
