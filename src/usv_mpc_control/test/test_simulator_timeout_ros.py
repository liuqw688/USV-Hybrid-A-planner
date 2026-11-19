#!/usr/bin/env python3
"""隔离域190：验证速度指令限幅、超时回退和零指令缓慢漂移。"""
import os
os.environ.setdefault('ROS_DOMAIN_ID', '190')
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
    effort=node.create_publisher(Twist,'/usv/cmd_velocity',10)
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
        spin(1.);assert states and 0 < states[-1].twist.twist.linear.x < .08
        old=Twist();old.linear.x=4.;old.angular.z=.4
        spin(.5,velocity=old);assert 0 < states[-1].twist.twist.linear.x < .08
        assert obsolete.get_subscription_count()==0,'Removed velocity interface is still subscribed'
        command=Twist();command.linear.x=2.;command.angular.z=.2
        spin(2.,command=command);speed=states[-1].twist.twist.linear.x
        assert speed>.5,('Velocity command did not move vessel',speed)
        spin(12.,velocity=old)
        assert states[-1].twist.twist.linear.x<.09,'Timed-out command was held'
        assert all(0<=m.twist.twist.linear.x<=4.001 for m in states)
        assert all(math.isfinite(m.pose.pose.position.x) for m in states)
        assert not any(t=='/scan' for t,_ in node.get_topic_names_and_types()),'Empty sensor output remains'
        print('PASS: velocity motion, timeout to positive zero-command equilibrium, no legacy subscriber or fake scan')
    finally:
        process.terminate()
        try:process.wait(timeout=3)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
