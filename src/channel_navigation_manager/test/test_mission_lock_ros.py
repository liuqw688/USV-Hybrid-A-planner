#!/usr/bin/env python3
"""隔离ROS域182：目标锁存、旋转map/odom TF、迟到TF、转向/越过目标不换道。"""
import os
os.environ['ROS_DOMAIN_ID']='182'
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['ROS_LOG_DIR']='/tmp/usv_mission_lock_logs'
import json
import math
import subprocess
import time
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import StaticTransformBroadcaster


def main():
    rclpy.init();node=rclpy.create_node('mission_lock_fixture')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    lanes_pub=node.create_publisher(String,'/lock/lanes',qos)
    grid_pub=node.create_publisher(OccupancyGrid,'/lock/map',qos)
    odom_pub=node.create_publisher(Odometry,'/lock/odom',20)
    goal_pub=node.create_publisher(PoseStamped,'/lock/goal',10)
    maps=[];statuses=[]
    node.create_subscription(OccupancyGrid,'/lock/lane_map',lambda m:maps.append(m),qos)
    node.create_subscription(String,'/lock/status',lambda m:statuses.append(json.loads(m.data)),qos)
    log=open('/tmp/usv_mission_lock.log','w')
    proc=subprocess.Popen([get_package_prefix('channel_navigation_manager')+
      '/lib/channel_navigation_manager/channel_navigation_node','--ros-args',
      '-p','lane_data_topic:=/lock/lanes','-p','costmap_topic:=/lock/map',
      '-p','odom_topic:=/lock/odom','-p','goal_topic:=/lock/goal',
      '-p','lane_costmap_topic:=/lock/lane_map','-p','status_topic:=/lock/status'],stdout=log,stderr=log)
    boat=[0.,-5.,0.]
    def spin(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            m=Odometry();m.header.frame_id='odom';m.header.stamp=node.get_clock().now().to_msg()
            m.pose.pose.position.x,m.pose.pose.position.y=boat[:2]
            m.pose.pose.orientation.z=math.sin(boat[2]/2);m.pose.pose.orientation.w=math.cos(boat[2]/2)
            m.twist.twist.linear.x=2.;m.twist.twist.linear.y=.3
            odom_pub.publish(m);rclpy.spin_once(node,timeout_sec=.025)
    def goal(y):
        m=PoseStamped();m.header.frame_id='map';m.header.stamp=node.get_clock().now().to_msg()
        m.pose.position.x=25.;m.pose.position.y=y;goal_pub.publish(m)
    def value(chart_x,chart_y):
        m=maps[-1]
        x,y=chart_y+5.,20.-chart_x # inverse of map<-odom R90 + (20,-5)
        dx=x-m.info.origin.position.x;dy=y-m.info.origin.position.y
        a=2*math.atan2(m.info.origin.orientation.z,m.info.origin.orientation.w)
        ix=int(math.floor(math.cos(a)*dx+math.sin(a)*dy))
        iy=int(math.floor(-math.sin(a)*dx+math.cos(a)*dy))
        return m.data[iy*m.info.width+ix]
    try:
        lanes=String();lanes.data=json.dumps({'frame_id':'map','lanes':[
          {'lane_id':1,'orient_deg':0.,'points':[[20,-60],[30,-60],[30,80],[20,80],[20,-60]]},
          {'lane_id':2,'orient_deg':180.,'points':[[10,-60],[20,-60],[20,80],[10,80],[10,-60]]}]})
        lanes_pub.publish(lanes)
        grid=OccupancyGrid();grid.header.frame_id='odom';grid.info.resolution=1.
        grid.info.width=grid.info.height=180;grid.info.origin.position.x=grid.info.origin.position.y=-80.
        grid.info.origin.orientation.z=math.sin(.3/2);grid.info.origin.orientation.w=math.cos(.3/2)
        grid.data=[0]*32400;grid_pub.publish(grid);spin(1.)
        goal(50.);spin(1.5);assert not maps,'Missing TF published a guessed mission lane'
        boat[:]=[40.,5.,math.pi];spin(.5)
        tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom'
        tf.transform.translation.x=20.;tf.transform.translation.y=-5.
        tf.transform.rotation.z=math.sin(math.pi/4);tf.transform.rotation.w=math.cos(math.pi/4)
        broadcaster=StaticTransformBroadcaster(node);broadcaster.sendTransform(tf);spin(2.)
        assert maps and value(25,0)<=20 and value(15,0)==75,'North mission chose wrong lane'
        anchor=statuses[-1]['task_origin']
        assert math.hypot(anchor[0]-25.,anchor[1]+5.)<1e-6,('Late TF changed latched origin',statuses[-1])
        assert abs(statuses[-1]['policy_course_deg_ros']-90.)<.01
        count=len(maps)
        # 越过目标、转向180度、跨分道，均不能改变同一任务的左右角色。
        boat[:]=[90.,5.,-math.pi/2];spin(2.)
        assert len(maps)==count,'Odometry triggered lane reclassification during mission'
        assert value(25,0)<=20 and value(15,0)==75
        grid.info.origin.position.x+=1.;grid_pub.publish(grid);spin(1.)
        assert value(25,0)<=20 and value(15,0)==75,'Map rebuild changed latched lane roles'
        payload=json.loads(lanes.data);payload['lanes'].reverse();lanes.data=json.dumps(payload)
        lanes_pub.publish(lanes);spin(1.)
        assert value(25,0)<=20 and value(15,0)==75,'Chart polygon reorder swapped lane roles'
        goal(-50.);spin(1.5)
        assert value(15,0)<=20 and value(25,0)==75,'New rear goal did not swap right-hand lane'
        boat[:]=[-30.,-5.,0.];spin(1.)
        assert value(15,0)<=20 and value(25,0)==75,'Turn completion changed reverse mission lane'
        print('PASS: rotated TF/grid; late TF preserves origin; turn/cross/passed-goal/map rebuild keep roles; new goal swaps roles')
    finally:
        proc.terminate()
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        log.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
