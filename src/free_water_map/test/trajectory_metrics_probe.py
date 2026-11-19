#!/usr/bin/env python3
"""Capture first open-water front-end path and optimized B-spline from a fixed state."""
import json, math, sys, time
import numpy as np
from scipy.interpolate import BSpline
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from plan_manage.msg import Bspline
from visualization_msgs.msg import Marker

class Probe(Node):
 def __init__(self,goal):
  super().__init__('single_plan_probe');self.goal_xyz=goal;self.started=time.monotonic();self.sent=None;self.spline=None;self.front=None
  q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.map=self.create_publisher(OccupancyGrid,'/map',q);self.odom=self.create_publisher(Odometry,'/planner/odom',10);self.goal=self.create_publisher(Path,'/waypoint_generator/waypoints',10)
  self.create_subscription(Bspline,'/planning/bspline',self.on_spline,10);self.create_subscription(Marker,'/planning_vis/trajectory',self.on_marker,20)
 def on_spline(self,m):
  if self.spline is None:
   k=np.array(m.knots);p=np.array([[x.x,x.y,x.z] for x in m.pos_pts]);f=BSpline(k,p,m.order);t=np.linspace(k[m.order],k[len(p)],1001);a=f(t);d=np.diff(a[:,:2],axis=0);self.spline={'knots':k.tolist(),'control_points':p.tolist(),'order':int(m.order),'elapsed_after_goal':time.monotonic()-self.sent,'duration':float(t[-1]-t[0]),'points':len(p),'length':float(np.linalg.norm(d,axis=1).sum()),'max_abs_y':float(np.abs(a[:,1]).max()),'end':a[-1].tolist(),'samples':a[::10].tolist()}
 def on_marker(self,m):
  if self.sent is not None and self.front is None and m.id==200 and m.action==0 and len(m.points)>1:
   p=np.array([[v.x,v.y,v.z] for v in m.points]);self.front={'points':len(p),'length':float(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1).sum()),'max_abs_y':float(np.abs(p[:,1]).max()),'end':p[-1].tolist(),'samples':p[::max(1,len(p)//100)].tolist()}
 def update(self):
  now=self.get_clock().now().to_msg();m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=now;m.info.resolution=1.;m.info.width=300;m.info.height=200;m.info.origin.position.x=-120.;m.info.origin.position.y=-100.;m.info.origin.orientation.w=1.;m.data=[0]*60000;self.map.publish(m)
  o=Odometry();o.header.frame_id='map';o.header.stamp=now;o.child_frame_id='base_link';o.pose.pose.orientation.w=1.;self.odom.publish(o)
 def send(self):
  now=self.get_clock().now().to_msg();p=PoseStamped();p.header.frame_id='map';p.header.stamp=now;p.pose.position.x=self.goal_xyz[0];p.pose.position.y=self.goal_xyz[1];p.pose.orientation.z=math.sin(self.goal_xyz[2]/2);p.pose.orientation.w=math.cos(self.goal_xyz[2]/2);m=Path();m.header=p.header;m.poses=[p];self.sent=time.monotonic();self.goal.publish(m)
def main():
 rclpy.init();n=Probe(tuple(map(float,sys.argv[1:4])));last=0
 while time.monotonic()-n.started<25:
  rclpy.spin_once(n,timeout_sec=.05);t=time.monotonic()-n.started
  if t-last>.2:n.update();last=t
  if t>4 and n.sent is None and n.count_subscribers('/waypoint_generator/waypoints')>0:n.send()
  if n.spline is not None and n.front is not None:break
 print(json.dumps({'goal':n.goal_xyz,'sent':n.sent is not None,'spline':n.spline,'frontend':n.front},indent=2));n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
