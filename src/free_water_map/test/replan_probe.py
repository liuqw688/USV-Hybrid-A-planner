#!/usr/bin/env python3
"""Measure each new spline against the latest planner-frame odometry."""
import json, math, sys, time
import numpy as np
from scipy.interpolate import BSpline
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from plan_manage.msg import Bspline
from std_msgs.msg import Header

class Probe(Node):
    def __init__(self):
        super().__init__('replan_probe')
        self.pub=self.create_publisher(PoseStamped,'/goal_pose',10)
        self.create_subscription(Odometry,'/planner/odom',self.on_odom,20)
        self.create_subscription(Bspline,'/planning/bspline',self.on_spline,10)
        self.create_subscription(Header,'/planning/replan_failed',lambda m:self.failures.append(time.monotonic()-self.initial),10)
        self.odom=None; self.records=[]; self.initial=time.monotonic(); self.goals=[]; self.failures=[]
    def on_odom(self,m): self.odom=m
    def goal(self,x,y,yaw):
        m=PoseStamped();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
        m.pose.position.x=x;m.pose.position.y=y;m.pose.orientation.z=math.sin(yaw/2);m.pose.orientation.w=math.cos(yaw/2)
        self.goals.append({'elapsed':time.monotonic()-self.initial,'target':[x,y,yaw]});self.pub.publish(m)
    def on_spline(self,m):
        if self.odom is None:return
        q=np.array([[p.x,p.y,p.z] for p in m.pos_pts]);k=np.array(m.knots);s=BSpline(k,q,m.order)
        t0=k[m.order];t1=k[len(q)];x=s(t0);end=s(t1);now=self.get_clock().now();age=(now.nanoseconds-(m.start_time.sec*1000000000+m.start_time.nanosec))*1e-9
        active=s(min(t1,max(t0,t0+age)))
        od=self.odom; pose=od.pose.pose;op=np.array([pose.position.x,pose.position.y]);
        self.records.append({'elapsed':time.monotonic()-self.initial,'id':m.traj_id,'odom':op.tolist(), 'start':x.tolist(),'end':end.tolist(), 'initial_gap':float(np.linalg.norm(x[:2]-op)), 'current_gap':float(np.linalg.norm(active[:2]-op)), 'planning_age_sec':age,'control_count':len(q),'duration':float(t1-t0)})

def main():
    rclpy.init();n=Probe(); duration=float(sys.argv[1]); scenario=sys.argv[2]; flags=set()
    while time.monotonic()-n.initial<duration:
        rclpy.spin_once(n,timeout_sec=.05);t=time.monotonic()-n.initial
        if t>6 and 1 not in flags:
            n.goal(80.,0.,0.) if scenario=='long' else n.goal(40.,0.,0.);flags.add(1)
        if scenario=='retarget' and t>17 and 2 not in flags:
            n.goal(20.,25.,1.0);flags.add(2)
    p=n.odom.pose.pose if n.odom else None
    final=[p.position.x,p.position.y,math.atan2(2*(p.orientation.w*p.orientation.z+p.orientation.x*p.orientation.y),1-2*(p.orientation.y*p.orientation.y+p.orientation.z*p.orientation.z))] if p else None
    print(json.dumps({'scenario':scenario,'goals':n.goals,'records':n.records,'failures_elapsed':n.failures,'last_pose':final},indent=2));n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
