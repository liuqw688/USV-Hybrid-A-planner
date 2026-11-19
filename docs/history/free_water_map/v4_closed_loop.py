#!/usr/bin/env python3
"""Run real ROS nodes, measure actual odometry throughout each encounter."""
import argparse
import json
import math
import os
from pathlib import Path as FilePath
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from hybrid_a_star_planner.msg import EncounterArray

# [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

# [功能与联系] 执行此场景的输入构造和断言核验；用于回归验证，不参与正常航行控制。具体通过条件以函数内断言为准。
def run(scenario, duration, directory):
    directory.mkdir(parents=True,exist_ok=True)
    with (directory/(scenario+'.log')).open('w') as log:
        child=subprocess.Popen(['ros2','launch','free_water_map','colregs_demo.launch.py',
            'scenario:='+scenario,'use_rviz:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        rclpy.init()
        node=Node('v4_closed_loop_probe')
        state={}
        subscriptions=[]
        for name,topic,kind in [('own','/odom',Odometry),('target','/target_boat/odom',Odometry),
                ('policy','/colregs/policies',EncounterArray),('path','/hybrid_a_star_trajectory',Path),
                ('cmd','/cmd_vel',Twist)]:
            subscriptions.append(node.create_subscription(kind,topic,
                lambda m,k=name:state.__setitem__(k,m),20))
        command=node.create_publisher(Twist,'/target_boat/command',10)
        start=time.monotonic()
        minimum=float('inf'); max_port=0.0; min_heading=100; samples=0
        seen_types=set(); takeover=False; released=False; had_lock=False
        arrived=False; first_right=None; normal_heading=0; rows=[]
        swing_direction=0;swing_extreme=0.0;swing_reversals=0;starboard_passed=False
        crossing_side=None
        first_recovering=None; recovered_before_unlock=False
        recovery_command_delay=None
        try:
            while time.monotonic()-start<duration:
                rclpy.spin_once(node,timeout_sec=0.02)
                elapsed=time.monotonic()-start
                if scenario=='port_cooperative' and elapsed>=2:
                    m=Twist();m.linear.x=2.0
                    m.angular.z=-0.15 if elapsed<12.5 else 0.0
                    command.publish(m)
                if not all(k in state for k in ['own','target','policy']):continue
                own=state['own'];target=state['target']
                op=own.pose.pose.position;tp=target.pose.pose.position
                heading=yaw(own.pose.pose.orientation)
                # 无障碍单船右舷交叉允许一次“右转避让→回归”切换，
                # 但安全通过前不能反复做超过3度的左右切换；不禁止合理提前恢复。
                if scenario=='starboard' and not starboard_passed:
                    tyaw=yaw(target.pose.pose.orientation)
                    dx=op.x-tp.x;dy=op.y-tp.y
                    across=dx*math.sin(tyaw)-dy*math.cos(tyaw)
                    along=dx*math.cos(tyaw)+dy*math.sin(tyaw)
                    if crossing_side is None:crossing_side=-1 if across<0 else 1
                    starboard_passed=across*crossing_side < -7.5 and along < -7.5
                    change=heading-swing_extreme
                    if swing_direction==0:
                        if abs(change)>math.radians(3):
                            swing_direction=1 if change>0 else -1;swing_extreme=heading
                    elif swing_direction*change>=0:
                        swing_extreme=heading
                    elif abs(change)>math.radians(3):
                        swing_reversals+=1;swing_direction=-swing_direction;swing_extreme=heading
                minimum=min(minimum,math.hypot(op.x-tp.x,op.y-tp.y))
                samples+=1
                policy=state['policy']
                for e in policy.encounters:
                    if e.recovering and first_recovering is None:
                        first_recovering=elapsed
                        recovered_before_unlock=e.locked
                    if e.locked:
                        had_lock=True;seen_types.add(e.type);takeover|=e.takeover
                        if e.type==3 and not e.takeover and not e.recovering:
                            normal_heading=max(normal_heading,abs(heading-e.reference))
                        if (e.type in [1,2] or e.takeover) and not e.recovering:
                            max_port=max(max_port,heading-e.reference)
                    elif had_lock:released=True
                min_heading=min(min_heading,heading)
                if scenario=='starboard' and first_recovering is not None and recovery_command_delay is None:
                    if 'cmd' in state and state['cmd'].angular.z>0.02:
                        recovery_command_delay=elapsed-first_recovering
                if heading< -0.04 and first_right is None:first_right=elapsed
                if not rows or elapsed-rows[-1][0]>=0.2:
                    rows.append([elapsed,op.x,op.y,heading,tp.x,tp.y,own.twist.twist.linear.x])
                if math.hypot(op.x-55,op.y)<1.2:
                    arrived=True;break
        finally:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        expected={'head_on':1,'starboard':2,'port_cooperative':3,'port_takeover':3,'overtaking':4}
        errors=[]
        max_own_speed = max((row[6] for row in rows), default=0.0)
        if max_own_speed > 4.01: errors.append('own speed exceeds 4m/s')
        if samples<100:errors.append('insufficient odometry')
        if scenario in expected and expected[scenario] not in seen_types:errors.append('wrong encounter')
        if scenario in ['head_on','starboard','port_takeover'] and min_heading> -0.07:
            errors.append('no substantial starboard action')
        if max_port>math.radians(2):errors.append('port turn during give-way')
        if scenario=='port_takeover' and not takeover:errors.append('no Rule17 takeover')
        if scenario=='port_cooperative' and takeover:errors.append('unnecessary Rule17 takeover')
        if scenario=='port_cooperative' and normal_heading>math.radians(5.5):
            errors.append('stand-on heading changed')
        if scenario=='starboard' and swing_reversals>1:
            errors.append('repeated major turn reversals before passing')
        if scenario=='starboard' and not recovered_before_unlock:
            errors.append('recovery still waits for historical unlock')
        if minimum<(8.8 if scenario=='overtaking' else 7.5):errors.append('clearance violation')
        if not arrived:errors.append('goal not reached')
        result=dict(scenario=scenario,passed=not errors,errors=errors,minimum_distance=minimum,
            arrived=arrived,types=sorted(seen_types),takeover=takeover,released=released,
            max_own_speed=max_own_speed,
            first_recovering_seconds=first_recovering,
            recovered_before_unlock=recovered_before_unlock,
            recovery_command_delay_seconds=recovery_command_delay,
            starboard_turn_reversals_before_pass=swing_reversals,
            min_heading_deg=math.degrees(min_heading),max_port_deg=math.degrees(max_port),
            first_right_seconds=first_right,normal_heading_deg=math.degrees(normal_heading))
        (directory/(scenario+'.json')).write_text(json.dumps(result,indent=2))
        (directory/(scenario+'_trajectory.json')).write_text(json.dumps(rows))
        print(json.dumps(result),flush=True)
        return not errors

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('scenario')
    # Rule 17 takeover can create a deliberately large early manoeuvre.  Keep
    # enough wall-clock time for the vessel to rejoin the route and reach goal.
    p.add_argument('--duration',type=float,default=300)
    p.add_argument('--output',default='/home/l/work_ws/test_results/v4')
    args=p.parse_args()
    raise SystemExit(0 if run(args.scenario,args.duration,FilePath(args.output)) else 1)
