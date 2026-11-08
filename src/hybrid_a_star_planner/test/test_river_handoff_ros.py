#!/usr/bin/env python3
"""真实XML河图、全局A*、Hybrid、原MPC和欠驱动仿真闭环航点交接测试。

使用独立DDS域，仅清理本脚本启动的进程，不触碰用户RViz/仿真实例。
"""
import os
os.environ['ROS_DOMAIN_ID'] = '185'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/usv_river_handoff_logs'
import json
import math
import signal
import subprocess
import time
from pathlib import Path as FilePath
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.qos import QoSProfile, DurabilityPolicy


def main():
    rclpy.init()
    node = rclpy.create_node('river_handoff_probe')
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    children, logs, state, routes, goals, paths, samples, caps = [], [], {}, [], [], [], [], []
    out = FilePath(os.environ.get('RIVER_HANDOFF_OUTPUT','artifacts/river_handoff_update'))
    out.mkdir(parents=True, exist_ok=True)
    def launch(command, name):
        log = open(out / (name + '.log'), 'w')
        logs.append(log)
        children.append(subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True))
    def executable(package, binary, yaml, extra=()):
        launch([get_package_prefix(package)+'/lib/'+package+'/'+binary, '--ros-args',
            '--params-file', get_package_share_directory(package)+'/config/'+yaml,
            *extra], binary)
    def received_goal(m):
        point=(m.pose.position.x,m.pose.position.y)
        if not goals or point != goals[-1]['xy']:
            goals.append({'xy':point,'time':time.monotonic(),'odom':(
                state['odom'].pose.pose.position.x,state['odom'].pose.pose.position.y) if 'odom' in state else None})
            print('ACTIVE', point, flush=True)
    def received_path(m):
        paths.append((time.monotonic(),m))
        if m.poses:
            p=m.poses[-1].pose.position
            for g in goals:
                if 'delivery' not in g and math.hypot(p.x-g['xy'][0],p.y-g['xy'][1])<1:
                    g['delivery']=time.monotonic()-g['time']
                    print('DELIVERED', g['xy'], g['delivery'],flush=True)
    node.create_subscription(OccupancyGrid,'/channel/lane_costmap',lambda m:state.update(lane=m),qos)
    node.create_subscription(OccupancyGrid,'/local_costmap',lambda m:state.update(map=m),qos)
    node.create_subscription(Odometry,'/odom',lambda m:state.update(odom=m),20)
    node.create_subscription(Path,'/astar_channel_waypoints',routes.append,qos)
    node.create_subscription(PoseStamped,'/goal_pose_from_astar',received_goal,qos)
    node.create_subscription(Path,'/hybrid_a_star_trajectory',received_path,qos)
    node.create_subscription(TwistStamped,'/hybrid_a_star/speed_limit',caps.append,qos)
    pub=node.create_publisher(PoseStamped,'/goal_pose',10)
    def spin(seconds, predicate=None):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            assert all(p.poll() is None for p in children),'A child node exited'
            rclpy.spin_once(node,timeout_sec=.02)
            if 'odom' in state:
                m=state['odom'];samples.append([time.monotonic(),m.pose.pose.position.x,
                    m.pose.pose.position.y,m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z])
            if predicate and predicate(): return
        if predicate: raise AssertionError('Timed out waiting for river pipeline')
    result={'passed':False}
    try:
        launch(['ros2','launch','river_chart','river_chart.launch.py','rviz:=false','frame_id:=odom'],'chart')
        executable('local_costmap_generator','local_costmap_node','local_costmap.yaml')
        executable('channel_navigation_manager','channel_navigation_node','channel_navigation.yaml')
        executable('channel_astar_global_planner','channel_astar_node','channel_astar.yaml')
        executable('hybrid_a_star_planner','hybrid_a_star_node','hybrid_a_star_params.yaml',
            ['-p','global_route_topic:=/astar_channel_waypoints'])
        executable('usv_mpc_control','mpc_controller','mpc_params.yaml')
        executable('usv_mpc_control','boat_simulator','sim_params.yaml')
        spin(60,lambda: 'lane' in state and 'map' in state and 'odom' in state and pub.get_subscription_count()>=2)
        goal=PoseStamped();goal.header.frame_id='map';goal.header.stamp=node.get_clock().now().to_msg()
        goal.pose.position.x=-324.;goal.pose.position.y=-101.
        goal.pose.orientation.z=math.sin(math.radians(110)/2);goal.pose.orientation.w=math.cos(math.radians(110)/2)
        special=os.environ.get('RIVER_SPECIAL_GOAL')=='right'
        if special:
            own=state['odom'].pose.pose;heading=2*math.atan2(own.orientation.z,own.orientation.w)
            goal.pose.position.x=own.position.x+20*math.cos(heading)
            goal.pose.position.y=own.position.y+20*math.sin(heading)
            goal.pose.orientation.z=math.sin((heading-math.pi/2)/2)
            goal.pose.orientation.w=math.cos((heading-math.pi/2)/2)
        pub.publish(goal)
        spin(30,lambda: goals and 'delivery' in goals[0])
        if special:
            trajectory=next(p for _,p in paths if p.poses)
            terminal=trajectory.poses[-1].pose
            requested=2*math.atan2(goal.pose.orientation.z,goal.pose.orientation.w)
            error=abs(math.atan2(math.sin(2*math.atan2(terminal.orientation.z,terminal.orientation.w)-requested),
                math.cos(2*math.atan2(terminal.orientation.z,terminal.orientation.w)-requested)))
            assert error<=.523599,'Terminal heading outside configured tolerance'
            spin(8)
            length=0.;prefix=0.;turning=False;opposite=0.
            grid=state['lane']
            for a,b in zip(trajectory.poses,trajectory.poses[1:]):
                ds=math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y);length+=ds
                angle=2*math.atan2(b.pose.orientation.z,b.pose.orientation.w)-heading
                turning=turning or abs(math.atan2(math.sin(angle),math.cos(angle)))>.04
                if not turning:prefix+=ds
                ix=math.floor((b.pose.position.x-grid.info.origin.position.x)/grid.info.resolution)
                iy=math.floor((b.pose.position.y-grid.info.origin.position.y)/grid.info.resolution)
                if 0<=ix<grid.info.width and 0<=iy<grid.info.height and grid.data[iy*grid.info.width+ix]==75:opposite+=ds
            cap=next((c.twist.linear.x for c in caps if c.header==trajectory.header),None)
            result={'passed':False,'scenario':'actual river 20m forward, right-facing goal',
                'active_goals':goals,'heading_error_rad':error,'reference_speed':cap,'path_length_m':length,
                'initial_straight_m':prefix,'opposite_lane_length_m':opposite,
                'final_measured_speed':state['odom'].twist.twist.linear.x}
            assert length<55,'Unnecessarily large close-range detour'
            assert prefix<8,'Initial straight remains too long'
            result['passed']=True
            print(json.dumps(result),flush=True)
            return
        spin(150,lambda: len(goals)>=2 and 'delivery' in goals[1])
        spin(5)
        first,second=goals[:2]
        empty_during_handoff=sum(not p.poses for t,p in paths if second['time']<=t<=second['time']+second['delivery'])
        handoff_speeds=[s[3] for s in samples if second['time']-1<=s[0]<=second['time']+second['delivery']+5]
        result={'passed':False,'route_waypoints':len(next(r for r in routes if r.poses).poses),
            'active_goals':goals,'empty_paths_during_handoff':empty_during_handoff,
            'minimum_measured_speed_during_handoff':min(handoff_speeds),
            'final_measured_speed':state['odom'].twist.twist.linear.x}
        assert second['delivery']<3,'Second waypoint planning too slow'
        assert empty_during_handoff==0,'Path withdrawn during safe waypoint handoff'
        assert min(handoff_speeds)>.25,'Vessel stopped during waypoint handoff'
        assert len([r for r in routes if r.poses])==1,'Global route unexpectedly replanned'
        result['passed']=True
    finally:
        import csv
        with open(out/'tracking.csv','w') as f:
            writer=csv.writer(f);writer.writerow(['time','x','y','u','v','r']);writer.writerows(samples)
        baseline=goals[0]['time'] if goals else 0
        for g in goals:g['time']-=baseline
        json.dump(result,open(out/'river_result.json','w'),ensure_ascii=False,indent=2)
        for index,g in enumerate(goals):
            matched=[p for _,p in paths if p.poses and math.hypot(p.poses[-1].pose.position.x-g['xy'][0],p.poses[-1].pose.position.y-g['xy'][1])<1]
            if matched:
                with open(out/f'leg_{index+1}.csv','w') as f:
                    writer=csv.writer(f);writer.writerow(['x','y','yaw'])
                    for q in matched[0].poses:writer.writerow([q.pose.position.x,q.pose.position.y,2*math.atan2(q.pose.orientation.z,q.pose.orientation.w)])
        if 'lane' in state and samples:
            import numpy as np
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            grid=state['lane'];w,h=grid.info.width,grid.info.height
            a=np.asarray(grid.data).reshape(h,w)
            ox,oy=grid.info.origin.position.x,grid.info.origin.position.y
            extent=[ox,ox+w*grid.info.resolution,oy,oy+h*grid.info.resolution]
            fig,ax=plt.subplots(figsize=(7,8))
            ax.imshow(a,origin='lower',extent=extent,cmap='RdYlGn_r',vmin=0,vmax=100,alpha=.35)
            for index,g in enumerate(goals):
                file=out/f'leg_{index+1}.csv'
                if file.exists():
                    p=np.loadtxt(file,delimiter=',',skiprows=1)
                    ax.plot(p[:,0],p[:,1],label=f'Hybrid leg {index+1}')
                ax.scatter(*g['xy'],marker='x',c='black')
            t=np.asarray(samples);ax.plot(t[:,1],t[:,2],'b--',label='Measured vessel track')
            ax.set_xlim(min(t[:,1].min(),min(g['xy'][0] for g in goals))-20,
                max(t[:,1].max(),max(g['xy'][0] for g in goals))+20)
            ax.set_ylim(min(t[:,2].min(),min(g['xy'][1] for g in goals))-10,
                max(t[:,2].max(),max(g['xy'][1] for g in goals))+10)
            ax.set_aspect('equal');ax.grid(alpha=.2)
            ax.set_xlabel('ENU x (m)');ax.set_ylabel('ENU y (m)');ax.legend()
            ax.set_title('Actual river chart: forward goal, right-facing heading' if
                os.environ.get('RIVER_SPECIAL_GOAL')=='right' else
                'Actual river chart: intermediate waypoint handoff')
            fig.tight_layout();fig.savefig(out/'river_handoff.png',dpi=160);plt.close(fig)
        for p in reversed(children):
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in reversed(children):
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for f in logs:f.close()
        node.destroy_node();rclpy.shutdown()
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
