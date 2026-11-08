#!/usr/bin/env python3
"""Run the real ROS closed loop in an isolated DDS domain and report measured errors."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def distances(xy, reference):
    import numpy as np
    a=np.asarray(reference[:-1]); d=np.diff(np.asarray(reference),axis=0)
    denom=np.maximum((d*d).sum(axis=1),1e-16)
    output=[]
    for start in range(0,len(xy),64):
        delta=xy[start:start+64,None,:]-a[None,:,:]
        t=np.clip((delta*d[None,:,:]).sum(axis=2)/denom,0,1)
        squared=((delta-t[:,:,None]*d[None,:,:])**2).sum(axis=2)
        output.extend(np.sqrt(squared.min(axis=1)))
    return np.asarray(output)


def statistics(values):
    import numpy as np
    a=np.asarray(values); a=a[np.isfinite(a)]
    if not len(a): return {'count':0}
    return {'count':len(a),'rms_m':float(np.sqrt(np.mean(a*a))),
            'mean_m':float(a.mean()),'p95_m':float(np.percentile(a,95)),'maximum_m':float(a.max())}


def report(out, reason):
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    summary=json.loads((out/'replay_summary.json').read_text()) if (out/'replay_summary.json').exists() else {}
    metadata=json.loads((out/'route_metadata.json').read_text())
    trace=list(csv.DictReader(open(out/'tracking.csv')))
    if not trace: raise RuntimeError('No odometry samples recorded')
    t=np.asarray([float(r['elapsed_s']) for r in trace])
    xy=np.asarray([[float(r['x']),float(r['y'])] for r in trace])
    speed=np.asarray([math.hypot(float(r['u']),float(r['v'])) for r in trace])
    raw=np.loadtxt(out/'source_enu.csv',delimiter=',',skiprows=1)
    waypoints=np.loadtxt(out/'waypoints_30.csv',delimiter=',',skiprows=1)
    raw_error=distances(xy,raw); sparse_error=distances(xy,waypoints[:,2:4])
    hybrid_error=np.asarray([float(r['distance_to_hybrid_path_m']) if r['distance_to_hybrid_path_m'] else np.nan for r in trace])
    events=[json.loads(line) for line in (out/'waypoint_events.jsonl').read_text().splitlines()]
    paths=[json.loads(line) for line in (out/'local_paths.jsonl').read_text().splitlines()]
    final_distance=float(np.linalg.norm(xy[-1]-waypoints[-1,2:4]))
    final_yaw_error=abs(math.atan2(math.sin(float(trace[-1]['yaw'])-waypoints[-1,4]),
                                  math.cos(float(trace[-1]['yaw'])-waypoints[-1,4])))
    planned_indices=set()
    final_path_endpoint=None
    for p in paths:
        if len(p['points'])<2: continue
        endpoint=np.asarray(p['points'][-1]); error=np.linalg.norm(waypoints[:,2:4]-endpoint,axis=1)
        if error.min()<1.:
            planned_indices.add(int(error.argmin()))
            if int(error.argmin())==len(waypoints)-1:
                final_path_endpoint=endpoint
    # Exclude prolonged parked waiting from tracking metrics, including when a
    # strict arrival criterion was not reached. Preserve every original sample.
    near_hold=None; settled_at=None
    for row,stamp,ground_speed in zip(trace,t,speed):
        error=abs(math.atan2(math.sin(float(row['yaw'])-waypoints[-1,4]),math.cos(float(row['yaw'])-waypoints[-1,4])))
        near=(int(row['active_index'])==len(waypoints)-1 and
              float(row['distance_to_goal_m'])<=summary.get('advance_radius_m',3.) and
              ground_speed<=summary.get('final_speed_threshold_mps',.15) and error<=math.pi/6)
        if near:
            if near_hold is None: near_hold=stamp
            if stamp-near_hold>=2.:
                settled_at=float(stamp); break
        else: near_hold=None
    mission_end=settled_at if settled_at is not None else float(t[-1])
    mission_start=summary.get('started_elapsed_s') or 0.
    window=(t>=mission_start)&(t<=mission_end)
    run_config=json.loads((out/'run_config.json').read_text()) if (out/'run_config.json').exists() else {}
    measured={'reason':reason,'replay':summary,'source':metadata,'run_config':run_config,
        'terminal_settled_near_goal':settled_at is not None,
        'tracking_metric_window_s':[mission_start,mission_end],
        'recorded_duration_s':float(t[-1]),
        'source_raw_track_error':statistics(raw_error[window]),
        'sparse_route_error':statistics(sparse_error[window]),
        'hybrid_reference_error_when_available':statistics(hybrid_error[window]),
        'final_position_error_m':final_distance,'final_heading_error_deg':math.degrees(final_yaw_error),
        'final_speed_mps':float(speed[-1]),'maximum_speed_mps':float(speed.max()),
        'maximum_surge_speed_mps':max(float(r['u']) for r in trace),
        'final_hybrid_endpoint_error_m':float(np.linalg.norm(final_path_endpoint-waypoints[-1,2:4])) if final_path_endpoint is not None else None,
        'final_vessel_distance_to_hybrid_endpoint_m':float(np.linalg.norm(xy[-1]-final_path_endpoint)) if final_path_endpoint is not None else None,
        'measured_track_length_m':float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),
        'goals_with_published_path_indices':sorted(planned_indices),
        'maximum_intermediate_switch_distance_m':max((e['distance_m'] for e in events if e['event']=='reached'),default=None),
        'limitations':['Simulation of the existing six-state model; not a real-vessel validation.',
          '30 targets are equal arc samples; straight chords omit some recorded curvature.',
          'Intermediate targets advance within 3m; the last requires position, speed and heading hold.',
          'Source-time/speed profile is not replayed; existing Hybrid/MPC speed policy is used.',
          'Raw-track distance is the global nearest polyline distance and can understate progress errors at crossings.']}
    (out/'evaluation.json').write_text(json.dumps(measured,ensure_ascii=False,indent=2))
    with open(out/'tracking_errors.csv','w') as f:
        writer=csv.writer(f); writer.writerow(['elapsed_s','raw_track_error_m','sparse_route_error_m','hybrid_reference_error_m'])
        writer.writerows(zip(t,raw_error,sparse_error,hybrid_error))
    fig,axes=plt.subplots(1,2,figsize=(14,8),gridspec_kw={'width_ratios':[1,1.1]})
    ax=axes[0]; ax.plot(raw[:,0],raw[:,1],color='0.55',lw=2,label='Recorded INS (ENU)')
    ax.plot(waypoints[:,2],waypoints[:,3],'o--',color='#df8a22',ms=3,lw=1,label='30 equal-arc targets')
    shown=set()
    for p in paths:
        if len(p['points'])<2 or p['active_index'] in shown: continue
        shown.add(p['active_index']); a=np.asarray(p['points'])
        ax.plot(a[:,0],a[:,1],color='#159547',alpha=.5,lw=.8,label='Hybrid local paths' if len(shown)==1 else None)
    ax.plot(xy[:,0],xy[:,1],color='#2865d8',lw=1.4,label='MPC + ship model')
    for i,row in enumerate(waypoints): ax.annotate(str(i+1),(row[2],row[3]),fontsize=7,xytext=(3,3),textcoords='offset points')
    ax.scatter([0],[0],s=60,marker='*',color='black',label='First INS point / origin')
    ax.set_aspect('equal'); ax.set_xlabel('East (m)'); ax.set_ylabel('North (m)'); ax.grid(alpha=.25); ax.legend(fontsize=8)
    axes[1].plot(t[window],raw_error[window],label='Distance to recorded track',alpha=.8)
    axes[1].plot(t[window],sparse_error[window],label='Distance to 30-point route',alpha=.8)
    axes[1].plot(t[window],hybrid_error[window],label='Distance to active Hybrid path',alpha=.8)
    axes[1].set_xlabel('Wall elapsed (s)'); axes[1].set_ylabel('Position error (m)'); axes[1].grid(alpha=.25)
    axes[1].legend(fontsize=8)
    complete=bool(summary.get('completed'))
    result_label='STRICT ARRIVAL PASSED' if complete else ('TERMINAL STOPPED; STRICT ARRIVAL FAILED' if settled_at is not None else 'INCOMPLETE')
    fig.suptitle(f'INS → 30 goals → Hybrid A* → MPC | max speed {run_config.get("max_speed_mps",4.0):.1f} m/s\n{result_label}\n'
                 f'Raw-track RMS {measured["source_raw_track_error"]["rms_m"]:.2f} m; '
                 f'Hybrid RMS {measured["hybrid_reference_error_when_available"].get("rms_m",float("nan")):.2f} m; '
                 f'final position error {final_distance:.2f} m')
    fig.tight_layout(); fig.savefig(out/'tracking_overview.png',dpi=180); plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True)
    axes[0].plot(t[window],speed[window],label='Ground speed (includes sway)')
    axes[0].plot(t[window],np.asarray([float(r['u']) for r in trace])[window],label='Surge speed u',alpha=.7)
    axes[0].axhline(run_config.get('max_speed_mps',4.0),color='black',ls='--',lw=.8,label='Configured surge limit')
    axes[0].set_ylabel('Speed (m/s)'); axes[0].grid(alpha=.25); axes[0].legend(fontsize=8)
    axes[1].step(t[window],np.asarray([int(r['active_index'])+1 for r in trace])[window],where='post')
    axes[1].set_ylabel('Active target (1-based)'); axes[1].set_xlabel('Wall elapsed (s)'); axes[1].grid(alpha=.25)
    fig.tight_layout(); fig.savefig(out/'speed_and_progress.png',dpi=160); plt.close(fig)
    result_text='完成全部30点并通过严格终点验收' if complete else ('已执行到第30点并停稳，严格终点位置验收未通过' if settled_at is not None else '尚未完成路线')
    lines=['# INS 数据集闭环测试', '', f'结果：{result_text}；结束原因：{reason}。', '',
           f'记录轨迹 {metadata["raw_length_m"]:.2f} m；30点沿源轨迹等弧长间隔 {metadata["sample_arc_spacing_m"]:.2f} m。', '',
           f'本次启动速度上限：{run_config.get("max_speed_mps","原始4.0")} m/s；速度覆盖仅在回放启动中生效。', '',
           '| 指标 | 数值 |','|---|---|',f'| 完成目标点 | {summary.get("waypoints_reached",0)}/30 |',
           f'| 执行至末点停稳耗时（或未停稳时的记录长度） | {mission_end-mission_start:.1f} s |',
           f'| 指标统计区间（启动后秒数） | {mission_start:.2f}–{mission_end:.2f} s |',
           f'| 完整记录长度（包含停稳后等待） | {t[-1]:.1f} s |',
           f'| 原始数据轨迹距离 RMS / P95 / 最大值 | {measured["source_raw_track_error"]["rms_m"]:.3f} / {measured["source_raw_track_error"]["p95_m"]:.3f} / {measured["source_raw_track_error"]["maximum_m"]:.3f} m |',
           f'| Hybrid路径跟踪距离 RMS / P95 / 最大值 | {measured["hybrid_reference_error_when_available"].get("rms_m",float("nan")):.3f} / {measured["hybrid_reference_error_when_available"].get("p95_m",float("nan")):.3f} / {measured["hybrid_reference_error_when_available"].get("maximum_m",float("nan")):.3f} m |',
           f'| 终点位置误差 | {final_distance:.3f} m |',f'| 终点艏向误差 | {math.degrees(final_yaw_error):.2f}° |',
           f'| Hybrid末端距最终数据目标 | {measured["final_hybrid_endpoint_error_m"]:.3f} m |' if final_path_endpoint is not None else '',
           f'| 船停稳位置距Hybrid末端 | {measured["final_vessel_distance_to_hybrid_endpoint_m"]:.3f} m |' if final_path_endpoint is not None else '',
           f'| 终点速度 | {speed[-1]:.3f} m/s |','',
           f'最高实测纵向速度 {measured["maximum_surge_speed_mps"]:.3f} m/s；最高地面合速度 {speed.max():.3f} m/s（包含侧滑）。','',
           '![轨迹与误差](tracking_overview.png)','', '![速度与航点进度](speed_and_progress.png)','',
           '中间点在3m内交接，不要求逐点停车；末点要求0.6m内、船速≤0.15m/s、艏向误差≤30°持续2秒。',
           '跟踪误差只统计任务开始至末点附近停稳2秒的区间，避免长时间静止等待拉低RMS；全部原始记录仍保留。',
           '原始CSV未平滑或去除跳点。仅使用经纬度路线，不重放采集时间及实测速度；除启动速度覆盖外，控制器沿用原配置。',
           '终点误差需区分规划末端与数据目标的偏移、MPC相对局部路径末端的停车误差。现有动态Hybrid搜索允许目标位置1m内结束；本次保留原算法。',
           '原始轨迹距离采用整条折线最近距离，交叉/近邻路段可能使该误差偏低，应结合编号航点进度和完整轨迹查看。',
           '本结果来自原六状态船模闭环仿真，不能推断实船控制性能。','']
    (out/'REPORT.md').write_text('\n'.join(lines))
    print(json.dumps(measured,ensure_ascii=False,indent=2),flush=True)
    return complete


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True)
    parser.add_argument('--timeout',type=float,default=1800)
    parser.add_argument('--max-speed',type=float,default=1.2)
    parser.add_argument('--domain',type=int,default=186)
    parser.add_argument('--report-only',action='store_true')
    args=parser.parse_args(); out=Path(args.output).expanduser().resolve(); out.mkdir(parents=True,exist_ok=True)
    if args.report_only:
        prior=out/'evaluation.json'
        reason=json.loads(prior.read_text()).get('reason') if prior.exists() else 'report regenerated from recorded data'
        report(out,reason); return
    if (out/'tracking.csv').exists(): raise RuntimeError('Use a fresh output directory to preserve earlier runs')
    (out/'run_config.json').write_text(json.dumps({'max_speed_mps':args.max_speed,
        'timeout_s':args.timeout,'dds_domain':args.domain},indent=2))
    env=os.environ.copy(); env.update(ROS_DOMAIN_ID=str(args.domain),ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(out/'ros_log'))
    reason='timeout'; start=time.monotonic(); previous=None
    with open(out/'launch.log','w') as log:
        child=subprocess.Popen(['ros2','launch','ins_path_replay','ins_replay.launch.py','use_rviz:=false',
                                f'max_speed:={args.max_speed}',
                                f'output_dir:={out}'],env=env,stdout=log,stderr=log,start_new_session=True)
        try:
            while time.monotonic()-start<args.timeout:
                if child.poll() is not None:
                    reason=f'launch exited ({child.returncode})'; break
                summary_path=out/'replay_summary.json'
                if summary_path.exists():
                    status=json.loads(summary_path.read_text())
                    key=(status.get('active_index'),status.get('completed'))
                    if key!=previous:
                        print(json.dumps(status),flush=True); previous=key
                    if status.get('completed'):
                        reason='completed'; time.sleep(2.); break
                    if status.get('terminal_settled'):
                        reason='terminal stopped near goal, strict final radius NOT met'; break
                time.sleep(1.)
        except KeyboardInterrupt: reason='interrupted'
        finally:
            if child.poll() is None: os.killpg(child.pid,signal.SIGINT)
            try: child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL); child.wait()
    if not (out/'route_metadata.json').exists():
        raise RuntimeError(f'Pipeline failed before loading route; inspect {out}/launch.log')
    complete=report(out,reason)
    raise SystemExit(0 if complete else 1)


if __name__=='__main__': main()
