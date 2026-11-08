#!/usr/bin/env python3
"""Repeatable 20m / 15deg Hybrid pose matrix; offline, without moving ROS ships.

Rows: target bearing relative to initial bow. Columns: target heading in the
same initial-bow frame. Positive angles are counterclockwise (port).
The executable uses production planner code; all output is written to --output.
"""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
import argparse, csv, hashlib, json, math, os, subprocess, time

def reversal_sensitivity(path):
    """Same spatial circle metric, with 1/3/5 degree lobe thresholds."""
    import numpy as np
    with path.open() as f:
        rows=list(csv.DictReader(f))
    if len(rows)<3:return {f'reversals_{k}deg':0 for k in (1,3,5)}
    xy=np.array([[float(r['x']),float(r['y'])] for r in rows])
    arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
    sample=np.r_[np.arange(0,arc[-1],.25),arc[-1]]
    points=np.stack([np.interp(sample,arc,xy[:,k]) for k in (0,1)],axis=1)
    a=np.diff(points,axis=0)[:-1];b=np.diff(points,axis=0)[1:]
    ab=np.linalg.norm(a,axis=1);bc=np.linalg.norm(b,axis=1);ac=np.linalg.norm(a+b,axis=1)
    curv=2*(a[:,0]*b[:,1]-a[:,1]*b[:,0])/np.maximum(1e-12,ab*bc*ac)
    ds=(ab+bc)*.5;lobes=[];sign=0;angle=length=0
    for k,s in zip(curv,ds):
        new=1 if k>.002 else -1 if k<-.002 else 0
        if new and new!=sign:
            lobes.append((sign,abs(angle),length));sign=new;angle=length=0
        if new:angle+=k*s;length+=s
    lobes.append((sign,abs(angle),length))
    result={}
    for threshold in (1,3,5):
        signs=[s for s,a,l in lobes if a>=math.radians(threshold) and l>=.5]
        result[f'reversals_{threshold}deg']=sum(a!=b for a,b in zip(signs,signs[1:]))
    result['turn_lobes_deg']=[dict(sign=s,angle_deg=round(math.degrees(a),3),length_m=round(l,3)) for s,a,l in lobes if l>=.5]
    result['sample_peak_curvature']=float(np.max(np.abs(curv))) if len(curv) else 0
    return result

def run(args):
    output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=True)
    probe=Path(args.probe).resolve()
    manifest=dict(probe_sha256=hashlib.sha256(probe.read_bytes()).hexdigest(),
                  environments=args.environments,initial_u=args.u,initial_v=args.v,
                  initial_r=args.r,budget=args.budget,optimize=not args.raw,
                  distance_m=20,angle_step_deg=15,heading_tolerance_deg=30,
                  angle_convention='0=initial bow; positive=port/CCW',jobs=args.jobs)
    config=output/'manifest.json'
    if config.exists() and json.loads(config.read_text())!=manifest:
        raise RuntimeError('Output contains a different revision/configuration; use a new output directory.')
    config.write_text(json.dumps(manifest,indent=2))
    for env in args.environments:(output/env).mkdir(exist_ok=True)
    tasks=[(env,bearing,heading) for env in args.environments for bearing in range(0,360,15) for heading in range(0,360,15)]
    def case(item):
        env,bearing,heading=item;stem=output/env/f'b{bearing:03d}_h{heading:03d}'
        if not stem.with_suffix('.json').exists():
            with stem.with_suffix('.log').open('w') as log:
                subprocess.run([str(probe),str(stem),env,str(bearing),str(heading),
                                str(args.u),str(args.v),str(args.r),str(args.budget),str(int(not args.raw))],
                               stdout=log,stderr=log,timeout=args.budget+10,check=True)
        data=json.loads(stem.with_suffix('.json').read_text())
        data.update(reversal_sensitivity(stem.with_suffix('.csv')))
        data.update(environment=env,bearing=bearing,heading=heading,stem=f'{env}/{stem.name}')
        if data['success']:
            assert data['safe'] and data['position_error']<1.01 and data['heading_error']<=math.pi/6+1e-6,data
            assert data['peak']<=data['raw_peak']+1e-6,data
            assert data['reversals']<=data['raw_reversals'],data
        return data
    rows=[];started=time.monotonic()
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        for future in as_completed([pool.submit(case,c) for c in tasks]):
            rows.append(future.result())
            if len(rows)%48==0:print(f'{len(rows)}/{len(tasks)} cases, {time.monotonic()-started:.0f}s',flush=True)
    rows.sort(key=lambda r:(args.environments.index(r['environment']),r['bearing'],r['heading']))
    summary={}
    for env in args.environments:
        group=[r for r in rows if r['environment']==env];ok=[r for r in group if r['success']]
        summary[env]=dict(total=len(group),success=len(ok),failed=len(group)-len(ok),
                          multiple_reversals=sum(r['reversals']>1 for r in ok),
                          raw_multiple_reversals=sum(r['raw_reversals']>1 for r in ok),
                          maximum_reversals=max((r['reversals'] for r in ok),default=0),
                          maximum_seconds=max(r['seconds'] for r in group),
                          failure_attempted_lowest=sum(bool(r['attempted_speeds']) and min(r['attempted_speeds'])<=1.20001 for r in group if not r['success']))
    payload=dict(manifest=manifest,summary=summary,cases=rows)
    if args.compare:
        previous=json.loads(Path(args.compare).read_text());old={(r['environment'],r['bearing'],r['heading']):r for r in previous['cases']}
        pairs=[(r,old[(r['environment'],r['bearing'],r['heading'])]) for r in rows if (r['environment'],r['bearing'],r['heading']) in old]
        payload['comparison']=dict(paired=len(pairs),new_failures=[r['stem'] for r,b in pairs if b['success'] and not r['success']],
                                   new_successes=[r['stem'] for r,b in pairs if r['success'] and not b['success']],
                                   more_reversals=[r['stem'] for r,b in pairs if r['success'] and b['success'] and r['reversals']>b['reversals']])
    (output/'results.json').write_text(json.dumps(payload,indent=2))
    with (output/'summary.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print(json.dumps(summary,indent=2),flush=True)
    if args.render:render(output,payload)

def render(output,payload):
    os.environ.setdefault('MPLCONFIGDIR','/tmp/usv_matrix_matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    colors={0:'#15803d',1:'#2563eb',2:'#d97706'}
    def draw(ax,row,compact=False):
        env=row['environment'];bearing=row['bearing']*math.pi/180;heading=row['heading']*math.pi/180
        gx,gy=20*math.cos(bearing),20*math.sin(bearing)
        ax.axhline(0,color='#e5e7eb',lw=.4);ax.axvline(0,color='#e5e7eb',lw=.4)
        if env in ('main','opposite'):
            if env=='main':main=(-12,12);opposite=(12,36)
            else:main=(12,36);opposite=(-12,12)
            ax.axhspan(*main,color='#dbeafe',alpha=.65);ax.axhspan(*opposite,color='#fee2e2',alpha=.65)
            ax.axhspan(-100,-20,color='#555',alpha=.3);ax.axhspan(44,100,color='#555',alpha=.3)
        p=list(csv.DictReader((output/(row['stem']+'.csv')).open()))
        raw=list(csv.DictReader((output/(row['stem']+'_raw.csv')).open()))
        if raw:ax.plot([float(q['x']) for q in raw],[float(q['y']) for q in raw],color='#aaa',lw=.65,alpha=.7)
        if p:
            color=colors.get(row['reversals'],'#9333ea')
            ax.plot([float(q['x']) for q in p],[float(q['y']) for q in p],color=color,lw=1 if compact else 1.8)
            end=p[-1];actual_yaw=float(end['yaw'])
            ax.arrow(float(end['x']),float(end['y']),3*math.cos(actual_yaw),3*math.sin(actual_yaw),
                     width=.12,color='#0891b2',length_includes_head=True)
        ax.arrow(0,0,4,0,width=.2,color='#111',length_includes_head=True)
        ax.arrow(gx,gy,4*math.cos(heading),4*math.sin(heading),width=.2,color='#dc2626',length_includes_head=True)
        ax.plot([gx],[gy],'o',ms=2,color='#dc2626')
        xs=[0,gx]+[float(q['x']) for q in p];ys=[0,gy]+[float(q['y']) for q in p]
        bound=max(26,max(abs(v) for v in xs+ys)+4)
        ax.set_xlim(-bound,bound);ax.set_ylim(-bound,bound);ax.set_aspect('equal')
        if row['success']:
            text=f"R={row['reversals']}  k={row['peak']:.3f}  L={row['length']:.1f}"
            if 'reversals_1deg' in row:text+=f"\nR(1/3/5deg)={row['reversals_1deg']}/{row['reversals']}/{row['reversals_5deg']}"
            if not compact:text+=f"\nEndpoint error: {row['position_error']:.2f}m / {math.degrees(row['heading_error']):.1f}deg"
        else:text='NO PATH (within budget)'
        ax.set_title(f"B {row['bearing']} / H {row['heading']}\n{text}",fontsize=4 if compact else 8,color='#111' if row['success'] else '#dc2626')
        if compact:ax.set_xticks([]);ax.set_yticks([])
        else:ax.tick_params(labelsize=6);ax.grid(alpha=.15)
    detail,ax=plt.subplots(figsize=(4,4))
    for i,row in enumerate(payload['cases']):
        ax.clear();draw(ax,row);detail.tight_layout()
        detail.savefig(output/(row['stem']+'.svg'))
        if (i+1)%288==0:print(f"Rendered {i+1} individual trajectories",flush=True)
    plt.close(detail)
    for env in payload['manifest']['environments']:
        rows=[r for r in payload['cases'] if r['environment']==env]
        fig,axes=plt.subplots(24,24,figsize=(48,48))
        for ax,row in zip(axes.flat,rows):draw(ax,row,True)
        for c in range(24):axes[0,c].annotate(f'H {c*15} deg',(.5,1.5),xycoords='axes fraction',ha='center',fontsize=8)
        for r in range(24):axes[r,0].annotate(f'B {r*15} deg',(-.25,.5),xycoords='axes fraction',va='center',ha='right',fontsize=8)
        fig.suptitle(f'{env}: ROW=target bearing, COLUMN=target heading; 20m, step=15deg\nGreen R0, blue R1, orange R2, purple R>=3, grey=raw; each tile uses its own equal-axis scale',fontsize=20)
        fig.subplots_adjust(left=.045,right=.995,bottom=.01,top=.975,wspace=.12,hspace=.45)
        fig.savefig(output/f'matrix_{env}.png',dpi=150);plt.close(fig)
        with PdfPages(output/f'trajectories_{env}.pdf') as pdf:
            for r in range(24):
                fig,axes=plt.subplots(4,6,figsize=(17,12))
                for ax,row in zip(axes.flat,rows[r*24:(r+1)*24]):draw(ax,row)
                fig.suptitle(f'{env} | target bearing {r*15} deg | headings 0..345 deg | distance 20m',fontsize=14)
                fig.tight_layout(rect=(0,0,1,.96));pdf.savefig(fig);plt.close(fig)
        print(f'{env} full matrix and 24-page PDF complete',flush=True)
    html=['<!doctype html><meta charset="utf-8"><title>20m 姿态矩阵</title><style>body{font-family:sans-serif}table{border-collapse:collapse}th{position:sticky;top:0;background:#fff;z-index:1}td,th{border:1px solid #ddd;padding:3px}img{width:220px;height:220px}td:first-child{position:sticky;left:0;background:#fff}summary{font-size:20px;padding:12px;cursor:pointer}</style>',
          '<h1>20米／15°姿态矩阵</h1><p>行＝目标方位，列＝目标艏向；0°沿初始船艏，正角度向左。黑箭头：起船方向；红箭头：要求的目标姿态；青色箭头：轨迹输出的终点参考姿态。终点艏向容差为30°，所以要求姿态与输出参考可能不同，地面轨迹切线也不等于船艏。灰线：原积分轨迹。R：显著曲率反向次数；k：最大地面曲率(1/m)；L：长度(m)。每图等比例，范围自动适配，跨图大小不能直接比较半径。点图可看完整矢量图。</p><p>主航道蓝底，对向红底，岸侧灰底。默认R按3°/0.5米转向段统计；同时给出1°/3°/5°三种门槛，避免把微小摆动隐去。矩阵淡黄格表示1°门槛下仍有多次反向，橙色轨迹表示3°门槛下有2次，紫色表示更多。仅规划测试，不等同MPC闭环到达。</p>']
    for env in payload['manifest']['environments']:
        html.append(f'<details open><summary>{env} {payload["summary"][env]}</summary><a href="matrix_{env}.png">全矩阵PNG</a> · <a href="trajectories_{env}.pdf">24页详细PDF</a><table><tr><th>方位\\姿态</th>')
        html.extend(f'<th>{h}°</th>' for h in range(0,360,15));html.append('</tr>')
        for bearing in range(0,360,15):
            html.append(f'<tr><td>{bearing}°</td>')
            for row in [r for r in payload['cases'] if r['environment']==env and r['bearing']==bearing]:
                style=' style="background:#fef3c7"' if row.get('reversals_1deg',row['reversals'])>1 else ''
                html.append(f'<td{style}><a href="{row["stem"]}.svg"><img loading="lazy" src="{row["stem"]}.svg" alt="{row["stem"]}"></a></td>')
            html.append('</tr>')
        html.append('</table></details>')
    (output/'index.html').write_text('\n'.join(html))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--probe',required=True);p.add_argument('--output',required=True)
    p.add_argument('--environments',nargs='+',choices=['open','main','opposite','chart_main','chart_opposite'],default=['open','main','opposite'])
    p.add_argument('--u',type=float,default=4);p.add_argument('--v',type=float,default=0);p.add_argument('--r',type=float,default=0)
    p.add_argument('--budget',type=float,default=3);p.add_argument('--jobs',type=int,default=2)
    p.add_argument('--raw',action='store_true');p.add_argument('--render',action='store_true');p.add_argument('--render-only',action='store_true')
    p.add_argument('--compare')
    args=p.parse_args()
    if args.render_only:
        output=Path(args.output).resolve();payload=json.loads((output/'results.json').read_text())
        for row in payload['cases']:row.update(reversal_sensitivity(output/(row['stem']+'.csv')))
        (output/'results.json').write_text(json.dumps(payload,indent=2))
        render(output,payload)
    else:run(args)
if __name__=='__main__':main()
