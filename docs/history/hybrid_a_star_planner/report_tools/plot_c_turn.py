#!/usr/bin/env python3
"""静态规划回归图；非RViz截图，也不是MPC闭环跟踪图。"""
import argparse
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

def read(path):
    with path.open() as stream: rows=list(csv.DictReader(stream))
    return {k:np.array([float(r[k]) for r in rows]) for k in ('x','y','yaw','curvature')}

def main():
    p=argparse.ArgumentParser();p.add_argument('--input-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    fig,axes=plt.subplots(1,2,figsize=(11,4.2),constrained_layout=True)
    for file,label,color in [('c_before.csv','Previous multi-arc','#d97732'),
        ('c_-30_10_u4.csv','Adaptive single-turn C','#148561')]:
        data=read(a.input_dir/file)
        axes[0].plot(data['x'],data['y'],label=label,color=color,lw=2)
        arc=np.cumsum(np.hypot(np.diff(data['x'],prepend=0),np.diff(data['y'],prepend=0)))
        yaw=np.unwrap(data['yaw'])
        axes[1].plot(arc,np.rad2deg(yaw),label=label,color=color,lw=2)
    axes[0].scatter([0,-30],[0,10],color='#333',zorder=5)
    axes[0].set_aspect('equal');axes[0].set_xlabel('x (m)');axes[0].set_ylabel('y (m)')
    axes[0].set_title('Same initial u=4 m/s and 180-degree goal')
    axes[1].set_xlabel('Path arc length (m)');axes[1].set_ylabel('Unwrapped hull heading (deg)')
    axes[1].axhline(180,color='#999',ls='--');axes[1].set_title('C turn: monotone heading, no counter-turn')
    for axis in axes:axis.grid(alpha=.25);axis.legend(fontsize=8)
    a.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=150)
if __name__=='__main__':main()
