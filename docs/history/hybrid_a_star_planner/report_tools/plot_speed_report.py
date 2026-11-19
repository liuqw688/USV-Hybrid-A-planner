import os
os.environ['MPLCONFIGDIR']='/tmp/usv_plot_config'
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import csv
root=Path('artifacts/hybrid_speed_update')
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name(),'axes.unicode_minus':False,'figure.dpi':150,'font.size':10})
geometry=json.load(open(root/'chart_geometry.json'))
def route(ax,name):
 for phase,color,label in [('before','#ed8b2c','修改前'),('after','#146ba5','修改后')]:
  f=root/phase/(name+'.csv')
  if not f.exists():continue
  p=np.genfromtxt(f,delimiter=',',names=True)
  if p.size:
   ax.plot(p['x'],p['y'],color=color,lw=2,label=label,ls='--' if phase=='before' else '-')
   if phase=='after':
    ax.scatter(p['x'][0],p['y'][0],color='#258352',s=30,zorder=5,label='起点')
    ax.scatter(p['x'][-1],p['y'][-1],color='#bb3c3c',marker='*',s=85,zorder=5,label='目标')
 ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('x / m');ax.set_ylabel('y / m')
fig,axes=plt.subplots(2,1,figsize=(12,7))
for ax,name,title in zip(axes,['chart_straight','chart_bend'],['真实海图：航道直航','真实海图：航道缓弯']):
 for l in geometry['land']:ax.add_patch(Polygon(l,color='#a39276',alpha=.8))
 for l in geometry['lanes']:
  color='#c5eacd' if 50<l['orient']<110 else '#eec9c5'
  ax.add_patch(Polygon(l['points'],color=color,alpha=.65))
 route(ax,name);ax.set_xlim(-100,225);ax.set_ylim(45,110);ax.set_title(title+'（海图 scale=15，与项目 launch 一致）');ax.legend()
fig.tight_layout();fig.savefig(root/'chart_comparison.png');plt.close(fig)
fig,axes=plt.subplots(2,2,figsize=(12,9))
for ax,name,title in zip(axes.flat,['open_bend','open_uturn','narrow_uturn','small_j_turn'],['开阔水域缓弯','开阔水域掉头（允许安全借道）','窄水域掉头（宽度28m）','小角度转弯']):
 x=np.linspace(-100,100,200);center=np.tan(.349)*np.maximum(x,0)
 ax.fill_between(x,center-12,center+12,color='#c5eacd',alpha=.6,label='主航道')
 route(ax,name)
 p=np.genfromtxt(root/'after'/(name+'.csv'),delimiter=',',names=True)
 if p.size:
  ax.set_xlim(min(-40,float(np.min(p['x']))-5),max(10,float(np.max(p['x']))+5))
  ax.set_ylim(float(np.min(p['y']))-5,float(np.max(p['y']))+5)
 if name=='narrow_uturn':ax.axhline(14,c='#555',lw=2);ax.axhline(-14,c='#555',lw=2);ax.set_ylim(-16,16)
 ax.set_title(title);ax.legend(fontsize=9)
fig.tight_layout();fig.savefig(root/'open_water_comparison.png');plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(12,4.8))
for ax,stem,title in zip(axes,['c_-30_10_u4','c_-60_20_u4'],['近距离单向掉头','远距离掉头：恢复更高参考速度']):
 route(ax,stem);ax.set_title(title);ax.legend()
fig.tight_layout();fig.savefig(root/'turn_comparison.png');plt.close(fig)
records={phase:{r['scenario']:r for r in csv.DictReader(open(root/phase/'scenarios.csv'))} for phase in ['before','after']}
names=['chart_straight','chart_bend','open_bend','open_uturn','narrow_uturn','small_j_turn']
labels=['海图直航','海图缓弯','开阔缓弯','开阔掉头','窄水掉头','小转弯'];x=np.arange(len(names))
fig,axes=plt.subplots(1,2,figsize=(12,4.7))
for phase,color,delta,label in [('before','#ed8b2c',-.18,'修改前'),('after','#146ba5',.18,'修改后')]:
 axes[0].bar(x+delta,[float(records[phase][n]['speed']) for n in names],.36,label=label,color=color)
 axes[1].bar(x+delta,[1000*float(records[phase][n]['seconds']) for n in names],.36,label=label,color=color)
for ax in axes:ax.set_xticks(x,labels,rotation=20);ax.grid(axis='y',alpha=.2);ax.legend()
axes[0].set_ylabel('规划参考速度 / m/s');axes[1].set_ylabel('单次算法耗时 / ms');axes[1].set_title('同机、同为 -O2 的前后算法对比')
fig.tight_layout();fig.savefig(root/'metrics_comparison.png');plt.close(fig)

if (root/'after/recovery_fast.csv').exists():
 fig,ax=plt.subplots(figsize=(10,5))
 for name,color,label in [('recovery_turn','#ed8b2c','低速转向规划'),('recovery_fast','#146ba5','转向后最大航速重规划')]:
  p=np.genfromtxt(root/'after'/(name+'.csv'),delimiter=',',names=True)
  ax.plot(p['x'],p['y'],color=color,lw=2,label=label)
  ax.scatter(p['x'][0],p['y'][0],color=color,s=40,zorder=4)
 ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend();ax.set_xlabel('x / m');ax.set_ylabel('y / m')
 ax.set_title('真实ROS节点：注入转向后稳定船位，参考航速恢复至4 m/s')
 fig.tight_layout();fig.savefig(root/'recovery_comparison.png');plt.close(fig)
