import sys,json,math
import numpy as np
from shapely.geometry import Polygon,Point
from shapely.ops import unary_union
from shapely.prepared import prep
sys.path.insert(0,'src/river_chart')
from river_chart.parser import load_chart
c=load_chart('src/river_chart/resource/ecdis_data.xml',scale=15)
lanes=[];lands=[]
for f in c.features:
 for g in f.geometry:
  if g.geometry_type in ('POLYGON','MULTIPOLYGON'):
   for ring in g.parts:
    if len(ring)<4: continue
    poly=Polygon(ring).buffer(0)
    if f.group=='TSSLPT': lanes.append((poly,float(f.attributes['ORIENT'])))
    if f.group=='LNDARE': lands.append(poly)
# Crop of the actual chart at the central connected eastbound lanes.
x0,y0,w,h=-110,30,350,100
main=unary_union([p for p,o in lanes if 50<o<110]);opp=unary_union([p for p,o in lanes if 230<o<290]);land=unary_union(lands)
pm,po,pl=prep(main),prep(opp),prep(land)
rows=[]
for y in range(h):
 for x in range(w):
  p=Point(x0+x+.5,y0+y+.5)
  raw=100 if pl.contains(p) else 0
  value=45
  if pm.contains(p):value=round(20*max(0,1-main.boundary.distance(p)/7))
  elif po.contains(p):value=75
  rows.append((raw,value))
with open('artifacts/hybrid_speed_update/chart_grid.txt','w') as f:
 f.write(f'{w} {h} {x0} {y0}\n');f.writelines(f'{a} {b}\n' for a,b in rows)
json.dump({'lanes':[{'points':list(p.exterior.coords),'orient':o} for p,o in lanes], 'land':[list(p.exterior.coords) for p in lands], 'crop':[x0,y0,w,h],'scale':15},open('artifacts/hybrid_speed_update/chart_geometry.json','w'))
for x,y in [(-70,64),(-20,70),(70,65),(120,65),(180,74),(200,80)]:print(x,y,main.contains(Point(x,y)),land.contains(Point(x,y)))
