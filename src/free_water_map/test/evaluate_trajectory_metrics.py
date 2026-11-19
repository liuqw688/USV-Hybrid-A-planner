#!/usr/bin/env python3
"""Offline first-spline metrics; curvature excludes low-speed portions."""
import json, math, sys
import numpy as np
from scipy.interpolate import BSpline


def evaluate(record):
    sp = record['spline']
    if sp is None:
        return {'available': False}
    p, k, order = np.asarray(sp['control_points']), np.asarray(sp['knots']), sp['order']
    f = BSpline(k,p,order)
    t = np.linspace(k[order],k[len(p)],2001)
    x, v, a, j = (f(t,n) for n in range(4))
    speed = np.hypot(v[:,0],v[:,1]); mask=speed>=0.3
    curve=np.abs(v[:,0]*a[:,1]-v[:,1]*a[:,0])/np.maximum(speed,0.3)**3
    valid_length=np.trapz(speed*mask,t)
    yaw=x[:,2]; r=v[:,2]; c=np.cos(yaw); s=np.sin(yaw)
    u=c*v[:,0]+s*v[:,1]; sway=-s*v[:,0]+c*v[:,1]
    du=c*a[:,0]+s*a[:,1]+r*sway
    dv=-s*a[:,0]+c*a[:,1]-r*u
    vcmd=(du+.433264281*u-.032263988)/.400421432
    wcmd=(a[:,2]-.166118168*sway+.544509453*r-.000626529)/.837536662
    sway_model=-.313324916*sway-.006741061*r-.407958500*wcmd-.000993619
    goal=np.asarray(record['goal']); yaw_error=math.atan2(math.sin(x[-1,2]-goal[2]),math.cos(x[-1,2]-goal[2]))
    return {'available':True,'duration_s':float(t[-1]-t[0]),'length_m':float(np.trapz(speed,t)),
       'start':x[0].tolist(),'end':x[-1].tolist(),'end_goal_distance_m':float(np.linalg.norm(x[-1,:2]-goal[:2])),
       'end_heading_error_rad':yaw_error,'start_end_chord_m':float(np.linalg.norm(x[-1,:2]-x[0,:2])),
       'curvature_rms_per_m':float(np.sqrt(np.trapz(curve**2*speed*mask,t)/valid_length)) if valid_length>0 else None,
       'curvature_max_per_m':float(curve[mask].max()) if mask.any() else None,
       'curvature_valid_length_fraction':float(valid_length/max(np.trapz(speed,t),1e-9)),
       'max_yaw_rate_rad_s':float(np.max(np.abs(r))),'max_yaw_acc_rad_s2':float(np.max(np.abs(a[:,2]))),
       'yaw_jerk_rms_rad_s3':float(np.sqrt(np.mean(j[:,2]**2))),
       'total_yaw_variation_rad':float(np.trapz(np.abs(r),t)),
       'vcmd_min_est':float(vcmd.min()),'vcmd_max_est':float(vcmd.max()),
       'wcmd_abs_max_est':float(np.max(np.abs(wcmd))),
       'sway_dynamics_rms_m_s2':float(np.sqrt(np.mean((dv-sway_model)**2))),
       'first_spline_latency_s':sp['elapsed_after_goal']}

if __name__=='__main__':
    results={}
    for path in sys.argv[1:]:
        with open(path) as file:results[path]=evaluate(json.load(file))
    print(json.dumps(results,indent=2,ensure_ascii=False))
