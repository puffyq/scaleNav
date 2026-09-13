"""Plot supplementary 0-to-140 m trajectories with logged point-cloud context."""
from pathlib import Path
import json, numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa
ROOT=Path(__file__).resolve().parents[2]
SESSIONS={"TopoGuide (ScaleNav)":ROOT/'log_scalenav/session_20260912_183244_170',
          'FAR Planner':ROOT/'log_scalenav/session_20260912_183411_823'}
OUT=ROOT/'paper/scalenav/pics/experiments/supplementary_0_140'

def odom(s):
 p=[]
 for line in open(s/'index.jsonl'):
  try: d=json.loads(line)
  except json.JSONDecodeError: continue
  if d.get('kind')=='odom': p.append(d['data']['position'])
 return np.asarray(p,float)
def cloud(s, stride=40):
 od=[]; ts=[]
 for line in open(s/'index.jsonl'):
  try:d=json.loads(line)
  except json.JSONDecodeError: continue
  if d.get('kind')=='odom': ts.append(d['stamp_ns']); od.append(d['data'])
 od=np.asarray([x['position'] for x in od]); ts=np.asarray(ts)
 pts=[]
 for line in open(s/'index.jsonl'):
  try:d=json.loads(line)
  except json.JSONDecodeError: continue
  if d.get('kind')!='pointcloud' or not d.get('file') or d['data'].get('stored_points',0)==0: continue
  try:
   f=s/d['file']
   lines=f.read_text().splitlines(); start=next(i for i,x in enumerate(lines) if x.startswith('DATA'))+1
   if start >= len(lines): continue
   a=np.loadtxt(lines[start:],dtype=float)
   if a.ndim==1:a=a[None,:]
   if a.size and a.ndim==2 and a.shape[1] >= 3:
    i=np.argmin(abs(ts-d['stamp_ns'])); q=od[i]; yaw=2*np.arctan2(float(0.70710678),float(0.70710678))
    c,si=np.cos(yaw),np.sin(yaw); z=a[::stride,:3]; x,y=z[:,0].copy(),z[:,1].copy(); z[:,0]=c*x-si*y+q[0]; z[:,1]=si*x+c*y+q[1]; z[:,2]+=q[2]; pts.append(z)
  except Exception: pass
 return np.vstack(pts) if pts else np.empty((0,3))
plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.2})
fig=plt.figure(figsize=(11,4.5)); ax=fig.add_subplot(1,2,1); ax3=fig.add_subplot(1,2,2,projection='3d')
colors=['#d62728','#1f77b4']
for (label,s),c in zip(SESSIONS.items(),colors):
 t=odom(s); ax.plot(t[:,0],t[:,1],lw=2,label=label,color=c); ax.scatter(t[0,0],t[0,1],c='k',s=25,zorder=3); ax.scatter(t[-1,0],t[-1,1],c=c,marker='*',s=70,zorder=3)
 p=cloud(s)
 if len(p): ax3.scatter(p[:,0],p[:,1],p[:,2],s=.15,alpha=.12,color='gray',rasterized=True)
 ax3.plot(t[:,0],t[:,1],t[:,2],lw=2,color=c,label=label)
ax.set(xlabel='x (m)',ylabel='y (m)',title='Trajectory comparison'); ax.legend(frameon=True)
ax.set_aspect('equal', adjustable='box')
ax3.set(xlabel='x (m)',ylabel='y (m)',zlabel='z (m)',title='Trajectory over logged point cloud'); ax3.view_init(elev=25,azim=-65); ax3.legend(loc='upper left')
fig.tight_layout(); OUT.mkdir(parents=True,exist_ok=True)
fig.savefig(OUT/'trajectory_pointcloud_comparison.pdf',bbox_inches='tight'); fig.savefig(OUT/'trajectory_pointcloud_comparison.png',dpi=300,bbox_inches='tight'); print(OUT)
