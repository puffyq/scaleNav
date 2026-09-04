#!/usr/bin/env python3
"""Generate HTML viewer with graph, world pointcloud, sensor images and GCN scores."""
import argparse,base64,io,json,math,os
import torch
from PIL import Image
from train import FrontierGCN,session_groups

def img(path,heat=False):
 if not path or not os.path.isfile(path): return ''
 try:
  im=Image.open(path).convert('RGB')
  if heat: im=im.convert('L').convert('RGB')
  b=io.BytesIO();im.save(b,'PNG');return 'data:image/png;base64,'+base64.b64encode(b.getvalue()).decode()
 except: return ''
def pcd(path,limit=1200):
 out=[];on=False
 if path and os.path.isfile(path):
  for l in open(path,encoding='ascii',errors='ignore'):
   if l.lower().startswith('data'):on=True;continue
   if on and len(out)<limit:
    z=l.split()
    if len(z)>=3:
     try:out.append([float(z[0]),float(z[1]),float(z[2])])
     except:pass
 return out
def rot(q,v):
 x,y,z,w=q;vx,vy,vz=v;tx,ty,tz=2*(y*vz-z*vy),2*(z*vx-x*vz),2*(x*vy-y*vx)
 return [vx+w*tx+y*tz-z*ty,vy+w*ty+z*tx-x*tz,vz+w*tz+x*ty-y*tx]
HTML='''<!doctype html><meta charset="utf-8"><title>ScaleNav GCN 数据</title><style>body{margin:0;background:#10141a;color:#e8edf3;font:14px system-ui}header{padding:12px;border-bottom:1px solid #39434f}button,select{background:#202832;color:#fff;border:1px solid #536171;padding:6px;margin-right:6px}main{display:grid;grid-template-columns:1fr 350px;gap:12px;padding:12px}.p{background:#171d25;border:1px solid #39434f;padding:10px}.s{display:grid;grid-template-columns:1fr 1fr;gap:8px}.s img{width:100%}.st{display:grid;grid-template-columns:1fr 1fr;gap:6px}.st div{background:#202832;padding:7px}.l{color:#9eacbb;font-size:12px}.v{font-size:18px;font-weight:600}@media(max-width:850px){main{grid-template-columns:1fr}}</style><header><b>ScaleNav GCN</b> <button id="prev">上一帧</button><button id="next">下一帧</button><select id="sample"></select><select id="session"><option value="">全部 Session</option></select><span id="count"></span></header><main><section class="p"><canvas id="map" width="900" height="600"></canvas><h3>RGB / Depth / Heatmap</h3><div class="s"><img id="rgb"><img id="depth"><img id="semantic"></div><p class="l">点云已变换到 world，与 skeleton graph 叠加显示。</p></section><aside class="p"><div id="title"></div><div class="st" id="stats"></div><h3>五列模型分数 / 真值</h3><div id="cols"></div></aside></main><script>const D=__DATA__,S=document.querySelector('#sample'),F=document.querySelector('#session'),M=document.querySelector('#map'),C=M.getContext('2d');[...new Set(D.samples.map(s=>s.session))].sort().forEach(s=>F.add(new Option(s,s)));function L(){return D.samples.filter(s=>!F.value||s.session==F.value)}function fill(){S.innerHTML='';L().forEach((s,i)=>S.add(new Option(i+': '+s.session+' seq='+s.seq,i)));document.querySelector('#count').textContent=L().length+' / '+D.total;draw()}function step(d){let n=L().length;if(n){S.value=(+S.value+d+n)%n;draw()}}function draw(){let a=L(),s=a[+S.value||0];if(!s)return;['rgb','depth','semantic'].forEach(k=>document.querySelector('#'+k).src=s[k]||'');let X=s.nodes.map(p=>p[0]),Y=s.nodes.map(p=>p[1]);s.cloud.forEach(p=>{X.push(p[0]);Y.push(p[1])});let ax=Math.min(...X),bx=Math.max(...X),ay=Math.min(...Y),by=Math.max(...Y),q=35,k=Math.min((M.width-2*q)/(bx-ax||1),(M.height-2*q)/(by-ay||1)),px=v=>q+(v-ax)*k,py=v=>M.height-q-(v-ay)*k;C.clearRect(0,0,M.width,M.height);C.fillStyle='#65788c';s.cloud.forEach(p=>C.fillRect(px(p[0]),py(p[1]),2,2));C.strokeStyle='#3d4956';s.edges.forEach(e=>{C.beginPath();C.moveTo(px(s.nodes[e[0]][0]),py(s.nodes[e[0]][1]));C.lineTo(px(s.nodes[e[1]][0]),py(s.nodes[e[1]][1]));C.stroke()});let z=new Map(s.frontier.map((v,i)=>[v,i]));s.nodes.forEach((p,i)=>{let j=z.get(i),c=j===s.target?'#45d483':j===s.planner?'#f0a04a':'#71808e',r=j===undefined?2.5:6;if(i===s.odom){c='#ef5b65';r=7}C.fillStyle=c;C.beginPath();C.arc(px(p[0]),py(p[1]),r,0,7);C.fill()});if(s.frontier[s.model]!==undefined){let p=s.nodes[s.frontier[s.model]],u=px(p[0]),v=py(p[1]);C.strokeStyle='#ba7cff';C.lineWidth=3;C.beginPath();C.arc(u,v,12,0,7);C.stroke();C.beginPath();C.moveTo(u-10,v);C.lineTo(u+10,v);C.moveTo(u,v-10);C.lineTo(u,v+10);C.stroke()}document.querySelector('#title').innerHTML='<b>'+s.session+'</b> seq='+s.seq;document.querySelector('#stats').innerHTML=[['GCN 输出',s.model],['Map 真值列',s.target],['Planner',s.planner],['可达列',s.safe.map((v,i)=>v?i:'-').join(' ')],['节点/边',s.nodes.length+'/'+s.edges.length],['点云数',s.cloud.length],['位置',s.position.slice(0,2).map(v=>v.toFixed(2)).join(', ')],['GCN 命中',s.model==s.target?'是':'否']].map(v=>'<div><span class="l">'+v[0]+'</span><br><span class="v">'+v[1]+'</span></div>').join('');document.querySelector('#cols').innerHTML=s.scores.map((v,i)=>'<div>column '+i+'　模型 <b style="color:#ba7cff">'+v.toFixed(3)+'</b>　真值 '+(s.truth[i]==null?'不可达':s.truth[i].toFixed(2))+(i==s.model?' ← GCN':'')+(i==s.target?' ← Map':'')+(i==s.planner?' ← Planner':'')+'</div>').join('')}F.onchange=fill;S.onchange=draw;prev.onclick=()=>step(-1);next.onclick=()=>step(1);document.onkeydown=e=>{if(e.key=='ArrowLeft')step(-1);if(e.key=='ArrowRight')step(1)};fill();</script>'''
ORIENTATION = r'''<script>
const drawBase=draw;
draw=function(){
  drawBase(); const s=L()[+S.value||0]; if(!s)return;
  const realCount=s.frontier[0], realNodes=s.nodes.slice(0,realCount), rayLength=25;
  const rays=[40,20,0,-20,-40].map(d=>{const a=s.yaw+d*Math.PI/180;return [s.position[0]+Math.cos(a)*rayLength,s.position[1]+Math.sin(a)*rayLength]});
  const X=realNodes.map(p=>p[0]).concat(s.cloud.map(p=>p[0]),rays.map(p=>p[0]),[s.position[0]]);
  const Y=realNodes.map(p=>p[1]).concat(s.cloud.map(p=>p[1]),rays.map(p=>p[1]),[s.position[1]]);
  const ax=Math.min(...X),bx=Math.max(...X),ay=Math.min(...Y),by=Math.max(...Y),q=35;
  const k=Math.min((M.width-2*q)/(bx-ax||1),(M.height-2*q)/(by-ay||1)),px=v=>q+(v-ax)*k,py=v=>M.height-q-(v-ay)*k;
  C.clearRect(0,0,M.width,M.height); C.fillStyle='#65788c'; s.cloud.forEach(p=>C.fillRect(px(p[0]),py(p[1]),2,2));
  C.strokeStyle='#3d4956';C.lineWidth=1;s.edges.filter(e=>e[0]<realCount&&e[1]<realCount).forEach(e=>{C.beginPath();C.moveTo(px(s.nodes[e[0]][0]),py(s.nodes[e[0]][1]));C.lineTo(px(s.nodes[e[1]][0]),py(s.nodes[e[1]][1]));C.stroke()});
  C.fillStyle='#71808e';realNodes.forEach(p=>{C.beginPath();C.arc(px(p[0]),py(p[1]),2.5,0,7);C.fill()});
  rays.forEach((p,i)=>{C.strokeStyle=i===s.model?'#ba7cff':i===s.target?'#45d483':i===s.planner?'#f0a04a':'#596775';C.lineWidth=i===s.model?5:2;C.beginPath();C.moveTo(px(s.position[0]),py(s.position[1]));C.lineTo(px(p[0]),py(p[1]));C.stroke();C.fillStyle=C.strokeStyle;C.beginPath();C.arc(px(p[0]),py(p[1]),i===s.model?7:4,0,7);C.fill()});
  const u=px(s.position[0]),v=py(s.position[1]),hx=Math.cos(s.yaw),hy=Math.sin(s.yaw),n=30;
  C.fillStyle='#ef5b65';C.beginPath();C.arc(u,v,7,0,7);C.fill();C.strokeStyle='#ef5b65';C.lineWidth=3;C.beginPath();C.moveTo(u,v);C.lineTo(u+hx*n,v-hy*n);C.stroke();
  document.querySelector('#stats').innerHTML+=`<div><span class="l">飞机 yaw</span><br><span class="v">${(s.yaw*180/Math.PI).toFixed(1)}°</span></div>`;
};fill();
</script>'''

def main():
 p=argparse.ArgumentParser();p.add_argument('--dataset',default='train_gcn/dataset.pt');p.add_argument('--model',default='train_gcn/frontier_gcn.pt');p.add_argument('--output',default='train_gcn/dataset_viewer.html');p.add_argument('--max-samples',type=int,default=300);a=p.parse_args();raw=torch.load(a.dataset,weights_only=False)['samples'];chosen=raw if a.max_samples<=0 else raw[:a.max_samples];model=FrontierGCN();model.load_state_dict(torch.load(a.model,map_location='cpu',weights_only=False)['model']);model.eval();pred={};scores={}
 for _,g in session_groups(chosen).items():
  hidden=None;previous=-1
  for s in g:
   with torch.no_grad(): l,hidden=model(s,hidden,previous)
   pred[id(s)]=int(l.argmax());scores[id(s)]=torch.softmax(l,0).tolist()
   previous=pred[id(s)];hidden=hidden.detach()
 cache={};out=[]
 for s in chosen:
  root=os.path.join('log_scalenav',s['session']);R=cache.setdefault(root,[])
  if not R:
   for line in open(os.path.join(root,'index.jsonl'),encoding='utf-8'):
    try:R.append(json.loads(line,parse_constant=float))
    except:pass
  t=next((r for r in R if r.get('kind')=='timing' and r.get('seq')==s['seq']),{});stamp=t.get('stamp_ns',0);A={}
  for kind in ('rgb','depth','semantic','pointcloud'):
   c=[r for r in R if r.get('kind')==kind and r.get('file')]
   if c:A[kind]=os.path.join(root,min(c,key=lambda r:abs(r['stamp_ns']-stamp))['file'])
  o=min((r for r in R if r.get('kind')=='odom'),key=lambda r:abs(r['stamp_ns']-stamp),default={});q=o.get('data',{}).get('orientation',[0,0,0,1]);yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]));cloud=[]
  for p in pcd(A.get('pointcloud')):
   v=rot(q,p);cloud.append([v[0]+s['position'][0],v[1]+s['position'][1],v[2]+s['position'][2]])
  nodes=[[round(float(r[0])*25,3),round(float(r[1])*80,3)] for r in s['x']];out.append({'nodes':nodes,'edges':s['edge_index'][:,::2].t().tolist(),'frontier':s['frontier_index'].tolist(),'model':pred[id(s)],'scores':scores[id(s)],'truth':truth_profile(s),'target':s['target'],'planner':s['planner_target'],'safe':s['safe_columns'].tolist(),'session':s['session'],'seq':s['seq'],'position':s['position'],'yaw':yaw,'odom':min(range(len(nodes)),key=lambda i:(nodes[i][0]-s['position'][0])**2+(nodes[i][1]-s['position'][1])**2),'cloud':cloud,'rgb':img(A.get('rgb')),'depth':img(A.get('depth')),'semantic':img(A.get('semantic'),True)})
 path=os.path.abspath(a.output);os.makedirs(os.path.dirname(path),exist_ok=True)
 page=HTML.replace('__DATA__',json.dumps({'total':len(raw),'samples':out},separators=(',',':')))
 with open(path,'w',encoding='utf-8') as f:f.write(page.replace('</script>','</script>'+ORIENTATION,1))
 print(f'wrote={path} embedded={len(out)} total={len(raw)}')
def truth_profile(s):
 n=min(s['frontier_index'].tolist());pos=s['position'];adj={i:[] for i in range(n)}
 for a,b in s['edge_index'][:,::2].t().tolist():
  if a<n and b<n:adj[a].append(b);adj[b].append(a)
 st=min(range(n),key=lambda i:(float(s['x'][i,0])*25-pos[0])**2+(float(s['x'][i,1])*80-pos[1])**2);seen,q={st},[st]
 while q:
  for j in adj[q.pop()]:
   if j not in seen:seen.add(j);q.append(j)
 r=[-1e9]*5;lim=math.radians(50)
 for i in seen:
  dx,dy=float(s['x'][i,0])*25-pos[0],float(s['x'][i,1])*80-pos[1];an=math.atan2(dx,dy)
  if dy>=3 and math.hypot(dx,dy)>=3 and abs(an)<=lim:r[max(0,min(4,int(round((an/lim+1)*2))))]=max(r[max(0,min(4,int(round((an/lim+1)*2))))],dy-.2*abs(dx))
 return [None if v<-1e8 else round(v,2) for v in r]
if __name__=='__main__':main()
