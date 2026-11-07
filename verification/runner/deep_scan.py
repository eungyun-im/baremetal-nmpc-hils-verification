#!/usr/bin/python3
# 판정 랩의 err_track_excl 과 EKF 발산 랩의 위치. 한 프로세스로 천천히 돈다.
import csv,json,os,re,types,sys,numpy as np,pandas as pd,rosbag,pyproj
ROOT=os.path.expanduser('~/catkin_ws/bags/exp_20261105'); os.chdir(ROOT+'/analysis/final/delay_qp_budget')
src=open('dqb_extract.py').read().rsplit('main()',1)[0]; q=types.ModuleType('q'); exec(src,q.__dict__)
R=list(csv.DictReader(open(ROOT+'/summary.csv'))); RO=set(json.load(open(os.path.expanduser('~/catkin_ws/src/bare-metal-mpc/bmpc_bridge/config/acceptance.json')))['report_only'])
to=pyproj.Proj(proj='utm',zone=52,ellps='WGS84')
def seg_d(pt,P):
    d=np.hypot(*(P-pt).T); i=int(np.argmin(d)); return float(d[i])
out=[];ek=[]
J=[x for x in R if x['valid']=='1' and x['condition'] not in RO and not x['fault'] and not x['condition'].startswith(('fault','ctrl','curve','wdt','sils_no','sils_lidar')) and x['bag']]
E=[x for x in R if x['invalid']=='ekf_diverged' and x['bag']]
print(len(J),len(E),flush=True)
for x in J:
    p=ROOT+'/bags/'+os.path.basename(x['bag'])
    if not os.path.exists(p): continue
    npz='cache/'+os.path.basename(x['bag'])+'.npz'
    try:
        if os.path.exists(npz): G=np.load(npz)['gps']; N=json.load(open(npz[:-3]+'json'))['names']
        else:
            res,a=q.process(p); G=a['gps']; N=res['names']
        Tp=[];Ee=[]
        with rosbag.Bag(p) as b:
            for tp,m,t in b.read_messages(topics=['/local_path','/ekf_pose_with_covariance']):
                if tp=='/local_path': Tp.append((t.to_sec(),np.array([[w.pose.position.x,w.pose.position.y] for w in m.poses[:150]])))
                else: Ee.append((t.to_sec(),m.pose.pose.position.x,m.pose.pose.position.y))
        Ee=np.array(Ee); ev=[]; prev=None
        for t,P in Tp:
            if not len(P): continue
            if prev is not None and len(prev):
                e=Ee[min(np.searchsorted(Ee[:,0],t),len(Ee)-1)]; a_=P[int(np.argmin(np.hypot(*(P-e[1:3]).T)))]
                if seg_d(a_,prev)>=.3: ev.append(t)
            prev=P
        ev=np.array(ev); t=G[:,0]; mask=np.ones(len(t),bool)
        for v in ev: mask&=~((t>=v)&(t<v+3))
        val=np.nan_to_num(G[:,1],nan=-1); pk=int(np.argmax(val)); rest=np.where(mask,val,-1); pk2=int(np.argmax(rest))
        name=lambda i:N[int(i)] if 0<=int(i)<len(N) else '?'
        out.append(dict(c=x['condition'],a=x['attempt'],mode=x['mode'],stamp=x['stamp'],raw=val[pk],raw_sec=name(G[pk,2]),excl=rest[pk2],excl_sec=name(G[pk2,2]),frac=1-mask.mean(),events=len(ev)))
    except Exception as e: print('err',x['bag'],repr(e)[:80],flush=True)
    pd.DataFrame(out).to_csv('../figs/err_track_excl.csv',index=False)
print('metric done',flush=True)
for x in E:
    p=ROOT+'/bags/'+os.path.basename(x['bag'])
    if not os.path.exists(p): continue
    Gp=[];Eo=[];Se=[];Sp=[]
    with rosbag.Bag(p) as b:
        for tp,m,t in b.read_messages(topics=['/gps','/ekf_pose_with_covariance','/mission/active_section','/Competition_topic']):
            tt=t.to_sec()
            if tp=='/gps':
                Gp.append((tt,)+to(m.longitude,m.latitude))
            elif tp[1]=='e': Eo.append((tt,m.pose.pose.position.x,m.pose.pose.position.y))
            elif tp[1]=='m': Se.append((tt,m.section_id))
            else: Sp.append((tt,m.velocity.x))
    Gp=np.array(Gp);Eo=np.array(Eo);Sp=np.array(Sp); Gp=Gp[np.hypot(Gp[:,1]-302595,Gp[:,2]-4124145)<5000]
    if not len(Gp) or not len(Eo): continue
    ex=np.interp(Gp[:,0],Eo[:,0],Eo[:,1]);ey=np.interp(Gp[:,0],Eo[:,0],Eo[:,2]);d=np.hypot(ex-Gp[:,1],ey-Gp[:,2]); pk=int(np.argmax(d)); tm=Gp[pk,0]
    secs=[s for tt,s in Se if tt<=tm]; over=d>1.0; t0=Gp[np.argmax(over),0] if over.any() else tm
    gd=np.diff(Gp[:,0]); ek.append(dict(c=x['condition'],a=x['attempt'],stamp=x['stamp'],peak=d[pk],peak_sec=str(secs[-1]) if secs else '?',t_from_start=tm-Sp[0,0],v=float(np.interp(tm,Sp[:,0],Sp[:,1])),over1_s=float(over.sum()*np.median(gd)),gps_gap=float(gd.max()),E=round(Gp[pk,1]-302500,1),N=round(Gp[pk,2]-4124000,1),first_over1=t0-Sp[0,0]))
    pd.DataFrame(ek).to_csv('../figs/ekf_diverged.csv',index=False)
print('done',flush=True)
