#!/usr/bin/python3
import numpy as np,glob,json,re,math,os,collections,matplotlib,subprocess
matplotlib.use("Agg"); import matplotlib.pyplot as plt
plt.rcParams.update({"font.family":"Noto Sans CJK JP","axes.unicode_minus":False,"figure.dpi":150,"axes.spines.top":False,"axes.spines.right":False,"font.size":9})
E0,N0=302500,4124000
csvp=subprocess.run("find ~/catkin_ws/src -name 0.csv | head -1",shell=True,capture_output=True,text=True).stdout.strip()
W=np.genfromtxt(csvp,delimiter=",",invalid_raise=False,skip_header=1); W=W[:,:2] if W[np.isfinite(W[:,0]),0].mean()>1e5 else W[:,1:3]
if W[np.isfinite(W[:,0]),0].mean()<1e3: W=W[:,::-1]
W=W[np.isfinite(W).all(1)]; print("csv sample",W[3400])
m=(W[:,0]>E0-5)&(W[:,0]<E0+75)&(W[:,1]>N0-20)&(W[:,1]<N0+60); C=W[m]-[E0,N0]
print("csv",csvp,"path E at N=0,20,30,40",[round(float(C[np.argmin(abs(C[:,1]-n)),0]),2) for n in(0,20,30,40)])
allp=[];objs=collections.defaultdict(list);times=[]
for f in sorted(glob.glob("zone_*.npz")):
    d=np.load(f,allow_pickle=True)
    for p,xyz in zip(d["pose"],d["pts"]):
        c,s=math.cos(p[2]),math.sin(p[2]);x,y,z=xyz.T
        allp.append(np.c_[p[0]+c*x-s*y-E0,p[1]+s*x+c*y-N0,z])
    for t,pp,txt in json.loads(str(d["info"])):
        try:J=json.loads(txt)
        except: continue
        for o in J.get("collision_objects",[])+J.get("obstacles",[]):
            fp=np.array(o.get("footprint_world",[]))
            if len(fp)<3: continue
            sp=np.array(o.get("support_points_world",[])) if o.get("support_points_world") else np.empty((0,3))
            objs[(round(fp[:,0].mean()-E0),round(fp[:,1].mean()-N0))].append((fp[:,0].max()-fp[:,0].min(),fp[:,1].max()-fp[:,1].min(),sp[:,2].max()-sp[:,2].min() if len(sp) else 0, f))
A=np.vstack(allp);print("pts",len(A),"z pct",np.percentile(A[:,2],[1,20,50,90,99]).round(2))
sel=A[(A[:,2]>-1.0)&(A[:,2]<4)]
H,xe,ye=np.histogram2d(sel[:,0],sel[:,1],bins=[np.arange(-5,75,.25),np.arange(-20,60,.25)])
H0,_,_=np.histogram2d(A[:,0],A[:,1],bins=[xe,ye])
print("objects clusters (E,N rel): count wE wN hz laps")
rows=[]
for k,v in objs.items():
    a=np.array([u[:3] for u in v],float); rows.append((k,len(v),*np.median(a,0).round(2),len({u[3] for u in v})))
rows.sort(key=lambda r:-r[1]);[print(" ",r) for r in rows[:18]]
stop=[]
for f in sorted(glob.glob("../../../bags/sils_lidar_m3_*.log")):
    T=re.sub(r"\x1b\[[0-9;]*m","",open(f,errors="ignore").read())
    for z in glob.glob("zone_*.npz"):
        d=np.load(z,allow_pickle=True)
        for m in re.finditer(r"\[(\d{10}\.\d+)\]: \[obstacle_stop\] STOP TRIGGERED by id=(\d+) pos_ego=\(([-\d.]+),([-\d.]+)\)",T):
            t=float(m.group(1)); i=int(np.argmin(abs(d["t"]-t)))
            if abs(d["t"][i]-t)<.5:
                p=d["pose"][i];c,s=math.cos(p[2]),math.sin(p[2]);px,py=float(m.group(3)),float(m.group(4))
                stop.append((round(p[0]+c*px-s*py-E0,1),round(p[1]+s*px+c*py-N0,1),round(p[0]-E0,1),round(p[1]-N0,1),m.group(2)))
print("stop this run (objE,objN,egoE,egoN,id)",stop)
old=[(36.7,29.3),(33.7,33.0),(34.6,26.6),(33.9,33.5),(34.7,30.2),(35.5,32.7),(34.9,28.1),(35.1,34.3),(33.9,30.4),(34.7,37.9),(34.7,28.9),(36.4,26.4),(32.6,34.9),(36.1,30.1),(35.1,28.9),(34.9,30.1),(34.7,26.3),(35.0,36.7),(35.3,26.7),(34.6,27.2)]
for E,N in old[:0]:pass
# 정지 위치 주변의 높이 분포
for nm,(e,n) in {"정지군 중심":(35,30),"박스":(34.39,2.82)}.items():
    q=A[(abs(A[:,0]-e)<2.5)&(abs(A[:,1]-n)<4)]
    print(nm,"pts",len(q),"z pct",np.percentile(q[:,2],[1,50,99]).round(2) if len(q) else "", "above -1:",int((q[:,2]>-1).sum()))
fig,ax=plt.subplots(figsize=(7.2,6.4))
ax.imshow(np.log1p(H).T,origin="lower",extent=[xe[0],xe[-1],ye[0],ye[-1]],cmap="Greys",vmax=np.log1p(H).max()*.8,aspect="equal")
ax.plot(C[:,0],C[:,1],color="#2878d6",lw=1.2,label="기준 경로")
ax.scatter(*zip(*old),s=16,marker="x",color="#c0443c",lw=1,label="M3 정지 물체 (앞선 20랩)")
if stop: ax.scatter([s[0] for s in stop],[s[1] for s in stop],s=26,marker="x",color="#d07a00",lw=1.2,label="M3 정지 물체 (이번 랩)")
ax.scatter(34.39,2.82,s=70,marker="s",facecolor="none",edgecolor="#2878d6",lw=1.3,label="시나리오 물체 (좌표 어림)")
ax.axhline(20.5,color="#888",lw=.8,ls=":");ax.text(70,21,"M3 경계 부근",ha="right",color="#666")
ax.set(xlim=(10,62),ylim=(-16,56),xlabel=f"동쪽 (m, 기준 {E0})",ylabel=f"북쪽 (m, 기준 {N0})");ax.legend(frameon=False,loc="upper left",fontsize=8);ax.set_title("삼거리와 M3의 라이다 누적 지도 (NPC 없음, 3랩)",loc="left",fontsize=11)
fig.tight_layout();fig.savefig("../figs/fig13_lidar_map.png");
H2=H.T.copy();np.savez("../figs/lidar_grid.npz",H=H2,xe=xe,ye=ye)
