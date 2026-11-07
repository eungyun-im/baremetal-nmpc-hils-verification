import csv,json,itertools,numpy as np,statistics as s,collections,random
R=[x for x in csv.DictReader(open('../../summary.csv')) if x['condition'].startswith('xover') and x['valid']=='1']
CFG=['sqp1_qp2','sqp1_qp4','sqp2_qp4','sqp2_qp16']
L=collections.defaultdict(list)
def metrics(b):
    J=json.load(open('cache/'+b+'.json')); A=np.load('cache/'+b+'.npz'); G=A['gps']; Sp=A['speed']; t0=J['t0']; seq=J['seq']
    t=G[:,0]; e=G[:,1]; ok=~np.isnan(e); core=np.zeros(len(t),bool)
    sec={}
    for i,(a,l) in enumerate(seq):
        A0=a+t0; B0=seq[i+1][0]+t0 if i+1<len(seq) else J['t1']; m=(t>=A0)&(t<B0)&ok
        sec[l]=(A0,B0)
        if l not in('M3','M8','EXIT'): core|=m
    out=dict(core=float(e[core].mean()) if core.any() else None)
    out['m5w']=None
    if 'M5' in sec:
        a,b_=sec['M5']; sp=Sp[(Sp[:,0]>=a)&(Sp[:,0]<=b_)]
        if len(sp)>5:
            d=np.r_[0,np.cumsum(np.diff(sp[:,0])*sp[:-1,1])]; m=(t>=a)&(t<=b_)&ok; dd=np.interp(t[m],sp[:,0],d); ee=e[m][(dd>=60)&(dd<80)]
            if len(ee): out['m5w']=float(ee.mean())
    return out
for x in R:
    m=metrics(x['bag']); m.update(fin=x['finished']=='1',emax=float(x['err_gps_max'] or 0),cap=float(x['maxiter_ratio'] or 0),st=x['stamp'])
    L[x['condition'][6:]].append(m)
for k in L: L[k].sort(key=lambda m:m['st'])
def signp(d):
    d=[x for x in d if x is not None]; o=abs(sum(d)); n=len(d)
    if n<=16:
        c=sum(1 for z in itertools.product([1,-1],repeat=n) if abs(sum(a*b for a,b in zip(z,d)))>=o-1e-12); return c/2**n
    rng=random.Random(1); c=sum(1 for _ in range(200000) if abs(sum(v*rng.choice((1,-1)) for v in d))>=o-1e-12); return c/200000
def st(v): v=[x for x in v if x is not None]; return (round(s.mean(v),4),round(s.stdev(v),4),len(v))
print('cfg fw core m5w emax>0.8 cap')
for c in CFG:
    for f in('before','after'):
        v=L[f+'_'+c]; print(c,f,'core',st([m['core'] for m in v if m['fin']]),'m5w',st([m['m5w'] for m in v if m['fin']]),'emax>.8',sum(m['emax']>.8 for m in v),'/',len(v),'cap',round(s.mean(m['cap'] for m in v),4))
pc=[];pm=[]
print('paired after-before by block (finished both)')
for c in CFG:
    a,b=L['after_'+c],L['before_'+c]; dc=[];dm=[]
    for i in range(min(len(a),len(b))):
        if a[i]['fin'] and b[i]['fin']:
            dc.append(a[i]['core']-b[i]['core'])
            if a[i]['m5w'] is not None and b[i]['m5w'] is not None: dm.append(a[i]['m5w']-b[i]['m5w'])
    pc+=dc;pm+=dm
    print(c,'n',len(dc),'core d',round(s.mean(dc),4),'p',round(signp(dc),3),'m5w d',round(s.mean(dm),4),'p',round(signp(dm),3))
print('pooled n',len(pc),'core d',round(s.mean(pc),4),'p',round(signp(pc),4),'neg',sum(x<0 for x in pc),'| m5w d',round(s.mean(pm),4),'p',round(signp(pm),4),'neg',sum(x<0 for x in pm))
for f in('before','after'):
    base=[m for m in L[f+'_sqp2_qp16'] if m['fin']]
    for c in CFG[:3]:
        v=[m for m in L[f+'_'+c] if m['fin']]
        print(f,c,'vs qp16 core',round(s.mean(m['core'] for m in v)-s.mean(m['core'] for m in base),4),'m5w',round(s.mean(m['m5w'] for m in v if m['m5w'])-s.mean(m['m5w'] for m in base if m['m5w']),4))
