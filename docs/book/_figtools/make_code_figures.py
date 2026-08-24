import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt, numpy as np, math, pathlib
EN=pathlib.Path("/home/anatoly/atom/adas/docs/book")
def save(fig, rel):
    for t in (EN, EN.parent/"book_ru"):
        p=t/rel; p.parent.mkdir(parents=True,exist_ok=True); fig.savefig(p,dpi=130,bbox_inches="tight",facecolor="white")
    plt.close(fig)

# ---- ToyCar: P-only oscillates, heading+offset settles (BicycleModel) ----
DT,L,V=0.05,2.636,10.0; A,LAM=1.75,120.0; MAX=math.radians(25)
yref=lambda x:A*math.sin(2*math.pi*x/LAM)
psiref=lambda x:math.atan(A*2*math.pi/LAM*math.cos(2*math.pi*x/LAM))
def sim(ctrl,t=60.0):
    x,y,psi=0.,1.,0.; T=[];C=[]
    for k in range(int(t/DT)):
        cte=y-yref(x); d=ctrl(x,y,psi,cte)
        x+=V*math.cos(psi)*DT; y+=V*math.sin(psi)*DT; psi+=V*math.tan(d)/L*DT
        T.append(k*DT); C.append(cte)
    return np.array(T),np.array(C)
p=lambda x,y,psi,cte:float(np.clip(-0.10*cte,-MAX,MAX))
h=lambda x,y,psi,cte:float(np.clip((psiref(x)-psi)-math.atan2(2.0*cte,V),-MAX,MAX))
tp,cp=sim(p); th,ch=sim(h)
fig,ax=plt.subplots(figsize=(7,3.4))
ax.plot(tp,cp,color="tab:red",lw=1.2,label="P on offset (undamped)")
ax.plot(th,ch,color="tab:blue",lw=1.5,label="heading + offset (Stanley)")
ax.axhline(0,color="0.7",lw=.7); ax.set_xlabel("time [s]"); ax.set_ylabel("cross-track error [m]")
ax.set_ylim(-3,3); ax.legend(fontsize=8); ax.grid(alpha=.3)
ax.set_title("Same plant, two controllers: P swings forever, heading damps it")
save(fig,"Planner/figures/toycar_control.png")

# ---- PurePursuit trade-off: |CTE| along a 60 m arc for Ld 3/8/20 ----
def arc_path(R=60.,length=180.):
    s=np.arange(0,length,0.5); pts=[]
    for si in s:
        if si<40: pts.append((si,0.))
        else:
            t=(si-40)/R; pts.append((40+R*math.sin(t),R*(1-math.cos(t))))
    return np.array(pts)
def drive(path,ld,t_end=45.):
    x,y,psi=path[0,0],path[0,1]+1.0,0.; near=0; C=[];D=[]
    for _ in range(int(t_end/DT)):
        d=np.hypot(path[near:,0]-x,path[near:,1]-y); near+=int(np.argmin(d[:max(1,int(4*ld))]))
        tgt=None
        for i in range(near,len(path)):
            if math.hypot(path[i,0]-x,path[i,1]-y)>=ld: tgt=path[i]; break
        if tgt is None: break
        al=math.atan2(tgt[1]-y,tgt[0]-x)-psi; delta=float(np.clip(math.atan2(2*L*math.sin(al),ld),-MAX,MAX))
        x+=V*math.cos(psi)*DT; y+=V*math.sin(psi)*DT; psi+=V*math.tan(delta)/L*DT
        C.append(float(np.hypot(path[:,0]-x,path[:,1]-y).min())); D.append(x)
        if near>=len(path)-3: break
    return np.array(D),np.array(C)
arc=arc_path()
fig,ax=plt.subplots(figsize=(7,3.4))
for ld,col in ((3,"tab:green"),(8,"tab:blue"),(20,"tab:red")):
    D,C=drive(arc,ld); ax.plot(D,C,col,lw=1.5,label=f"Ld={ld} m")
ax.axvline(40,color="0.7",ls=":",lw=.8); ax.text(41,ax.get_ylim()[1]*0.8,"arc starts",fontsize=7,color="0.5")
ax.set_xlabel("distance travelled [m]"); ax.set_ylabel("|cross-track error| [m]")
ax.legend(fontsize=8); ax.grid(alpha=.3)
ax.set_title("Pure Pursuit on R=60 m: long look-ahead settles metres inside the bend")
save(fig,"Planner/figures/pp_tradeoff.png")

# ---- VehicleModel: arc drift uncompensated vs /G ; and delay oscillation ----
def G(v): return float(np.interp(v,[8,13.5,23.5],[0.97,0.80,0.54]))
KAPPA=1/210.; Vv=23.5
def arc_drift(comp,t_end=5.):
    x=y=psi=0.; d=math.atan(L*KAPPA/(G(Vv) if comp else 1.0)); off=[]
    for _ in range(int(t_end/DT)):
        x+=Vv*math.cos(psi)*DT; y+=Vv*math.sin(psi)*DT; psi+=Vv*G(Vv)*math.tan(d)/L*DT
        off.append(math.hypot(x,y-1/KAPPA)-1/KAPPA)
    return np.array(off)
from collections import deque
def straight(delay,predict,kp=2.0,t_end=12.,v=23.5):
    sx,sy,sp=0.,1.,0.; q=deque([0.]*max(0,round(delay/DT))); C=[]
    for _ in range(int(t_end/DT)):
        ex,ey,ep=sx,sy,sp
        if predict:
            for dd in q:
                ex+=v*math.cos(ep)*DT; ey+=v*math.sin(ep)*DT; ep+=v*G(v)*math.tan(dd)/L*DT
        cmd=float(np.clip(-ep-math.atan2(kp*ey,v),-MAX,MAX)); q.append(cmd); dd=q.popleft()
        sx+=v*math.cos(sp)*DT; sy+=v*math.sin(sp)*DT; sp+=v*G(v)*math.tan(dd)/L*DT; C.append(abs(sy))
    return np.array(C)
fig,(a1,a2)=plt.subplots(1,2,figsize=(9,3.3))
tt=np.arange(0,5,DT)
a1.plot(tt,arc_drift(False),"tab:red",label="kinematic δ (no G)")
a1.plot(tt,arc_drift(True),"tab:blue",label="δ divided by G(v)")
a1.set_title("Understeer on R=210 m @ 23.5 m/s"); a1.set_xlabel("time [s]"); a1.set_ylabel("offset outside arc [m]")
a1.legend(fontsize=8); a1.grid(alpha=.3)
tt2=np.arange(0,12,DT)
a2.plot(tt2,straight(0.0,False),"0.5",label="no delay")
a2.plot(tt2,straight(0.35,False),"tab:red",label="0.35 s delay")
a2.plot(tt2,straight(0.35,True),"tab:blue",label="delay + prediction")
a2.set_title("Delay turns a good controller into an oscillator"); a2.set_xlabel("time [s]"); a2.set_ylabel("|CTE| [m]")
a2.set_ylim(0,3); a2.legend(fontsize=8); a2.grid(alpha=.3)
save(fig,"Control/figures/slip_delay.png")

# ---- MPC time grid + lateral reach ----
tnode=lambda i:10.0*(i/32.0)**2; N=16
fig,ax=plt.subplots(figsize=(7,3.3))
i=np.arange(N+1); t=np.array([tnode(k) for k in i]); reach=0.5*3.0*t*t
ax.stem(t,np.ones_like(t),linefmt="0.7",markerfmt="o",basefmt=" ")
ax.plot(t,reach,"tab:red",lw=1.5,label="lateral reach ½·3·t² [m]")
for k in (0,4,8,12,16): ax.text(tnode(k),1.05,f"n{k}",fontsize=7,ha="center",color="0.4")
ax.set_xlabel("horizon time [s]  (nodes t=10(i/32)²)"); ax.set_ylabel("reach [m]")
ax.legend(fontsize=8); ax.grid(alpha=.3)
ax.set_title("fp horizon: near nodes are dense but cannot move sideways yet")
save(fig,"Planner/figures/mpc_timegrid.png")

# ---- Latency ceiling function ----
fig,ax=plt.subplots(figsize=(7,3.3))
W=np.linspace(20,120,400)
for T,col in ((33,"tab:blue"),(50,"tab:green")):
    f=1000.0/(T*np.ceil(W/T)); ax.plot(W,f,col,lw=1.5,label=f"period T={T} ms")
ax.set_xlabel("processing work per frame W [ms]"); ax.set_ylabel("achieved rate [Hz]")
ax.set_title("Rate is a staircase: crossing a multiple of T costs a third of it")
ax.legend(fontsize=8); ax.grid(alpha=.3)
save(fig,"Latency/figures/ceiling.png")

# ---- Planner understeer bars ----
fig,ax=plt.subplots(figsize=(6.5,3.3))
v=["6–9","12–15","21–26"]; meas=[0.97,0.80,0.54]; exp=[0.96,0.87,0.69]
xx=np.arange(3); w=0.35
ax.bar(xx-w/2,meas,w,label="measured",color="tab:blue")
ax.bar(xx+w/2,exp,w,label="textbook expects",color="0.7")
ax.axhline(1.0,color="tab:green",ls=":",lw=1,label="kinematic (1.0)")
ax.set_xticks(xx); ax.set_xticklabels([f"{s} m/s" for s in v]); ax.set_ylim(0,1.1)
ax.set_ylabel("κ_fact / κ_kin"); ax.legend(fontsize=8); ax.grid(alpha=.3,axis="y")
ax.set_title("The one number: curvature achieved vs commanded, below textbook at every speed")
save(fig,"Planner/figures/understeer_bars.png")
print("mkfigs2: 6 figures into both trees")
