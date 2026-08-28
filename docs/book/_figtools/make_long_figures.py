import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt, numpy as np, math, pathlib
EN=pathlib.Path(__file__).resolve().parents[1]
def save(fig, rel):
    for t in (EN, EN.parent/"book_ru"):
        p=t/rel; p.parent.mkdir(parents=True,exist_ok=True); fig.savefig(p,dpi=130,bbox_inches="tight",facecolor="white")
    plt.close(fig)

# === safe distance and desired gap vs speed ===
def safe(v,tf=1.45,b=2.5,d0=6.0): return v*v/(2*b)+tf*v+d0
v=np.linspace(0,35,200)
fig,ax=plt.subplots(figsize=(6.5,3.4))
ax.plot(v,safe(v),"tab:blue",lw=2,label="safe distance to a stopped obstacle")
ax.plot(v,safe(v)-v*v/5.0,"tab:green",lw=2,label="desired gap behind a lead at the same speed")
ax.plot(v,1.45*v+6,"k:",lw=1,label="1.45 s + 6 m")
ax.axvspan(0,8,color="0.9"); ax.text(0.5,150,"city",fontsize=8)
ax.set_xlabel("speed [m/s]"); ax.set_ylabel("distance [m]"); ax.grid(alpha=.3); ax.legend(fontsize=8)
ax.set_title("The two distances the plan is built on (T_FOLLOW 1.45 s, 2.5 m/s², 6 m)")
save(fig,"Planner/figures/long_safe_distance.png")

# === toy 1: P on the gap vs a braking lead ===
def sim_p(kp,kv,T=40,dt=0.05):
    v_l,v_e,gap=20.0,20.0,30.0; out=[]
    for k in range(int(T/dt)):
        t=k*dt
        if 5<=t<10: v_l=max(5.0,v_l-2.0*dt)
        want=1.45*v_e+6.0
        a=kp*(gap-want)+kv*(v_l-v_e); a=max(-3.5,min(2.0,a))
        v_e=max(0.0,v_e+a*dt); gap+= (v_l-v_e)*dt; out.append((t,gap,want,a,v_l,v_e))
    return np.array(out)
fig,axs=plt.subplots(1,2,figsize=(9,3.4))
for kp,kv,c,lab in ((0.35,0.0,"tab:red","kp=0.35, no speed term"),(0.15,0.5,"tab:blue","kp=0.15, kv=0.5")):
    r=sim_p(kp,kv); axs[0].plot(r[:,0],r[:,1],c,lw=1.8,label=lab); axs[1].plot(r[:,0],r[:,3],c,lw=1.8,label=lab)
r=sim_p(0.15,0.5); axs[0].plot(r[:,0],r[:,2],"k:",lw=1,label="desired gap")
axs[0].axvspan(5,10,color="tab:orange",alpha=.12); axs[0].set_ylabel("gap [m]"); axs[0].set_xlabel("t [s]"); axs[0].grid(alpha=.3); axs[0].legend(fontsize=7)
axs[1].axhline(-3.5,color="0.5",ls="--",lw=1); axs[1].axhline(2.0,color="0.5",ls="--",lw=1)
axs[1].set_ylabel("accel command [m/s²]"); axs[1].set_xlabel("t [s]"); axs[1].grid(alpha=.3); axs[1].legend(fontsize=7)
fig.suptitle("Toy 1: a proportional law on the gap — the lead brakes from 20 to 5 m/s at −2 m/s²",fontsize=10)
save(fig,"Planner/figures/long_gap_p_toy.png")

# === the MPC grid and the cost scaling ===
T=np.array([10*(i/12)**2 for i in range(13)]); dT=np.diff(T)
fig,ax=plt.subplots(figsize=(7,2.6))
ax.vlines(T,0,1,color="tab:blue",lw=1.5); ax.scatter(T,np.ones_like(T),color="tab:blue",s=14)
for i,(t0,t1) in enumerate(zip(T[:-1],T[1:])): ax.text((t0+t1)/2,0.55,f"{t1-t0:.2f}",ha="center",fontsize=6.5,color="tab:red")
ax.set_ylim(0,1.3); ax.set_yticks([]); ax.set_xlabel("horizon time [s]  — red: interval Δt, the weight of each stage")
ax.set_title("12 intervals on a quadratic grid: 0.07 s at the wheel, 1.6 s at the end of the 10 s horizon")
save(fig,"Planner/figures/long_mpc_grid.png")

# === MPC plan behind a braking lead (numpy Gauss-Newton on the same cost) ===
def rollout(u,v0,a0):
    x=np.zeros(13);v=np.zeros(13);a=np.zeros(13); v[0]=v0;a[0]=a0
    for k in range(12):
        d=dT[k]; x[k+1]=x[k]+v[k]*d+0.5*a[k]*d*d+u[k]*d**3/6; v[k+1]=v[k]+a[k]*d+0.5*u[k]*d*d; a[k+1]=a[k]+u[k]*d
    return x,v,a
def resid(u,v0,a0,xobs,amin=-3.5,amax=1.2):
    x,v,a=rollout(u,v0,a0); sc=np.sqrt(np.append(dT,1.0)); r=[]
    for i in range(13):
        sd=v[i]**2/5+1.45*v[i]+6; g=(xobs[i]-x[i])-sd; h=v[i]+10
        r.append(sc[i]*math.sqrt(3.0)*g/h)
        if i<12: r.append(sc[i]*math.sqrt(5.0)*u[i])
    for i in range(12):
        z=sc[i]*1000.0; r.append(z*max(0,-v[i])); r.append(z*max(0,amin-a[i])); r.append(z*max(0,a[i]-amax))
        sd=v[i]**2/5+1.45*v[i]+6; g=(xobs[i]-x[i])-0.75*sd; r.append(sc[i]*10.0*max(0,-g/(v[i]+10)))
    return np.array(r)
def solve(v0,a0,xobs,iters=60):
    u=np.zeros(12)
    for _ in range(iters):
        r=resid(u,v0,a0,xobs); J=np.zeros((len(r),12)); h=1e-6
        for k in range(12):
            u2=u.copy(); u2[k]+=h; J[:,k]=(resid(u2,v0,a0,xobs)-r)/h
        step=np.linalg.solve(J.T@J+1e-3*np.eye(12),-J.T@r); c0=r@r
        for al in (1,.5,.25,.125,.0625):
            r2=resid(u+al*step,v0,a0,xobs)
            if r2@r2<c0: u=u+al*step; break
        else: break
    return rollout(u,v0,a0)
# lead 35 m ahead at 15 m/s braking −3 m/s² (decaying), ego 20 m/s
xl=np.zeros(13); vl=np.zeros(13); vv=15.0; xx=35.0
for i in range(13):
    d=0 if i==0 else dT[i-1]; al=-3.0*math.exp(-1.5*T[i]**2/2); vv=max(0,vv+d*al); xx+=d*vv; xl[i]=xx; vl[i]=vv
xobs=xl+vl**2/5
x,v,a=solve(20.0,0.0,xobs)
fig,axs=plt.subplots(1,3,figsize=(11,3.3))
axs[0].plot(T,xl,"tab:red",lw=2,label="lead position"); axs[0].plot(T,x,"tab:blue",lw=2,label="ego plan")
axs[0].plot(T,xobs-(v**2/5+1.45*v+6),"k:",lw=1,label="where the gap would be exactly safe")
axs[0].set_xlabel("t [s]"); axs[0].set_ylabel("x [m]"); axs[0].legend(fontsize=7); axs[0].grid(alpha=.3)
axs[1].plot(T,vl,"tab:red",lw=2,label="lead"); axs[1].plot(T,v,"tab:blue",lw=2,label="ego plan"); axs[1].set_xlabel("t [s]"); axs[1].set_ylabel("v [m/s]"); axs[1].legend(fontsize=7); axs[1].grid(alpha=.3)
axs[2].plot(T,a,"tab:blue",lw=2); axs[2].axhline(-3.5,color="0.5",ls="--",lw=1); axs[2].axhline(1.2,color="0.5",ls="--",lw=1)
axs[2].set_xlabel("t [s]"); axs[2].set_ylabel("a [m/s²]"); axs[2].grid(alpha=.3)
fig.suptitle("The MPC plan: lead 35 m ahead braking from 15 m/s, ego at 20 m/s — brake now, settle at the safe gap",fontsize=10)
save(fig,"Planner/figures/long_mpc_plan.png")

# === cruise obstacle ===
v0,vc,amax=10.0,25.0,1.2
vlow=v0+T*(-1.2)*1.05; vup=v0+T*amax*1.05; vcl=np.clip(vc,vlow,vup)
xc=np.cumsum(np.append(0,dT)*vcl)+vcl**2/5+1.45*vcl+6
xobs=np.full(13,1e9); xobs=np.minimum(xobs,xc)
x,v,a=solve(v0,0.0,xc)
fig,axs=plt.subplots(1,2,figsize=(9,3.3))
axs[0].plot(T,xc,"tab:orange",lw=2,label="cruise obstacle"); axs[0].plot(T,x,"tab:blue",lw=2,label="ego plan")
axs[0].set_xlabel("t [s]"); axs[0].set_ylabel("x [m]"); axs[0].legend(fontsize=7); axs[0].grid(alpha=.3)
axs[1].plot(T,vcl,"tab:orange",lw=2,label="cruise speed, clipped to what ego can reach"); axs[1].plot(T,v,"tab:blue",lw=2,label="ego plan")
axs[1].axhline(vc,color="0.5",ls=":",lw=1); axs[1].set_xlabel("t [s]"); axs[1].set_ylabel("v [m/s]"); axs[1].legend(fontsize=7); axs[1].grid(alpha=.3)
fig.suptitle("No lead: the set speed is a phantom car already at 25 m/s — the plan chases it at ≤1.2 m/s²",fontsize=10)
save(fig,"Planner/figures/long_cruise_obstacle.png")

# === control law: delay compensation + states ===
Tm=np.array([10*(i/32)**2 for i in range(33)])[:17]
speeds=np.maximum(0,20-1.5*Tm)  # a braking plan
def interp(x,xp,fp): return np.interp(x,xp,fp)
fig,axs=plt.subplots(1,2,figsize=(9,3.3))
axs[0].plot(Tm,speeds,"tab:blue",lw=2,label="plan speeds"); 
for d,c in ((0.0,"0.5"),(0.15,"tab:red")):
    vt=interp(d,Tm,speeds); axs[0].scatter([d],[vt],color=c,zorder=3)
vn=interp(0,Tm,speeds); vt=interp(0.15,Tm,speeds); an=-1.5
axs[0].annotate(f"a_target = 2·(v(0.15)−v(0))/0.15 − a(0)\n= {2*(vt-vn)/0.15-an:.2f} m/s²",(0.15,vt),(0.8,19.3),fontsize=8,arrowprops=dict(arrowstyle="->"))
axs[0].set_xlim(0,2.6); axs[0].set_xlabel("t [s]"); axs[0].set_ylabel("v [m/s]"); axs[0].grid(alpha=.3); axs[0].set_title("Read the plan 0.15 s ahead — the actuator's delay",fontsize=9)
# state machine timeline
t=np.linspace(0,14,600); v=np.where(t<6,np.maximum(0,8-1.6*t),np.where(t<9,0,np.minimum(8,1.0*(t-9))))
st=np.where(v>1.0,1,np.where(t<9,2,3)); st[t<0.5]=1
axs[1].plot(t,v,"tab:blue",lw=2,label="v_ego"); ax2=axs[1].twinx(); ax2.step(t,st,"tab:red",lw=1.5,where="post"); ax2.set_yticks([1,2,3]); ax2.set_yticklabels(["pid","stopping","starting"],color="tab:red",fontsize=8)
axs[1].set_xlabel("t [s]"); axs[1].set_ylabel("v [m/s]"); axs[1].grid(alpha=.3); axs[1].set_title("pid → stopping (v<1, plan at rest) → starting (plan goes) → pid (v>1)",fontsize=9)
save(fig,"Planner/figures/long_control_law.png")

# === sim results table figure ===
scen=["cruise","lead 15 m/s","lead brakes 20→5","lead stops","stopped car"]
mingap=[np.nan,27.7,10.3,4.6,5.4]; desired=[np.nan,27.7,13.3,6.0,6.0]; amin=[-0.07,-2.42,-2.52,-2.90,-2.44]
fig,axs=plt.subplots(1,2,figsize=(9,3.2))
xi=np.arange(5)
axs[0].bar(xi-0.18,mingap,0.36,color="tab:blue",label="min gap [m]"); axs[0].bar(xi+0.18,desired,0.36,color="0.7",label="desired at the end [m]")
axs[0].set_xticks(xi); axs[0].set_xticklabels(scen,rotation=20,fontsize=8); axs[0].legend(fontsize=8); axs[0].grid(alpha=.3,axis="y")
axs[1].bar(xi,amin,color="tab:red"); axs[1].axhline(-3.5,color="k",ls="--",lw=1); axs[1].text(0.1,-3.4,"panda floor −3.5",fontsize=7)
axs[1].set_xticks(xi); axs[1].set_xticklabels(scen,rotation=20,fontsize=8); axs[1].set_ylabel("strongest deceleration [m/s²]"); axs[1].grid(alpha=.3,axis="y")
fig.suptitle("MetaDrive, sim.eval --scenario …: five encounters, no contact, envelope respected",fontsize=10)
save(fig,"Planner/figures/long_sim_scenarios.png")
print("make_long_figures: 7 figures into both trees")
