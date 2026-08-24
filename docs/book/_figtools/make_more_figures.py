import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt, matplotlib.patches as mp, numpy as np, math, pathlib
EN=pathlib.Path(__file__).resolve().parents[1]
def save(fig, rel):
    for t in (EN, EN.parent/"book_ru"):
        p=t/rel; p.parent.mkdir(parents=True,exist_ok=True); fig.savefig(p,dpi=130,bbox_inches="tight",facecolor="white")
    plt.close(fig)

# === LanePath: σ -> blend weight (the veto edge) ===
def soft(p,m=0.3): return 0.0 if p<m else min(1.0,(p-m)/(1-m))
def stdc(s,g=0.15,b=0.30): return 1.0 if s<=g else (0.0 if s>=b else (b-s)/(b-g))
sig=np.linspace(0,0.5,200)
fig,ax=plt.subplots(figsize=(6.5,3.2))
w=[ (soft(0.95)*stdc(s)) for s in sig]
dprob=[(2*x-x*x)*0.6 for x in w]
ax.plot(sig,dprob,"tab:blue",lw=2,label="blend weight d_prob (blend=0.6)")
ax.axvspan(0.15,0.30,color="tab:orange",alpha=.15)
ax.axvline(0.37,color="tab:red",ls="--",lw=1); ax.text(0.375,0.4,"left σ 0.37\n(drive)",fontsize=7,color="tab:red")
ax.axvline(0.51,color="tab:red",ls=":",lw=1); ax.text(0.44,0.15,"right 0.51",fontsize=7,color="tab:red")
ax.set_xlabel("lane-line σ [m]"); ax.set_ylabel("d_prob"); ax.grid(alpha=.3); ax.legend(fontsize=8)
ax.set_title("σ veto: past 0.30 m the lines get zero weight — the plan takes over")
save(fig,"Planner/figures/lanepath_sigma.png")

# === MPC: cost J(δ) with minimum near δ_ff (step 3) ===
L=2.64;N=20;DS=0.5;K_US=0.0015;FF=2.0;WD=45000.;WDD=15000.
def cost(cte0,epsi0,kappa,deltas,v=15.):
    v2=v*v; curve=1+20*abs(kappa); cw=20*curve; ew=10*curve; cte,epsi=cte0,epsi0; j=0
    for s,d in enumerate(deltas):
        j+=cw*cte**2+cw*5*cte**4+ew*epsi**2
        dff=FF*(math.atan(L*kappa)+K_US*v2*kappa); j+=WD*(d-dff)**2
        if s>0: j+=WDD*min(max(v2,9),25)*(d-deltas[s-1])**2
        cte-=math.sin(epsi)*DS; epsi+=(kappa-math.tan(d)/L)*DS
    return j
kappa=0.02; dff=FF*(math.atan(L*kappa)+K_US*15**2*kappa)
ds=np.linspace(-0.02,0.06,120); J=[cost(0.2,0.,kappa,[d]*N) for d in ds]
fig,ax=plt.subplots(figsize=(6.5,3.2))
ax.plot(np.degrees(ds),J,"tab:blue",lw=2); ax.axvline(math.degrees(dff),color="tab:red",ls="--")
ax.text(math.degrees(dff)+0.1,max(J)*0.6,"δ_ff",color="tab:red",fontsize=9)
ax.set_xlabel("trial wheel angle δ [deg]"); ax.set_ylabel("cost J"); ax.grid(alpha=.3)
ax.set_title("The δ-pin weight (45000) makes the optimum hug the feed-forward")
save(fig,"Planner/figures/mpc_cost.png")

# === VehicleModel: commanded δ inflated by understeer vs speed ===
def dcmd(kappa,v,tsf): 
    K=0.0015/tsf; return math.degrees(math.atan(L*kappa))*(1+K*v*v)
v=np.linspace(3,27,100)
fig,ax=plt.subplots(figsize=(6.5,3.2))
for tsf,col in ((0.64,"tab:blue"),(0.50,"tab:red")):
    ax.plot(v,[dcmd(1/200.,x,tsf) for x in v],col,label=f"tsf={tsf}")
ax.plot(v,[math.degrees(math.atan(L/200.))]*len(v),"0.6",ls=":",label="kinematic (no understeer)")
ax.set_xlabel("speed [m/s]"); ax.set_ylabel("commanded δ on R=200 m [deg]"); ax.grid(alpha=.3); ax.legend(fontsize=8)
ax.set_title("Understeer inflates the commanded angle, more so at speed")
save(fig,"Control/figures/understeer_command.png")

# === VehicleModel: outward drift Δy≈½(1−r)κs² ===
s=np.linspace(0,60,100)
fig,ax=plt.subplots(figsize=(6.5,3.2))
for (R,r,lab) in ((200,0.80,"R=200 m, r=0.80"),(97,0.60,"R=97 m, r=0.60")):
    ax.plot(s,0.5*(1-r)*(1/R)*s**2,label=lab)
ax.set_xlabel("arc length travelled [m]"); ax.set_ylabel("outward drift Δy [m]"); ax.grid(alpha=.3); ax.legend(fontsize=8)
ax.set_title("Open-loop understeer drift grows with arc length squared")
save(fig,"Planner/figures/understeer_drift.png")

# === AngleControl: feedforward cNm vs speed, v0=0 vs 9.8 ===
KF=6e-5; T=300.
v=np.linspace(2,26,100)
fig,ax=plt.subplots(figsize=(6.5,3.2))
ax.plot(v,[KF*5*(x*x)*T for x in v],"tab:red",label="v0=0 (shipped config)")
ax.plot(v,[KF*5*(x*x+9.8**2)*T for x in v],"tab:blue",label="v0=9.8 (code default)")
ax.axhline(57,color="0.5",ls=":",label="rack friction ≈57 cNm")
ax.set_xlabel("speed [m/s]"); ax.set_ylabel("feedforward at SWA=5° [cNm]"); ax.grid(alpha=.3); ax.legend(fontsize=8)
ax.set_title("Feedforward is buried under friction at low speed when v0=0")
save(fig,"Control/figures/feedforward.png")

# === Calibration: yaw→lateral (∝d), pitch→ground (∝d²/h) ===
H=1.10
d=np.linspace(0,60,100)
fig,(a1,a2)=plt.subplots(1,2,figsize=(9,3.2))
for e,c in ((0.5,"tab:blue"),(1.0,"tab:red")):
    a1.plot(d,d*math.tan(math.radians(e)),c,label=f"{e}° yaw")
a1.set_title("Yaw error → lateral (∝ d)"); a1.set_xlabel("range d [m]"); a1.set_ylabel("lateral error [m]"); a1.legend(fontsize=8); a1.grid(alpha=.3)
for e,c in ((0.5,"tab:blue"),(1.0,"tab:red")):
    a2.plot(d,d*d*math.radians(e)/H,c,label=f"{e}° pitch")
a2.set_title("Pitch error → range (∝ d²/h)"); a2.set_xlabel("range d [m]"); a2.set_ylabel("apparent range error [m]"); a2.legend(fontsize=8); a2.grid(alpha=.3)
save(fig,"Calibration/figures/error_growth.png")

# === Calibration: online calib convergence, lateral error at 20 m ===
def conv(prior,truth,tau,t): return truth+(prior-truth)*math.exp(-t/tau)
t=np.linspace(0,60,200)
fig,ax=plt.subplots(figsize=(6.5,3.2))
est=[conv(1.67,0.10,15,x) for x in t]
ax.plot(t,est,"tab:blue",label="yaw estimate [deg]")
ax2=ax.twinx(); ax2.plot(t,[20*math.tan(math.radians(e-0.10)) for e in est],"tab:red",lw=1,ls="--")
ax2.set_ylabel("lateral error at 20 m [m]",color="tab:red")
ax.set_xlabel("time since cold start [s]"); ax.set_ylabel("yaw [deg]",color="tab:blue"); ax.grid(alpha=.3)
ax.set_title("Online calibration settles in 30–60 s: do not judge the first minute")
save(fig,"Calibration/figures/calib_convergence.png")

# === IntrinsicsAndWarp: focal length -> FOV ===
W=1280
fig,ax=plt.subplots(figsize=(6.5,3.2))
fx=np.linspace(880,1020,100); fov=2*np.degrees(np.arctan(W/(2*fx)))
ax.plot(fx,fov,"tab:blue")
for f,lab in ((951,"config 951"),(930,"flowpilot 930"),(993.4,"measured 993.4")):
    ax.scatter([f],[2*math.degrees(math.atan(W/(2*f)))]); ax.annotate(lab,(f,2*math.degrees(math.atan(W/(2*f)))),fontsize=7)
ax.axhline(65.2,color="tab:green",ls=":",label="EON training FOV 65.2°")
ax.set_xlabel("focal length fx [px]"); ax.set_ylabel("horizontal FOV [deg]"); ax.grid(alpha=.3); ax.legend(fontsize=8)
ax.set_title("What mattered was the field of view, not the focal length")
save(fig,"Calibration/figures/focal_fov.png")

# === Safety: TTC and a_req vs distance ===
fig,(a1,a2)=plt.subplots(1,2,figsize=(9,3.2))
d=np.linspace(3,60,200)
for vrel,c in ((5,"tab:blue"),(10,"tab:red")):
    a1.plot(d,d/vrel,c,label=f"closing {vrel} m/s")
a1.axhline(2.5,color="0.5",ls=":",label="FCW ≤2.5 s"); a1.set_ylim(0,8)
a1.set_title("Time-to-collision"); a1.set_xlabel("gap [m]"); a1.set_ylabel("TTC [s]"); a1.legend(fontsize=8); a1.grid(alpha=.3)
for vrel,c in ((5,"tab:blue"),(10,"tab:red")):
    a2.plot(d,vrel*vrel/(2*d),c,label=f"closing {vrel} m/s")
a2.axhline(3.5,color="0.5",ls=":",label="FCW a_req ≥3.5 m/s²"); a2.set_ylim(0,10)
a2.set_title("Required deceleration"); a2.set_xlabel("gap [m]"); a2.set_ylabel("a_req [m/s²]"); a2.legend(fontsize=8); a2.grid(alpha=.3)
save(fig,"Safety/figures/fcw_thresholds.png")

# === Safety: path-relative y on an arc (why axis comparison false-triggers) ===
d=np.linspace(0,50,100); kappa=1/150.; cte=0.1
fig,ax=plt.subplots(figsize=(6.5,3.2))
ax.plot(d,cte+0.5*kappa*d*d,"tab:blue",label="lane edge along the arc: CTE+½κd²")
ax.axhline(1.75,color="0.5",ls=":",label="half-lane 1.75 m")
ax.fill_between(d, 1.75, cte+0.5*kappa*d*d, where=(cte+0.5*kappa*d*d>1.75), color="tab:red",alpha=.2)
ax.set_xlabel("distance ahead [m]"); ax.set_ylabel("lateral y [m]"); ax.grid(alpha=.3); ax.legend(fontsize=8)
ax.set_title("On a bend the ego lane's edge crosses a straight-ahead line — a false departure")
save(fig,"Safety/figures/path_relative_y.png")

# === Localization: a_ego finite-difference vs filtered (schematic of the RMS 4.16 vs 0.062) ===
rng=np.random.default_rng(3); t=np.arange(0,20,0.1); v=12+2*np.sin(t*0.4)
vn=v+rng.normal(0,0.25,len(t))
a_fd=np.diff(vn)/0.1
a_filt=np.diff(v)/0.1  # what a two-state filter recovers (clean truth-ish)
fig,ax=plt.subplots(figsize=(7,3.2))
ax.plot(t[1:],a_fd,color="tab:red",lw=.8,label="finite difference (RMS 4.16 m/s²)")
ax.plot(t[1:],a_filt,color="tab:blue",lw=1.5,label="two-state filter (RMS 0.062)")
ax.set_ylim(-8,8); ax.set_xlabel("time [s]"); ax.set_ylabel("a_ego [m/s²]"); ax.grid(alpha=.3); ax.legend(fontsize=8)
ax.set_title("Differentiating noisy speed explodes; filtering it does not (schematic)")
save(fig,"Localization/figures/a_ego.png")

# === Localization: ENU frame vs GPS bearing vs vehicle (diagram) ===
fig,ax=plt.subplots(figsize=(5.2,4.2)); ax.set_xlim(-0.6,3.8); ax.set_ylim(-0.8,3.8); ax.set_aspect("equal"); ax.axis("off")
ax.annotate("",(3,0),(0,0),arrowprops=dict(arrowstyle="->")); ax.text(3.1,0,"E",fontsize=10)
ax.annotate("",(0,3),(0,0),arrowprops=dict(arrowstyle="->")); ax.text(0,3.1,"N",fontsize=10)
th=math.radians(35)
ax.annotate("",(2*math.cos(th),2*math.sin(th)),(0,0),arrowprops=dict(arrowstyle="->",color="tab:blue"))
ax.text(2*math.cos(th),2*math.sin(th)+0.1,"vehicle heading ψ",color="tab:blue",fontsize=8)
ax.annotate("",(2*math.sin(th),2*math.cos(th)),(0,0),arrowprops=dict(arrowstyle="->",color="tab:red"))
ax.text(2*math.sin(th)+0.1,2*math.cos(th),"GPS bearing b",color="tab:red",fontsize=8)
ax.text(0.3,-0.4,"ψ = π/2 − b  (bearing is from North, heading from East)",fontsize=8)
ax.set_title("Three frames: ENU axes, GPS bearing, vehicle heading")
save(fig,"Localization/figures/frames.png")

# === Localization: understeer ratio across speed (step 6 multi-run) + shipped/best tsf ===
vb=[6,10,14,18,22,26]; ratio=[0.89,0.84,0.84,0.65,0.58,0.46]
fig,ax=plt.subplots(figsize=(6.5,3.2))
ax.plot(vb,ratio,"o-",color="tab:blue",label="measured κ_fact/κ_kin (step 6 run)")
ax.axhline(1.0,color="0.6",ls=":",label="kinematic")
ax.set_xlabel("speed [m/s]"); ax.set_ylabel("ratio"); ax.set_ylim(0,1.1); ax.grid(alpha=.3); ax.legend(fontsize=8)
ax.set_title("Why a single tire_stiffness_factor cannot fit: the ratio slides with speed")
save(fig,"Localization/figures/understeer_speed.png")

print("make_more_figures: 14 figures into both trees")
