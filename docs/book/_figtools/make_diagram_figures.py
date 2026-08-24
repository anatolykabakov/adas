import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mp
import numpy as np, pathlib

EN = pathlib.Path("/home/anatoly/atom/adas/docs/book")
def save(fig, rel):
    for tree in (EN, EN.parent/"book_ru"):
        p = tree/rel; p.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(p, dpi=130, bbox_inches="tight", facecolor="white")
    plt.close(fig)

# 1. LanePath: two lines + plan cutting the arc + fused references
fig, ax = plt.subplots(figsize=(7,4))
x = np.linspace(0,40,81); c = 0.5*0.004*x**2  # gentle left arc centre (device y right+, up=+)
L = c + 1.75; R = c - 1.75
plan = c - 0.40*(x/40)                          # plan cuts inside
for b,ls in ((0.0,":"),(0.6,"--"),(1.0,"-")):
    fused = b*c + (1-b)*plan
    ax.plot(x, fused, ls, color="tab:blue", lw=2, label=f"fused, blend={b}")
ax.plot(x, L, color="0.6"); ax.plot(x, R, color="0.6")
ax.plot(x, c, color="tab:green", lw=1, label="true lane centre")
ax.plot(x, plan, color="tab:red", lw=1.5, label="model plan (cuts in)")
ax.set_xlabel("distance ahead x [m]"); ax.set_ylabel("lateral y [m]")
ax.legend(loc="upper left", fontsize=8); ax.grid(alpha=.3)
ax.set_title("Lane fusion on an arc: blend=0 rides the cutting plan, blend=1 the paint")
save(fig, "Planner/figures/lanepath_blend.png")

# 2. Friction deadband
fig, ax = plt.subplots(figsize=(6,3.6))
err = np.linspace(-3,3,400); KP=180.0; Tsat=0.0167*15**2; Tfric=57.0; Tmax=300.0
# steady applied torque a P controller reaches = clip(KP*err) but only moves rack if |net|>Tfric
cmd = np.clip(KP*err, -Tmax, Tmax)
dead = (Tsat+Tfric)  # in cNm terms at swa~err; deadband half-width in deg:
db = dead/KP
ax.plot(err, cmd, color="tab:blue", lw=2, label="P command = kp·error")
ax.axvspan(-db, db, color="tab:red", alpha=.15)
ax.axhline(Tfric+Tsat, color="0.5", ls=":"); ax.axhline(-(Tfric+Tsat), color="0.5", ls=":")
ax.annotate("rack does not move\n(|torque| < friction+self-align)", (0,0), (0.15,120),
            fontsize=8, ha="left", color="tab:red")
ax.set_xlabel("angle error [deg]"); ax.set_ylabel("commanded torque [cNm]")
ax.set_title("P alone: a friction deadband it cannot close"); ax.grid(alpha=.3); ax.legend(fontsize=8)
save(fig, "Control/figures/friction_deadband.png")

# 3. Supercombo 6504 layout bar
fig, ax = plt.subplots(figsize=(9,1.9))
slices = [("plan\n(x,y,ψ…)",0,990,"tab:blue"),("lane\nlines",990,2154,"tab:green"),
          ("lane σ\n(log)",2154,2286,"tab:olive"),("road\nedges",2286,2418,"tab:brown"),
          ("leads",2418,2622,"tab:red"),("pose /\nodometry",5755,5875,"tab:purple"),
          ("features\nbuffer",5875,6504,"0.6")]
for name,a,b,col in slices:
    ax.add_patch(mp.Rectangle((a,0), b-a, 1, facecolor=col, alpha=.5, edgecolor="k"))
    ax.text((a+b)/2, 0.5, name, ha="center", va="center", fontsize=7)
ax.set_xlim(0,6504); ax.set_ylim(0,1); ax.set_yticks([])
ax.set_xlabel("index into the 6504-element output vector (offsets are illustrative)")
ax.set_title("Supercombo output: one flat vector, read by slice — the most misread thing in the project")
save(fig, "Vision/figures/supercombo_layout.png")

# 4. Control cascade with rates
fig, ax = plt.subplots(figsize=(9,2.4)); ax.axis("off")
boxes = [("Planner\nκ*","~24 Hz\n(vision)"),("Vehicle model\nκ*→δ","~24 Hz"),
         ("Angle PID\nδ→steer_norm","~100 Hz\n(chassis)"),("Platform\n×300, panda limit","50 Hz\n(HCA)"),
         ("EPS\nrack","car")]
xs = np.linspace(0.02,0.82,5)
for (t,r),x in zip(boxes,xs):
    ax.add_patch(mp.FancyBboxPatch((x,0.35),0.15,0.3, boxstyle="round,pad=0.01",
                 facecolor="tab:blue", alpha=.15, edgecolor="k"))
    ax.text(x+0.075,0.5,t,ha="center",va="center",fontsize=8)
    ax.text(x+0.075,0.72,r,ha="center",va="center",fontsize=7,color="0.4")
    if x<xs[-1]: ax.annotate("",(x+0.19,0.5),(x+0.15,0.5),arrowprops=dict(arrowstyle="->"))
ax.set_xlim(0,1); ax.set_ylim(0,1)
ax.set_title("The lateral cascade: each stage runs at its own rate", fontsize=10)
save(fig, "Control/figures/control_cascade.png")

# 5. HCA_01 bit-field
fig, ax = plt.subplots(figsize=(9,2.2)); ax.axis("off")
# 8 bytes, byte0=checksum, byte1: counter(low nibble)+const, byte2/3: magnitude 14b + sign + lkas
fields = [(0,8,"CRC-8\nchecksum","tab:red"),(8,4,"counter","tab:blue"),(12,4,"const","0.7"),
          (16,14,"|torque| (14 bits)","tab:green"),(30,1,"lkas","tab:olive"),(31,1,"sign","tab:orange"),
          (32,32,"… status / const …","0.85")]
for start,ln,name,col in fields:
    ax.add_patch(mp.Rectangle((start,0), ln, 1, facecolor=col, alpha=.5, edgecolor="k"))
    ax.text(start+ln/2,0.5,name,ha="center",va="center",fontsize=7)
ax.set_xlim(0,64); ax.set_ylim(-0.3,1.2)
for b in range(0,65,8):
    ax.text(b,1.12,f"bit {b}",fontsize=6,ha="center",color="0.4")
ax.set_title("HCA_01 payload: magnitude and sign split, counter + CRC the EPS checks")
save(fig, "Platform/figures/hca01_bits.png")

# 6. Pinhole + vanishing point sign
fig, axes = plt.subplots(1,2,figsize=(9,3.4))
a=axes[0]  # pinhole
a.plot([0,6],[1.5,0.2],"tab:green"); a.plot([0,6],[-1.5,-0.2],"tab:green")
a.plot([0,6],[0,0],"k--",lw=.8); a.scatter([0],[0],c="k"); a.text(0,0.15,"camera",fontsize=8)
a.text(6,0.25,"vanishing\npoint",fontsize=8,color="tab:red"); a.scatter([6],[0],c="tab:red",zorder=5)
a.set_title("Parallel road lines meet at the VP"); a.axis("equal"); a.axis("off")
b=axes[1]  # VP shifts with yaw
u=np.linspace(0,1280,50)
for yaw,col,lab in ((0,"0.5","yaw 0 → VP centre"),(1.67,"tab:blue","yaw +1.67° → VP left")):
    vp=640-yaw*17.4  # ~px/deg illustrative
    b.plot([0,vp],[720,360],col); b.plot([1280,vp],[720,360],col)
    b.scatter([vp],[360],c=col,zorder=5)
b.text(300,380,"yaw 0",fontsize=8,color="0.5"); b.text(120,400,"yaw +1.67°",fontsize=8,color="tab:blue")
b.set_xlim(0,1280); b.set_ylim(720,300); b.set_title("Camera yaw shifts the VP sideways")
b.set_xlabel("u [px]"); b.set_ylabel("v [px]")
save(fig, "Calibration/figures/pinhole_vp.png")

print("figures generated for both trees")
