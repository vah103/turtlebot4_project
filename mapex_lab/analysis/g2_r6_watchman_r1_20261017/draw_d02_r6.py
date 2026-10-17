"""Visualize frozen causal D02 data. Do not use GT / later observations."""
from pathlib import Path
from io import BytesIO
import base64
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
from PIL import Image

from trial_d02 import make

ROOT = Path(__file__).resolve().parent
p, obs, native, sem = make(2)
plan = p.plan()
goals = [tuple(g) for g in plan["goal_sequence_core_rc"]]
route = p._route_order(goals)["route"]
nominal, stress = p._route_order(goals)["cover"]
candidate = p.candidates()
robot = p.robot
print("bounds", len(candidate), len(goals), len(route),int(nominal.sum()),int(stress.sum()))
np.savez_compressed(ROOT/"D02_R6_VISUAL_EVIDENCE.npz",
    observed=obs, mean=p.mean, variance=p.var, semantic=sem,
    room_mask=p.room.astype(np.uint8), native_free=p.legal.astype(np.uint8),
    predicted_nominal=nominal.astype(np.uint8), predicted_stress=stress.astype(np.uint8),
    candidate_core_rc=np.asarray(candidate,np.int16),
    goal_sequence_core_rc=np.asarray(goals,np.int16),
    route_core_rc=np.asarray(route,np.int16),
    robot_core_rc=np.asarray(robot,np.int16))
np.savez_compressed(ROOT/"D02_R6_CAUSAL_MAP_ONLY.npz",
    observed=obs,mean=p.mean,variance=p.var,semantic=sem,
    native_walkable=native,robot_core_rc=np.asarray(robot,np.int16))

plt.rcParams.update({"font.family":"DejaVu Sans", "font.size":11,
                     "axes.titleweight":"bold", "figure.facecolor":"#F5F7F9",
                     "savefig.facecolor":"#F5F7F9"})
colors = np.array([[243,246,249],[193,205,216],[53,63,73]],dtype=np.uint8)
rgb = colors[obs]
# Outside the selected room leave lightly washed-out, to focus on D02.
base = rgb.copy()
mask = p.room
base[~mask] = np.round(.45*base[~mask]+.55*np.array([249,249,250])).astype(np.uint8)

x0,x1,y0,y1 = 83,222,135,203
roi = (slice(y0,y1),slice(x0,x1))
extent=[x0-.5,x1-.5,y1-.5,y0-.5]

def axes_style(ax):
    ax.set_xlim(x0,x1)
    ax.set_ylim(y1,y0)
    ax.set_xlabel("Grid col (x)")
    ax.set_ylabel("Grid row (y)")
    ax.set_xticks([90,110,130,150,170,190,210])
    ax.set_yticks([140,150,160,170,180,190,200])
    ax.grid(color="#69717a",linewidth=.25,alpha=.23)
    ax.set_aspect("equal")
    ax.contour(np.arange(obs.shape[1]),np.arange(obs.shape[0]),p.room.astype(float),
               levels=[.5],colors=["#D94343"],linewidths=1.55)

def show_base(ax):
    ax.imshow(base[roi],extent=extent,interpolation="nearest")
    axes_style(ax)

def mark_robot(ax):
    ax.scatter([robot[1]],[robot[0]],s=140,marker="o",facecolor="#FAA43A",
                edgecolor="#563809",linewidth=1.8,zorder=20)
    ax.text(robot[1]-2.5,robot[0]-5.0,"START",color="#88580D",fontweight="bold",fontsize=9,zorder=21)

def mark_goals(ax):
    for i,(y,x) in enumerate(goals,1):
        ax.scatter([x],[y],s=220,marker="o",facecolor="#6139D1",
                   edgecolor="white",linewidth=2,zorder=30)
        ax.text(x,y,str(i),fontsize=10,color="white",fontweight="bold",
                horizontalalignment="center",verticalalignment="center",zorder=31)

def draw_route(ax):
    v=np.asarray(route)
    ax.plot(v[:,1],v[:,0],color="#FF9F42",lw=3.8,zorder=14,solid_joinstyle="round")
    ax.plot(v[:,1],v[:,0],color="#C65D15",lw=.65,zorder=15,alpha=.7)

fig, ax = plt.subplots(2,2,figsize=(15.2,10.8),dpi=115)
fig.suptitle("New Room D02 | G2-R6 offline Watchman planning",fontweight="bold",fontsize=21,color="#263341")
show_base(ax[0,0]);mark_robot(ax[0,0])
ax[0,0].set_title("1. Observed LiDAR at D02 (no future scans)")
ax[0,0].text(.02,.02,"Pale: FREE   Blue-gray: UNKNOWN   Dark: wall   Red: Room boundary",
              transform=ax[0,0].transAxes,fontsize=8,bbox=dict(facecolor="white",alpha=.8,edgecolor="none"))

# LaMa occupancy only in UNKNOWN; observed remains ground evidence.
show_base(ax[0,1])
loc = p.room & (obs==1)
mval=np.ma.array(p.mean,mask=~loc)
im=ax[0,1].imshow(mval[roi],extent=extent,vmin=0,vmax=1,cmap="RdYlBu_r",alpha=.90,interpolation="nearest")
mark_robot(ax[0,1])
ax[0,1].set_title("2. LaMa mean occupancy (UNKNOWN cells)")
cax=fig.add_axes([.67,.505,.23,.015])
cbar=fig.colorbar(im,cax=cax,orientation="horizontal")
cbar.set_label("LaMa occupancy: blue = FREE, red = WALL",fontsize=9)

show_base(ax[1,0])
for y,x in candidate:
    ax[1,0].scatter([x],[y],marker="x",color="#008EAE",s=36,lw=1.4,zorder=10)
draw_route(ax[1,0]);mark_robot(ax[1,0]);mark_goals(ax[1,0])
ax[1,0].set_title("3. 17 candidate viewpoints | 5 selected goals")
ax[1,0].text(.02,.02,"Orange line = planned FREE-cell route (15.5 m)\n"
              "Cyan X = proposals; purple numbers = visit order",
              transform=ax[1,0].transAxes,fontsize=8,bbox=dict(facecolor="white",alpha=.85,edgecolor="none"))

show_base(ax[1,1])
# Separate stress from nominal; orange remaining cells are not observed (even when predicted).
overlay = np.zeros((*obs.shape,4),dtype=np.float32)
unknown = p.mission
overlay[unknown] = [1.0,.73,.33,.85]  # unresolved / unpredicted visible
overlay[unknown&nominal] = [.31,.76,.66,.9]  # predicted nominal visible
overlay[unknown&stress] = [.17,.52,.80,.93]  # stress visible
ax[1,1].imshow(overlay[roi],extent=extent,interpolation="nearest")
draw_route(ax[1,1]);mark_robot(ax[1,1]);mark_goals(ax[1,1])
ax[1,1].set_title("4. Predicted visibility ALONG the route")
ax[1,1].text(.02,.02,"Blue = stress visible, green = nominal only,\n"
              "orange = not projected visible. NOT actual coverage.",
              transform=ax[1,1].transAxes,fontsize=8,bbox=dict(facecolor="white",alpha=.88,edgecolor="none"))
fig.subplots_adjust(left=.06,right=.965,top=.89,bottom=.13,hspace=.39,wspace=.14)
fig.text(.5,.04,
         "Frozen Run11 D02 observed/mean/variance/Room Node   |   "
         "No GT or future LiDAR used   |   G2-R6 planning only, NOT a closed-loop run",
         fontsize=10,ha="center",color="#555E69")

# PNG color palette for a modest standalone SVG with embedded raster.
def save_png_svg(fig,name,palette_colors=128):
    stream=BytesIO()
    fig.savefig(stream,format="png",dpi=110,facecolor="#F5F7F9")
    stream.seek(0)
    image=Image.open(stream).convert("RGB")
    palette=image.quantize(colors=palette_colors,method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
    out=ROOT/(name+".png")
    palette.save(out,optimize=True,compress_level=9)
    raw=out.read_bytes()
    w,h=image.size
    encoded=base64.b64encode(raw).decode("ascii")
    wrap="\n".join(encoded[i:i+110] for i in range(0,len(encoded),110))
    svg = ('<svg xmlns="http://www.w3.org/2000/svg" width="'+str(w)+
           '" height="'+str(h)+'" viewBox="0 0 '+str(w)+' '+str(h)+'">\n'
           '<image width="'+str(w)+'" height="'+str(h)+
           '" href="data:image/png;base64,\n'+wrap+'"/>\n</svg>\n')
    svgpath=ROOT/(name+".svg")
    svgpath.write_text(svg,encoding="utf8")
    print(out.name,len(raw),"SVG",len(svg),w,h)
    return svgpath
save_png_svg(fig,"D02_G2_R6_4_PANEL")
plt.close(fig)

fig, ax=plt.subplots(figsize=(13.2,6.6),dpi=125)
fig.suptitle("G2-R6 | D02: chosen Watchman route in observed FREE",fontsize=19,color="#2F3949")
show_base(ax)
for y,x in candidate:
    ax.scatter([x],[y],marker="x",color="#008EAE",s=60,lw=1.6,zorder=10)
draw_route(ax);mark_robot(ax);mark_goals(ax)
ax.set_title("5 selected goals, 15.5 m model route; no future scan used",fontsize=12,fontweight="normal")
fig.tight_layout()
save_png_svg(fig,"D02_G2_R6_ROUTE_DETAIL")
plt.close(fig)
print("RESULT:",json.dumps({k:plan[k] for k in ("mission_unknown","projected_unique_unknown_nominal",
  "projected_unique_unknown_stress","goal_sequence_core_rc","predicted_route_distance_m",
  "extra_G2_planning_wall_s")},ensure_ascii=False))