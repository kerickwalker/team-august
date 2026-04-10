"""
plot_field_3d.py
================
Reads from the FIELD object and draws the field in 3D.
Coordinates come entirely from objects in field_map_2026.py.
Usage: python3 plot_field_3d.py
"""
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math, re
from field_map_2026 import FIELD, RC, RR, inf_cx, inf_cy, inf_rx, inf_ry

fig = plt.figure(figsize=(12, 9))
ax  = fig.add_subplot(111, projection='3d')
fig.patch.set_facecolor('#1a1a1a')
ax.set_facecolor('#2e2e2e')
ax.set_xlim(0, 7); ax.set_ylim(0, 6); ax.set_zlim(0, 2)
ax.set_xlabel('X (m)', color='white', labelpad=8)
ax.set_ylabel('Y (m)', color='white', labelpad=8)
ax.set_zlabel('Z (m)', color='white', labelpad=8)
ax.set_title('Field Map 3D', color='white', fontsize=13)
ax.tick_params(colors='white')
ax.xaxis.pane.fill = False; ax.yaxis.pane.fill = False; ax.zaxis.pane.fill = False
ax.xaxis.pane.set_edgecolor('#444'); ax.yaxis.pane.set_edgecolor('#444'); ax.zaxis.pane.set_edgecolor('#444')
ax.grid(True, alpha=0.15, color='white')
ax.view_init(elev=35, azim=-60)

_off = 0.035

# ── INFO BOXES ────────────────────────────────────────────────────────
coord_text = fig.text(0.02, 0.97, '',
                      color='white', fontsize=10, fontweight='bold', va='top',
                      family='monospace',
                      bbox=dict(boxstyle='round,pad=0.5', facecolor='#111', alpha=0.85))

info_text = fig.text(0.02, 0.88, '',
                     color='#aaddff', fontsize=9, va='top', family='monospace',
                     bbox=dict(boxstyle='round,pad=0.5', facecolor='#0a0a1a', alpha=0.80))

def _find_tape_info(name):
    for tl in FIELD.tape_lines:
        if tl.name == name or name.endswith('/' + tl.name):
            return {'task': tl.task, 'connects': tl.connects}
    for np_ in FIELD.nav_paths:
        for seg in np_.segments:
            sn = getattr(seg, 'name', '')
            if sn == name or name == f'{np_.name}/{sn}':
                return {'task': np_.task, 'connects': np_.connects}
    return {}

def _update_info(px, py):
    if not (0 <= px <= 7 and 0 <= py <= 6):
        return
    pz   = FIELD.pz_at(px, py)
    zone = FIELD.current_zone(px, py)
    coord_text.set_text(f'x={px:.3f}  y={py:.3f}  z={pz:.3f}m   zone: {zone}')
    tape = FIELD.nearest_tape_segment(px, py)
    gate, gate_dist = FIELD.nearest_gate(px, py)
    aruco = FIELD.nearby_aruco(px, py, radius=1.0)
    lines = []
    if tape:
        lines.append(f'── TAPE ──────────────')
        lines.append(f'name   : {tape["name"]}')
        lines.append(f'dist   : {tape["dist"]*100:.1f} cm')
        lines.append(f'lateral: {tape["lateral_error"]*100:+.1f} cm')
        ti = _find_tape_info(tape["name"])
        if ti:
            lines.append(f'task   : {ti.get("task") or "-"}')
            if ti.get("connects"):
                lines.append(f'connects: {" → ".join(ti["connects"])}')
    if gate and gate_dist < 2.0:
        lines.append(f'── NEAREST GATE ──────')
        lines.append(f'name   : {gate.name}')
        lines.append(f'dist   : {gate_dist*100:.1f} cm')
        lines.append(f'sat    : {"✓" if gate.has_satellite else "✗"}  +{gate.points}pt')
    if aruco:
        lines.append(f'── NEARBY ARUCO ──────')
        for d, m in aruco[:2]:
            lines.append(f'ID={m.id}  {d*100:.0f}cm')
    info_text.set_text('\n'.join(lines))
    fig.canvas.draw_idle()

# format_coord override - capture toolbar coordinates as mouse moves over map
_last = {'x': None, 'y': None}
_orig_fmt = ax.format_coord
def _fmt(x, y):
    s = _orig_fmt(x, y)
    nums = re.findall(r'[-+]?\d+\.?\d*', s)
    if len(nums) >= 2:
        try:
            wx, wy = float(nums[0]), float(nums[1])
            if _last['x'] != round(wx,2) or _last['y'] != round(wy,2):
                _last['x'] = round(wx,2); _last['y'] = round(wy,2)
                _update_info(wx, wy)
        except Exception:
            pass
    return s
ax.format_coord = _fmt

# ── GATE DRAW FUNCTION ────────────────────────────────────────────────
def draw_gate(g):
    cx, cy, cz = g.center.x, g.center.y, g.center.z
    h = g.height
    if g.radial_from is not None:
        rx, ry = g.radial_from
        ax.plot([rx,rx],[ry,ry],[cz,cz+h], color='#ddaa00',linewidth=2.5,zorder=9,solid_capstyle='round')
        ax.plot([rx,cx],[ry,cy],[cz+h,cz+h], color='#ddaa00',linewidth=2.5,zorder=9,solid_capstyle='round')
        ax.plot([cx,cx],[cy,cy],[cz+h,cz], color='#ddaa00',linewidth=2.5,zorder=9,solid_capstyle='round')
        if g.has_satellite:
            mx=(rx+cx)/2; my=(ry+cy)/2
            ax.plot([mx],[my],[cz+h+0.06],'*',color='cyan',markersize=10,zorder=10)
    else:
        angle = g.line_angle_deg if g.line_angle_deg is not None else g.orientation_deg+90
        rad = math.radians(angle)
        dx = math.cos(rad)*g.width/2; dy = math.sin(rad)*g.width/2
        ax.plot([cx-dx,cx-dx],[cy-dy,cy-dy],[cz,cz+h], color='#ddaa00',linewidth=2.5,zorder=9,solid_capstyle='round')
        ax.plot([cx-dx,cx+dx],[cy-dy,cy+dy],[cz+h,cz+h], color='#ddaa00',linewidth=2.5,zorder=9,solid_capstyle='round')
        ax.plot([cx+dx,cx+dx],[cy+dy,cy+dy],[cz+h,cz], color='#ddaa00',linewidth=2.5,zorder=9,solid_capstyle='round')
        if g.has_satellite:
            ax.plot([cx],[cy],[cz+h+0.06],'*',color='cyan',markersize=10,zorder=10)

# ── GRID + BORDER ─────────────────────────────────────────────────────
for i in range(8): ax.plot([i,i],[0,6],[0,0], color='white', alpha=0.15, linewidth=0.6)
for j in range(7): ax.plot([0,7],[j,j],[0,0], color='white', alpha=0.15, linewidth=0.6)
for x0,y0,x1,y1 in [(0,0,7,0),(7,0,7,6),(7,6,0,6),(0,6,0,0)]:
    ax.plot([x0,x1],[y0,y1],[0,0], color='deepskyblue', linewidth=2)

# ── GREEN BORDER ──────────────────────────────────────────────────────
ax.plot([0,0],[0,2.5],[0,0], color='lime', linewidth=3, zorder=5)
ax.plot([FIELD.shuttle.path_start.x, FIELD.shuttle.path_end.x],
        [FIELD.shuttle.path_start.y, FIELD.shuttle.path_end.y],
        [0,0], color='lime', linewidth=3, zorder=5)
ax.text(1.75,0.30,0,'Luggage shuttle',color='lime',fontsize=8,ha='center',fontweight='bold')
ax.plot([0],[0],[0],'*',color='lime',markersize=12,zorder=7)
ax.text(0.1,0.2,0,'(0,0,0)',color='lime',fontsize=8,fontweight='bold')

# ── ROUNDABOUT ────────────────────────────────────────────────────────
ax.plot([3.5,3.5],[0,2],[0,0], color='white', linewidth=3, zorder=5)
ax.plot([3.5],[2],[0],'o',color='white',markersize=6,zorder=6)
theta = np.linspace(0,2*np.pi,100)
ax.plot(RC[0]+RR*np.cos(theta), RC[1]+RR*np.sin(theta), np.zeros(100), color='white', linewidth=2, zorder=5)
ax.text(RC[0],RC[1],0.05,'3',color='white',fontsize=10,ha='center',va='center',fontweight='bold')
ax.plot([RC[0]-RR,RC[0]-RR-2.6],[RC[1],RC[1]],[0,0],color='white',linewidth=3,zorder=5)
ax.plot([RC[0]-RR-2.6],[RC[1]],[0],'o',color='white',markersize=6,zorder=6)
ax.plot([0.27,0.27],[2.60,3.30],[0,0],color='white',linewidth=3,zorder=5)
ax.plot([0.27],[3.30],[0],'o',color='white',markersize=6,zorder=6)

# ── SHORT RAMP ────────────────────────────────────────────────────────
sr = FIELD.short_ramp
nx = sr.width/2
ax.add_collection3d(Poly3DCollection([[(sr.base.x-nx,sr.base.y,sr.base.z),(sr.base.x+nx,sr.base.y,sr.base.z),(sr.top.x+nx,sr.top.y,sr.top.z),(sr.top.x-nx,sr.top.y,sr.top.z)]],alpha=0.35,facecolor='#886644',edgecolor='white',linewidth=0.8))
ax.add_collection3d(Poly3DCollection([[(sr.base.x-nx,sr.base.y,0),(sr.base.x+nx,sr.base.y,0),(sr.top.x+nx,sr.top.y,0),(sr.top.x-nx,sr.top.y,0)]],alpha=0.25,facecolor='#5a3a1a',edgecolor='none'))
ax.add_collection3d(Poly3DCollection([[(sr.base.x-nx,sr.base.y,0),(sr.base.x-nx,sr.base.y,sr.base.z),(sr.top.x-nx,sr.top.y,sr.top.z),(sr.top.x-nx,sr.top.y,0)]],alpha=0.25,facecolor='#6a4a2a',edgecolor='white',linewidth=0.5))
ax.add_collection3d(Poly3DCollection([[(sr.base.x+nx,sr.base.y,0),(sr.base.x+nx,sr.base.y,sr.base.z),(sr.top.x+nx,sr.top.y,sr.top.z),(sr.top.x+nx,sr.top.y,0)]],alpha=0.25,facecolor='#6a4a2a',edgecolor='white',linewidth=0.5))
ax.add_collection3d(Poly3DCollection([[(sr.base.x-nx,sr.base.y,0),(sr.base.x+nx,sr.base.y,0),(sr.base.x+nx,sr.base.y,sr.base.z),(sr.base.x-nx,sr.base.y,sr.base.z)]],alpha=0.25,facecolor='#7a5a3a',edgecolor='white',linewidth=0.5))
t_rp=np.linspace(0,1,30)
ax.plot(np.full(30,sr.base.x), sr.base.y+t_rp*(sr.top.y-sr.base.y), t_rp*sr.top.z+_off, color='white',linewidth=3,zorder=8)
ax.plot([sr.top.x],[sr.top.y],[sr.top.z],'o',color='white',markersize=6,zorder=6)
ax.text(sr.top.x+0.1,sr.top.y,sr.top.z+0.03,f'({sr.top.x:.2f},{sr.top.y:.2f},{sr.top.z})',color='white',fontsize=7)

# ── PLATFORM ──────────────────────────────────────────────────────────
p=FIELD.platform
x0,y0,x1,y1,pz = p.origin.x, p.origin.y, p.origin.x+p.width, p.origin.y+p.depth, p.pz
ax.add_collection3d(Poly3DCollection([[(x0,y0,pz),(x1,y0,pz),(x1,y1,pz),(x0,y1,pz)]],alpha=0.35,facecolor='#556655',edgecolor='lightgreen',linewidth=1.0))
ax.add_collection3d(Poly3DCollection([[(x0,y0,0),(x1,y0,0),(x1,y1,0),(x0,y1,0)]],alpha=0.25,facecolor='#334433',edgecolor='none'))
ax.add_collection3d(Poly3DCollection([[(x0,y0,0),(x1,y0,0),(x1,y0,pz),(x0,y0,pz)]],alpha=0.25,facecolor='#445544',edgecolor='lightgreen',linewidth=0.5))
ax.add_collection3d(Poly3DCollection([[(x1,y0,0),(x1,y1,0),(x1,y1,pz),(x1,y0,pz)]],alpha=0.25,facecolor='#445544',edgecolor='lightgreen',linewidth=0.5))
ax.add_collection3d(Poly3DCollection([[(x1,y1,0),(x0,y1,0),(x0,y1,pz),(x1,y1,pz)]],alpha=0.25,facecolor='#445544',edgecolor='lightgreen',linewidth=0.5))
ax.add_collection3d(Poly3DCollection([[(x0,y1,0),(x0,y0,0),(x0,y0,pz),(x0,y1,pz)]],alpha=0.25,facecolor='#445544',edgecolor='lightgreen',linewidth=0.5))
ax.text((x0+x1)/2,(y0+y1)/2,pz+0.05,f'Platform\n1.5×1.0m\nz={pz}m',color='lightgreen',fontsize=7,ha='center',va='bottom')
ax.plot([p.hole.x],[p.hole.y],[pz+_off],'o',color='black',markersize=10,zorder=10,markeredgecolor='white',markeredgewidth=1.5)
ax.text(p.hole.x+0.08,p.hole.y,pz+0.04,f'Hole',color='white',fontsize=7)
ax.plot([p.golf_ball.x],[p.golf_ball.y],[pz+_off],'o',color='orange',markersize=10,zorder=10,markeredgecolor='white',markeredgewidth=1.5)
ax.text(p.golf_ball.x+0.08,p.golf_ball.y,pz+0.04,f'Ball',color='orange',fontsize=7)

# ── STAIRS ────────────────────────────────────────────────────────────
s=FIELD.stairs
from field_map_2026 import stair_x0, stair_x1
sx0,sx1 = stair_x0, stair_x1
sy_top=p.origin.y
sy_bot=sy_top - s.n_steps*s.step_depth
for side_x in [sx0, sx1]:
    pts = [(side_x,sy_bot,0)]
    for i in range(s.n_steps):
        sy = sy_top-(s.n_steps-i)*s.step_depth; sz=(i+1)*s.step_height
        pts += [(side_x,sy,i*s.step_height),(side_x,sy,sz)]
    pts += [(side_x,sy_top,s.n_steps*s.step_height)]
    for i in range(len(pts)-1):
        p1s=pts[i]; p2s=pts[i+1]
        ax.add_collection3d(Poly3DCollection([[(p1s[0],p1s[1],0),(p2s[0],p2s[1],0),(p2s[0],p2s[1],p2s[2]),(p1s[0],p1s[1],p1s[2])]],alpha=0.25,facecolor='#445566',edgecolor='none'))
for i in range(s.n_steps):
    sy0_s=sy_top-(s.n_steps-i)*s.step_depth; sy1_s=sy_top-(s.n_steps-i-1)*s.step_depth; sz=(i+1)*s.step_height
    ax.add_collection3d(Poly3DCollection([[(sx0,sy0_s,sz),(sx1,sy0_s,sz),(sx1,sy1_s,sz),(sx0,sy1_s,sz)]],alpha=0.35,facecolor='#667788',edgecolor='white',linewidth=0.6))
    ax.add_collection3d(Poly3DCollection([[(sx0,sy0_s,i*s.step_height),(sx1,sy0_s,i*s.step_height),(sx1,sy0_s,sz),(sx0,sy0_s,sz)]],alpha=0.25,facecolor='#556677',edgecolor='white',linewidth=0.6))
    if i in [1,3]:
        ax.text(sx1+0.05,(sy0_s+sy1_s)/2,(i+1)*s.step_height+0.05,'Gate (+1)' if i==1 else 'Gate (+2)',color='#ddaa00',fontsize=7)
ax.text((sx0+sx1)/2,sy_top-s.n_steps*s.step_depth/2,s.n_steps*s.step_height/2,'Stairs\n4 steps',color='#aaccff',fontsize=7,ha='center')
stair_mid_x=(sx0+sx1)/2
t_s=np.linspace(0,1,20)
ax.plot(np.full(20,stair_mid_x), sy_bot+t_s*(sy_top-sy_bot), t_s*s.n_steps*s.step_height+_off, color='white',linewidth=3,zorder=8)

# ── LONG RAMP ─────────────────────────────────────────────────────────
lr=FIELD.long_ramp
lx0,lx1=lr.top.x,lr.base.x
ly0,ly1=lr.top.y-lr.width/2,lr.top.y+lr.width/2
lzt,lzb=lr.top.z,lr.base.z
ax.add_collection3d(Poly3DCollection([[(lx0,ly0,lzt),(lx0,ly1,lzt),(lx1,ly1,lzb),(lx1,ly0,lzb)]],alpha=0.35,facecolor='#886644',edgecolor='white',linewidth=0.8))
ax.add_collection3d(Poly3DCollection([[(lx0,ly0,0),(lx0,ly1,0),(lx1,ly1,0),(lx1,ly0,0)]],alpha=0.25,facecolor='#5a3a1a',edgecolor='none'))
ax.add_collection3d(Poly3DCollection([[(lx0,ly0,0),(lx0,ly0,lzt),(lx1,ly0,lzb),(lx1,ly0,0)]],alpha=0.25,facecolor='#6a4a2a',edgecolor='white',linewidth=0.5))
ax.add_collection3d(Poly3DCollection([[(lx0,ly1,0),(lx0,ly1,lzt),(lx1,ly1,lzb),(lx1,ly1,0)]],alpha=0.25,facecolor='#6a4a2a',edgecolor='white',linewidth=0.5))
ax.add_collection3d(Poly3DCollection([[(lx0,ly0,0),(lx0,ly1,0),(lx0,ly1,lzt),(lx0,ly0,lzt)]],alpha=0.25,facecolor='#7a5a3a',edgecolor='white',linewidth=0.5))
t_lr=np.linspace(0,1,40)
ax.plot(lx0+t_lr*(lx1-lx0), np.full(40,(ly0+ly1)/2), lzt+t_lr*(lzb-lzt)+_off, color='white',linewidth=3,zorder=8)
ax.text((lx0+lx1)/2,(ly0+ly1)/2,lzt/2+0.05,'Long ramp\n3.64m',color='#ffcc88',fontsize=7,ha='center')

# ── SEESAW ────────────────────────────────────────────────────────────
sw=FIELD.seesaw
swx=sw.pivot.x; swz=sw.pivot.z; sww=sw.width
swy0=sw.pivot.y-sw.length/2; swy1=sw.pivot.y+sw.length/2
ax.add_collection3d(Poly3DCollection([[(swx-sww/2,swy0,swz),(swx+sww/2,swy0,swz),(swx+sww/2,swy1,swz),(swx-sww/2,swy1,swz)]],alpha=1.0,facecolor='#cc8833',edgecolor='orange',linewidth=1.2))
ax.plot([swx],[sw.pivot.y],[swz],'^',color='orange',markersize=10,zorder=8,markeredgecolor='white',markeredgewidth=0.8)
if sw.golf_ball_pos:
    ax.plot([sw.golf_ball_pos.x],[sw.golf_ball_pos.y],[sw.golf_ball_pos.z],'o',color='orange',markersize=9,zorder=10,markeredgecolor='white',markeredgewidth=1.2)
ax.text(swx+0.15,sw.pivot.y,swz+0.08,f'Seesaw\n1.80m\n(z≈{swz:.2f}m)',color='orange',fontsize=7)

# ── POST-RAMP U-TURN + ROUNDABOUT ─────────────────────────────────────
ry_mid=(ly0+ly1)/2
straight1_end_x=lx1+0.39
ax.plot([lx1,straight1_end_x],[ry_mid,ry_mid],[0,0],color='white',linewidth=3,zorder=5)
u_r=0.60; u_cy=ry_mid-u_r
theta_u=np.linspace(np.pi/2,-np.pi/2,60)
ax.plot(straight1_end_x+u_r*np.cos(theta_u),u_cy+u_r*np.sin(theta_u),np.zeros(60),color='white',linewidth=3,zorder=5)
u_exit_y=ry_mid-2*u_r; round_top_y=RC[1]+RR
arc_r2=u_exit_y-round_top_y; p_str_end_x=RC[0]+arc_r2
ax.plot([straight1_end_x,p_str_end_x],[u_exit_y,u_exit_y],[0,0],color='white',linewidth=3,zorder=5)
theta_arc2=np.linspace(np.pi/2,np.pi,30)
ax.plot(p_str_end_x+arc_r2*np.cos(theta_arc2),round_top_y+arc_r2*np.sin(theta_arc2),np.zeros(30),color='white',linewidth=3,zorder=5)
ax.plot([RC[0],RC[0]],[RC[1]-RR,0.50],[0,0],color='white',linewidth=3,zorder=5)

# ── BALL BOWL ─────────────────────────────────────────────────────────
bd=FIELD.ball_dispenser
bx,by=bd.position.x,bd.position.y
bowl_angles=np.linspace(0,2*np.pi,7); bowl_r=0.18
for a1,a2 in zip(bowl_angles[:-1],bowl_angles[1:]):
    ax.add_collection3d(Poly3DCollection([[(bx+bowl_r*np.cos(a1),by+bowl_r*np.sin(a1),0.0),(bx+bowl_r*np.cos(a2),by+bowl_r*np.sin(a2),0.0),(bx+bowl_r*np.cos(a2),by+bowl_r*np.sin(a2),0.18),(bx+bowl_r*np.cos(a1),by+bowl_r*np.sin(a1),0.18)]],alpha=0.4,facecolor='#aaaaaa',edgecolor='white',linewidth=0.5))
ball_color_map={'blue':'#2266ff','red':'#ff3333','white':'white'}
ball_offsets=[(0,0,.20),(.08,.05,.20),(-.08,.05,.20),(.04,-.08,.20),(-.04,-.08,.20)]
for ball,(ox,oy,oz) in zip(bd.balls,ball_offsets):
    ax.plot([bx+ox],[by+oy],[oz],'o',color=ball_color_map.get(ball.color,'white'),markersize=8,zorder=10,markeredgecolor='gray',markeredgewidth=0.8)
ax.text(bx+0.25,by,0.22,'Ball bowl\n(7)',color='#ff9966',fontsize=7)

# ── +90 SEC BUTTONS ───────────────────────────────────────────────────
for tb in FIELD.time_buttons:
    ax.plot([tb.center.x],[tb.center.y],[0.01],'o',color='red',markersize=14,zorder=9,markeredgecolor='darkred',markeredgewidth=1.5)
    ax.text(tb.center.x+0.15,tb.center.y+0.15,0.08,'+90s',color='white',fontsize=7,fontweight='bold',ha='left')

# ── INFINITY PATH ─────────────────────────────────────────────────────
theta_inf=np.linspace(0,2*np.pi,100)
lc_x=inf_cx-inf_rx; rc_x=inf_cx+inf_rx
ax.plot(lc_x+inf_rx*np.cos(theta_inf),inf_cy+inf_ry*np.sin(theta_inf),np.zeros(100),color='white',linewidth=2.5,zorder=5)
ax.plot(rc_x+inf_rx*np.cos(theta_inf),inf_cy+inf_ry*np.sin(theta_inf),np.zeros(100),color='white',linewidth=2.5,zorder=5)
ax.text(lc_x-inf_rx,inf_cy+0.2,0.05,'9',color='white',fontsize=8,ha='center')
ax.text(inf_cx,inf_cy+0.2,0.05,'1',color='white',fontsize=8,ha='center')

# ── ABCD SORTING CENTER ───────────────────────────────────────────────
sc=FIELD.sorting_center
abcd_d=0.60/np.sqrt(2); abcd_cx=sc.center.x; abcd_cy=sc.center.y
abcd_pts=[(abcd_cx,abcd_cy-abcd_d,0),(abcd_cx+abcd_d,abcd_cy,0),(abcd_cx,abcd_cy+abcd_d,0),(abcd_cx-abcd_d,abcd_cy,0)]
zone_colors={'A':'#4488ff','B':'#44ff88','C':'#ff8844','D':'#ff44aa'}
cpt=(abcd_cx,abcd_cy,0)
for i,(label,color) in enumerate(zone_colors.items()):
    p1z=abcd_pts[i]; p2z=abcd_pts[(i+1)%4]
    ax.add_collection3d(Poly3DCollection([[cpt,p1z,p2z]],alpha=1.0,facecolor=color,edgecolor='white',linewidth=0.8))
    ax.text((cpt[0]+p1z[0]+p2z[0])/3,(cpt[1]+p1z[1]+p2z[1])/3,0.05,label,color='white',fontsize=8,fontweight='bold',ha='center')
for i,(p1z,p2z) in enumerate(zip(abcd_pts,[abcd_pts[(j+1)%4] for j in range(4)])):
    mx=(p1z[0]+p2z[0])/2; my=(p1z[1]+p2z[1])/2
    ax.plot([mx],[my],[0.02],'D',color='orange',markersize=5,zorder=8,markeredgecolor='darkorange',markeredgewidth=0.5)
    ax.text(mx+0.04,my+0.04,0.04,f'{10+i*2}',color='orange',fontsize=5)

# ── SMALL WHITE LINE ──────────────────────────────────────────────────
ax.plot([3.0,3.5],[1.3,1.3],[0,0],color='white',linewidth=3,zorder=5)

# ── START ─────────────────────────────────────────────────────────────
sa=FIELD.start_area
ax.plot([sa.origin.x,sa.origin.x+sa.width],[0,0],[0,0],color='deepskyblue',linewidth=3,zorder=6)
ax.plot([sa.origin.x,sa.origin.x],[0,sa.depth],[0,0],color='deepskyblue',linewidth=3,zorder=6)
ax.plot([sa.origin.x+sa.width,sa.origin.x+sa.width],[0,sa.depth],[0,0],color='deepskyblue',linewidth=3,zorder=6)
ax.text(sa.center.x,sa.depth/2,0.05,'Start',color='deepskyblue',fontsize=7,ha='center',fontweight='bold')

# ── GOAL ──────────────────────────────────────────────────────────────
gp=FIELD.goal_point; gs=0.18
ax.add_collection3d(Poly3DCollection([[(gp.x-gs/2,0,0.01),(gp.x+gs/2,0,0.01),(gp.x+gs/2,gs,0.01),(gp.x-gs/2,gs,0.01)]],alpha=1.0,facecolor='#2244cc',edgecolor='deepskyblue',linewidth=1.5))
ax.text(gp.x,gs/2,0.06,'Goal',color='deepskyblue',fontsize=7,ha='center',fontweight='bold')
ax.plot([gp.x],[gs+0.05],[0.02],'D',color='orange',markersize=8,zorder=10,markeredgecolor='darkorange',markeredgewidth=1)
ax.text(gp.x+0.1,gs+0.05,0.04,'ArUco',color='orange',fontsize=6)

# ── START-GOAL LOOP ───────────────────────────────────────────────────
p_s=(gp.x,gs); p_e=(sa.center.x,sa.depth/2)
p1x,p1y=p_s[0],p_s[1]+1.40; p2x,p2y=p_e[0],p_e[1]+1.40
acx=(p1x+p2x)/2; acy=max(p1y,p2y)+0.40
t=np.linspace(0,1,60)
bx=(1-t)**3*p1x+3*(1-t)**2*t*acx+3*(1-t)*t**2*acx+t**3*p2x
by=(1-t)**3*p1y+3*(1-t)**2*t*acy+3*(1-t)*t**2*acy+t**3*p2y
ax.plot([p_s[0],p1x],[p_s[1],p1y],[0,0],color='white',linewidth=3,zorder=5)
ax.plot(bx,by,np.zeros(60),color='white',linewidth=3,zorder=5)
ax.plot([p2x,p_e[0]],[p2y,p_e[1]],[0,0],color='white',linewidth=3,zorder=5)
ax.text(5.5,2.15,0.05,'2',color='#ddaa00',fontsize=8,ha='center')

# ── START LEFT CURVE ──────────────────────────────────────────────────
stx=sa.center.x; sty=sa.depth/2+1.40; tx=3.92; ty=2.17
t2=np.linspace(0,1,50)
bx2=(1-t2)**3*stx+3*(1-t2)**2*t2*stx+3*(1-t2)*t2**2*tx+t2**3*tx
by2=(1-t2)**3*sty+3*(1-t2)**2*t2*ty+3*(1-t2)*t2**2*ty+t2**3*ty
ax.plot(bx2,by2,np.zeros(50),color='white',linewidth=3,zorder=5)
ax.plot([tx],[ty],[0.01],'o',color='white',markersize=5,zorder=6)

# ── SHUTTLE ───────────────────────────────────────────────────────────
sh=FIELD.shuttle
sx=(sh.path_start.x+sh.path_end.x)/2; sd=0.13; sht=sh.surface_height_m; sw2=0.30
ax.add_collection3d(Poly3DCollection([[(sx-sw2/2,0,0),(sx+sw2/2,0,0),(sx+sw2/2,sd,0),(sx-sw2/2,sd,0)]],alpha=0.9,facecolor='#cc9900',edgecolor='white',linewidth=1.0))
ax.add_collection3d(Poly3DCollection([[(sx-sw2/2,0,sht),(sx+sw2/2,0,sht),(sx+sw2/2,sd,sht),(sx-sw2/2,sd,sht)]],alpha=0.9,facecolor='#ddaa00',edgecolor='white',linewidth=1.0))
ax.plot([sx],[sd],[sht/2],'D',color='orange',markersize=6,zorder=10,markeredgecolor='darkorange',markeredgewidth=0.8)
ax.text(sx+0.08,sd,sht/2+0.03,f'ID{sh.aruco_id}',color='orange',fontsize=6)
for lug,lxo in zip(sh.luggage,[-0.07,0.07]):
    lx=sx+lxo; ls=0.06
    ax.add_collection3d(Poly3DCollection([[(lx-ls/2,sd/2-ls/2,sht),(lx+ls/2,sd/2-ls/2,sht),(lx+ls/2,sd/2+ls/2,sht),(lx-ls/2,sd/2+ls/2,sht)]],alpha=0.9,facecolor='#4466aa',edgecolor='white',linewidth=0.8))
    ax.plot([lx],[sd/2],[sht+0.03],'D',color='orange',markersize=4,zorder=11,markeredgecolor='darkorange',markeredgewidth=0.5)
    ax.text(lx,sd/2+0.05,sht+0.04,str(lug.aruco_id),color='orange',fontsize=5)

# ── TAPE LINES ────────────────────────────────────────────────────────
stair_bot_y=sy_top-s.n_steps*s.step_depth
bowl_px=bd.position.x; bowl_py=bd.position.y
ax.plot([bowl_px,stair_mid_x],[bowl_py,stair_bot_y],[_off,_off],color='white',linewidth=3,zorder=6)
lr_entry=(lx0,(ly0+ly1)/2,pz+_off)
for pt_xy in [(sr.top.x,sr.top.y),(stair_mid_x,sy_top)]:
    ctrl_x=pt_xy[0]; ctrl_y=lr_entry[1]
    t_pl=np.linspace(0,1,50)
    bx1=(1-t_pl)**2*pt_xy[0]+2*(1-t_pl)*t_pl*ctrl_x+t_pl**2*lr_entry[0]
    by1=(1-t_pl)**2*pt_xy[1]+2*(1-t_pl)*t_pl*ctrl_y+t_pl**2*lr_entry[1]
    ax.plot(bx1,by1,np.full(50,pz+_off),color='white',linewidth=3,zorder=8)

# ── ALL GATES ─────────────────────────────────────────────────────────
for g in FIELD.all_gates():
    draw_gate(g)

plt.tight_layout()
plt.savefig('field_map_3d.png', dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
print("Saved: field_map_3d.png")
print("Move mouse over the map to update the top-left info panel.")
plt.show()