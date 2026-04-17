"""
plot_field_3d.py
================
Draws the field in 3D. All coordinates come from the FIELD object.
No hardcoded coordinates — everything is read from field_map_2026.py objects.

Usage: python3 plot_field_3d.py
"""
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math, re

from field_map_2026 import FIELD
from field_objects import (
    TapeLine, ArcSegment, BezierSegment, NavPath,
    Ramp, Stairs, Platform, Seesaw, Roundabout,
    InfinityPath, SortingCenter, BallDispenser,
    LuggageShuttle, Gate, ArucoMarker,
)

# ─────────────────────────────────────────────────────────────────────
# FIGURE SETUP
# ─────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(13, 10))
ax  = fig.add_subplot(111, projection='3d')
fig.patch.set_facecolor('#1a1a1a')
ax.set_facecolor('#2e2e2e')
ax.set_xlim(0, FIELD.width_m)
ax.set_ylim(0, FIELD.height_m)
ax.set_zlim(0, 2)
ax.set_xlabel('X (m)', color='white', labelpad=8)
ax.set_ylabel('Y (m)', color='white', labelpad=8)
ax.set_zlabel('Z (m)', color='white', labelpad=8)
ax.set_title('DTU Robocup 2026 — Field Map 3D', color='white', fontsize=13)
ax.tick_params(colors='white')
ax.xaxis.pane.fill = False; ax.yaxis.pane.fill = False; ax.zaxis.pane.fill = False
ax.xaxis.pane.set_edgecolor('#444')
ax.yaxis.pane.set_edgecolor('#444')
ax.zaxis.pane.set_edgecolor('#444')
ax.grid(True, alpha=0.15, color='white')
ax.view_init(elev=35, azim=-60)

_off = 0.035   # z offset so tape lines float just above floor

# ─────────────────────────────────────────────────────────────────────
# INTERACTIVE INFO BOXES
# ─────────────────────────────────────────────────────────────────────
coord_text = fig.text(0.02, 0.97, '', color='white', fontsize=10,
                      fontweight='bold', va='top', family='monospace',
                      bbox=dict(boxstyle='round,pad=0.5', facecolor='#111', alpha=0.85))
info_text  = fig.text(0.02, 0.88, '', color='#aaddff', fontsize=9,
                      va='top', family='monospace',
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
    if not (0 <= px <= FIELD.width_m and 0 <= py <= FIELD.height_m):
        return
    pz   = FIELD.pz_at(px, py)
    zone = FIELD.current_zone(px, py)
    coord_text.set_text(f'x={px:.3f}  y={py:.3f}  z={pz:.3f}m   zone: {zone}')
    tape = FIELD.nearest_tape_segment(px, py)
    gate, gate_dist = FIELD.nearest_gate(px, py)
    aruco = FIELD.nearby_aruco(px, py, radius=1.0)
    lines = []
    if tape:
        lines.append('── TAPE ──────────────')
        lines.append(f'name    : {tape["name"]}')
        lines.append(f'dist    : {tape["dist"]*100:.1f} cm')
        lines.append(f'lateral : {tape["lateral_error"]*100:+.1f} cm')
        ti = _find_tape_info(tape["name"])
        if ti:
            lines.append(f'task    : {ti.get("task") or "-"}')
            if ti.get("connects"):
                lines.append(f'connects: {" → ".join(ti["connects"])}')
    if gate and gate_dist < 2.0:
        lines.append('── NEAREST GATE ──────')
        lines.append(f'name    : {gate.name}')
        lines.append(f'dist    : {gate_dist*100:.1f} cm')
        lines.append(f'sat     : {"✓" if gate.has_satellite else "✗"}  +{gate.points}pt')
    if aruco:
        lines.append('── NEARBY ARUCO ──────')
        for d, m in aruco[:3]:
            lines.append(f'ID={m.id}  {d*100:.0f}cm')
    info_text.set_text('\n'.join(lines))
    fig.canvas.draw_idle()

_last = {'x': None, 'y': None}
_orig_fmt = ax.format_coord
def _fmt(x, y):
    s = _orig_fmt(x, y)
    nums = re.findall(r'[-+]?\d+\.?\d*', s)
    if len(nums) >= 2:
        try:
            wx, wy = float(nums[0]), float(nums[1])
            if _last['x'] != round(wx, 2) or _last['y'] != round(wy, 2):
                _last['x'] = round(wx, 2); _last['y'] = round(wy, 2)
                _update_info(wx, wy)
        except Exception:
            pass
    return s
ax.format_coord = _fmt

# ─────────────────────────────────────────────────────────────────────
# DRAW HELPERS
# ─────────────────────────────────────────────────────────────────────

def draw_gate(g: Gate):
    """Draw a gate using all values from the FIELD object."""
    cx, cy = g.center.x, g.center.y
    bz = g.center.z + g.bottom_z   # bottom edge z
    tz = bz + g.height              # top edge z  (= g.top_z + center.z)

    color_gate = '#ddaa00'
    color_sat  = 'cyan'

    if g.radial_from is not None:
        # Roundabout type: radial line from center → gate edge
        rx, ry = g.radial_from
        ax.plot([rx, rx], [ry, ry], [bz, tz],
                color=color_gate, linewidth=2.5, zorder=9, solid_capstyle='round')
        ax.plot([rx, cx], [ry, cy], [tz, tz],
                color=color_gate, linewidth=2.5, zorder=9, solid_capstyle='round')
        ax.plot([cx, cx], [cy, cy], [tz, bz],
                color=color_gate, linewidth=2.5, zorder=9, solid_capstyle='round')
        if g.has_satellite:
            mx = (rx + cx) / 2; my = (ry + cy) / 2
            sat_z = tz + g.satellite_height / 2
            ax.plot([mx], [my], [sat_z], '*', color=color_sat,
                    markersize=10, zorder=10)
            ax.plot([mx, mx], [my, my], [tz, sat_z],
                    color=color_sat, linewidth=1, linestyle='--', zorder=9)
    else:
        angle = g.line_angle_deg if g.line_angle_deg is not None else g.orientation_deg + 90
        rad = math.radians(angle)
        dx = math.cos(rad) * g.width / 2
        dy = math.sin(rad) * g.width / 2
        # Left vertical
        ax.plot([cx-dx, cx-dx], [cy-dy, cy-dy], [bz, tz],
                color=color_gate, linewidth=2.5, zorder=9, solid_capstyle='round')
        # Top horizontal
        ax.plot([cx-dx, cx+dx], [cy-dy, cy+dy], [tz, tz],
                color=color_gate, linewidth=2.5, zorder=9, solid_capstyle='round')
        # Right vertical
        ax.plot([cx+dx, cx+dx], [cy+dy, cy+dy], [tz, bz],
                color=color_gate, linewidth=2.5, zorder=9, solid_capstyle='round')
        if g.has_satellite:
            sat_z = tz + g.satellite_height / 2
            ax.plot([cx], [cy], [sat_z], '*', color=color_sat,
                    markersize=10, zorder=10)
            ax.plot([cx, cx], [cy, cy], [tz, sat_z],
                    color=color_sat, linewidth=1, linestyle='--', zorder=9)


def draw_aruco(m: ArucoMarker, color='orange', size=6):
    """Draw an ArUco marker as a diamond marker + ID label."""
    ax.plot([m.position.x], [m.position.y], [m.position.z + _off],
            'D', color=color, markersize=size, zorder=10,
            markeredgecolor='darkorange', markeredgewidth=0.8)
    ax.text(m.position.x + 0.05, m.position.y + 0.05, m.position.z + _off + 0.03,
            f'ID{m.id}', color=color, fontsize=6, zorder=11)


def draw_tape_segment(p0, p1, z=0.0, color='white', lw=3):
    """Draw a straight tape between two points."""
    ax.plot([p0.x, p1.x], [p0.y, p1.y], [z + _off, z + _off],
            color=color, linewidth=lw, zorder=6)


def draw_segment(seg, base_z=0.0, color='white', lw=3):
    """Draw a TapeLine / ArcSegment / BezierSegment."""
    if isinstance(seg, TapeLine):
        pts = seg.waypoints
        for a, b in zip(pts[:-1], pts[1:]):
            z = (a.z + b.z) / 2 + _off
            ax.plot([a.x, b.x], [a.y, b.y], [z, z],
                    color=color, linewidth=lw, zorder=6)
    elif isinstance(seg, ArcSegment):
        xs, ys, zs = seg.sample()
        ax.plot(xs, ys, zs + _off, color=color, linewidth=lw, zorder=6)
    elif isinstance(seg, BezierSegment):
        xs, ys, zs = seg.sample()
        ax.plot(xs, ys, zs + _off, color=color, linewidth=lw, zorder=6)


def draw_ramp_solid(ramp: Ramp, face='#886644', edge='white'):
    """Draw the ramp surface as a 3D quadrilateral."""
    b, t = ramp.base, ramp.top
    nx_vec = (-(t.y - b.y), (t.x - b.x))
    length = math.hypot(*nx_vec)
    if length < 1e-6: return
    nx = nx_vec[0] / length * ramp.width / 2
    ny = nx_vec[1] / length * ramp.width / 2
    verts = [
        [(b.x-nx, b.y-ny, b.z), (b.x+nx, b.y+ny, b.z),
         (t.x+nx, t.y+ny, t.z), (t.x-nx, t.y-ny, t.z)]
    ]
    ax.add_collection3d(Poly3DCollection(verts, alpha=0.40,
                        facecolor=face, edgecolor=edge, linewidth=0.8))
    # Shadow on floor
    shadow = [[(b.x-nx, b.y-ny, 0), (b.x+nx, b.y+ny, 0),
               (t.x+nx, t.y+ny, 0), (t.x-nx, t.y-ny, 0)]]
    ax.add_collection3d(Poly3DCollection(shadow, alpha=0.20,
                        facecolor='#5a3a1a', edgecolor='none'))


# ─────────────────────────────────────────────────────────────────────
# 1. GRID + FIELD BORDER
# ─────────────────────────────────────────────────────────────────────
for i in range(int(FIELD.width_m) + 1):
    ax.plot([i, i], [0, FIELD.height_m], [0, 0], color='white', alpha=0.12, linewidth=0.5)
for j in range(int(FIELD.height_m) + 1):
    ax.plot([0, FIELD.width_m], [j, j], [0, 0], color='white', alpha=0.12, linewidth=0.5)
W, H = FIELD.width_m, FIELD.height_m
for x0, y0, x1, y1 in [(0,0,W,0),(W,0,W,H),(W,H,0,H),(0,H,0,0)]:
    ax.plot([x0, x1], [y0, y1], [0, 0], color='deepskyblue', linewidth=2)

ax.plot([0], [0], [0], '*', color='lime', markersize=12, zorder=7)
ax.text(0.1, 0.2, 0, '(0,0,0)', color='lime', fontsize=8, fontweight='bold')

# ─────────────────────────────────────────────────────────────────────
# 2. GREEN BARRIERS (from FIELD object, z = GREEN_BARRIER_Z)
# ─────────────────────────────────────────────────────────────────────
bz_green = FIELD.GREEN_BARRIER_Z   # 0.09 m

# Vertical green line
p0g, p1g = FIELD.green_y_line
ax.plot([p0g.x, p1g.x], [p0g.y, p1g.y], [bz_green, bz_green],
        color='lime', linewidth=3, zorder=5)
ax.plot([p0g.x, p1g.x], [p0g.y, p1g.y], [0, 0],
        color='lime', linewidth=1, alpha=0.3, zorder=4)

# Shuttle green border
p0s, p1s = FIELD.shuttle_green_border
ax.plot([p0s.x, p1s.x], [p0s.y, p1s.y], [bz_green, bz_green],
        color='lime', linewidth=3, zorder=5)
ax.plot([p0s.x, p1s.x], [p0s.y, p1s.y], [0, 0],
        color='lime', linewidth=1, alpha=0.3, zorder=4)
ax.text((p0s.x + p1s.x)/2, p0s.y + 0.20, bz_green + 0.02,
        'Luggage shuttle', color='lime', fontsize=8, ha='center', fontweight='bold')

# +90s buttons on top of barrier
for tb in FIELD.time_buttons:
    ax.plot([tb.center.x], [tb.center.y], [tb.z_height + 0.03],
            'o', color='red', markersize=14, zorder=9,
            markeredgecolor='darkred', markeredgewidth=1.5)
    ax.text(tb.center.x + 0.15, tb.center.y + 0.10, tb.z_height + 0.05,
            '+90s', color='white', fontsize=7, fontweight='bold')

# ─────────────────────────────────────────────────────────────────────
# 3. START AREA  (from FIELD.start_area)
# ─────────────────────────────────────────────────────────────────────
sa = FIELD.start_area
ax.plot([sa.origin.x, sa.origin.x + sa.width], [0, 0], [0, 0],
        color='deepskyblue', linewidth=3, zorder=6)
ax.plot([sa.origin.x, sa.origin.x], [0, sa.depth], [0, 0],
        color='deepskyblue', linewidth=3, zorder=6)
ax.plot([sa.origin.x + sa.width, sa.origin.x + sa.width], [0, sa.depth], [0, 0],
        color='deepskyblue', linewidth=3, zorder=6)
ax.text(sa.center.x, sa.depth / 2, 0.05, 'Start',
        color='deepskyblue', fontsize=8, ha='center', fontweight='bold')

# ─────────────────────────────────────────────────────────────────────
# 4. GOAL  (from FIELD.goal_point + FIELD.goal_aruco)
# ─────────────────────────────────────────────────────────────────────
gp = FIELD.goal_point
ga = FIELD.goal_aruco
gs = 0.18   # goal square size — intrinsic to goal, keep here
ax.add_collection3d(Poly3DCollection(
    [[(gp.x-gs/2, 0, 0.01), (gp.x+gs/2, 0, 0.01),
      (gp.x+gs/2, gs, 0.01), (gp.x-gs/2, gs, 0.01)]],
    alpha=1.0, facecolor='#2244cc', edgecolor='deepskyblue', linewidth=1.5))
ax.text(gp.x, gs/2, 0.06, 'Goal',
        color='deepskyblue', fontsize=8, ha='center', fontweight='bold')
# Goal ArUco — from FIELD.goal_aruco (ID comes from object)
draw_aruco(ga, color='orange', size=8)

# ─────────────────────────────────────────────────────────────────────
# 5. ROUNDABOUT  (from FIELD.roundabout)
# ─────────────────────────────────────────────────────────────────────
ro = FIELD.roundabout
cx_r, cy_r = ro.center.x, ro.center.y
theta = np.linspace(0, 2*np.pi, 100)
ax.plot(cx_r + ro.radius*np.cos(theta), cy_r + ro.radius*np.sin(theta),
        np.zeros(100), color='white', linewidth=2, zorder=5)
ax.text(cx_r, cy_r, 0.05, 'Roundabout',
        color='white', fontsize=7, ha='center', va='center', fontweight='bold')

# ─────────────────────────────────────────────────────────────────────
# 6. INFINITY PATH  (from FIELD.infinity_path)
# ─────────────────────────────────────────────────────────────────────
inf = FIELD.infinity_path
lc = inf.left_center
rc_inf = inf.right_center
theta_inf = np.linspace(0, 2*np.pi, 100)
ax.plot(lc.x + inf.loop_radius*np.cos(theta_inf),
        lc.y + inf.loop_radius*np.sin(theta_inf),
        np.zeros(100), color='white', linewidth=2.5, zorder=5)
ax.plot(rc_inf.x + inf.loop_radius*np.cos(theta_inf),
        rc_inf.y + inf.loop_radius*np.sin(theta_inf),
        np.zeros(100), color='white', linewidth=2.5, zorder=5)
ax.text(lc.x, lc.y + inf.loop_radius + 0.15, 0.05,
        'Infinity\n(guard)', color='#ccccff', fontsize=7, ha='center')

# ─────────────────────────────────────────────────────────────────────
# 7. SHORT RAMP  (from FIELD.short_ramp)
# ─────────────────────────────────────────────────────────────────────
sr = FIELD.short_ramp
draw_ramp_solid(sr)
# Tape line along ramp centre
t_r = np.linspace(0, 1, 30)
ax.plot(sr.base.x + t_r*(sr.top.x - sr.base.x),
        sr.base.y + t_r*(sr.top.y - sr.base.y),
        sr.base.z + t_r*(sr.top.z - sr.base.z) + _off,
        color='white', linewidth=3, zorder=8)
ax.text(sr.top.x + 0.1, sr.top.y, sr.top.z + 0.04,
        f'Short ramp\n{sr.run:.2f}m / rise={sr.rise}m',
        color='white', fontsize=6)

# ─────────────────────────────────────────────────────────────────────
# 8. PLATFORM  (from FIELD.platform)
# ─────────────────────────────────────────────────────────────────────
pl = FIELD.platform
px0 = pl.origin.x; py0 = pl.origin.y
px1 = px0 + pl.width; py1 = py0 + pl.depth; pz = pl.pz
faces = [
    ([(px0,py0,pz),(px1,py0,pz),(px1,py1,pz),(px0,py1,pz)], '#556655', 'lightgreen', 1.0),
    ([(px0,py0,0),(px1,py0,0),(px1,py0,pz),(px0,py0,pz)],    '#445544', 'lightgreen', 0.4),
    ([(px1,py0,0),(px1,py1,0),(px1,py1,pz),(px1,py0,pz)],    '#445544', 'lightgreen', 0.4),
    ([(px1,py1,0),(px0,py1,0),(px0,py1,pz),(px1,py1,pz)],    '#445544', 'lightgreen', 0.4),
    ([(px0,py1,0),(px0,py0,0),(px0,py0,pz),(px0,py1,pz)],    '#445544', 'lightgreen', 0.4),
]
for verts, fc, ec, alph in faces:
    ax.add_collection3d(Poly3DCollection([verts], alpha=alph*0.4,
                        facecolor=fc, edgecolor=ec, linewidth=0.8))
ax.text((px0+px1)/2, (py0+py1)/2, pz+0.06,
        f'Platform\n{pl.width:.1f}×{pl.depth:.1f}m  z={pz}m',
        color='lightgreen', fontsize=7, ha='center', va='bottom')
# Hole
if pl.hole:
    ax.plot([pl.hole.x], [pl.hole.y], [pz + _off],
            'o', color='black', markersize=10, zorder=10,
            markeredgecolor='white', markeredgewidth=1.5)
    ax.text(pl.hole.x + 0.08, pl.hole.y, pz + 0.04, 'Hole',
            color='white', fontsize=6)
# Golf ball on platform
if pl.golf_ball:
    ax.plot([pl.golf_ball.x], [pl.golf_ball.y], [pz + _off],
            'o', color='orange', markersize=10, zorder=10,
            markeredgecolor='white', markeredgewidth=1.5)
    ax.text(pl.golf_ball.x + 0.08, pl.golf_ball.y, pz + 0.04, 'Ball',
            color='orange', fontsize=6)

# ─────────────────────────────────────────────────────────────────────
# 9. STAIRS  (from FIELD.stairs)
# ─────────────────────────────────────────────────────────────────────
st = FIELD.stairs
st_x0 = st.base.x
st_x1 = st.base.x + st.width
st_bot_y = st.base.y
st_top_y = st.top.y

for i in range(st.n_steps):
    sy0 = st_bot_y + i * st.step_depth
    sy1 = sy0 + st.step_depth
    sz0 = i * st.step_height
    sz1 = (i + 1) * st.step_height
    # Tread (horizontal face)
    ax.add_collection3d(Poly3DCollection(
        [[(st_x0,sy0,sz1),(st_x1,sy0,sz1),(st_x1,sy1,sz1),(st_x0,sy1,sz1)]],
        alpha=0.45, facecolor='#667788', edgecolor='white', linewidth=0.6))
    # Riser (vertical face)
    ax.add_collection3d(Poly3DCollection(
        [[(st_x0,sy0,sz0),(st_x1,sy0,sz0),(st_x1,sy0,sz1),(st_x0,sy0,sz1)]],
        alpha=0.30, facecolor='#556677', edgecolor='white', linewidth=0.6))

stair_mid_x = (st_x0 + st_x1) / 2
t_s = np.linspace(0, 1, 20)
ax.plot(np.full(20, stair_mid_x),
        st_bot_y + t_s * (st_top_y - st_bot_y),
        t_s * st.total_rise + _off,
        color='white', linewidth=3, zorder=8)
ax.text(stair_mid_x, (st_bot_y + st_top_y)/2, st.total_rise/2 + 0.05,
        f'Stairs\n{st.n_steps} steps', color='#aaccff', fontsize=7, ha='center')

# ─────────────────────────────────────────────────────────────────────
# 10. LONG RAMP  (from FIELD.long_ramp)
# ─────────────────────────────────────────────────────────────────────
lr = FIELD.long_ramp
draw_ramp_solid(lr, face='#886644')
t_lr = np.linspace(0, 1, 40)
ax.plot(lr.top.x + t_lr*(lr.base.x - lr.top.x),
        np.full(40, (lr.top.y)),
        lr.top.z + t_lr*(lr.base.z - lr.top.z) + _off,
        color='white', linewidth=3, zorder=8)
ax.text((lr.top.x+lr.base.x)/2, lr.top.y, lr.top.z/2 + 0.05,
        f'Long ramp\n{lr.run:.2f}m', color='#ffcc88', fontsize=7, ha='center')

# ─────────────────────────────────────────────────────────────────────
# 11. SEESAW  (from FIELD.seesaw)
# ─────────────────────────────────────────────────────────────────────
sw = FIELD.seesaw
swx = sw.pivot.x; swz = sw.pivot.z; sww = sw.width
swy0 = sw.pivot.y - sw.length / 2
swy1 = sw.pivot.y + sw.length / 2
ax.add_collection3d(Poly3DCollection(
    [[(swx-sww/2, swy0, swz), (swx+sww/2, swy0, swz),
      (swx+sww/2, swy1, swz), (swx-sww/2, swy1, swz)]],
    alpha=1.0, facecolor='#cc8833', edgecolor='orange', linewidth=1.2))
ax.plot([swx], [sw.pivot.y], [swz], '^', color='orange', markersize=10,
        zorder=8, markeredgecolor='white', markeredgewidth=0.8)
if sw.golf_ball_pos:
    ax.plot([sw.golf_ball_pos.x], [sw.golf_ball_pos.y], [sw.golf_ball_pos.z],
            'o', color='orange', markersize=9, zorder=10,
            markeredgecolor='white', markeredgewidth=1.2)
ax.text(swx + 0.15, sw.pivot.y, swz + 0.08,
        f'Seesaw\n{sw.length}m\nz≈{swz:.2f}m', color='orange', fontsize=7)

# ─────────────────────────────────────────────────────────────────────
# 12. BALL DISPENSER  (from FIELD.ball_dispenser)
# ─────────────────────────────────────────────────────────────────────
bd = FIELD.ball_dispenser
bx, by = bd.position.x, bd.position.y
bh = bd.trigger_height_m
bowl_angles = np.linspace(0, 2*np.pi, 7)
bowl_r = 0.18
for a1, a2 in zip(bowl_angles[:-1], bowl_angles[1:]):
    ax.add_collection3d(Poly3DCollection(
        [[(bx+bowl_r*np.cos(a1), by+bowl_r*np.sin(a1), 0.0),
          (bx+bowl_r*np.cos(a2), by+bowl_r*np.sin(a2), 0.0),
          (bx+bowl_r*np.cos(a2), by+bowl_r*np.sin(a2), bh),
          (bx+bowl_r*np.cos(a1), by+bowl_r*np.sin(a1), bh)]],
        alpha=0.4, facecolor='#aaaaaa', edgecolor='white', linewidth=0.5))
ball_color_map = {'blue': '#2266ff', 'red': '#ff3333', 'white': 'white'}
ball_offsets = [(0, 0, .20), (.08, .05, .20), (-.08, .05, .20),
                (.04, -.08, .20), (-.04, -.08, .20)]
for ball, (ox, oy, oz) in zip(bd.balls, ball_offsets):
    ax.plot([bx+ox], [by+oy], [oz], 'o',
            color=ball_color_map.get(ball.color, 'white'),
            markersize=8, zorder=10, markeredgecolor='gray', markeredgewidth=0.8)
ax.text(bx + 0.25, by, bh + 0.05,
        f'Ball dispenser\n({len(bd.balls)} balls)', color='#ff9966', fontsize=7)

# ─────────────────────────────────────────────────────────────────────
# 13. LUGGAGE SHUTTLE  (from FIELD.shuttle)
# ─────────────────────────────────────────────────────────────────────
sh = FIELD.shuttle
sx = (sh.path_start.x + sh.path_end.x) / 2
sd = 0.13; sht = sh.surface_height_m; sw2 = 0.30
ax.add_collection3d(Poly3DCollection(
    [[(sx-sw2/2,0,0),(sx+sw2/2,0,0),(sx+sw2/2,sd,0),(sx-sw2/2,sd,0)]],
    alpha=0.9, facecolor='#cc9900', edgecolor='white', linewidth=1.0))
ax.add_collection3d(Poly3DCollection(
    [[(sx-sw2/2,0,sht),(sx+sw2/2,0,sht),(sx+sw2/2,sd,sht),(sx-sw2/2,sd,sht)]],
    alpha=0.9, facecolor='#ddaa00', edgecolor='white', linewidth=1.0))
# Shuttle's own ArUco
ax.plot([sx], [sd], [sht/2], 'D', color='orange', markersize=6, zorder=10,
        markeredgecolor='darkorange', markeredgewidth=0.8)
ax.text(sx + 0.08, sd, sht/2 + 0.03, f'ID{sh.aruco_id}', color='orange', fontsize=6)
# Luggage on shuttle
for lug, lxo in zip(sh.luggage, [-0.07, 0.07]):
    lx = sx + lxo; ls = 0.06
    ax.add_collection3d(Poly3DCollection(
        [[(lx-ls/2, sd/2-ls/2, sht), (lx+ls/2, sd/2-ls/2, sht),
          (lx+ls/2, sd/2+ls/2, sht), (lx-ls/2, sd/2+ls/2, sht)]],
        alpha=0.9, facecolor='#4466aa', edgecolor='white', linewidth=0.8))
    ax.plot([lx], [sd/2], [sht+0.03], 'D', color='orange', markersize=4,
            zorder=11, markeredgecolor='darkorange', markeredgewidth=0.5)
    ax.text(lx, sd/2+0.05, sht+0.04,
            f'ID{lug.aruco_id}\n→{lug.target_zone}', color='orange', fontsize=5)

# ─────────────────────────────────────────────────────────────────────
# 14. SORTING CENTER  (from FIELD.sorting_center — zones + aruco)
# ─────────────────────────────────────────────────────────────────────
sc = FIELD.sorting_center
zone_colors = {'A': '#4488ff', 'B': '#44ff88', 'C': '#ff8844', 'D': '#ff44aa'}
cpt = (sc.center.x, sc.center.y, 0)
sorted_zones = ['A', 'B', 'C', 'D']

# Zone quadrant triangles — semi-transparent so grid shows through
for i, label in enumerate(sorted_zones):
    z1 = sc.zones[label]
    z2 = sc.zones[sorted_zones[(i+1) % 4]]
    ax.add_collection3d(Poly3DCollection(
        [[cpt, (z1.x, z1.y, 0), (z2.x, z2.y, 0)]],
        alpha=0.35, facecolor=zone_colors[label], edgecolor='white', linewidth=0.6))
    # Zone label at centroid
    mx = (cpt[0] + z1.x + z2.x) / 3
    my = (cpt[1] + z1.y + z2.y) / 3
    ax.text(mx, my, 0.06, label, color='white', fontsize=9,
            fontweight='bold', ha='center', va='center')

# ArUco markers: small diamonds, coloured by zone, offset label
zone_aruco_color = {
    10: zone_colors['A'], 11: zone_colors['A'],   # A = down
    12: zone_colors['B'], 13: zone_colors['B'],   # B = right
    14: zone_colors['C'], 15: zone_colors['C'],   # C = up
    16: zone_colors['D'], 17: zone_colors['D'],   # D = left
}
for m in sc.aruco_markers:
    color = zone_aruco_color.get(m.id, 'orange')
    ax.plot([m.position.x], [m.position.y], [_off],
            'D', color=color, markersize=4, zorder=10,
            markeredgecolor='white', markeredgewidth=0.5)
    ax.text(m.position.x + 0.04, m.position.y + 0.04, _off + 0.02,
            str(m.id), color=color, fontsize=5, zorder=11)

# ─────────────────────────────────────────────────────────────────────
# 15. ALL TAPE LINES + NAV PATHS  (from FIELD.tape_lines + FIELD.nav_paths)
# ─────────────────────────────────────────────────────────────────────
for tl in FIELD.tape_lines:
    draw_segment(tl)

for np_ in FIELD.nav_paths:
    for seg in np_.segments:
        draw_segment(seg)

# ─────────────────────────────────────────────────────────────────────
# 16. ALL GATES  (from FIELD.all_gates())
# ─────────────────────────────────────────────────────────────────────
for g in FIELD.all_gates():
    draw_gate(g)

# ─────────────────────────────────────────────────────────────────────
# 17. ALL ARUCO MARKERS  (from FIELD.all_aruco())
#     Sorting center markers already drawn above; skip duplicates
# ─────────────────────────────────────────────────────────────────────
sc_aruco_ids = {m.id for m in sc.aruco_markers}
for m in FIELD.all_aruco():
    if m.id not in sc_aruco_ids:
        draw_aruco(m, color='orange', size=8)

# ─────────────────────────────────────────────────────────────────────
# SAVE + SHOW
# ─────────────────────────────────────────────────────────────────────
plt.tight_layout()
plt.savefig('field_map_3d.png', dpi=150, bbox_inches='tight',
            facecolor=fig.get_facecolor())
print("Saved: field_map_3d.png")
print("The top-left panel updates as you move the mouse over the map.")
plt.show()