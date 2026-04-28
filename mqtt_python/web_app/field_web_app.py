import math
from typing import Optional, Tuple

import numpy as np
import plotly.graph_objects as go
from dash import Dash, dcc, html, Input, Output
from datetime import datetime

from field_map.field_map_2026 import FIELD
from field_map.field_objects import TapeLine, ArcSegment, BezierSegment, Ramp
from web_app.telemetry_hub import (
    hub,
    TOPIC_ENC_POSE, TOPIC_ENC_VEL, TOPIC_IMU_GYRO, TOPIC_IMU_ACC,
    TOPIC_KALMAN_STATE, TOPIC_VISION_POSE, TOPIC_VISION_ARUCO, TOPIC_VISION_LANDMARK,
)


OFF = 0.035


def context_text(px, py):
    ctx = FIELD.context(px, py)

    lines = [
        f"x = {ctx['px']:.3f} m",
        f"y = {ctx['py']:.3f} m",
        f"z = {ctx['pz']:.3f} m",
        f"zone = {ctx['zone']}",
        "",
    ]

    tape = ctx["nearest_tape"]
    if tape:
        lines += [
            "── NEAREST TAPE ─────",
            f"name     = {tape['name']}",
            f"dist     = {tape['dist'] * 100:.1f} cm",
            f"lateral  = {tape['lateral_error'] * 100:+.1f} cm",
            f"heading  = {tape['heading_deg']:.1f} deg",
            "",
        ]

    gate = ctx["nearest_gate"]
    if gate and gate["name"]:
        lines += [
            "── NEAREST GATE ─────",
            f"name     = {gate['name']}",
            f"dist     = {gate['dist'] * 100:.1f} cm",
            f"sat      = {'yes' if gate['has_satellite'] else 'no'}",
            "",
        ]

    if ctx["nearby_aruco"]:
        lines += ["── NEARBY ARUCO ─────"]
        for m in ctx["nearby_aruco"][:4]:
            lines.append(f"ID {m['id']}   {m['dist'] * 100:.0f} cm")

    return "\n".join(lines)


def add_line(fig, xs, ys, zs, color="white", width=5, name="line"):
    fig.add_trace(go.Scatter3d(
        x=xs,
        y=ys,
        z=zs,
        mode="lines",
        line=dict(color=color, width=width),
        name=name,
        hoverinfo="skip",
        showlegend=False,
    ))


def add_marker(fig, x, y, z, color="orange", size=5, name="marker", text=None):
    fig.add_trace(go.Scatter3d(
        x=[x],
        y=[y],
        z=[z],
        mode="markers+text" if text else "markers",
        marker=dict(
            size=size,
            color=color,
            line=dict(color="white", width=1) if text else None,
        ),
        text=[text] if text else None,
        textposition="top center",
        textfont=dict(color="white", size=12),
        name=name,
        hoverinfo="skip",
        showlegend=False,
    ))


def add_mesh_box(fig, x0, x1, y0, y1, z0, z1, color="#667788", opacity=0.55, name="box"):
    vertices = [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),
    ]

    x, y, z = zip(*vertices)

    i = [0, 0, 0, 1, 4, 4, 2, 3, 0, 1, 5, 6]
    j = [1, 2, 4, 5, 5, 6, 6, 7, 3, 2, 6, 7]
    k = [2, 3, 5, 6, 6, 7, 7, 4, 7, 6, 7, 4]

    fig.add_trace(go.Mesh3d(
        x=x,
        y=y,
        z=z,
        i=i,
        j=j,
        k=k,
        color=color,
        opacity=opacity,
        name=name,
        hoverinfo="skip",
        showlegend=False,
    ))


def add_disk(fig, cx, cy, radius, z_bottom, z_top, color, opacity=1.0, n_segments=72, name="disk"):
    """Draw a solid cylindrical slab (top disk + side wall). Bottom face omitted (hidden by floor)."""
    angles = np.linspace(0.0, 2.0 * np.pi, n_segments, endpoint=False)
    cos_a = np.cos(angles)
    sin_a = np.sin(angles)

    # Vertex layout:
    #   0                 : top centre
    #   1 .. n            : top ring
    #   n+1 .. 2n         : bottom ring
    top_ring_x = cx + radius * cos_a
    top_ring_y = cy + radius * sin_a

    xs = [cx] + list(top_ring_x) + list(top_ring_x)
    ys = [cy] + list(top_ring_y) + list(top_ring_y)
    zs = [z_top] + [z_top] * n_segments + [z_bottom] * n_segments

    i, j, k = [], [], []
    for s in range(n_segments):
        t0 = 1 + s
        t1 = 1 + (s + 1) % n_segments
        b0 = 1 + n_segments + s
        b1 = 1 + n_segments + (s + 1) % n_segments

        # top fan triangle
        i.append(0); j.append(t0); k.append(t1)
        # side quad = two triangles
        i.append(t0); j.append(t1); k.append(b0)
        i.append(t1); j.append(b1); k.append(b0)

    fig.add_trace(go.Mesh3d(
        x=xs, y=ys, z=zs, i=i, j=j, k=k,
        color=color, opacity=opacity, name=name,
        hoverinfo="skip", showlegend=False,
        flatshading=True,
    ))


def add_wall(fig, p0, p1, height, thickness=0.03, color="lime", opacity=1.0, name="wall"):
    """Draw a vertical wall slab from p0 to p1 (2D points) with given height and thickness."""
    x0, y0 = p0
    x1, y1 = p1
    dx = x1 - x0
    dy = y1 - y0
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return

    nx = -dy / length * thickness / 2
    ny = dx / length * thickness / 2

    add_mesh_box_oriented(
        fig,
        corners_2d=[
            (x0 - nx, y0 - ny),
            (x1 - nx, y1 - ny),
            (x1 + nx, y1 + ny),
            (x0 + nx, y0 + ny),
        ],
        z0=0.0,
        z1=height,
        color=color,
        opacity=opacity,
        name=name,
    )


def add_mesh_box_oriented(fig, corners_2d, z0, z1, color, opacity=1.0, name="box"):
    (ax, ay), (bx, by), (cx, cy), (dx_, dy_) = corners_2d
    vertices = [
        (ax, ay, z0), (bx, by, z0), (cx, cy, z0), (dx_, dy_, z0),
        (ax, ay, z1), (bx, by, z1), (cx, cy, z1), (dx_, dy_, z1),
    ]
    x, y, z = zip(*vertices)

    i = [0, 0, 4, 4, 0, 0, 1, 1, 2, 2, 3, 3]
    j = [1, 2, 5, 6, 1, 4, 2, 5, 3, 6, 0, 7]
    k = [2, 3, 6, 7, 4, 5, 5, 6, 6, 7, 7, 4]

    fig.add_trace(go.Mesh3d(
        x=x, y=y, z=z, i=i, j=j, k=k,
        color=color, opacity=opacity, name=name,
        hoverinfo="skip", showlegend=False,
        flatshading=True,
    ))


def add_ramp_solid(fig, ramp: Ramp, color="#886644", opacity=0.7):
    b = ramp.base
    t = ramp.top

    dx = t.x - b.x
    dy = t.y - b.y
    run = math.hypot(dx, dy)

    if run < 1e-6:
        return

    px = -dy / run * ramp.width / 2
    py = dx / run * ramp.width / 2

    vertices = [
        (b.x - px, b.y - py, 0.0),
        (b.x + px, b.y + py, 0.0),
        (t.x + px, t.y + py, 0.0),
        (t.x - px, t.y - py, 0.0),

        (b.x - px, b.y - py, b.z),
        (b.x + px, b.y + py, b.z),
        (t.x + px, t.y + py, t.z),
        (t.x - px, t.y - py, t.z),
    ]

    x, y, z = zip(*vertices)

    i = [4, 4, 0, 0, 1, 1, 0, 0, 3, 3, 0, 0]
    j = [5, 6, 1, 2, 5, 6, 4, 7, 7, 6, 1, 5]
    k = [6, 7, 2, 3, 6, 2, 7, 3, 6, 2, 5, 4]

    fig.add_trace(go.Mesh3d(
        x=x,
        y=y,
        z=z,
        i=i,
        j=j,
        k=k,
        color=color,
        opacity=opacity,
        name=ramp.name,
        hoverinfo="skip",
        showlegend=False,
    ))


def add_segment(fig, seg, color="white", width=6):
    if isinstance(seg, TapeLine):
        xs = [p.x for p in seg.waypoints]
        ys = [p.y for p in seg.waypoints]
        zs = [p.z + OFF for p in seg.waypoints]
        add_line(fig, xs, ys, zs, color=color, width=width, name=seg.name)

    elif isinstance(seg, ArcSegment):
        xs, ys, zs = seg.sample()
        add_line(fig, xs, ys, zs + OFF, color=color, width=width)

    elif isinstance(seg, BezierSegment):
        xs, ys, zs = seg.sample()
        add_line(fig, xs, ys, zs + OFF, color=color, width=width)


def add_gate(fig, g):
    cx, cy = g.center.x, g.center.y
    bz = g.center.z + g.bottom_z
    tz = bz + g.height

    color = "#ddaa00"

    if g.radial_from is not None:
        rx, ry = g.radial_from

        add_line(fig, [rx, rx], [ry, ry], [bz, tz], color=color, width=7)
        add_line(fig, [rx, cx], [ry, cy], [tz, tz], color=color, width=7)
        add_line(fig, [cx, cx], [cy, cy], [tz, bz], color=color, width=7)

        if g.has_satellite:
            mx = (rx + cx) / 2
            my = (ry + cy) / 2
            sz = tz + g.satellite_height
            add_marker(fig, mx, my, sz, color="cyan", size=6, text="SAT")

    else:
        angle = g.line_angle_deg if g.line_angle_deg is not None else g.orientation_deg + 90
        rad = math.radians(angle)

        dx = math.cos(rad) * g.width / 2
        dy = math.sin(rad) * g.width / 2

        add_line(fig, [cx - dx, cx - dx], [cy - dy, cy - dy], [bz, tz], color=color, width=7)
        add_line(fig, [cx - dx, cx + dx], [cy - dy, cy + dy], [tz, tz], color=color, width=7)
        add_line(fig, [cx + dx, cx + dx], [cy + dy, cy + dy], [tz, bz], color=color, width=7)

        if g.has_satellite:
            sz = tz + g.satellite_height
            add_marker(fig, cx, cy, sz, color="cyan", size=6, text="SAT")


def build_figure():
    fig = go.Figure()

    # Invisible hover grid
    xs = np.arange(0, FIELD.width_m + 0.001, 0.05)
    ys = np.arange(0, FIELD.height_m + 0.001, 0.05)

    gx, gy, gz, custom = [], [], [], []

    for x in xs:
        for y in ys:
            z = FIELD.pz_at(float(x), float(y))
            gx.append(float(x))
            gy.append(float(y))
            gz.append(float(z) + 0.01)
            custom.append([float(x), float(y)])

    fig.add_trace(go.Scatter3d(
        x=gx,
        y=gy,
        z=gz,
        mode="markers",
        marker=dict(size=2, color="rgba(255,255,255,0)"),
        customdata=custom,
        hovertemplate="x=%{customdata[0]:.2f}<br>y=%{customdata[1]:.2f}<extra></extra>",
        name="hover-grid",
        showlegend=False,
    ))

    # Outer visual margin floor
    fig.add_trace(go.Mesh3d(
        x=[-0.5, FIELD.width_m + 0.5, FIELD.width_m + 0.5, -0.5],
        y=[-0.5, -0.5, FIELD.height_m + 0.5, FIELD.height_m + 0.5],
        z=[-0.005, -0.005, -0.005, -0.005],
        i=[0, 0],
        j=[1, 2],
        k=[2, 3],
        color="#202020",
        opacity=0.65,
        hoverinfo="skip",
        showlegend=False,
    ))

    # Field grid
    for i in range(int(FIELD.width_m) + 1):
        add_line(fig, [i, i], [0, FIELD.height_m], [0, 0], color="rgba(255,255,255,0.18)", width=1)

    for j in range(int(FIELD.height_m) + 1):
        add_line(fig, [0, FIELD.width_m], [j, j], [0, 0], color="rgba(255,255,255,0.18)", width=1)

    # Field border
    W, H = FIELD.width_m, FIELD.height_m
    borders = [
        ([0, W], [0, 0], [0, 0]),
        ([W, W], [0, H], [0, 0]),
        ([W, 0], [H, H], [0, 0]),
        ([0, 0], [H, 0], [0, 0]),
    ]

    for xs_, ys_, zs_ in borders:
        add_line(fig, xs_, ys_, zs_, color="deepskyblue", width=6)

    # Green barriers — solid walls, height = FIELD.green_barrier_z
    p0, p1 = FIELD.green_y_line
    add_wall(fig, (p0.x, p0.y), (p1.x, p1.y),
             height=FIELD.green_barrier_z, color="lime", name="green_y_wall")

    p0, p1 = FIELD.shuttle_green_border
    add_wall(fig, (p0.x, p0.y), (p1.x, p1.y),
             height=FIELD.green_barrier_z, color="lime", name="shuttle_border_wall")

    # Time extension buttons
    for tb in FIELD.time_buttons:
        fig.add_trace(go.Scatter3d(
            x=[tb.center.x],
            y=[tb.center.y],
            z=[tb.z_height],
            mode="markers+text",
            marker=dict(
                size=14,
                color="red",
                symbol="circle",
                line=dict(color="white", width=2),
            ),
            text=["+90s"],
            textposition="top center",
            textfont=dict(color="white", size=13),
            name=tb.name,
            hovertemplate=(
                f"{tb.name}<br>"
                "x=%{x:.2f}<br>"
                "y=%{y:.2f}<br>"
                "z=%{z:.2f}<extra></extra>"
            ),
            showlegend=False,
        ))

    # Start
    sa = FIELD.start_area
    add_line(fig, [sa.origin.x, sa.origin.x + sa.width], [0, 0], [0, 0], color="deepskyblue", width=8)
    add_line(fig, [sa.origin.x, sa.origin.x], [0, sa.depth], [0, 0], color="deepskyblue", width=8)
    add_line(fig, [sa.origin.x + sa.width, sa.origin.x + sa.width], [0, sa.depth], [0, 0], color="deepskyblue", width=8)
    add_marker(fig, sa.center.x, sa.center.y, 0.08, color="deepskyblue", size=3, text="Start")

    # Goal
    gp = FIELD.goal_point
    gs = 0.18
    add_mesh_box(fig, gp.x - gs / 2, gp.x + gs / 2, 0, gs, 0, 0.02, color="#2244cc", opacity=0.9)
    add_marker(fig, gp.x, gs / 2, 0.08, color="deepskyblue", size=3, text="Goal")

    # Roundabout — 1 cm raised opaque plate + white rim on top
    ro = FIELD.roundabout
    add_disk(
        fig,
        cx=ro.center.x, cy=ro.center.y,
        radius=ro.radius,
        z_bottom=0.0, z_top=ro.plate_z,
        color="#cccccc", opacity=1.0,
        name="roundabout_plate",
    )
    theta = np.linspace(0, 2 * np.pi, 150)
    add_line(
        fig,
        ro.center.x + ro.radius * np.cos(theta),
        ro.center.y + ro.radius * np.sin(theta),
        np.full_like(theta, ro.plate_z + 0.001),
        color="white",
        width=5,
    )
    add_marker(fig, ro.center.x, ro.center.y, ro.plate_z + 0.08,
               color="white", size=3, text="Roundabout")

    # Infinity path as two ellipses
    inf = FIELD.infinity_path
    theta = np.linspace(0, 2 * np.pi, 180)

    for c in [inf.left_center, inf.right_center]:
        add_line(
            fig,
            c.x + inf.radius_x * np.cos(theta),
            c.y + inf.radius_y * np.sin(theta),
            np.zeros_like(theta),
            color="white",
            width=6,
        )

    add_marker(fig, inf.center.x, inf.center.y, 0.08, color="#ccccff", size=3, text="Infinity")

    # Ramps
    add_ramp_solid(fig, FIELD.short_ramp)
    add_ramp_solid(fig, FIELD.long_ramp)

    # Platform
    pl = FIELD.platform
    add_mesh_box(
        fig,
        pl.origin.x,
        pl.origin.x + pl.width,
        pl.origin.y,
        pl.origin.y + pl.depth,
        0,
        pl.pz,
        color="#556655",
        opacity=0.55,
    )
    add_marker(fig, pl.center.x, pl.center.y, pl.pz + 0.08, color="lightgreen", size=3, text="Platform")

    if pl.hole:
        add_marker(fig, pl.hole.x, pl.hole.y, pl.pz + OFF, color="black", size=6, text="Hole")

    if pl.golf_ball:
        add_marker(fig, pl.golf_ball.x, pl.golf_ball.y, pl.pz + OFF, color="orange", size=6, text="Ball")

    # Stairs
    st = FIELD.stairs
    sx0 = st.base.x - st.width / 2
    sx1 = st.base.x + st.width / 2

    # All steps drawn — the last step's top is at z=plat_z, so its top face
    # blends with the platform surface (no visible gap, no overlap).
    for i in range(st.n_steps):
        sy0 = st.base.y + i * st.step_depth
        sy1 = sy0 + st.step_depth
        z1 = (i + 1) * st.step_height

        add_mesh_box(
            fig,
            sx0,
            sx1,
            sy0,
            sy1,
            0,
            z1,
            color="#667788",
            opacity=0.55,
        )

    add_marker(
        fig,
        st.base.x,
        (st.base.y + st.top.y) / 2,
        st.total_rise / 2 + 0.08,
        color="#aaccff",
        size=3,
        text="Stairs",
    )

    # Seesaw
    sw = FIELD.seesaw
    x0 = sw.pivot.x - sw.width / 2
    x1 = sw.pivot.x + sw.width / 2
    y0 = sw.pivot.y - sw.length / 2
    y1 = sw.pivot.y + sw.length / 2

    fig.add_trace(go.Mesh3d(
        x=[x0, x1, x1, x0],
        y=[y0, y0, y1, y1],
        z=[sw.pivot.z, sw.pivot.z, sw.pivot.z, sw.pivot.z],
        i=[0, 0],
        j=[1, 2],
        k=[2, 3],
        color="#cc8833",
        opacity=0.9,
        hoverinfo="skip",
        showlegend=False,
    ))

    add_marker(fig, sw.pivot.x, sw.pivot.y, sw.pivot.z + 0.08,
               color="lightgreen", size=3, text="Seesaw")

    if sw.golf_ball_pos:
        add_marker(
            fig,
            sw.golf_ball_pos.x,
            sw.golf_ball_pos.y,
            sw.golf_ball_pos.z,
            color="orange",
            size=6,
            text="Ball",
        )

    # Ball dispenser
    bd = FIELD.ball_dispenser
    add_marker(fig, bd.position.x, bd.position.y, bd.trigger_height_m, color="#ff9966", size=8, text="Ball dispenser")

    # Shuttle
    sh = FIELD.shuttle
    sx = (sh.path_start.x + sh.path_end.x) / 2
    add_mesh_box(fig, sx - 0.15, sx + 0.15, 0, 0.13, 0, sh.surface_height_m, color="#ddaa00", opacity=0.8)
    add_marker(fig, sx, 0.13, sh.surface_height_m + 0.03, color="orange", size=5, text=f"ID{sh.aruco_id}")

    # Sorting center
    sc = FIELD.sorting_center
    add_marker(fig, sc.center.x, sc.center.y, 0.08, color="white", size=4, text="Sorting")

    zone_colors = {
        "A": "#4488ff",
        "B": "#44ff88",
        "C": "#ff8844",
        "D": "#ff44aa",
    }

    # Diamond outlined by NSEW apexes, divided internally with an "X"
    # (lines from centre to each EDGE midpoint). Each zone is a quadrilateral
    # (centre, edge_mid_a, apex, edge_mid_b) drawn as two triangles.
    h = sc.zone_size
    cx, cy = sc.center.x, sc.center.y
    # Edge midpoints (lie on the diamond's edges, at 45° from the centre):
    NE_m = (cx + h / 2, cy + h / 2)
    SE_m = (cx + h / 2, cy - h / 2)
    SW_m = (cx - h / 2, cy - h / 2)
    NW_m = (cx - h / 2, cy + h / 2)
    zone_quads = {
        "B": (NW_m, NE_m),  # north apex flanked by NW and NE edge mids
        "A": (NE_m, SE_m),  # east apex
        "D": (SE_m, SW_m),  # south apex
        "C": (SW_m, NW_m),  # west apex
    }
    for label, (em1, em2) in zone_quads.items():
        apex = sc.zones[label]
        # 4-vertex quad: centre, em1, apex, em2 → split into two triangles.
        fig.add_trace(go.Mesh3d(
            x=[cx,    em1[0], apex.x, em2[0]],
            y=[cy,    em1[1], apex.y, em2[1]],
            z=[0.01, 0.01,   0.01,   0.01],
            i=[0, 0],
            j=[1, 2],
            k=[2, 3],
            color=zone_colors[label],
            opacity=0.45,
            hoverinfo="skip",
            showlegend=False,
        ))
        # Label sits at the quad centroid (apex pulled slightly toward centre).
        mx = (cx + em1[0] + apex.x + em2[0]) / 4
        my = (cy + em1[1] + apex.y + em2[1]) / 4
        add_marker(fig, mx, my, 0.08, color=zone_colors[label], size=3, text=label)

    # ArUco markers
    for m in FIELD.all_aruco():
        add_marker(fig, m.position.x, m.position.y, m.position.z + OFF, color="orange", size=5, text=f"ID{m.id}")

    # Tape lines
    for tl in FIELD.tape_lines:
        add_segment(fig, tl)

    for np_ in FIELD.nav_paths:
        for seg in np_.segments:
            add_segment(fig, seg)

    # Gates
    for g in FIELD.all_gates():
        add_gate(fig, g)

    # Landmarks — geometrically unique points the camera will recognise to fix absolute pose
    kind_colors = {
        "fork_y":          "#ff44ff",
        "t_intersection":  "#cc44ff",
        "ramp_start":      "#44ddff",
        "ramp_end":        "#44ddff",
        "stair_step":      "#ffaa44",
        "fixture_anchor":  "#88ff88",
    }
    for lm in FIELD.landmarks:
        col = kind_colors.get(lm.kind, "#ff44ff")
        fig.add_trace(go.Scatter3d(
            x=[lm.position.x],
            y=[lm.position.y],
            z=[lm.position.z + 0.18],
            mode="markers",
            marker=dict(size=10, color=col, symbol="diamond",
                        line=dict(color="white", width=1.5)),
            name=lm.name,
            hovertemplate=(
                f"<b>{lm.name}</b><br>"
                f"kind={lm.kind}<br>"
                "x=%{x:.2f}  y=%{y:.2f}  z=%{z:.2f}"
                f"<br>radius={lm.confidence_radius*100:.0f} cm<extra></extra>"
            ),
            showlegend=False,
        ))
        # confidence radius footprint on the floor
        theta = np.linspace(0, 2*np.pi, 64)
        fig.add_trace(go.Scatter3d(
            x=lm.position.x + lm.confidence_radius * np.cos(theta),
            y=lm.position.y + lm.confidence_radius * np.sin(theta),
            z=np.full_like(theta, lm.position.z + 0.005),
            mode="lines",
            line=dict(color=col, width=2, dash="dot"),
            hoverinfo="skip",
            showlegend=False,
        ))

    fig.update_layout(
        paper_bgcolor="#1a1a1a",
        plot_bgcolor="#1a1a1a",
        # uirevision is checked when the figure is replaced: matching values
        # mean "preserve user-applied camera angle / zoom". Without it, every
        # 500 ms tick from the live-overlay callback would reset the view.
        uirevision="field-map",
        margin=dict(l=0, r=0, t=35, b=0),
        title=dict(
            text="DTU Robocup 2026 — Interactive 3D Field Map",
            font=dict(color="white", size=20),
            x=0.5,
        ),
        scene=dict(
            bgcolor="#2e2e2e",

            xaxis=dict(
                range=[-0.5, FIELD.width_m + 0.5],
                title="X (m)",
                color="white",
                backgroundcolor="#2e2e2e",
                gridcolor="rgba(255,255,255,0.14)",
                zerolinecolor="rgba(255,255,255,0.25)",
                showbackground=True,
                showgrid=True,
            ),

            yaxis=dict(
                range=[-0.5, FIELD.height_m + 0.5],
                title="Y (m)",
                color="white",
                backgroundcolor="#2e2e2e",
                gridcolor="rgba(255,255,255,0.14)",
                zerolinecolor="rgba(255,255,255,0.25)",
                showbackground=True,
                showgrid=True,
            ),

            zaxis=dict(
                range=[0, 2],
                title="Z (m)",
                color="white",
                backgroundcolor="#2e2e2e",
                gridcolor="rgba(255,255,255,0.14)",
                zerolinecolor="rgba(255,255,255,0.25)",
                showbackground=True,
                showgrid=True,
            ),

            aspectmode="manual",
            aspectratio=dict(x=8, y=7, z=2),

            camera=dict(
                eye=dict(x=1.7, y=-1.8, z=1.1),
                center=dict(x=0, y=0, z=0),
            ),
        ),
        font=dict(color="white"),
    )

    return fig


# Parse CLI once at import time so the layout can reflect e.g. --camera-stream.
import argparse as _argparse
import sys as _sys


def _parse_cli():
    p = _argparse.ArgumentParser(add_help=False)
    p.add_argument("--mock",          action="store_true")
    p.add_argument("--broker",        default="localhost")
    p.add_argument("--port",          type=int, default=1883)
    p.add_argument("--host",          default="127.0.0.1")
    p.add_argument("--web-port",      type=int, default=8050)
    p.add_argument("--camera-stream", default=None,
                   help="Raw camera MJPEG URL, e.g. http://10.197.216.254:7123/stream.mjpg")
    p.add_argument("--annotated-stream", default=None,
                   help="Annotated/perception MJPEG URL (overlay with line, fork, ArUco)")
    args, _unknown = p.parse_known_args(_sys.argv[1:])
    return args


CLI_ARGS = _parse_cli()


app = Dash(__name__)

# Eat the browser's default <html>/<body> margins and match the page background
# to the field map's dark theme so there's no white border around the app.
app.index_string = """
<!DOCTYPE html>
<html>
    <head>
        {%metas%}
        <title>{%title%}</title>
        {%favicon%}
        {%css%}
        <style>
            html, body {
                margin: 0;
                padding: 0;
                background-color: #1a1a1a;
                overflow: hidden;
            }
            #_dash-app-content { background-color: #1a1a1a; }
        </style>
    </head>
    <body>
        {%app_entry%}
        <footer>
            {%config%}
            {%scripts%}
            {%renderer%}
        </footer>
    </body>
</html>
"""


# ---------------------------------------------------------------------------
# Live telemetry panel — colour-coded streams + event log fed from telemetry_hub
# ---------------------------------------------------------------------------

# Per-topic display config: (topic, label, formatter, expected_max_age_s)
# `expected_max_age_s` is the cell's "yellow" threshold; > 2.5× this is "red".
def _fmt_pose(v):
    if not v: return "—"
    return f"x={v.get('x', 0):+.3f}  y={v.get('y', 0):+.3f}  yaw={math.degrees(v.get('yaw', 0)):+.1f}°"

def _fmt_vel(v):
    if not v: return "—"
    return f"vL={v.get('vL', 0):+.3f}  vR={v.get('vR', 0):+.3f}"

def _fmt_gyro(v):
    if not v: return "—"
    return f"gx={v.get('gx', 0):+.3f}  gy={v.get('gy', 0):+.3f}  gz={v.get('gz', 0):+.3f}"

def _fmt_acc(v):
    if not v: return "—"
    return f"ax={v.get('ax', 0):+.2f}  ay={v.get('ay', 0):+.2f}  az={v.get('az', 0):+.2f}"

def _fmt_kalman(v):
    if not v: return "—"
    return (f"x={v.get('x', 0):+.3f}  y={v.get('y', 0):+.3f}  "
            f"yaw={math.degrees(v.get('yaw', 0)):+.1f}°  "
            f"v={v.get('velocity', 0):+.2f}m/s")

def _fmt_vision_pose(v):
    if not v: return "—"
    src = v.get("source", "?")
    return f"{src}:  x={v.get('x', 0):+.3f}  y={v.get('y', 0):+.3f}  yaw={math.degrees(v.get('yaw', 0)):+.1f}°"

def _fmt_aruco(v):
    if not v: return "—"
    return f"ID={v.get('id', '?')}  range={v.get('range', 0):.2f}m  bearing={math.degrees(v.get('bearing', 0)):+.1f}°"

def _fmt_landmark(v):
    if not v: return "—"
    return f"{v.get('name', '?')}  conf={v.get('confidence', 0):.2f}"


RAW_STREAMS = [
    (TOPIC_ENC_POSE, "encoder pose",   _fmt_pose,   1.0),
    (TOPIC_ENC_VEL,  "wheel vel",      _fmt_vel,    1.0),
    (TOPIC_IMU_GYRO, "IMU gyro",       _fmt_gyro,   0.5),
    (TOPIC_IMU_ACC,  "IMU accel",      _fmt_acc,    0.5),
]

FUSION_STREAMS = [
    (TOPIC_KALMAN_STATE,    "Kalman state",    _fmt_kalman,      0.5),
    (TOPIC_VISION_POSE,     "vision pose",     _fmt_vision_pose, 8.0),
    (TOPIC_VISION_ARUCO,    "ArUco fix",       _fmt_aruco,       8.0),
    (TOPIC_VISION_LANDMARK, "landmark fix",    _fmt_landmark,   12.0),
]


def _age_color(age_s, expected_max):
    """Pick a cell colour based on how stale the value is."""
    if age_s is None:
        return "#444"          # never received
    if age_s < 0.4:
        return "#1f8a3a"       # green-flash: fresh value just arrived
    if age_s < expected_max:
        return "#888"          # normal live
    if age_s < expected_max * 2.5:
        return "#a07a00"       # stale-yellow
    return "#a02020"           # red: probably offline


def _format_age(age_s):
    if age_s is None:
        return "no data"
    if age_s < 1.0:
        return f"{age_s*1000:.0f} ms"
    if age_s < 60.0:
        return f"{age_s:.1f} s"
    return f"{age_s/60.0:.1f} min"


def _build_stream_row(label, value_text, age_s, expected_max):
    color = _age_color(age_s, expected_max)
    age_text = _format_age(age_s)
    return html.Div(
        style={
            "display": "flex",
            "alignItems": "center",
            "padding": "4px 10px",
            "borderLeft": f"4px solid {color}",
            "marginBottom": "3px",
            "backgroundColor": "#1c1c1c",
            "fontFamily": "Menlo, monospace",
            "fontSize": "12px",
        },
        children=[
            html.Div(label, style={"width": "120px", "color": "#bbb"}),
            html.Div(value_text, style={"flex": 1, "color": "#e0e0e0"}),
            html.Div(age_text, style={"width": "80px", "textAlign": "right",
                                      "color": color, "fontWeight": "bold"}),
        ],
    )


def _build_event_row(ev, now):
    age = now - ev["t"]
    src_color = {
        "aruco": "#4ad",  "landmark": "#f4f", "vision": "#4f4",
        "kalman": "#fa4", "system": "#888",  "sensor": "#bbb",
    }.get(ev["source"], "#bbb")
    ts = datetime.fromtimestamp(ev["t"]).strftime("%H:%M:%S")
    return html.Div(
        style={"fontFamily": "Menlo, monospace", "fontSize": "11.5px",
               "padding": "2px 10px", "color": "#ccc"},
        children=[
            html.Span(ts, style={"color": "#777", "marginRight": "8px"}),
            html.Span(f"[{ev['source']}]", style={"color": src_color, "marginRight": "8px"}),
            html.Span(ev["text"]),
            html.Span(f"  ({age:.1f}s ago)", style={"color": "#555", "marginLeft": "6px"}),
        ],
    )


def _telemetry_block_title(text, color="#9cf"):
    return html.Div(text, style={
        "fontSize": "12px", "fontWeight": "bold", "color": color,
        "letterSpacing": "1px", "marginBottom": "6px", "marginTop": "4px",
    })


def render_telemetry(snap):
    streams = snap["streams"]
    now = snap["now"]

    def row(topic, label, formatter, expected_max):
        s = streams.get(topic, {})
        last = s.get("last_received") or 0.0
        age = (now - last) if last > 0 else None
        return _build_stream_row(label, formatter(s.get("value")), age, expected_max)

    raw_rows    = [row(*cfg) for cfg in RAW_STREAMS]
    fusion_rows = [row(*cfg) for cfg in FUSION_STREAMS]
    event_rows  = [_build_event_row(ev, now) for ev in reversed(snap["events"])]

    mode_label = {"mqtt": "● LIVE (MQTT)", "mock": "○ MOCK", "idle": "○ idle"}.get(
        snap.get("mode", "idle"), "?")
    mode_color = {"mqtt": "#3a3", "mock": "#a83", "idle": "#666"}.get(
        snap.get("mode", "idle"), "#666")

    return html.Div(
        style={"display": "grid", "gridTemplateColumns": "1fr 1fr 1fr", "gap": "16px"},
        children=[
            html.Div(children=[
                _telemetry_block_title("RAW SENSORS  (robobot/drive/T0/...)"),
                *raw_rows,
            ]),
            html.Div(children=[
                _telemetry_block_title("FUSION  (robobot/...)"),
                *fusion_rows,
            ]),
            html.Div(children=[
                html.Div(style={"display": "flex", "justifyContent": "space-between",
                                "marginBottom": "6px"},
                         children=[
                             _telemetry_block_title("EVENT LOG"),
                             html.Span(mode_label,
                                       style={"fontSize": "11px", "color": mode_color,
                                              "fontWeight": "bold"}),
                         ]),
                html.Div(style={"maxHeight": "260px", "overflowY": "auto"},
                         children=event_rows or [html.Div("(no events yet)",
                                                          style={"color": "#666",
                                                                 "fontStyle": "italic",
                                                                 "padding": "4px 10px"})]),
            ]),
        ],
    )


app.layout = html.Div(
    style={
        "height": "100vh",
        "backgroundColor": "#1a1a1a",
        "display": "flex",
        "flexDirection": "column",
        "overflow": "hidden",
        "fontFamily": "Arial, sans-serif",
    },
    children=[
        # ---- Top half: camera | field info | 3D map ----
        html.Div(
            style={"display": "flex", "flexDirection": "row", "flex": "1 1 auto",
                   "minHeight": 0, "borderBottom": "1px solid #333"},
            children=[
                # LEFT — camera feed with raw/annotated toggle
                html.Div(
                    style={
                        "width": "420px",
                        "backgroundColor": "#000",
                        "color": "#888",
                        "boxSizing": "border-box",
                        "borderRight": "1px solid #333",
                        "display": "flex",
                        "flexDirection": "column",
                        "alignItems": "stretch",
                    },
                    children=[
                        # Header bar: title + raw/annotated toggle
                        html.Div(
                            style={
                                "display": "flex", "alignItems": "center",
                                "justifyContent": "space-between",
                                "padding": "8px 14px", "borderBottom": "1px solid #222",
                            },
                            children=[
                                html.Div("CAMERA",
                                         style={"fontSize": "12px",
                                                "letterSpacing": "1px",
                                                "color": "#9cf"}),
                                dcc.RadioItems(
                                    id="camera-mode",
                                    options=[
                                        {"label": "raw",
                                         "value": "raw",
                                         "disabled": CLI_ARGS.camera_stream is None},
                                        {"label": "annotated",
                                         "value": "annotated",
                                         "disabled": CLI_ARGS.annotated_stream is None},
                                    ],
                                    value=("annotated"
                                           if CLI_ARGS.annotated_stream and not CLI_ARGS.camera_stream
                                           else "raw"),
                                    inline=True,
                                    inputStyle={"marginRight": "4px",
                                                "marginLeft": "10px"},
                                    labelStyle={"color": "#bbb", "fontSize": "12px",
                                                "cursor": "pointer"},
                                ),
                            ],
                        ),
                        # Content area: filled by the camera-content callback
                        html.Div(
                            id="camera-content",
                            style={"flex": 1, "display": "flex",
                                   "alignItems": "center", "justifyContent": "center",
                                   "minHeight": 0, "backgroundColor": "#000"},
                        ),
                    ],
                ),
                # MIDDLE — field info (hover details only)
                html.Div(
                    style={
                        "width": "320px",
                        "backgroundColor": "#111",
                        "color": "white",
                        "padding": "16px",
                        "boxSizing": "border-box",
                        "borderRight": "1px solid #444",
                        "overflowY": "auto",
                    },
                    children=[
                        html.H2("Field Info", style={"marginTop": "0"}),
                        html.Pre(
                            id="info-box",
                            children="",
                            style={
                                "fontSize": "14px",
                                "lineHeight": "1.35",
                                "whiteSpace": "pre-wrap",
                                "color": "#aaddff",
                            },
                        ),
                    ],
                ),
                # RIGHT — 3D map
                html.Div(
                    style={"flex": "1", "minWidth": 0, "position": "relative"},
                    children=[
                        # Floating "Follow Robot" toggle in the upper-right
                        # corner of the map. When on, the 3D camera tracks the
                        # robot's pose; when off, the camera is fully manual.
                        html.Div(
                            style={
                                "position": "absolute",
                                "top": "8px",
                                "right": "12px",
                                "zIndex": 5,
                                "backgroundColor": "rgba(0,0,0,0.55)",
                                "padding": "4px 10px",
                                "borderRadius": "4px",
                            },
                            children=[
                                dcc.Checklist(
                                    id="follow-robot",
                                    options=[{"label": " Follow Robot", "value": "follow"}],
                                    value=["follow"],
                                    inputStyle={"marginRight": "4px"},
                                    labelStyle={"color": "#bbb", "fontSize": "12px",
                                                "cursor": "pointer"},
                                ),
                            ],
                        ),
                        dcc.Graph(
                            id="field-graph",
                            figure=build_figure(),
                            clear_on_unhover=False,
                            style={"height": "100%", "width": "100%"},
                            config={
                                "displayModeBar": True,
                                "scrollZoom": True,
                            },
                        ),
                        # Drives _update_field_graph: redraws the live overlay
                        # (robot dot, trail, vision-fix marker) on top of the
                        # cached static field map. Static parts of the figure
                        # don't re-render thanks to uirevision='field-map'.
                        # Interval bumped to 1.5 s so the browser has time to
                        # process drag/rotate gestures between redraws —
                        # 500 ms was making the map feel laggy.
                        dcc.Interval(id="map-tick", interval=1500, n_intervals=0),
                    ],
                ),
            ],
        ),
        # ---- Bottom half: live telemetry panel ----
        html.Div(
            style={
                "height": "320px",
                "backgroundColor": "#0f0f0f",
                "color": "white",
                "padding": "12px 16px",
                "boxSizing": "border-box",
                "overflowY": "auto",
            },
            children=[
                html.Div(id="telemetry-panel"),
                dcc.Interval(id="telemetry-tick", interval=200, n_intervals=0),
            ],
        ),
    ],
)


@app.callback(
    Output("info-box", "children"),
    [Input("field-graph", "hoverData"),
     Input("map-tick", "n_intervals")],
)
def update_info(hover_data, _n):
    """Default: robotun mevcut Kalman pozisyonu için context_text.
    Kullanıcı haritada bir noktanın üstüne hover yaparsa o noktayı gösterir.
    Hover bilgisi yoksa map-tick her 500 ms'de robot pozisyonunu yeniden okur,
    yani panel canlı kalır.

    Heading prepended ("POINTER" vs "ROBOT") so the cursor-hover view isn't
    mistaken for live robot pose — they sit in the same panel.
    """
    if hover_data:
        point = hover_data["points"][0]
        if "customdata" in point:
            px, py = point["customdata"]
        else:
            px = point.get("x", 0.0)
            py = point.get("y", 0.0)
        return "[POINTER — hover]\n" + context_text(float(px), float(py))

    with _LIVE_LOCK:
        kx = _LIVE.get("kalman_x")
        ky = _LIVE.get("kalman_y")

    if kx is not None and ky is not None:
        return "[ROBOT — live Kalman]\n" + context_text(float(kx), float(ky))

    return "robot pose not yet received — start mqtt_client_core / drive the robot"


@app.callback(
    Output("telemetry-panel", "children"),
    Input("telemetry-tick", "n_intervals"),
)
def update_telemetry_panel(_):
    return render_telemetry(hub.snapshot())


@app.callback(
    Output("camera-content", "children"),
    Input("camera-mode", "value"),
)
def update_camera_view(mode):
    url = (CLI_ARGS.annotated_stream if mode == "annotated"
           else CLI_ARGS.camera_stream)

    if url:
        return html.Img(
            src=url,
            style={"width": "100%", "height": "100%",
                   "objectFit": "contain", "backgroundColor": "#000"},
        )

    # No URL configured for the selected mode → friendly placeholder.
    flag = "--annotated-stream" if mode == "annotated" else "--camera-stream"
    return html.Div(
        style={"fontFamily": "Menlo, monospace", "fontSize": "12px",
               "color": "#555", "textAlign": "center", "padding": "20px"},
        children=[
            html.Div(f"{mode} feed not configured",
                     style={"marginBottom": "8px"}),
            html.Div(f"start with  {flag} <MJPEG_URL>",
                     style={"color": "#444"}),
        ],
    )


def _start_hub():
    """Try real MQTT first; fall back to mock so the panel is never empty."""
    if CLI_ARGS.mock:
        hub.start_mock()
    elif not hub.start_mqtt(CLI_ARGS.broker, CLI_ARGS.port):
        print("% field_web_app: MQTT unavailable — starting mock mode for the panel.")
        hub.start_mock()


# ---------------------------------------------------------------------------
# Live map overlay: independent MQTT subscriber for the field-graph callback.
# telemetry_hub uses a different topic namespace (august/*); rather than
# refactor that, we add a lightweight side-channel that listens to the topics
# the actual robot stack publishes (robobot/kalman/state, robobot/drive/T0/
# vision_pose). The hub keeps powering the bottom telemetry panel; this
# channel powers the live robot dot + trail + vision-fix marker on the map.
# ---------------------------------------------------------------------------
import json as _json
import threading as _threading
import time
from collections import deque as _deque

# Topics published by skalman.publish_state() and live_perception_overlay
# _send_vision_pose() respectively. Single source of truth lives there.
_LIVE_TOPIC_KALMAN = "robobot/kalman/state"          # JSON
_LIVE_TOPIC_VISION = "robobot/drive/T0/vision_pose"  # space-delimited
# Fallback: Teensy-side encoder odometry. Always running, drift-prone but at
# least gives us a robot dot on the map even when the Kalman master isn't up.
# Format: "TS x y heading tilt"
_LIVE_TOPIC_ENC_POSE = "robobot/drive/T0/pose"       # space-delimited

# Module-level state. Mutated from the MQTT thread, read from the Dash
# callback thread; we hold a lock for any compound read/write.
_LIVE_LOCK = _threading.Lock()
_LIVE: dict = {
    "kalman_x":    None,
    "kalman_y":    None,
    "kalman_yaw":  None,
    "kalman_z":    None,
    "kalman_x_std": None,
    "kalman_y_std": None,
    "kalman_t":    None,
    "vision_x":    None,
    "vision_y":    None,
    "vision_yaw":  None,
    "vision_src":  None,
    "vision_sigma_xy": None,
    "vision_t":    None,
    # Encoder pose (Teensy odometry) — used as fallback when Kalman master
    # isn't running. The encoder reports position relative to its own boot,
    # so we apply START_POSE_OFFSET to land it in field-map coordinates.
    "enc_x":       None,
    "enc_y":       None,
    "enc_yaw":     None,
    "enc_t":       None,
}

# Field-map coordinates of the start area (must match the Kalman reset seed
# used in live_perception_overlay.py). Encoder pose is shifted by these so
# the dot appears in the right field cell.
_START_X = 4.775
_START_Y = 0.235
_START_YAW = 1.5708
_TRAIL_KALMAN: _deque = _deque(maxlen=600)   # ~3 minutes at 300 ms tick

# Built once and reused — the static base figure is identical every tick.
_BASE_FIG = None


def _decode_kalman_payload(raw: str):
    try:
        msg = _json.loads(raw)
    except (ValueError, TypeError):
        return None
    pos = msg.get("position", {})
    ori = msg.get("orientation", {})
    cov = msg.get("covariance_diag", {})
    return {
        "x":     float(pos.get("x", 0.0)),
        "y":     float(pos.get("y", 0.0)),
        "z":     float(pos.get("z", 0.0)),
        "yaw":   float(ori.get("yaw", 0.0)),
        "x_std": float(cov.get("x_std", 0.0)),
        "y_std": float(cov.get("y_std", 0.0)),
    }


def _decode_enc_pose_payload(raw: str):
    """Parse Teensy odometry pose: 'HOST_TS TEENSY_TS x y heading tilt'.

    The Teensy publishes six fields plus a leading host-side timestamp; the
    actual pose values start at index 2 (matches sensors/spose.py:208-227).

    Coordinate transform: identity + start-pose offset only. We don't rotate
    by start-yaw because past field tests showed the encoder/world axis
    convention disagrees in unknown ways — applying the rotation pointed
    the dot to the wrong corner. Using identity at least keeps the dot
    *moving* with the encoder, even if the absolute direction is suspect.
    On-bench calibration is needed to determine the correct rotation.
    """
    parts = raw.strip().split()
    if len(parts) < 5:
        return None
    try:
        ex = float(parts[2])
        ey = float(parts[3])
        eh = float(parts[4])
    except (ValueError, IndexError):
        return None
    return {
        "x":   _START_X + ex,
        "y":   _START_Y + ey,
        "yaw": _START_YAW + eh,
    }


def _decode_vision_payload(raw: str):
    """Parse the live_perception_overlay vision_pose wire format.

    Layout (frozen positionals + KEY=VALUE extras):
        TIMESTAMP X Y Z YAW PITCH SOURCE [sxy=… syaw=… n=… conf=…]
    """
    parts = raw.strip().split()
    if len(parts) < 5:
        return None
    try:
        x = float(parts[1]); y = float(parts[2]); yaw = float(parts[4])
    except (ValueError, IndexError):
        return None
    source = parts[6] if len(parts) >= 7 and "=" not in parts[6] else "unknown"
    sigma_xy = None
    extras_start = 7 if (len(parts) >= 7 and "=" not in parts[6]) else 6
    for tok in parts[extras_start:]:
        if tok.startswith("sxy="):
            try:
                sigma_xy = float(tok.split("=", 1)[1])
            except ValueError:
                pass
    return {"x": x, "y": y, "yaw": yaw, "source": source, "sigma_xy": sigma_xy}


def _start_live_subscriber(broker: str, port: int) -> bool:
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        print("% field_web_app: paho-mqtt missing; live map overlay disabled.")
        return False

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

    def _on_connect(c, userdata, flags, reason_code, properties=None):
        c.subscribe(_LIVE_TOPIC_KALMAN)
        c.subscribe(_LIVE_TOPIC_VISION)
        c.subscribe(_LIVE_TOPIC_ENC_POSE)
        print(f"% field_web_app: live overlay subscribed @ {broker}:{port}")

    def _on_message(c, userdata, msg):
        raw = msg.payload.decode("utf-8", errors="replace")
        now = time.time()
        if msg.topic == _LIVE_TOPIC_KALMAN:
            parsed = _decode_kalman_payload(raw)
            if parsed is None:
                return
            with _LIVE_LOCK:
                _LIVE["kalman_x"]   = parsed["x"]
                _LIVE["kalman_y"]   = parsed["y"]
                _LIVE["kalman_yaw"] = parsed["yaw"]
                _LIVE["kalman_z"]   = parsed["z"]
                _LIVE["kalman_x_std"] = parsed["x_std"]
                _LIVE["kalman_y_std"] = parsed["y_std"]
                _LIVE["kalman_t"]   = now
                _TRAIL_KALMAN.append((parsed["x"], parsed["y"], parsed["z"]))
        elif msg.topic == _LIVE_TOPIC_VISION:
            parsed = _decode_vision_payload(raw)
            if parsed is None:
                return
            with _LIVE_LOCK:
                _LIVE["vision_x"]        = parsed["x"]
                _LIVE["vision_y"]        = parsed["y"]
                _LIVE["vision_yaw"]      = parsed["yaw"]
                _LIVE["vision_src"]      = parsed["source"]
                _LIVE["vision_sigma_xy"] = parsed["sigma_xy"]
                _LIVE["vision_t"]        = now
        elif msg.topic == _LIVE_TOPIC_ENC_POSE:
            parsed = _decode_enc_pose_payload(raw)
            if parsed is None:
                return
            with _LIVE_LOCK:
                _LIVE["enc_x"]   = parsed["x"]
                _LIVE["enc_y"]   = parsed["y"]
                _LIVE["enc_yaw"] = parsed["yaw"]
                _LIVE["enc_t"]   = now
                # Fallback (identity transform): if Kalman master hasn't
                # published in the last 2 s, use encoder pose as the trail
                # source. Direction may be wrong (axis convention not
                # confirmed on this robot) but at least the dot moves and
                # confirms the robot is actually driving.
                kt = _LIVE.get("kalman_t")
                if (kt is None) or (now - kt > 2.0):
                    _LIVE["kalman_x"]   = parsed["x"]
                    _LIVE["kalman_y"]   = parsed["y"]
                    _LIVE["kalman_yaw"] = parsed["yaw"]
                    _LIVE["kalman_z"]   = 0.0
                    _TRAIL_KALMAN.append((parsed["x"], parsed["y"], 0.0))

    client.on_connect = _on_connect
    client.on_message = _on_message
    try:
        client.connect(broker, port, keepalive=30)
    except Exception as exc:
        print(f"% field_web_app: live MQTT connect failed ({exc}); overlay disabled.")
        return False
    client.loop_start()
    return True


def _live_overlay_traces(snapshot_live: dict, trail_pts: list):
    """Return the three Scatter3d traces drawn on top of the base map:
       1) Kalman trail (orange line, last ~3 min).
       2) Robot marker + heading vector (orange diamond, line for facing).
       3) Latest vision fix (blue/green/red dot, color-coded by source).

    Each call rebuilds these traces from scratch — they're tiny.
    """
    traces = []

    # 1) Trail
    if len(trail_pts) >= 2:
        xs = [p[0] for p in trail_pts]
        ys = [p[1] for p in trail_pts]
        zs = [p[2] + 0.02 for p in trail_pts]
        traces.append(go.Scatter3d(
            x=xs, y=ys, z=zs,
            mode="lines",
            line=dict(color="rgba(255,160,60,0.85)", width=4),
            name="Kalman trail",
            hoverinfo="skip",
            showlegend=False,
        ))

    # 2) Current robot pose
    kx, ky = snapshot_live.get("kalman_x"), snapshot_live.get("kalman_y")
    kyaw  = snapshot_live.get("kalman_yaw")
    kz    = snapshot_live.get("kalman_z") or 0.0
    if kx is not None and ky is not None:
        traces.append(go.Scatter3d(
            x=[kx], y=[ky], z=[kz + 0.04],
            mode="markers",
            marker=dict(size=8, color="#ffa040", symbol="diamond",
                        line=dict(color="black", width=1)),
            name="Robot (Kalman)",
            customdata=[[kx, ky]],
            hovertemplate=f"Robot Kalman<br>x=%{{customdata[0]:.3f}}<br>"
                          f"y=%{{customdata[1]:.3f}}<br>"
                          f"yaw={math.degrees(kyaw or 0):.1f}°<extra></extra>",
            showlegend=False,
        ))
        # Heading vector — 30 cm long
        if kyaw is not None:
            hx = kx + 0.30 * math.cos(kyaw)
            hy = ky + 0.30 * math.sin(kyaw)
            traces.append(go.Scatter3d(
                x=[kx, hx], y=[ky, hy], z=[kz + 0.04, kz + 0.04],
                mode="lines",
                line=dict(color="#ffa040", width=6),
                hoverinfo="skip",
                showlegend=False,
            ))

    # 3) Latest vision fix (color-coded by source)
    vx, vy = snapshot_live.get("vision_x"), snapshot_live.get("vision_y")
    vsrc = snapshot_live.get("vision_src") or "unknown"
    vsig = snapshot_live.get("vision_sigma_xy")
    if vx is not None and vy is not None:
        color = {"aruco": "#5fd6ff", "landmark": "#ffd13f",
                 "tape": "#9f9f9f"}.get(vsrc, "#888888")
        sigma_str = f" σ={vsig*1000:.0f} mm" if vsig is not None else ""
        traces.append(go.Scatter3d(
            x=[vx], y=[vy], z=[(kz if kz is not None else 0.0) + 0.06],
            mode="markers",
            marker=dict(size=7, color=color, symbol="circle",
                        line=dict(color="white", width=1)),
            name=f"Vision fix ({vsrc})",
            hovertemplate=f"Vision fix<br>source={vsrc}{sigma_str}<br>"
                          f"x={vx:.3f}<br>y={vy:.3f}<extra></extra>",
            showlegend=False,
        ))
    return traces


def _chase_camera(kx: float, ky: float, yaw: float) -> dict:
    """Build a Plotly scene.camera dict that follows the robot.

    Plotly's 3D camera coordinates are in the scene's normalized cube — the
    static layout's eye=(1.7,-1.8,1.1) gives a feel for the natural scale.
    We normalize world (kx, ky) into that cube, place the eye behind the
    robot along its heading, and lift it well above so a sensible chunk of
    the field around the robot is visible (a tight zoom hides everything
    except the robot's immediate surroundings).
    """
    x_min, x_max = -0.5, FIELD.width_m + 0.5
    y_min, y_max = -0.5, FIELD.height_m + 0.5
    cx = -1.0 + 2.0 * (kx - x_min) / (x_max - x_min)
    cy = -1.0 + 2.0 * (ky - y_min) / (y_max - y_min)
    back = 5.0      # how far behind the robot the eye sits (normalized)
    height = 3.0    # camera height above the ground plane — height/back = 0.6
                    # so the heading stays obvious while the framing is wide
    ex = cx - back * math.cos(yaw)
    ey = cy - back * math.sin(yaw)
    return dict(
        center=dict(x=cx, y=cy, z=0.0),
        eye=dict(x=ex, y=ey, z=height),
        up=dict(x=0, y=0, z=1),
    )


def _best_robot_pose(snap: dict, now_t: float) -> Optional[Tuple[float, float, float]]:
    """Pick the freshest available (x, y, yaw) source for chase camera.

    Priority: Kalman (≤5 s old) → vision pose (≤5 s old) → encoder pose
    (≤5 s old). Without this fallback, a stale Kalman state pins the camera
    to (0, 0) even when vision_pose has the correct location — which is what
    happened in the field test on 2026-04-28.
    """
    fresh = 5.0
    for kx_key, ky_key, kyaw_key, t_key in (
        ("kalman_x", "kalman_y", "kalman_yaw", "kalman_t"),
        ("vision_x", "vision_y", "vision_yaw", "vision_t"),
        ("enc_x",    "enc_y",    "enc_yaw",    "enc_t"),
    ):
        x = snap.get(kx_key); y = snap.get(ky_key)
        yaw = snap.get(kyaw_key); t = snap.get(t_key)
        if x is None or y is None or t is None:
            continue
        if (now_t - t) > fresh:
            continue
        return float(x), float(y), float(yaw if yaw is not None else math.pi / 2)
    return None


@app.callback(
    Output("field-graph", "figure"),
    [Input("map-tick", "n_intervals"),
     Input("follow-robot", "value")],
)
def _update_field_graph(n, follow_value):
    global _BASE_FIG
    if _BASE_FIG is None:
        _BASE_FIG = build_figure()

    with _LIVE_LOCK:
        snap = dict(_LIVE)
        trail = list(_TRAIL_KALMAN)

    # Deep-copy via to_dict() round-trip: this preserves each trace's `type`
    # field (e.g. 'scatter3d'), which `go.Figure(_BASE_FIG)` was dropping —
    # causing Plotly to fall back to 2D Scatter and reject the `z=` argument
    # on every overlay trace.
    import copy
    fig = go.Figure(copy.deepcopy(_BASE_FIG.to_dict()))
    for t in _live_overlay_traces(snap, trail):
        fig.add_trace(t)

    follow_on = bool(follow_value) and "follow" in follow_value
    pose_xyz = _best_robot_pose(snap, time.time())

    if follow_on and pose_xyz is not None:
        kx, ky, kyaw = pose_xyz
        cam = _chase_camera(kx, ky, kyaw)
        # Bumping uirevision per tick lets our camera override take effect;
        # otherwise Plotly preserves the prior user-applied camera state.
        fig.update_layout(
            scene_camera=cam,
            uirevision=f"follow-{n}",
        )
    else:
        fig.update_layout(uirevision="field-map")

    return fig


if __name__ == "__main__":
    _start_hub()
    if not CLI_ARGS.mock:
        _start_live_subscriber(CLI_ARGS.broker, CLI_ARGS.port)
    app.run(debug=False, host=CLI_ARGS.host, port=CLI_ARGS.web_port)