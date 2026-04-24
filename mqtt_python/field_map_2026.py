from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import math
import numpy as np
from field_objects import (
    Point3D, ArucoMarker, Gate, TapeLine,
    StartArea, Roundabout, InfinityPath,
    Ramp, Stairs, Seesaw, Platform,
    SortingCenter, TimeExtButton,
    LuggageShuttle, BallDispenser, GolfBall, Luggage,
    BezierSegment, ArcSegment, NavPath,
)

RR = 0.60
RC = (3.5, 2 + RR)   

ramp_base = (0.27, 3.30)
ramp_top  = (0.27, 3.30 + 1.58)   # (0.27, 4.88)
ramp_w    = 0.55
ramp_rise = 0.55

plat_x0 = ramp_top[0] - ramp_w/2   # -0.005
plat_x1 = plat_x0 + 1.50            # 1.495
plat_y0 = ramp_top[1]               # 4.88
plat_y1 = plat_y0 + 1.00            # 5.88
plat_z  = ramp_rise                  # 0.55

stair_x0 = plat_x0 + ramp_w         # 0.545
stair_x1 = stair_x0 + 0.75          # 1.295
stair_mid_x = (stair_x0 + stair_x1) / 2

long_ramp_x0 = plat_x1
long_ramp_x1 = plat_x0 + 1.50 + 3.64
long_ramp_y0 = plat_y0
long_ramp_y1 = plat_y0 + 1.00
lr_mid_y = (long_ramp_y0 + long_ramp_y1) / 2

seesaw_x  = long_ramp_x0 + 0.40
seesaw_z  = plat_z - (0.40 / 3.64) * 0.55
seesaw_y1 = long_ramp_y0
seesaw_y0 = seesaw_y1 - 1.80

start_x0 = 4.55; start_x1 = 5.0; start_h = 0.47
goal_cx  = 6.5;  goal_s   = 0.18

inf_cx = (4.5 + 6.6) / 2; inf_cy = 3.0
inf_rx = (6.6 - 4.5) / 4; inf_ry = inf_rx * 0.45

# ---------------------------------------------------------------------------
# FIELD MAP
# ---------------------------------------------------------------------------

@dataclass
class CompetitionField2026:
    name: str = "DTU Robocup 2026"
    width_m:  float = 7.0
    height_m: float = 6.0

    GREEN_BARRIER_Z: float = field(default=0.09)
    green_y_line:  Tuple = (Point3D(0,0,0), Point3D(0,2.5,0))
    shuttle_green_border: Tuple = (Point3D(0,0.10,0), Point3D(3.5,0.10,0))  

    # --- Start & Goal ---
    start_area: StartArea = None
    goal_point: Point3D   = None
    goal_aruco: ArucoMarker = None

    # --- Roundabout ---
    roundabout: Roundabout = None

    # --- Short ramp ---
    short_ramp: Ramp    = None
    platform:   Platform = None
    stairs:     Stairs  = None
    seesaw:     Seesaw  = None

    # --- Long ramp ---
    long_ramp: Ramp = None

    sorting_center: SortingCenter  = None
    shuttle:        LuggageShuttle = None
    ball_dispenser: BallDispenser  = None
    infinity_path:  InfinityPath   = None

    # --- List ---
    golf_balls:   List[GolfBall]      = field(default_factory=list)
    time_buttons: List[TimeExtButton] = field(default_factory=list)
    gates:        List[Gate]           = field(default_factory=list)
    tape_lines:   List[TapeLine]       = field(default_factory=list)
    aruco_markers: List[ArucoMarker]   = field(default_factory=list)
    nav_paths:     List               = field(default_factory=list)  # NavPath list

    def all_gates(self):
        gates = list(self.gates)
        for obj in [self.roundabout, self.infinity_path, self.stairs, self.long_ramp]:
            if obj and hasattr(obj, 'gates'):
                gates.extend(obj.gates or [])
        if self.seesaw and self.seesaw.gate:
            gates.append(self.seesaw.gate)
        return gates

    def all_aruco(self):
        markers = list(self.aruco_markers)
        if self.sorting_center:
            markers.extend(self.sorting_center.aruco_markers)
        if self.goal_aruco:
            markers.append(self.goal_aruco)
        return markers

    def pz_at(self, x: float, y: float) -> float:
        if self.platform and self.platform.contains_2d(x, y):
            return self.platform.pz
        if self.long_ramp:
            pz = self.long_ramp.pz_at(x, y)
            if pz > 0.01: return pz
        if self.short_ramp:
            pz = self.short_ramp.pz_at(x, y)
            if pz > 0.01: return pz
        if self.stairs:
            pz = self.stairs.pz_at(x, y)
            if pz > 0.01: return pz
        return 0.0

    def nearest_tape_segment(self, px: float, py: float, yaw: float = None,
                              world_line_points: list = None,
                              max_yaw_diff_deg: float = 60.0):
        all_segments = []
        for tl in self.tape_lines:
            for i in range(len(tl.waypoints) - 1):
                all_segments.append((tl.name, tl.waypoints[i], tl.waypoints[i+1]))
        for np_ in self.nav_paths:
            for seg in np_.segments:
                if hasattr(seg, 'waypoints') and len(seg.waypoints) >= 2:
                    for i in range(len(seg.waypoints) - 1):
                        all_segments.append((
                            np_.name + '/' + (seg.name if hasattr(seg, 'name') else 'seg'),
                            seg.waypoints[i], seg.waypoints[i+1],
                        ))

        best      = None
        best_score = float('inf')

        for name, p0, p1 in all_segments:
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            seg_len = math.hypot(dx, dy)
            if seg_len < 1e-6:
                continue

            t = max(0.0, min(1.0, ((px - p0.x)*dx + (py - p0.y)*dy) / (seg_len**2)))
            cx = p0.x + t * dx
            cy = p0.y + t * dy
            dist = math.hypot(px - cx, py - cy)

            seg_heading_rad = math.atan2(dy, dx)
            seg_heading_deg = math.degrees(seg_heading_rad)

            lateral = ((px - p0.x)*dy - (py - p0.y)*dx) / seg_len

            heading_err_deg = 0.0
            if yaw is not None:
                raw = math.degrees(yaw) - seg_heading_deg
                heading_err_deg = (raw + 180) % 360 - 180  # [-180, 180]

                yaw_diff = abs(heading_err_deg)
                if yaw_diff > 90:
                    yaw_diff = 180 - yaw_diff

                if yaw_diff > max_yaw_diff_deg:
                    continue

                yaw_score = yaw_diff / max_yaw_diff_deg  # [0, 1]
            else:
                yaw_score = 0.0

            point_score = 0.0
            if world_line_points and len(world_line_points) >= 2:
                total_dist = 0.0
                for wx, wy in world_line_points:
                    tp = max(0.0, min(1.0,
                             ((wx - p0.x)*dx + (wy - p0.y)*dy) / (seg_len**2)))
                    cpx = p0.x + tp * dx
                    cpy = p0.y + tp * dy
                    total_dist += math.hypot(wx - cpx, wy - cpy)
                avg_dist = total_dist / len(world_line_points)
                point_score = min(1.0, avg_dist / 0.5)

            w_dist   = 1.0
            w_yaw    = 0.4
            w_points = 0.6 if world_line_points else 0.0

            score = (w_dist * dist
                     + w_yaw * yaw_score
                     + w_points * point_score)

            if score < best_score:
                best_score = score
                best = {
                    'name':              name,
                    'segment_start':     p0,
                    'segment_end':       p1,
                    'closest_pt':        (cx, cy),
                    'dist':              dist,
                    'heading_deg':       seg_heading_deg,
                    'lateral_error':     lateral,
                    'heading_error_deg': heading_err_deg,
                    'score':             score,
                    'p0': (p0.x, p0.y),
                    'p1': (p1.x, p1.y),
                }

        return best

    def nearby_aruco(self, px: float, py: float, radius: float = 1.5):
        result = []
        for m in self.all_aruco():
            d = math.hypot(m.position.x - px, m.position.y - py)
            if d <= radius:
                result.append((d, m))
        result.sort(key=lambda x: x[0])
        return [(d, m) for d, m in result]

    def nearest_gate(self, px: float, py: float):
        best_gate = None
        best_dist = float('inf')
        for g in self.all_gates():
            d = math.hypot(g.center.x - px, g.center.y - py)
            if d < best_dist:
                best_dist = d
                best_gate = g
        return best_gate, best_dist

    def current_zone(self, px: float, py: float) -> str:
        pz = self.pz_at(px, py)
        if self.platform and self.platform.contains_2d(px, py):
            return "platform"
        lr = self.long_ramp
        if lr:
            lx0, lx1 = min(lr.base.x, lr.top.x), max(lr.base.x, lr.top.x)
            ly0, ly1 = lr.top.y - lr.width/2, lr.top.y + lr.width/2
            if lx0 <= px <= lx1 and ly0 <= py <= ly1:
                return "long_ramp"
        sr = self.short_ramp
        if sr:
            if abs(px - sr.base.x) < sr.width/2 and sr.base.y <= py <= sr.top.y:
                return "short_ramp"
        s = self.stairs
        if s:
            from field_map_2026 import stair_x0, stair_x1
            if stair_x0 <= px <= stair_x1 and s.base.y <= py <= s.top.y:
                return "stairs"
        if self.sorting_center:
            sc = self.sorting_center
            d = sc.zone_size * 0.60 * math.sqrt(2)
            if math.hypot(px - sc.center.x, py - sc.center.y) < d:
                return "sorting_center"
        r = self.roundabout
        if r and math.hypot(px - r.center.x, py - r.center.y) < r.radius:
            return "roundabout"
        return "floor"

    def context(self, px: float, py: float, yaw: float = None,
                world_line_points: list = None,
                max_yaw_diff_deg: float = 60.0) -> dict:
        tape  = self.nearest_tape_segment(px, py, yaw,
                                          world_line_points=world_line_points,
                                          max_yaw_diff_deg=max_yaw_diff_deg)
        gate, gate_dist = self.nearest_gate(px, py)
        aruco = self.nearby_aruco(px, py, radius=1.5)
        zone  = self.current_zone(px, py)
        pz    = self.pz_at(px, py)
        return {
            'px': px, 'py': py, 'pz': pz, 'yaw': yaw,
            'zone': zone,
            'nearest_tape': tape,
            'nearest_gate': {'name': gate.name if gate else None,
                              'dist': gate_dist,
                              'has_satellite': gate.has_satellite if gate else False},
            'nearby_aruco': [{'id': m.id, 'dist': d,
                               'pos': m.position} for d, m in aruco],
        }

    def summary(self):
        print(f"\n=== {self.name} ===")
        print(f"Field: {self.width_m}m x {self.height_m}m")
        print(f"Start exit : {self.start_area.exit_point}")
        print(f"Goal       : {self.goal_point}")
        gates = self.all_gates()
        print(f"\nGates ({len(gates)}):")
        for g in gates: print(f"  {g}")
        aruco = self.all_aruco()
        print(f"\nArUco markers ({len(aruco)}):")
        for m in aruco: print(f"  {m}")
        print(f"\nPz test:")
        for x,y,lbl in [
            (6.5, 0.1, "goal"),
            (long_ramp_x0+1.0, lr_mid_y, "long ramp middle"),
            (plat_x0+0.5, plat_y0+0.5, "platform"),
            (stair_mid_x, plat_y0-0.8, "stairs middle"),
        ]:
            print(f"  ({x:.2f},{y:.2f}) [{lbl}] -> Pz={self.pz_at(x,y):.3f}m")


# ---------------------------------------------------------------------------
# OBJECTS
# ---------------------------------------------------------------------------
# Start
START_AREA = StartArea(
    origin=Point3D(start_x0, 0.0),
    width=start_x1-start_x0, depth=start_h,
    open_side="top",
    note="U shape, closed at the bottom, open at the top",
)

# Goal
GOAL_POINT = Point3D(goal_cx, goal_s/2, 0.0)
GOAL_ARUCO = ArucoMarker(
    id=25, position=Point3D(goal_cx, goal_s+0.05, 0.02),
    size_m=0.10, note="ArUco on top of Goal",
)

# Roundabout
ROUNDABOUT = Roundabout(
    center=Point3D(RC[0], RC[1]),
    diameter=RR*2,
    gate_angles_deg=[0.0, 120.0, 240.0],
    n_drones=2,
)
ROUNDABOUT.build_gates(has_satellite=True)
ROUNDABOUT.gates[0].has_satellite = False  # 0° right — no satellite
ROUNDABOUT.gates[1].has_satellite = True   # 120° upper left — has satellite
ROUNDABOUT.gates[2].has_satellite = True   # 240° lower left — has satellite

# Short ramp (adjacent to stairs)
SHORT_RAMP = Ramp(
    name="short_ramp",
    base=Point3D(ramp_base[0], ramp_base[1], 0.0),
    top=Point3D(ramp_top[0],  ramp_top[1],  ramp_rise),
    width=ramp_w, rise=ramp_rise,
    note="Short ramp adjacent to stairs, along y axis, 1.58m",
)

# Platform
PLATFORM = Platform(
    name="platform",
    origin=Point3D(plat_x0, plat_y0, plat_z),
    width=plat_x1-plat_x0, depth=plat_y1-plat_y0, pz=plat_z,
    hole=Point3D(plat_x0+0.10, plat_y1-0.10, plat_z),
    golf_ball=Point3D(plat_x1-0.10, plat_y0+0.10, plat_z),
    connections=["short_ramp", "stairs", "long_ramp"],
)

# Stairs
STAIRS = Stairs(
    name="stairs",
    base=Point3D(stair_x0, plat_y0 - 4*0.40, 0.0),
    top=Point3D(stair_x0, plat_y0, plat_z),
    width=stair_x1-stair_x0,
    n_steps=4, step_height=0.11, step_depth=0.40,
)
STAIRS.build_gates()

# Long ramp
LONG_RAMP = Ramp(
    name="long_ramp",
    base=Point3D(long_ramp_x1, lr_mid_y, 0.0),
    top=Point3D(long_ramp_x0,  lr_mid_y, plat_z),
    width=1.00, rise=plat_z,
    note="Long ramp, along x axis, 3.64m, from the right side of platform",
)
LONG_RAMP.gates = [
    Gate("long_ramp_gate", Point3D(long_ramp_x0+0.70, lr_mid_y, plat_z-(0.70/3.64)*plat_z),
         orientation_deg=180, line_angle_deg=90, has_satellite=True, points=1),
]

# Seesaw
SEESAW = Seesaw(
    name="seesaw",
    pivot=Point3D(seesaw_x, (seesaw_y0+seesaw_y1)/2, seesaw_z),
    length=1.80, width=0.30,
)
SEESAW.build_gate()
SEESAW.golf_ball_pos = Point3D(seesaw_x, seesaw_y0+1.80*0.25, seesaw_z+0.03)

# Sorting center ABCD
SORTING = SortingCenter(
    center=Point3D(1.8, 0.9+0.30),
    zone_size=0.30, pz=0.0,
)
SORTING.build_zones()
SORTING.build_aruco(marker_size=0.10)

# Ball dispenser
BALL_DISPENSER = BallDispenser(
    position=Point3D(1.0, 2.0, 0.0),
    trigger_height_m=0.18,
)
BALL_DISPENSER.build_balls()

# Luggage shuttle
SHUTTLE = LuggageShuttle(
    path_start=Point3D(0.0, 0.10),
    path_end=Point3D(3.5, 0.10),
    speed_cms=20.0, aruco_id=5,
)
SHUTTLE.build_luggage()

# Infinity path (guard)
INFINITY = InfinityPath(
    center=Point3D(inf_cx, inf_cy),
    loop_radius=inf_rx,
)
INFINITY.build_gates()
INFINITY.gates[0].has_satellite = True
INFINITY.gates[0].center = Point3D(inf_cx - inf_rx*2, inf_cy)
INFINITY.gates[0].orientation_deg = 0
INFINITY.gates[0].line_angle_deg = 0    # along x axis
INFINITY.gates[1].has_satellite = False
INFINITY.gates[1].center = Point3D(inf_cx, inf_cy)
INFINITY.gates[1].orientation_deg = 0
INFINITY.gates[1].line_angle_deg = 90   # along y axis

# Golf balls
GOLF_BALLS = [
    GolfBall(position=Point3D(seesaw_x, seesaw_y0+1.80*0.25, seesaw_z+0.03),
             target_hole=PLATFORM.hole, points=2, note="Seesaw ball"),
    GolfBall(position=Point3D(plat_x1-0.10, plat_y0+0.10, plat_z),
             target_hole=PLATFORM.hole, points=2, note="Platform ball"),
]

# +90sec buttons
TIME_BUTTONS = [
    TimeExtButton("time_ext_10", Point3D(0.0, 1.0), extra_sec=90, note="Left button"),
    TimeExtButton("time_ext_11", Point3D(7.0, 3.75), extra_sec=90, note="Right button"),
]

# Extra gates
EXTRA_GATES = [
    Gate("gate_ramp_area_left",  Point3D(4.5, 4.2), orientation_deg=0, line_angle_deg=90, has_satellite=True,  points=1),
    Gate("gate_ramp_area_right", Point3D(5.4, 4.2), orientation_deg=0, line_angle_deg=90, has_satellite=True,  points=1),
    Gate("gate_start_loop",      Point3D(5.5, 2.0), orientation_deg=0, line_angle_deg=90, has_satellite=True,  points=1, note="Start-Goal loop gate"),
    Gate("gate_short_ramp_top",  Point3D(ramp_base[0], ramp_top[1], ramp_rise), orientation_deg=90, line_angle_deg=0, has_satellite=True, points=1, note="Short ramp top start"),
]

# Tape segments (tape lines)
TAPE_LINES = [
    TapeLine("roundabout_left_exit",
             [Point3D(RC[0]-RR, RC[1]), Point3D(RC[0]-RR-2.6, RC[1])],
             task="roundabout",
             connects=["roundabout", "short_ramp_approach"]),
    TapeLine("roundabout_to_start_vertical",
             [Point3D(3.5, 0.0), Point3D(3.5, 2.0)],
             task="navigation",
             connects=["start", "roundabout"]),
    TapeLine("small_line",
             [Point3D(3.0, 1.3), Point3D(3.5, 1.3)],
             task="navigation",
             connects=["roundabout_to_start_vertical", "sorting_center"]),
    TapeLine("shuttle_path",
             [Point3D(0.0, 0.10), Point3D(3.5, 0.10)],
             task="luggage_shuttle",
             connects=["floor_left", "roundabout_bottom"]),
    TapeLine("short_ramp_approach",
             [Point3D(0.27, 2.60), Point3D(0.27, 3.30)],
             task="ramp_up",
             connects=["roundabout_left_exit", "short_ramp"]),
    TapeLine("short_ramp_surface",
             [Point3D(ramp_base[0], ramp_base[1], 0.0),
              Point3D(ramp_top[0],  ramp_top[1],  ramp_rise)],
             task="ramp_up",
             connects=["short_ramp_approach", "platform"]),
]

# ---------------------------------------------------------------------------
# NAV PATHS — curved line definitions
# ---------------------------------------------------------------------------

# After long ramp: straight → U turn → to the top of roundabout
_rx1        = long_ramp_x1
_ry_mid     = lr_mid_y
_str1_end_x = _rx1 + 0.39
_u_r        = 0.60
_u_exit_y   = _ry_mid - 2 * _u_r
_round_top_y = RC[1] + RR
_arc_r2      = _u_exit_y - _round_top_y
_p_str_end_x = RC[0] + _arc_r2

PATH_LONG_RAMP_TO_ROUNDABOUT = NavPath(
    name="long_ramp_to_roundabout",
    task="ramp_down",
    connects=["long_ramp", "roundabout"],
    segments=[
        TapeLine("lr_straight1",
                 [Point3D(_rx1, _ry_mid), Point3D(_str1_end_x, _ry_mid)]),
        ArcSegment(center=Point3D(_str1_end_x, _ry_mid - _u_r),
                   radius=_u_r,
                   angle_start_deg=90, angle_end_deg=-90, n_samples=60),
        TapeLine("lr_straight2",
                 [Point3D(_str1_end_x, _u_exit_y), Point3D(_p_str_end_x, _u_exit_y)]),
        ArcSegment(center=Point3D(_p_str_end_x, _round_top_y),
                   radius=abs(_arc_r2),
                   angle_start_deg=90, angle_end_deg=180, n_samples=30),
        TapeLine("roundabout_down",
                 [Point3D(RC[0], RC[1]-RR), Point3D(RC[0], 0.50)]),
    ],
    note="Line descending from the end of long ramp to the roundabout",
)

# Start-Goal loop: inverted U from above goal to start
_goal_top_y  = goal_s
_goal_top_x  = goal_cx
_start_mid_x = (start_x0 + start_x1) / 2
_start_mid_y = start_h / 2
_straight_len = 1.40
_arc_bomb     = 0.40

_p1 = Point3D(_goal_top_x,  _goal_top_y  + _straight_len)
_p2 = Point3D(_start_mid_x, _start_mid_y + _straight_len)
_ctrl = Point3D((_p1.x + _p2.x) / 2, max(_p1.y, _p2.y) + _arc_bomb)

PATH_START_GOAL_LOOP = NavPath(
    name="start_goal_loop",
    task="goal",
    connects=["goal", "start"],
    segments=[
        TapeLine("goal_straight",
                 [Point3D(_goal_top_x, _goal_top_y), _p1]),
        BezierSegment(p0=_p1, p3=_p2,
                      p1=_ctrl, p2=_ctrl, n_samples=60,
                      note="Inverted U bulge"),
        TapeLine("start_straight",
                 [_p2, Point3D(_start_mid_x, _start_mid_y)]),
    ],
    note="Inverted U loop from above goal to start, bulging gate (5.5, 2.0)",
)

# Line curving left from top of start → (3.92, 2.17)
_s_top   = Point3D(_start_mid_x, _start_mid_y + _straight_len)
_s_target = Point3D(3.92, 2.17)

PATH_START_LEFT_BRANCH = NavPath(
    name="start_left_branch",
    task="navigation",
    connects=["start", "roundabout"],
    segments=[
        BezierSegment(
            p0=_s_top, p3=_s_target,
            p1=Point3D(_s_top.x,    _s_target.y),
            p2=Point3D(_s_target.x, _s_target.y),
            n_samples=50,
            note="Curve from top of start to the left edge of roundabout",
        ),
    ],
    note="Line branching off to the left from the start straight line",
)

# Stairs tape line: ball dispenser → stairs bottom → stairs top (3D)
_stair_mid_x  = stair_mid_x
_stair_bot_y  = plat_y0 - 4 * 0.40   # n_steps=4, step_d=0.40
_stair_top_y  = plat_y0
_stair_top_z  = 4 * 0.11              # n_steps * step_h

PATH_BALL_TO_STAIRS = NavPath(
    name="ball_to_stairs",
    task="golf_balls",
    connects=["ball_dispenser", "stairs", "platform"],
    segments=[
        TapeLine("bowl_to_stair_base",
                 [Point3D(1.0, 2.0), Point3D(_stair_mid_x, _stair_bot_y)]),
        TapeLine("stair_climb",
                 [Point3D(_stair_mid_x, _stair_bot_y, 0.0),
                  Point3D(_stair_mid_x, _stair_top_y, _stair_top_z)]),
    ],
    note="Line going from the ball bowl up along the stairs",
)

# ---------------------------------------------------------------------------
# ADD nav_paths TO THE FIELD OBJECT
# ---------------------------------------------------------------------------
FIELD = CompetitionField2026(
    start_area     = START_AREA,
    goal_point     = GOAL_POINT,
    goal_aruco     = GOAL_ARUCO,
    roundabout     = ROUNDABOUT,
    short_ramp     = SHORT_RAMP,
    platform       = PLATFORM,
    stairs         = STAIRS,
    long_ramp      = LONG_RAMP,
    seesaw         = SEESAW,
    sorting_center = SORTING,
    shuttle        = SHUTTLE,
    ball_dispenser = BALL_DISPENSER,
    infinity_path  = INFINITY,
    golf_balls     = GOLF_BALLS,
    time_buttons   = TIME_BUTTONS,
    gates          = EXTRA_GATES,
    tape_lines     = TAPE_LINES,
    aruco_markers  = [],  # goal_aruco is already a separate field, not added here
    nav_paths      = [
        PATH_LONG_RAMP_TO_ROUNDABOUT,
        PATH_START_GOAL_LOOP,
        PATH_START_LEFT_BRANCH,
        PATH_BALL_TO_STAIRS,
    ],
)

if __name__ == "__main__":
    FIELD.summary()