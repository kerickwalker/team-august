from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import math

from field_map.field_objects import (
    Point3D, ArucoMarker, MobileArucoSpec, Gate, TapeLine,
    StartArea, Roundabout, InfinityPath,
    Ramp, Stairs, Seesaw, Platform,
    SortingCenter, TimeExtButton,
    LuggageShuttle, BallDispenser, GolfBall,
    BezierSegment, ArcSegment, NavPath,
    Landmark,
    MARKER_ROLE_LOCALIZATION,
    MARKER_ROLE_LUGGAGE,
    MARKER_ROLE_SHUTTLE,
)


@dataclass(frozen=True)
class FieldSpec:
    """Field dimensions and layout constants used across the map."""
    width_m: float = 6.20    # field width, was 7.0
    height_m: float = 6.67   # field height, was 6.0

    roundabout_center: Tuple[float, float] = (2.90, 2.60)   # was (3.5, 2.6):
                                                            # west edge at x=2.30, south edge at y=2.00,
                                                            # radius 0.60, so centre is (2.90, 2.60).
    roundabout_radius: float = 0.60
    roundabout_plate_z: float = 0.01  # raised plate thickness, 1 cm
    roundabout_gate_angles_deg: Tuple[float, ...] = (0.0, 120.0, 240.0)

    short_ramp_base: Tuple[float, float] = (0.31, 3.59)   # x was 0.27; y keeps the platform flush with the north wall:
                                                          # 3.59 + short_ramp_length(2.08) + platform_depth(1.00) = 6.67.
    short_ramp_length: float = 2.08    # horizontal run, was 1.58
    short_ramp_width: float = 0.60     # was 0.55
    short_ramp_rise: float = 0.55      # same height as the platform
    # Check: hypotenuse = sqrt(2.08² + 0.55²) = 2.15 m.

    platform_width: float = 1.50    # platform long edge
    platform_depth: float = 1.00    # platform short edge

    stair_run_width: float = 0.60   # X-axis tread width, was 0.75
    stair_n_steps: int = 4     # 4 visible step boxes; the last top surface
                               # is flush with the platform (4 × 13.75 cm = 55 cm).
    stair_depth: float = 0.40       # per-step Y-axis run

    long_ramp_length: float = 3.64
    long_ramp_width: float = 0.60   # narrower than platform_depth, was 1.00

    # U-turn after the long ramp is a 180° elliptic arc.
    # Bounding box: 600 mm wide × 1400 mm tall.
    # Since the U opens to one side, horizontal extent = rx and vertical extent = 2·ry.
    lr_uturn_rx: float = 0.60
    lr_uturn_ry: float = 0.70
    lr_uturn_extra_straight: float = 0.15   # straight section after long_ramp_x1 before the U-turn arc.
                                            # Keeps the U-turn east edge 30 cm clear of the field wall.
    lr_north_approach_length: float = 1.30  # straight tape from the north side of the roundabout.
                                            # The second arc bends east after this section.

    seesaw_length: float = 2.00             # was 1.80
    seesaw_width: float = 0.44              # was 0.30
    seesaw_offset_on_ramp: float = 0.55     # X-distance along the long ramp, was 0.40
    seesaw_y_drop_from_ramp: float = 0.30   # short connector tape going in -Y from the ramp centerline
                                            # to the seesaw's north end
    seesaw_z_height: float = 0.43           # pivot height above the floor, independent of ramp slope

    start_x0: float = 3.60   # start area west edge, was 4.55
    start_x1: float = 4.05   # start area east edge, was 5.00; width = 45 cm
    start_depth: float = 0.53  # start area depth, was 0.47

    goal_cx: float = 4.985   # goal centre x; was 6.5
    goal_size: float = 0.18

    infinity_x_range: Tuple[float, float] = (3.90, 6.00)   # was (4.5, 6.6):
                                                           # 40 cm gap east of the roundabout edge,
                                                           # with total length kept at 2.10 m.
    infinity_cy: float = 3.0
    infinity_y_scale: float = 0.45

    # Sorting center: 60×60 cm plate split by a "+" divider into four zones.
    # The square is rotated 45°, so it appears as a diamond in the field frame.
    sorting_center_xy: Tuple[float, float] = (1.5843, 1.3243)  # bounding box SW corner = (1.16, 0.90);
                                                               # diamond side = 60 cm, so bbox is 84.85×84.85 cm;
                                                               # centre = SW + bbox_half = (1.5843, 1.3243).
    sorting_square_side: float = 0.60                          # diamond side length, not bbox width

    green_barrier_z: float = 0.10        # green wall height
    green_wall_thickness: float = 0.04   # Y-axis thickness of the luggage-zone green wall
    time_button_trigger_z: float = 0.12  # buttons sit on top of the green wall, trigger around 10-14 cm
    green_y_line: Tuple[Tuple[float, float], Tuple[float, float]] = ((0.0, 0.0), (0.0, 1.95))   # green wall along the west edge
    shuttle_border: Tuple[Tuple[float, float], Tuple[float, float]] = ((0.0, 0.20), (3.0, 0.20))   # shuttle wall line:
                                                                                                  # 3 m along X at Y=0.20;
                                                                                                  # thickness comes from green_wall_thickness.


SPEC = FieldSpec()


@dataclass(frozen=True)
class _Geometry:
    ramp_top: Tuple[float, float]
    plat_x0: float
    plat_x1: float
    plat_y0: float
    plat_y1: float
    plat_z: float
    stair_x0: float
    stair_x1: float
    stair_mid_x: float
    long_ramp_x0: float
    long_ramp_x1: float
    lr_mid_y: float
    seesaw_x: float
    seesaw_z: float
    ramp_z_at_seesaw: float   # long ramp surface height at seesaw_x, used for the connector tape
    seesaw_y0: float
    seesaw_y1: float
    seesaw_y_mid: float
    inf_cx: float
    inf_rx: float
    inf_ry: float


def _derive_geometry(spec: FieldSpec) -> _Geometry:
    ramp_top = (spec.short_ramp_base[0], spec.short_ramp_base[1] + spec.short_ramp_length)

    plat_x0 = ramp_top[0] - spec.short_ramp_width / 2
    plat_x1 = plat_x0 + spec.platform_width
    plat_y0 = ramp_top[1]
    plat_y1 = plat_y0 + spec.platform_depth
    plat_z = spec.short_ramp_rise

    stair_x0 = plat_x0 + spec.short_ramp_width
    stair_x1 = stair_x0 + spec.stair_run_width
    stair_mid_x = (stair_x0 + stair_x1) / 2

    long_ramp_x0 = plat_x1
    long_ramp_x1 = plat_x0 + spec.platform_width + spec.long_ramp_length
    # Long ramp aligns with the platform's north edge and covers only
    # `long_ramp_width` metres south of it. The rest of the platform edge stays open.
    lr_mid_y = plat_y1 - spec.long_ramp_width / 2

    # Seesaw pivot height is set directly, not derived from the long-ramp slope.
    # The ramp height at the connector start is still kept separately.
    seesaw_progress = spec.seesaw_offset_on_ramp / spec.long_ramp_length
    ramp_z_at_seesaw = plat_z * (1.0 - seesaw_progress)
    seesaw_z = spec.seesaw_z_height
    seesaw_x = long_ramp_x0 + spec.seesaw_offset_on_ramp
    # The seesaw's north end sits south of the long-ramp centerline by
    # `seesaw_y_drop_from_ramp`.
    seesaw_y1 = lr_mid_y - spec.seesaw_y_drop_from_ramp
    seesaw_y0 = seesaw_y1 - spec.seesaw_length
    seesaw_y_mid = (seesaw_y0 + seesaw_y1) / 2

    inf_cx = (spec.infinity_x_range[0] + spec.infinity_x_range[1]) / 2
    inf_rx = (spec.infinity_x_range[1] - spec.infinity_x_range[0]) / 4
    inf_ry = inf_rx * spec.infinity_y_scale

    return _Geometry(
        ramp_top=ramp_top,
        plat_x0=plat_x0, plat_x1=plat_x1, plat_y0=plat_y0, plat_y1=plat_y1, plat_z=plat_z,
        stair_x0=stair_x0, stair_x1=stair_x1, stair_mid_x=stair_mid_x,
        long_ramp_x0=long_ramp_x0, long_ramp_x1=long_ramp_x1, lr_mid_y=lr_mid_y,
        seesaw_x=seesaw_x, seesaw_z=seesaw_z, ramp_z_at_seesaw=ramp_z_at_seesaw,
        seesaw_y0=seesaw_y0, seesaw_y1=seesaw_y1, seesaw_y_mid=seesaw_y_mid,
        inf_cx=inf_cx, inf_rx=inf_rx, inf_ry=inf_ry,
    )


@dataclass
class CompetitionField2026:
    name: str = "DTU Robocup 2026"
    width_m: float = 6.20    # field width
    height_m: float = 6.67   # field height

    green_barrier_z: float = 0.10
    green_y_line: Tuple[Point3D, Point3D] = (Point3D(0, 0, 0), Point3D(0, 2.5, 0))
    shuttle_green_border: Tuple[Point3D, Point3D] = (Point3D(0, 0.10, 0), Point3D(3.5, 0.10, 0))

    start_area: Optional[StartArea] = None
    goal_point: Optional[Point3D] = None
    goal_aruco: Optional[ArucoMarker] = None

    roundabout: Optional[Roundabout] = None
    short_ramp: Optional[Ramp] = None
    platform: Optional[Platform] = None
    stairs: Optional[Stairs] = None
    seesaw: Optional[Seesaw] = None
    long_ramp: Optional[Ramp] = None

    sorting_center: Optional[SortingCenter] = None
    shuttle: Optional[LuggageShuttle] = None
    ball_dispenser: Optional[BallDispenser] = None
    infinity_path: Optional[InfinityPath] = None

    golf_balls: List[GolfBall] = field(default_factory=list)
    time_buttons: List[TimeExtButton] = field(default_factory=list)
    gates: List[Gate] = field(default_factory=list)
    tape_lines: List[TapeLine] = field(default_factory=list)
    aruco_markers: List[ArucoMarker] = field(default_factory=list)
    mobile_arucos: List[MobileArucoSpec] = field(default_factory=list)
    nav_paths: List[NavPath] = field(default_factory=list)
    landmarks: List[Landmark] = field(default_factory=list)

    def all_gates(self) -> List[Gate]:
        gates = list(self.gates)
        for obj in (self.roundabout, self.infinity_path, self.stairs, self.long_ramp):
            if obj and getattr(obj, "gates", None):
                gates.extend(obj.gates)
        if self.seesaw and self.seesaw.gate:
            gates.append(self.seesaw.gate)
        return gates

    def all_aruco(self) -> List[ArucoMarker]:
        markers = list(self.aruco_markers)
        if self.sorting_center:
            markers.extend(self.sorting_center.aruco_markers)
        if self.goal_aruco:
            markers.append(self.goal_aruco)
        return markers

    def all_mobile_arucos(self) -> List[MobileArucoSpec]:
        """Specs for markers whose world position is not fixed, such as luggage and shuttle markers.

        The detector classifies these markers by role and converts marker pose
        to the attached object's geometric centre. They are deliberately not
        returned by `all_aruco()` so they cannot be used for localization fixes.
        """
        return list(self.mobile_arucos)

    def pz_at(self, x: float, y: float) -> float:
        if self.platform and self.platform.contains_2d(x, y):
            return self.platform.pz
        for ramp in (self.long_ramp, self.short_ramp):
            if ramp:
                pz = ramp.pz_at(x, y)
                if pz > 0.01:
                    return pz
        if self.stairs:
            pz = self.stairs.pz_at(x, y)
            if pz > 0.01:
                return pz
        return 0.0

    def current_zone(self, px: float, py: float) -> str:
        if self.platform and self.platform.contains_2d(px, py):
            return "platform"
        if self.long_ramp and self.long_ramp.contains_2d(px, py):
            return "long_ramp"
        if self.short_ramp and self.short_ramp.contains_2d(px, py):
            return "short_ramp"
        if self.stairs and self.stairs.contains_2d(px, py):
            return "stairs"

        if self.sorting_center:
            sc = self.sorting_center
            if math.hypot(px - sc.center.x, py - sc.center.y) < sc.zone_size * 0.60 * math.sqrt(2):
                return "sorting_center"

        if self.roundabout:
            r = self.roundabout
            if math.hypot(px - r.center.x, py - r.center.y) < r.radius:
                return "roundabout"

        if self.infinity_path:
            inf = self.infinity_path
            d_left = math.hypot(px - inf.left_center.x, py - inf.left_center.y)
            d_right = math.hypot(px - inf.right_center.x, py - inf.right_center.y)
            if d_left < inf.radius_x or d_right < inf.radius_x:
                return "infinity_path"

        return "floor"

    def nearest_tape_segment(
        self,
        px: float,
        py: float,
        yaw: Optional[float] = None,
        world_line_points: Optional[List[Tuple[float, float]]] = None,
        max_yaw_diff_deg: float = 60.0,
    ) -> Optional[dict]:
        segments: List[Tuple[str, Point3D, Point3D]] = []

        for tl in self.tape_lines:
            for i in range(len(tl.waypoints) - 1):
                segments.append((tl.name, tl.waypoints[i], tl.waypoints[i + 1]))

        for np_ in self.nav_paths:
            for seg in np_.segments:
                if hasattr(seg, "waypoints") and len(seg.waypoints) >= 2:
                    for i in range(len(seg.waypoints) - 1):
                        segments.append((
                            f"{np_.name}/{getattr(seg, 'name', 'seg')}",
                            seg.waypoints[i],
                            seg.waypoints[i + 1],
                        ))

        best: Optional[dict] = None
        best_score = float("inf")

        for name, p0, p1 in segments:
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            seg_len = math.hypot(dx, dy)
            if seg_len < 1e-6:
                continue

            t = max(0.0, min(1.0, ((px - p0.x) * dx + (py - p0.y) * dy) / (seg_len ** 2)))
            cx = p0.x + t * dx
            cy = p0.y + t * dy

            dist = math.hypot(px - cx, py - cy)
            seg_heading_deg = math.degrees(math.atan2(dy, dx))
            lateral = ((px - p0.x) * dy - (py - p0.y) * dx) / seg_len

            heading_err_deg = 0.0
            if yaw is not None:
                raw = math.degrees(yaw) - seg_heading_deg
                heading_err_deg = (raw + 180) % 360 - 180
                yaw_diff = abs(heading_err_deg)
                if yaw_diff > 90:
                    yaw_diff = 180 - yaw_diff
                if yaw_diff > max_yaw_diff_deg:
                    continue
                yaw_score = yaw_diff / max_yaw_diff_deg
            else:
                yaw_score = 0.0

            point_score = 0.0
            if world_line_points and len(world_line_points) >= 2:
                total = 0.0
                for wx, wy in world_line_points:
                    tp = max(0.0, min(1.0, ((wx - p0.x) * dx + (wy - p0.y) * dy) / (seg_len ** 2)))
                    total += math.hypot(wx - (p0.x + tp * dx), wy - (p0.y + tp * dy))
                point_score = min(1.0, (total / len(world_line_points)) / 0.5)

            w_points = 0.6 if world_line_points else 0.0
            score = dist + 0.4 * yaw_score + w_points * point_score

            if score < best_score:
                best_score = score
                best = {
                    "name": name,
                    "segment_start": p0,
                    "segment_end": p1,
                    "closest_pt": (cx, cy),
                    "dist": dist,
                    "heading_deg": seg_heading_deg,
                    "lateral_error": lateral,
                    "heading_error_deg": heading_err_deg,
                    "score": score,
                    "p0": (p0.x, p0.y),
                    "p1": (p1.x, p1.y),
                }

        return best

    def nearby_aruco(self, px: float, py: float, radius: float = 1.5) -> List[Tuple[float, ArucoMarker]]:
        result = [
            (math.hypot(m.position.x - px, m.position.y - py), m)
            for m in self.all_aruco()
        ]
        result = [r for r in result if r[0] <= radius]
        result.sort(key=lambda x: x[0])
        return result

    def nearest_gate(self, px: float, py: float) -> Tuple[Optional[Gate], float]:
        best_gate: Optional[Gate] = None
        best_dist = float("inf")
        for g in self.all_gates():
            d = math.hypot(g.center.x - px, g.center.y - py)
            if d < best_dist:
                best_dist = d
                best_gate = g
        return best_gate, best_dist

    def context(
        self,
        px: float,
        py: float,
        yaw: Optional[float] = None,
        world_line_points: Optional[List[Tuple[float, float]]] = None,
        max_yaw_diff_deg: float = 60.0,
    ) -> dict:
        gate, gate_dist = self.nearest_gate(px, py)
        return {
            "px": px,
            "py": py,
            "pz": self.pz_at(px, py),
            "yaw": yaw,
            "zone": self.current_zone(px, py),
            "nearest_tape": self.nearest_tape_segment(
                px, py, yaw,
                world_line_points=world_line_points,
                max_yaw_diff_deg=max_yaw_diff_deg,
            ),
            "nearest_gate": {
                "name": gate.name if gate else None,
                "dist": gate_dist,
                "has_satellite": gate.has_satellite if gate else False,
            },
            "nearby_aruco": [
                {"id": m.id, "dist": d, "pos": m.position}
                for d, m in self.nearby_aruco(px, py, radius=1.5)
            ],
        }

    def summary(self) -> None:
        print(f"\n=== {self.name} ===")
        print(f"Field: {self.width_m}m x {self.height_m}m")
        print(f"Start exit : {self.start_area.exit_point if self.start_area else None}")
        print(f"Goal       : {self.goal_point}")

        gates = self.all_gates()
        print(f"\nGates ({len(gates)}):")
        for g in gates:
            print(f"  {g}")

        aruco = self.all_aruco()
        print(f"\nArUco markers ({len(aruco)}):")
        for m in aruco:
            print(f"  {m}")


def _build_start_and_goal(spec: FieldSpec) -> Tuple[StartArea, Point3D, ArucoMarker]:
    start_area = StartArea(
        origin=Point3D(spec.start_x0, 0.0),
        width=spec.start_x1 - spec.start_x0,
        depth=spec.start_depth,
        open_side="top",
    )
    goal_point = Point3D(spec.goal_cx, spec.goal_size / 2, 0.0)
    goal_aruco = ArucoMarker(
        id=25,
        position=Point3D(spec.goal_cx, spec.goal_size + 0.05, 0.02),
        size_m=0.10,
        role=MARKER_ROLE_LOCALIZATION,
        note="ArUco on top of Goal",
    )
    return start_area, goal_point, goal_aruco


def _build_roundabout(spec: FieldSpec) -> Roundabout:
    r = Roundabout(
        center=Point3D(*spec.roundabout_center),
        diameter=spec.roundabout_radius * 2,
        gate_angles_deg=list(spec.roundabout_gate_angles_deg),
        n_drones=2,
        plate_z=spec.roundabout_plate_z,
    )
    r.build_gates(has_satellite=True)
    # First gate is the entry, so it does not have a satellite. The other gates guard the circle.
    r.gates[0].has_satellite = False
    return r


def _build_traversal(spec: FieldSpec, g: _Geometry) -> Tuple[Ramp, Platform, Stairs, Ramp, Seesaw]:
    short_ramp = Ramp(
        name="short_ramp",
        base=Point3D(*spec.short_ramp_base, 0.0),
        top=Point3D(*g.ramp_top, g.plat_z),
        width=spec.short_ramp_width,
        rise=spec.short_ramp_rise,
        note=f"Short ramp adjacent to stairs, along y axis, {spec.short_ramp_length}m",
    )

    platform = Platform(
        name="platform",
        origin=Point3D(g.plat_x0, g.plat_y0, g.plat_z),
        width=spec.platform_width,
        depth=spec.platform_depth,
        pz=g.plat_z,
        hole=Point3D(g.plat_x0 + 0.10, g.plat_y1 - 0.10, g.plat_z),
        golf_ball=Point3D(g.plat_x1 - 0.10, g.plat_y0 + 0.10, g.plat_z),
        connections=["short_ramp", "stairs", "long_ramp"],
    )

    stairs = Stairs(
        name="stairs",
        base=Point3D(g.stair_mid_x, g.plat_y0 - spec.stair_n_steps * spec.stair_depth, 0.0),
        top=Point3D(g.stair_mid_x, g.plat_y0, g.plat_z),
        width=spec.stair_run_width,
        n_steps=spec.stair_n_steps,
        step_height=g.plat_z / spec.stair_n_steps,
        step_depth=spec.stair_depth,
    )
    stairs.build_gates()

    long_ramp = Ramp(
        name="long_ramp",
        base=Point3D(g.long_ramp_x1, g.lr_mid_y, 0.0),
        top=Point3D(g.long_ramp_x0, g.lr_mid_y, g.plat_z),
        width=spec.long_ramp_width,
        rise=g.plat_z,
        note=f"Long ramp, along x axis, {spec.long_ramp_length}m, from right side of platform",
    )
    long_ramp.gates = [
        Gate(
            # This gate sits 55 cm after the seesaw branch on the long ramp.
            # It is placed at `seesaw_offset_on_ramp + 0.55` from the platform edge.
            "long_ramp_gate",
            Point3D(
                g.long_ramp_x0 + spec.seesaw_offset_on_ramp + 0.55,
                g.lr_mid_y,
                g.plat_z * (1.0 - (spec.seesaw_offset_on_ramp + 0.55) / spec.long_ramp_length),
            ),
            orientation_deg=180,
            line_angle_deg=90,
            has_satellite=True,
            points=1,
        ),
    ]

    seesaw = Seesaw(
        name="seesaw",
        pivot=Point3D(g.seesaw_x, g.seesaw_y_mid, g.seesaw_z),
        length=spec.seesaw_length,
        width=spec.seesaw_width,
    )
    seesaw.build_gate()
    seesaw.golf_ball_pos = Point3D(
        g.seesaw_x,
        g.seesaw_y0 + spec.seesaw_length * 0.25,
        g.seesaw_z + 0.03,
    )

    return short_ramp, platform, stairs, long_ramp, seesaw


def _build_tasks(
    spec: FieldSpec, g: _Geometry,
) -> Tuple[SortingCenter, LuggageShuttle, BallDispenser, InfinityPath]:
    # Sorting square is rotated 45° into a diamond shape.
    # The 60 cm value is the side length, not the bounding-box span.
    # The half diagonal, which is the distance from centre to any apex, is side / √2.
    # SortingCenter uses this as `zone_size`; build_zones and build_aruco then place
    # the four zones in the NE, SE, SW, and NW quarters.
    sorting = SortingCenter(
        center=Point3D(*spec.sorting_center_xy),
        zone_size=spec.sorting_square_side / math.sqrt(2),
        pz=0.0,
    )
    sorting.build_zones()
    sorting.build_aruco(marker_size=0.10)

    shuttle = LuggageShuttle(
        path_start=Point3D(*spec.shuttle_border[0]),
        path_end=Point3D(*spec.shuttle_border[1]),
        speed_cms=20.0,
        aruco_id=5,
    )
    shuttle.build_luggage()

    ball_dispenser = BallDispenser(
        position=Point3D(1.0, 2.0, 0.0),
        trigger_height_m=0.18,
    )
    ball_dispenser.build_balls()

    infinity = InfinityPath(
        center=Point3D(g.inf_cx, spec.infinity_cy),
        loop_radius=g.inf_rx,
        loop_radius_y=g.inf_ry,
    )
    infinity.gates = [
        Gate(
            "infinity_gate_left",
            Point3D(g.inf_cx - g.inf_rx * 2, spec.infinity_cy),
            orientation_deg=0,
            line_angle_deg=0,
            has_satellite=True,
            points=1,
        ),
        Gate(
            "infinity_gate_center",
            Point3D(g.inf_cx, spec.infinity_cy),
            orientation_deg=0,
            line_angle_deg=90,
            has_satellite=False,
            points=1,
        ),
    ]

    return sorting, shuttle, ball_dispenser, infinity


def _build_extra_gates(spec: FieldSpec, g: _Geometry) -> List[Gate]:
    # Two satellite gates sit on the lr_straight2 segment, west of the U-turn exit.
    # The exit is at x = long_ramp_x1 + lr_uturn_extra_straight and y = lr_mid_y - 2·ry.
    # The gates are placed 30 cm and 130 cm along that straight.
    lr_str2_y = g.lr_mid_y - 2 * spec.lr_uturn_ry
    lr_str2_start_x = g.long_ramp_x1 + spec.lr_uturn_extra_straight
    return [
        Gate("gate_ramp_area_left",  # first gate the robot meets after the U-turn
             Point3D(lr_str2_start_x - 0.30, lr_str2_y),
             orientation_deg=0, line_angle_deg=90, has_satellite=True, points=1),
        Gate("gate_ramp_area_right",  # second gate, 1 m further along the same straight
             Point3D(lr_str2_start_x - 1.30, lr_str2_y),
             orientation_deg=0, line_angle_deg=90, has_satellite=True, points=1),
        # Gate at the midpoint of the Bezier arch between start_straight and goal_straight.
        # With the current spec values, that midpoint is (4.405, 1.955).
        Gate("gate_start_loop", Point3D(4.405, 1.955),
             orientation_deg=0, line_angle_deg=90, has_satellite=True, points=1),
        Gate("gate_short_ramp_top",
             Point3D(spec.short_ramp_base[0], g.ramp_top[1], spec.short_ramp_rise),
             orientation_deg=90, line_angle_deg=0, has_satellite=True, points=1),
    ]


def _build_tape_lines(spec: FieldSpec, g: _Geometry) -> List[TapeLine]:
    rc = spec.roundabout_center
    rr = spec.roundabout_radius
    return [
        TapeLine("roundabout_left_exit",
                 # Stops at the short-ramp approach line instead of extending past the west edge.
                 [Point3D(rc[0] - rr, rc[1]),
                  Point3D(spec.short_ramp_base[0], rc[1])],
                 task="roundabout", connects=["roundabout", "short_ramp_approach"]),
        TapeLine("roundabout_to_start_vertical",
                 # Shares the same vertical line as roundabout_down to avoid duplicate near-parallel
                 # lines around the T-junction with small_line.
                 [Point3D(rc[0], 0.0), Point3D(rc[0], 2.0)],
                 task="navigation", connects=["start", "roundabout"]),
        TapeLine("small_line",
                 # Branches west from the roundabout_to_start_vertical line toward the sorting center.
                 [Point3D(rc[0], 1.30), Point3D(rc[0] - 0.40, 1.30)],
                 task="navigation", connects=["roundabout_to_start_vertical", "sorting_center"]),
        # There is no floor tape here; this area is represented by the green wall model instead.
        TapeLine("short_ramp_approach",
                 [Point3D(spec.short_ramp_base[0], 2.60),
                  Point3D(spec.short_ramp_base[0], spec.short_ramp_base[1])],
                 task="ramp_up", connects=["roundabout_left_exit", "short_ramp"]),
        TapeLine("short_ramp_surface",
                 [Point3D(*spec.short_ramp_base, 0.0),
                  Point3D(*g.ramp_top, spec.short_ramp_rise)],
                 task="ramp_up", connects=["short_ramp_approach", "platform"]),
        TapeLine("long_ramp_surface",
                 [Point3D(g.long_ramp_x0, g.lr_mid_y, g.plat_z),
                  Point3D(g.long_ramp_x1, g.lr_mid_y, 0.0)],
                 task="ramp_down", connects=["platform", "long_ramp_exit"]),
        # Short connector running south from the long-ramp centerline to the seesaw's north end.
        TapeLine("ramp_to_seesaw",
                 # Starts on the long-ramp surface and ends at the seesaw pivot height.
                 [Point3D(g.seesaw_x, g.lr_mid_y, g.ramp_z_at_seesaw),
                  Point3D(g.seesaw_x, g.seesaw_y1, g.seesaw_z)],
                 task="ramp_down", connects=["long_ramp_surface", "seesaw"]),
    ]


def _build_nav_paths(spec: FieldSpec, g: _Geometry) -> List[NavPath]:
    # --- long ramp -> roundabout ---
    # U-turn is an ellipse: rx is the horizontal half-width, ry is the vertical half-height.
    # Total dimensions are 2·rx × 2·ry, currently 60 × 140 cm.
    rx = spec.lr_uturn_rx
    ry = spec.lr_uturn_ry
    straight1_end_x = g.long_ramp_x1 + spec.lr_uturn_extra_straight
    u_exit_y = g.lr_mid_y - 2 * ry
    round_top_y = spec.roundabout_center[1] + spec.roundabout_radius
    # The second arc bends east to south and brings the path to the top of
    # the straight tape that runs down into the roundabout.
    # The arc radius is the remaining distance between the U-turn exit and
    # the top of that straight section:
    #     arc2_radius = u_exit_y - (round_top_y + lr_north_approach_length)
    # Smaller values make this bend tighter.
    north_top_y = round_top_y + spec.lr_north_approach_length
    arc2_radius = u_exit_y - north_top_y
    arc2_end_x = spec.roundabout_center[0] + arc2_radius

    path_long_ramp = NavPath(
        name="long_ramp_to_roundabout",
        task="ramp_down",
        connects=["long_ramp", "roundabout"],
        segments=[
            TapeLine("lr_straight1",
                     [Point3D(g.long_ramp_x1, g.lr_mid_y), Point3D(straight1_end_x, g.lr_mid_y)]),
            ArcSegment(
                center=Point3D(straight1_end_x, g.lr_mid_y - ry),
                radius=rx,
                radius_y=ry,
                angle_start_deg=90, angle_end_deg=-90,
                n_samples=80,
            ),
            TapeLine("lr_straight2",
                     [Point3D(straight1_end_x, u_exit_y), Point3D(arc2_end_x, u_exit_y)]),
            ArcSegment(
                center=Point3D(arc2_end_x, north_top_y),
                radius=abs(arc2_radius),
                angle_start_deg=90, angle_end_deg=180,
                n_samples=30,
            ),
            TapeLine("roundabout_north_approach",
                     [Point3D(spec.roundabout_center[0], north_top_y),
                      Point3D(spec.roundabout_center[0], round_top_y)]),
            TapeLine("roundabout_down",
                     [Point3D(spec.roundabout_center[0], spec.roundabout_center[1] - spec.roundabout_radius),
                      Point3D(spec.roundabout_center[0], 0.50)]),
        ],
    )

    # --- start <-> goal loop (Bezier arch over the obstacle area) ---
    start_mid_x = (spec.start_x0 + spec.start_x1) / 2
    start_mid_y = spec.start_depth / 2
    straight_len = 1.40
    arc_height = 0.40

    p1 = Point3D(spec.goal_cx, spec.goal_size + straight_len)
    p2 = Point3D(start_mid_x, start_mid_y + straight_len)
    ctrl = Point3D((p1.x + p2.x) / 2, max(p1.y, p2.y) + arc_height)

    path_start_goal = NavPath(
        name="start_goal_loop",
        task="goal",
        connects=["goal", "start"],
        segments=[
            TapeLine("goal_straight", [Point3D(spec.goal_cx, spec.goal_size), p1]),
            BezierSegment(p0=p1, p3=p2, p1=ctrl, p2=ctrl, n_samples=60),
            TapeLine("start_straight", [p2, Point3D(start_mid_x, start_mid_y)]),
        ],
    )

    # --- start -> roundabout (left branch) ---
    s_top = Point3D(start_mid_x, start_mid_y + straight_len)
    s_target = Point3D(3.20, 2.05)   # left-branch target point
    path_start_left = NavPath(
        name="start_left_branch",
        task="navigation",
        connects=["start", "roundabout"],
        segments=[
            BezierSegment(
                p0=s_top, p3=s_target,
                p1=Point3D(s_top.x, s_target.y),
                p2=Point3D(s_target.x, s_target.y),
                n_samples=50,
            ),
        ],
    )

    # --- ball dispenser -> stairs -> platform ---
    stair_bot_y = g.plat_y0 - spec.stair_n_steps * spec.stair_depth
    path_ball_stairs = NavPath(
        name="ball_to_stairs",
        task="golf_balls",
        connects=["ball_dispenser", "stairs", "platform"],
        segments=[
            TapeLine("bowl_to_stair_base",
                     [Point3D(1.0, 2.0), Point3D(g.stair_mid_x, stair_bot_y)]),
            TapeLine("stair_climb",
                     [Point3D(g.stair_mid_x, stair_bot_y, 0.0),
                      Point3D(g.stair_mid_x, g.plat_y0, g.plat_z)]),
        ],
    )

    return [path_long_ramp, path_start_goal, path_start_left, path_ball_stairs]


def _build_landmarks(spec: FieldSpec, g: _Geometry) -> List[Landmark]:
    """Geometrically unique points on the map.

    Each landmark is a place where the line or curve detector should produce a
    clear signature, such as a Y-fork, T-junction, ramp start, or stair top.
    The matcher compares live detector output against `expected_signature` and,
    on a match, publishes an (x, y, yaw) fix similar to an ArUco correction.
    """
    start_mid_x = (spec.start_x0 + spec.start_x1) / 2
    start_mid_y = spec.start_depth / 2
    s_top_y = start_mid_y + 1.40   # straight_len from _build_nav_paths

    # Confidence radius sizing
    # ------------------------
    # The matcher uses this radius, scaled by MIN_PRIOR_GATE_RATIO, as a position
    # gate on the Kalman prior. If the radius is too tight, drift between fixes
    # can push the prior outside the gate. Then the next landmark gets rejected
    # even when the camera sees it clearly. A 0.50-0.80 m range leaves room for
    # realistic odometry and IMU drift, while signature scoring still filters out
    # accidental matches.
    return [
        Landmark(
            name="start_fork",
            position=Point3D(start_mid_x, s_top_y, 0.0),
            kind="fork_y",
            expected_signature={"branch_count": 2, "fork_angle_deg": 90.0},
            expected_yaw_deg=90.0,
            confidence_radius=0.60,
            note="Tape splits: straight goes to goal, left branch heads to roundabout",
        ),
        Landmark(
            name="roundabout_north_entry",
            position=Point3D(spec.roundabout_center[0],
                             spec.roundabout_center[1] + spec.roundabout_radius, 0.0),
            kind="t_intersection",
            expected_signature={"branch_count": 2, "fork_angle_deg": 90.0},
            expected_yaw_deg=270.0,
            confidence_radius=0.50,
            note="Long-ramp descent path meets roundabout at its top",
        ),
        Landmark(
            name="long_ramp_top",
            position=Point3D(g.long_ramp_x0, g.lr_mid_y, g.plat_z),
            kind="ramp_end",
            expected_signature={"slope_change_deg": 8.7},   # approx atan(0.55 / 3.64)
            expected_yaw_deg=0.0,
            confidence_radius=0.60,
            note="Top of long ramp where it meets the platform",
        ),
        Landmark(
            name="long_ramp_bottom",
            position=Point3D(g.long_ramp_x1, g.lr_mid_y, 0.0),
            kind="ramp_start",
            expected_signature={"slope_change_deg": 8.7},
            expected_yaw_deg=180.0,
            confidence_radius=0.60,
            note="Bottom of long ramp at floor level",
        ),
        Landmark(
            name="short_ramp_top",
            position=Point3D(*g.ramp_top, g.plat_z),
            kind="ramp_end",
            expected_signature={"slope_change_deg": 19.2},  # approx atan(0.55 / 1.58)
            expected_yaw_deg=90.0,
            confidence_radius=0.50,
            note="Short ramp meets platform",
        ),
        Landmark(
            name="stair_top",
            position=Point3D(g.stair_mid_x, g.plat_y0, g.plat_z),
            kind="stair_step",
            expected_signature={"step_count": spec.stair_n_steps},
            expected_yaw_deg=90.0,
            confidence_radius=0.50,
            note="Top of stairs onto platform",
        ),
        Landmark(
            name="ball_dispenser",
            position=Point3D(1.0, 2.0, 0.0),
            kind="fixture_anchor",
            expected_signature={"tape_endpoint": True},
            expected_yaw_deg=270.0,
            confidence_radius=0.60,
            note="Tape 'bowl_to_stair_base' starts here; physical dispenser is a large fixed landmark",
        ),
    ]


def _build_mobile_arucos(shuttle: LuggageShuttle) -> List[MobileArucoSpec]:
    """Specs for moving and placeable task markers.

    Sources:
        - Luggage cubes have one ArUco marker on the visible top face.
          The cube's geometric centre sits behind that face, at -cube_side/2
          along the marker's outward normal (+Z in OpenCV ArUco).
        - Shuttle carrier marker id 5 lies flat on the shuttle deck.
          Its planar centre matches the marker centre, so the offset is 0.
          Vertical offset is intentionally omitted because downstream tasks
          only use the (x, y) position.

    Marker physical size matches `MARKER_SIZE_M` in saruco.py.
    This is the size used by solvePnP, not the cube edge length.
    """
    specs: List[MobileArucoSpec] = []

    cube_side = 0.06   # luggage cube edge length in metres; cf. Luggage.size_m
    cube_marker_size = 0.035

    for lug in shuttle.luggage:
        specs.append(MobileArucoSpec(
            id=int(lug.aruco_id),
            role=MARKER_ROLE_LUGGAGE,
            size_m=cube_marker_size,
            object_offset_marker_frame=(0.0, 0.0, -cube_side / 2.0),
            object_kind="cube",
            name=f"luggage_zone{lug.target_zone}" if lug.target_zone else f"luggage_{lug.aruco_id}",
            note=f"target zone={lug.target_zone or '-'}, points_correct={lug.points_correct}",
        ))

    specs.append(MobileArucoSpec(
        id=int(shuttle.aruco_id),
        role=MARKER_ROLE_SHUTTLE,
        size_m=cube_marker_size,
        object_offset_marker_frame=(0.0, 0.0, 0.0),
        object_kind="platform",
        name="luggage_shuttle",
        note="marker lies flat on shuttle deck, so planar centre = marker centre",
    ))

    return specs


def build_field(spec: FieldSpec = SPEC) -> CompetitionField2026:
    g = _derive_geometry(spec)

    start_area, goal_point, goal_aruco = _build_start_and_goal(spec)
    roundabout = _build_roundabout(spec)
    short_ramp, platform, stairs, long_ramp, seesaw = _build_traversal(spec, g)
    sorting, shuttle, ball_dispenser, infinity = _build_tasks(spec, g)

    golf_balls = [
        GolfBall(
            position=Point3D(g.seesaw_x, g.seesaw_y0 + spec.seesaw_length * 0.25, g.seesaw_z + 0.03),
            target_hole=platform.hole,
            points=2,
            note="Seesaw ball",
        ),
        GolfBall(
            position=Point3D(g.plat_x1 - 0.10, g.plat_y0 + 0.10, g.plat_z),
            target_hole=platform.hole,
            points=2,
            note="Platform ball",
        ),
    ]

    time_buttons = [
        # Left time-extension button on the west green wall.
        TimeExtButton("time_ext_10", Point3D(0.0, 0.68),
                      extra_sec=90, z_height=spec.time_button_trigger_z, note="Left button"),
        # Right time-extension button on the east green wall.
        TimeExtButton("time_ext_11", Point3D(spec.width_m, 3.60),
                      extra_sec=90, z_height=spec.time_button_trigger_z, note="Right button"),
    ]

    return CompetitionField2026(
        width_m=spec.width_m,
        height_m=spec.height_m,
        green_barrier_z=spec.green_barrier_z,
        green_y_line=(Point3D(*spec.green_y_line[0], 0), Point3D(*spec.green_y_line[1], 0)),
        shuttle_green_border=(
            Point3D(*spec.shuttle_border[0], 0),
            Point3D(*spec.shuttle_border[1], 0),
        ),
        start_area=start_area,
        goal_point=goal_point,
        goal_aruco=goal_aruco,
        roundabout=roundabout,
        short_ramp=short_ramp,
        platform=platform,
        stairs=stairs,
        long_ramp=long_ramp,
        seesaw=seesaw,
        sorting_center=sorting,
        shuttle=shuttle,
        ball_dispenser=ball_dispenser,
        infinity_path=infinity,
        golf_balls=golf_balls,
        time_buttons=time_buttons,
        gates=_build_extra_gates(spec, g),
        tape_lines=_build_tape_lines(spec, g),
        aruco_markers=[],
        mobile_arucos=_build_mobile_arucos(shuttle),
        nav_paths=_build_nav_paths(spec, g),
        landmarks=_build_landmarks(spec, g),
    )


FIELD = build_field()


if __name__ == "__main__":
    FIELD.summary()