from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
import math


@dataclass
class Point3D:
    x: float
    y: float
    z: float = 0.0

    def as_tuple(self) -> Tuple[float, float, float]:
        return (round(self.x, 3), round(self.y, 3), round(self.z, 3))

    def xy(self) -> Tuple[float, float]:
        return (round(self.x, 3), round(self.y, 3))

    def dist_2d(self, other: "Point3D") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def dist_3d(self, other: "Point3D") -> float:
        return math.sqrt(
            (self.x - other.x) ** 2 +
            (self.y - other.y) ** 2 +
            (self.z - other.z) ** 2
        )

    def __repr__(self):
        return f"Point3D({self.x:.3f}, {self.y:.3f}, {self.z:.3f})"


# Roles an ArUco marker can have on the field.
# Only "localization" markers are allowed to update the robot pose.
# Other roles are used for task-specific markers such as luggage, shuttle, and goal markers.
MARKER_ROLE_LOCALIZATION = "localization"
MARKER_ROLE_LUGGAGE      = "luggage"
MARKER_ROLE_SHUTTLE      = "shuttle"
MARKER_ROLE_GOAL         = "goal_marker"
MARKER_ROLE_UNKNOWN      = "unknown"

VALID_MARKER_ROLES = frozenset({
    MARKER_ROLE_LOCALIZATION,
    MARKER_ROLE_LUGGAGE,
    MARKER_ROLE_SHUTTLE,
    MARKER_ROLE_GOAL,
    MARKER_ROLE_UNKNOWN,
})


@dataclass
class ArucoMarker:
    id: int
    position: Point3D
    size_m: float
    facing_deg: float = 0.0
    role: str = MARKER_ROLE_LOCALIZATION
    note: str = ""

    def __post_init__(self):
        if self.role not in VALID_MARKER_ROLES:
            raise ValueError(
                f"ArucoMarker(id={self.id}): unknown role {self.role!r}; "
                f"expected one of {sorted(VALID_MARKER_ROLES)}"
            )

    def __repr__(self):
        return (
            f"ArucoMarker(id={self.id}, role={self.role}, "
            f"pos={self.position.xy()}, size={self.size_m}m)"
        )


@dataclass
class MobileArucoSpec:
    """Spec for a marker whose world position is not fixed.

    Used for task markers that move with an object, such as luggage cubes or
    the shuttle. The detector still sees these markers every frame, and this
    spec tells us how to classify them and convert marker pose to the object's
    geometric centre.

    `object_offset_marker_frame` is a 3-vector in the marker's local frame.
    OpenCV ArUco convention is +X to the marker's right edge, +Y to the marker's
    lower edge, and +Z along the outward normal. For a cube with a marker on one
    face, the cube centre sits behind that face at (0, 0, -cube_side / 2).
    """
    id: int
    role: str
    size_m: float
    object_offset_marker_frame: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    object_kind: str = ""    # free-form object type, for example "cube" or "platform"
    name: str = ""           # human-readable label, for example "luggage_zoneA"
    note: str = ""

    def __post_init__(self):
        if self.role == MARKER_ROLE_LOCALIZATION:
            raise ValueError(
                f"MobileArucoSpec(id={self.id}): role 'localization' is reserved "
                f"for fixed markers; use ArucoMarker for those."
            )
        if self.role not in VALID_MARKER_ROLES:
            raise ValueError(
                f"MobileArucoSpec(id={self.id}): unknown role {self.role!r}; "
                f"expected one of {sorted(VALID_MARKER_ROLES)}"
            )


@dataclass
class Gate:
    name: str
    center: Point3D
    width: float = 0.45            # gate opening width
    height: float = 0.50           # vertical post height
    bottom_z: float = 0.0          # post starts directly on the floor
    satellite_height: float = 0.10 # distance from gate top edge to satellite tip
    orientation_deg: float = 0.0
    line_angle_deg: float = None
    radial_from: tuple = None
    has_satellite: bool = False
    points: int = 1
    note: str = ""

    @property
    def top_z(self) -> float:
        return self.bottom_z + self.height

    @property
    def satellite_z(self) -> float:
        """Satellite tip height above the centre of the gate's top edge."""
        return self.top_z + self.satellite_height

    def __repr__(self):
        sat = " [SAT]" if self.has_satellite else ""
        return f"Gate({self.name}, center={self.center.xy()}, +{self.points}pt{sat})"


@dataclass
class BezierSegment:
    p0: Point3D
    p3: Point3D
    p1: Point3D = None
    p2: Point3D = None
    z: float = 0.0
    n_samples: int = 60
    note: str = ""

    def __post_init__(self):
        if self.p1 is None:
            self.p1 = self.p0
        if self.p2 is None:
            self.p2 = self.p3

    def sample(self, n: int = None):
        import numpy as np
        n = n or self.n_samples
        t = np.linspace(0, 1, n)

        x = (
            (1 - t) ** 3 * self.p0.x +
            3 * (1 - t) ** 2 * t * self.p1.x +
            3 * (1 - t) * t ** 2 * self.p2.x +
            t ** 3 * self.p3.x
        )
        y = (
            (1 - t) ** 3 * self.p0.y +
            3 * (1 - t) ** 2 * t * self.p1.y +
            3 * (1 - t) * t ** 2 * self.p2.y +
            t ** 3 * self.p3.y
        )
        z = np.full(n, self.z)
        return x, y, z

    @property
    def start(self) -> Point3D:
        return self.p0

    @property
    def end(self) -> Point3D:
        return self.p3


@dataclass
class ArcSegment:
    center: Point3D
    radius: float                        # horizontal x radius; also the circle radius when radius_y is None
    angle_start_deg: float
    angle_end_deg: float
    z: float = 0.0
    n_samples: int = 60
    radius_y: Optional[float] = None    # if set, the arc is elliptical with this vertical radius
    note: str = ""

    @property
    def rx(self) -> float:
        return self.radius

    @property
    def ry(self) -> float:
        return self.radius_y if self.radius_y is not None else self.radius

    def sample(self, n: int = None):
        import numpy as np
        n = n or self.n_samples
        angles = np.linspace(
            math.radians(self.angle_start_deg),
            math.radians(self.angle_end_deg),
            n
        )
        x = self.center.x + self.rx * np.cos(angles)
        y = self.center.y + self.ry * np.sin(angles)
        z = np.full(n, self.z)
        return x, y, z

    @property
    def start(self) -> Point3D:
        rad = math.radians(self.angle_start_deg)
        return Point3D(
            self.center.x + self.rx * math.cos(rad),
            self.center.y + self.ry * math.sin(rad),
            self.z
        )

    @property
    def end(self) -> Point3D:
        rad = math.radians(self.angle_end_deg)
        return Point3D(
            self.center.x + self.rx * math.cos(rad),
            self.center.y + self.ry * math.sin(rad),
            self.z
        )


@dataclass
class NavPath:
    name: str
    segments: List = field(default_factory=list)
    task: str = None
    connects: List[str] = field(default_factory=list)
    color: str = "white"
    note: str = ""

    def all_points(self) -> List[Point3D]:
        pts = []
        for seg in self.segments:
            if hasattr(seg, "waypoints"):
                pts.extend(seg.waypoints)
            elif hasattr(seg, "start"):
                pts.append(seg.start)
                pts.append(seg.end)
        return pts


@dataclass
class TapeLine:
    name: str
    waypoints: List[Point3D]
    width: float = 0.038
    task: str = None
    connects: List[str] = field(default_factory=list)
    note: str = ""

    @property
    def start(self) -> Point3D:
        return self.waypoints[0]

    @property
    def end(self) -> Point3D:
        return self.waypoints[-1]

    @property
    def length(self) -> float:
        total = 0.0
        for a, b in zip(self.waypoints[:-1], self.waypoints[1:]):
            total += a.dist_2d(b)
        return total

    @property
    def heading_deg(self) -> float:
        if len(self.waypoints) < 2:
            return 0.0
        dx = self.waypoints[1].x - self.waypoints[0].x
        dy = self.waypoints[1].y - self.waypoints[0].y
        return math.degrees(math.atan2(dy, dx))


@dataclass
class StartArea:
    origin: Point3D
    width: float
    depth: float
    open_side: str = "top"
    note: str = ""

    @property
    def center(self) -> Point3D:
        return Point3D(
            self.origin.x + self.width / 2,
            self.origin.y + self.depth / 2,
            self.origin.z,
        )

    @property
    def exit_point(self) -> Point3D:
        if self.open_side == "top":
            return Point3D(self.origin.x + self.width / 2, self.origin.y + self.depth, self.origin.z)
        if self.open_side == "bottom":
            return Point3D(self.origin.x + self.width / 2, self.origin.y, self.origin.z)
        if self.open_side == "right":
            return Point3D(self.origin.x + self.width, self.origin.y + self.depth / 2, self.origin.z)
        return Point3D(self.origin.x, self.origin.y + self.depth / 2, self.origin.z)


@dataclass
class Roundabout:
    center: Point3D
    diameter: float
    gate_angles_deg: List[float] = field(default_factory=lambda: [90.0, 210.0, 330.0])
    n_drones: int = 2
    plate_z: float = 0.01  # height of the raised circular plate above the floor
    gates: List[Gate] = field(default_factory=list)
    note: str = ""

    @property
    def radius(self) -> float:
        return self.diameter / 2.0

    def point_at_angle(self, angle_deg: float, r_scale: float = 1.0) -> Point3D:
        r = self.radius * r_scale
        rad = math.radians(angle_deg)
        return Point3D(
            self.center.x + r * math.cos(rad),
            self.center.y + r * math.sin(rad),
            self.center.z,
        )

    def build_gates(self, has_satellite: bool = True) -> List[Gate]:
        self.gates = []
        for i, angle in enumerate(self.gate_angles_deg):
            pt = self.point_at_angle(angle)
            pt.z = 0.025
            self.gates.append(Gate(
                name=f"roundabout_gate_{i + 1}",
                center=pt,
                orientation_deg=angle,
                radial_from=(self.center.x, self.center.y),
                has_satellite=has_satellite,
                points=1,
            ))
        return self.gates


@dataclass
class InfinityPath:
    center: Point3D
    loop_radius: float
    loop_radius_y: float = None
    guard_speed_cms: float = 30.0
    penalty_per_touch: int = -1
    max_penalty: int = -2
    gates: List[Gate] = field(default_factory=list)
    note: str = ""

    @property
    def radius_x(self) -> float:
        return self.loop_radius

    @property
    def radius_y(self) -> float:
        return self.loop_radius_y if self.loop_radius_y is not None else self.loop_radius

    @property
    def left_center(self) -> Point3D:
        return Point3D(self.center.x - self.radius_x, self.center.y, self.center.z)

    @property
    def right_center(self) -> Point3D:
        return Point3D(self.center.x + self.radius_x, self.center.y, self.center.z)

    def build_gates(self) -> List[Gate]:
        self.gates = [
            Gate("infinity_gate_left", self.left_center, orientation_deg=90, points=1),
            Gate("infinity_gate_right", self.right_center, orientation_deg=90, points=1),
        ]
        return self.gates


@dataclass
class Ramp:
    name: str
    base: Point3D
    top: Point3D
    width: float
    rise: float = 0.55
    gates: List[Gate] = field(default_factory=list)
    note: str = ""

    @property
    def run(self) -> float:
        return self.base.dist_2d(self.top)

    @property
    def slope_deg(self) -> float:
        if self.run < 1e-6:
            return 90.0
        return math.degrees(math.atan2(self.rise, self.run))

    def _local(self, x: float, y: float) -> Optional[Tuple[float, float, float]]:
        dx = self.top.x - self.base.x
        dy = self.top.y - self.base.y
        run = math.hypot(dx, dy)
        if run < 1e-6:
            return None
        ux, uy = dx / run, dy / run
        vx, vy = -uy, ux
        px = x - self.base.x
        py = y - self.base.y
        along = px * ux + py * uy
        lateral = abs(px * vx + py * vy)
        return along, lateral, run

    def contains_2d(self, x: float, y: float) -> bool:
        loc = self._local(x, y)
        if loc is None:
            return False
        along, lateral, run = loc
        return 0.0 <= along <= run and lateral <= self.width / 2

    def pz_at(self, x: float, y: float) -> float:
        loc = self._local(x, y)
        if loc is None:
            return 0.0
        along, lateral, run = loc
        if along < 0 or along > run or lateral > self.width / 2:
            return 0.0
        return (along / run) * self.rise


@dataclass
class Stairs:
    name: str
    base: Point3D
    top: Point3D
    width: float = 0.40
    n_steps: int = 4
    step_height: float = 0.1375
    step_depth: float = 0.40
    gates: List[Gate] = field(default_factory=list)
    note: str = ""

    @property
    def total_rise(self) -> float:
        return self.n_steps * self.step_height

    @property
    def total_run(self) -> float:
        return self.base.dist_2d(self.top)

    def _local(self, x: float, y: float) -> Optional[Tuple[float, float, float]]:
        total = self.total_run
        if total < 1e-6:
            return None
        dx = self.top.x - self.base.x
        dy = self.top.y - self.base.y
        ux, uy = dx / total, dy / total
        vx, vy = -uy, ux
        px = x - self.base.x
        py = y - self.base.y
        along = px * ux + py * uy
        lateral = abs(px * vx + py * vy)
        return along, lateral, total

    def contains_2d(self, x: float, y: float) -> bool:
        loc = self._local(x, y)
        if loc is None:
            return False
        along, lateral, total = loc
        return 0.0 <= along <= total and lateral <= self.width / 2

    def pz_at(self, x: float, y: float) -> float:
        loc = self._local(x, y)
        if loc is None:
            return 0.0
        along, lateral, total = loc
        if along < 0 or along > total or lateral > self.width / 2:
            return 0.0
        step_idx = min(int((along / total) * self.n_steps), self.n_steps - 1)
        return (step_idx + 1) * self.step_height

    def build_gates(self) -> List[Gate]:
        total = self.total_run
        if total < 1e-6:
            self.gates = []
            return self.gates

        dx = (self.top.x - self.base.x) / total
        dy = (self.top.y - self.base.y) / total

        self.gates = []
        for step_idx, pts in [(self.n_steps - 2, 1), (self.n_steps - 1, 2)]:
            t = (step_idx + 0.5) / self.n_steps
            gx = self.base.x + t * total * dx
            gy = self.base.y + t * total * dy
            gz = (step_idx + 1) * self.step_height

            self.gates.append(Gate(
                name=f"{self.name}_gate_{step_idx + 1}",
                center=Point3D(gx, gy, gz),
                width=self.width,
                orientation_deg=90,
                line_angle_deg=0,
                has_satellite=False,
                points=pts,
            ))

        return self.gates


@dataclass
class Seesaw:
    name: str
    pivot: Point3D
    length: float
    width: float = 0.60
    gate: Optional[Gate] = None
    golf_ball_pos: Optional[Point3D] = None
    note: str = ""

    @property
    def pz(self) -> str:
        return "dynamic (IMU pitch)"

    def build_gate(self) -> Gate:
        # The seesaw is Y-oriented. The gate stands on the floor directly under
        # the seesaw's south tip, using the same X/Y as the tip but z=0.
        # It is not lifted onto the seesaw surface and has no satellite.
        self.gate = Gate(
            name=f"{self.name}_gate",
            center=Point3D(
                self.pivot.x,
                self.pivot.y - self.length / 2,
                0.0,
            ),
            orientation_deg=0,
            line_angle_deg=0,
            has_satellite=False,
            points=1,
        )
        return self.gate


@dataclass
class Platform:
    name: str
    origin: Point3D
    width: float
    depth: float
    pz: float = 0.55
    hole: Optional[Point3D] = None
    golf_ball: Optional[Point3D] = None
    connections: List[str] = field(default_factory=list)
    note: str = ""

    @property
    def center(self) -> Point3D:
        return Point3D(
            self.origin.x + self.width / 2,
            self.origin.y + self.depth / 2,
            self.pz,
        )

    def contains_2d(self, x: float, y: float) -> bool:
        return (
            self.origin.x <= x <= self.origin.x + self.width and
            self.origin.y <= y <= self.origin.y + self.depth
        )


@dataclass
class SortingCenter:
    """Sorting plate split into four zones by a '+' divider.

    `zone_size` is the distance from the centre to the square edge when the
    plate is treated as an axis-aligned square.
    """
    center: Point3D
    zone_size: float = 0.30  # half-side; full square side = 2 · zone_size
    pz: float = 0.25
    zones: Dict[str, Point3D] = field(default_factory=dict)
    aruco_markers: List[ArucoMarker] = field(default_factory=list)
    note: str = ""

    def build_zones(self) -> Dict[str, Point3D]:
        # The sorting square is rotated 45°, so it appears as a diamond.
        # `zone_size` is the half-diagonal, equal to side / √2.
        # Zone centres sit at the four N/S/E/W apex points:
        #
        #       B (north)
        #   C (west)   A (east)
        #       D (south)
        h = self.zone_size
        cx, cy = self.center.x, self.center.y
        self.zones = {
            "A": Point3D(cx + h, cy,     self.pz),  # east
            "B": Point3D(cx,     cy + h, self.pz),  # north
            "C": Point3D(cx - h, cy,     self.pz),  # west
            "D": Point3D(cx,     cy - h, self.pz),  # south
        }
        return self.zones

    def build_aruco(self, marker_size: float = 0.10) -> List[ArucoMarker]:
        # The four ArUco plates sit at the diamond's N/S/E/W apexes.
        # Each plate is V-shaped, with two faces 90° apart, so it can serve
        # the two zones adjacent to that apex. The faces point outward along
        # the edge bisectors: NE, SE, SW, and NW.
        import math
        h = self.zone_size
        inset = 0.17                                  # distance along the edge from the apex, in metres
        d = inset / math.sqrt(2.0)                    # equivalent offset on each axis
        cx, cy = self.center.x, self.center.y

        # ID layout:
        #   A = east apex, B = north apex, C = west apex, D = south apex.
        # Each zone has two marker faces along the adjacent diamond edges.
        positions = [
            # A zone, east apex: NE face and SE face
            (11, cx + h - d, cy + d,         45.0, "A E-apex NE face"),
            (10, cx + h - d, cy - d,        -45.0, "A E-apex SE face"),
            # B zone, north apex: NE face and NW face
            (12, cx + d,     cy + h - d,     45.0, "B N-apex NE face"),
            (13, cx - d,     cy + h - d,    135.0, "B N-apex NW face"),
            # C zone, west apex: NW face and SW face
            (14, cx - h + d, cy + d,        135.0, "C W-apex NW face"),
            (15, cx - h + d, cy - d,       -135.0, "C W-apex SW face"),
            # D zone, south apex: SE face and SW face
            (17, cx + d,     cy - h + d,    -45.0, "D S-apex SE face"),
            (16, cx - d,     cy - h + d,   -135.0, "D S-apex SW face"),
        ]

        self.aruco_markers = [
            ArucoMarker(
                id=mid,
                position=Point3D(px, py, self.pz),
                size_m=marker_size,
                facing_deg=fdeg,
                role=MARKER_ROLE_LOCALIZATION,
                note=note,
            )
            for mid, px, py, fdeg, note in positions
        ]

        return self.aruco_markers

    def get_zone(self, name: str) -> Optional[Point3D]:
        return self.zones.get(name.upper())


@dataclass
class TimeExtButton:
    name: str
    center: Point3D
    extra_sec: int = 90
    trigger: str = "bump"
    z_height: float = 0.12  # sensor trigger height above ground
    note: str = ""


@dataclass
class Luggage:
    aruco_id: int
    size_m: float = 0.06
    target_zone: str = ""
    points_correct: int = 4
    points_any: int = 1
    note: str = ""


@dataclass
class LuggageShuttle:
    path_start: Point3D
    path_end: Point3D
    speed_cms: float = 20.0
    surface_height_m: float = 0.15   # shuttle deck height where luggage cubes sit
    aruco_id: int = 5
    luggage: List[Luggage] = field(default_factory=list)
    note: str = ""

    @property
    def path_length(self) -> float:
        return self.path_start.dist_2d(self.path_end)

    def build_luggage(self) -> List[Luggage]:
        self.luggage = [
            Luggage(aruco_id=20, target_zone="A", points_correct=4),
            Luggage(aruco_id=53, target_zone="D", points_correct=2),
        ]
        return self.luggage


@dataclass
class Ball:
    color: str
    target_zone: str
    points: int
    diameter_m: float = 0.043


@dataclass
class BallDispenser:
    position: Point3D
    trigger_height_m: float = 0.18
    balls: List[Ball] = field(default_factory=list)
    note: str = ""

    def build_balls(self) -> List[Ball]:
        self.balls = [
            Ball("blue", "C", points=4),
            Ball("red", "B", points=3),
            Ball("white", "", points=1),
            Ball("white", "", points=1),
            Ball("white", "", points=1),
        ]
        return self.balls


@dataclass
class Landmark:
    """A unique map feature that the camera can use for absolute pose correction.

    When the line or curve detector sees a pattern matching `expected_signature`,
    the robot can publish an absolute (x, y, yaw) update at this landmark,
    similar to an ArUco-based pose fix.
    """
    name: str
    position: Point3D
    kind: str                                              # e.g. "fork_y", "t_intersection", "ramp_start", "ramp_end", "stair_step"
    expected_signature: Dict[str, float] = field(default_factory=dict)
    confidence_radius: float = 0.20                        # tolerance around `position` in metres
    expected_yaw_deg: Optional[float] = None               # expected robot heading; None means any heading
    note: str = ""

    def __repr__(self):
        return f"Landmark({self.name}, {self.kind}, pos={self.position.xy()})"


@dataclass
class GolfBall:
    position: Point3D
    diameter_m: float = 0.043
    color: str = "orange"
    target_hole: Optional[Point3D] = None
    points: int = 2
    note: str = ""