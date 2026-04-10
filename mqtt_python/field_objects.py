"""
field_objects.py
================
DTU Robocup 2026 - Core object models.

Coordinate system (world frame):
    x = right  (from bottom-left corner of field)
    y = up     (from bottom-left corner of field)
    z = height (metres)

Floor objects z=0, platform z=0.55m, ramp z increases linearly.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
import math


# ---------------------------------------------------------------------------
# BASE TYPE
# ---------------------------------------------------------------------------

@dataclass
class Point3D:
    x: float
    y: float
    z: float = 0.0

    def as_tuple(self) -> Tuple[float, float, float]:
        return (round(self.x, 3), round(self.y, 3), round(self.z, 3))

    def xy(self) -> Tuple[float, float]:
        return (round(self.x, 3), round(self.y, 3))

    def dist_2d(self, other: Point3D) -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def dist_3d(self, other: Point3D) -> float:
        return math.sqrt((self.x-other.x)**2 + (self.y-other.y)**2 + (self.z-other.z)**2)

    def __repr__(self):
        return f"Point3D({self.x:.3f}, {self.y:.3f}, {self.z:.3f})"


# ---------------------------------------------------------------------------
# ARUCO MARKER
# ---------------------------------------------------------------------------

@dataclass
class ArucoMarker:
    """An ArUco marker on the field."""
    id: int
    position: Point3D        # marker centre (world coordinates)
    size_m: float            # marker edge length (metres)
    facing_deg: float = 0.0  # direction the marker faces (degrees, 0=+X axis)
    note: str = ""

    def __repr__(self):
        return f"ArucoMarker(id={self.id}, pos={self.position.xy()}, size={self.size_m}m)"


# ---------------------------------------------------------------------------
# GATE
# ---------------------------------------------------------------------------

@dataclass
class Gate:
    """
    Yellow gate. Some have a satellite (drone).
    Gate frame: width 0.45m, height 0.47m (standard).

    orientation_deg : robot's passing direction (world frame, degrees)
    line_angle_deg  : angle along which the line extends (0=along x, 90=along y)
                      If None, orientation_deg + 90 is used (perpendicular)
    radial_from     : (x, y) - radial line from this point to gate centre
                      Used for centre-to-edge gates such as roundabout gates
    """
    name: str
    center: Point3D
    width: float = 0.45
    height: float = 0.47
    orientation_deg: float = 0.0
    line_angle_deg: float = None   # None --> orientation_deg + 90
    radial_from: tuple = None      # (x, y) - e.g. roundabout centre
    has_satellite: bool = False
    points: int = 1
    note: str = ""

    def __repr__(self):
        sat = " [SAT]" if self.has_satellite else ""
        return f"Gate({self.name}, center={self.center.xy()}, +{self.points}pt{sat})"


# ---------------------------------------------------------------------------
# BEZIER SEGMENT
# ---------------------------------------------------------------------------

@dataclass
class BezierSegment:
    """
    Cubic Bezier curve. Defined by start, end, and control points.
    plot_field_3d.py samples this segment using np.linspace.
    """
    p0: Point3D          # start
    p3: Point3D          # end
    p1: Point3D = None   # control point 1 (defaults to p0 if None)
    p2: Point3D = None   # control point 2 (defaults to p3 if None)
    z: float = 0.0       # height (0 = floor)
    n_samples: int = 60  # number of samples
    note: str = ""

    def __post_init__(self):
        if self.p1 is None:
            self.p1 = self.p0
        if self.p2 is None:
            self.p2 = self.p3

    def sample(self, n: int = None):
        """Sample the Bezier curve, returns (xs, ys, zs)."""
        import numpy as np
        n = n or self.n_samples
        t = np.linspace(0, 1, n)
        x = ((1-t)**3*self.p0.x + 3*(1-t)**2*t*self.p1.x +
             3*(1-t)*t**2*self.p2.x + t**3*self.p3.x)
        y = ((1-t)**3*self.p0.y + 3*(1-t)**2*t*self.p1.y +
             3*(1-t)*t**2*self.p2.y + t**3*self.p3.y)
        z = np.full(n, self.z)
        return x, y, z

    @property
    def start(self) -> Point3D: return self.p0

    @property
    def end(self) -> Point3D: return self.p3

    def __repr__(self):
        return f"BezierSegment({self.p0.xy()} -> {self.p3.xy()})"


# ---------------------------------------------------------------------------
# ARC SEGMENT
# ---------------------------------------------------------------------------

@dataclass
class ArcSegment:
    """
    Circular arc. Defined by centre, radius, and angle range.
    """
    center: Point3D
    radius: float
    angle_start_deg: float   # start angle
    angle_end_deg: float     # end angle
    z: float = 0.0
    n_samples: int = 60
    note: str = ""

    def sample(self, n: int = None):
        """Sample the arc, returns (xs, ys, zs)."""
        import numpy as np
        n = n or self.n_samples
        angles = np.linspace(math.radians(self.angle_start_deg),
                              math.radians(self.angle_end_deg), n)
        x = self.center.x + self.radius * np.cos(angles)
        y = self.center.y + self.radius * np.sin(angles)
        z = np.full(n, self.z)
        return x, y, z

    @property
    def start(self) -> Point3D:
        rad = math.radians(self.angle_start_deg)
        return Point3D(self.center.x + self.radius*math.cos(rad),
                       self.center.y + self.radius*math.sin(rad), self.z)

    @property
    def end(self) -> Point3D:
        rad = math.radians(self.angle_end_deg)
        return Point3D(self.center.x + self.radius*math.cos(rad),
                       self.center.y + self.radius*math.sin(rad), self.z)

    def __repr__(self):
        return (f"ArcSegment(center={self.center.xy()}, r={self.radius:.2f}, "
                f"{self.angle_start_deg:.0f}°->{self.angle_end_deg:.0f}°)")


# ---------------------------------------------------------------------------
# NAV PATH (composite route: straight + arc + bezier)
# ---------------------------------------------------------------------------

@dataclass
class NavPath:
    """
    Navigation route. Can contain multiple segments (TapeLine, ArcSegment,
    BezierSegment). The localisation module follows this route.

    task     : which competition task this relates to
    connects : which zones this route bridges
    """
    name: str
    segments: List = field(default_factory=list)
    task: str = None
    connects: List[str] = field(default_factory=list)
    color: str = 'white'
    note: str = ""

    def all_points(self) -> List[Point3D]:
        pts = []
        for seg in self.segments:
            if hasattr(seg, 'waypoints'):
                pts.extend(seg.waypoints)
            elif hasattr(seg, 'start'):
                pts.append(seg.start)
                pts.append(seg.end)
        return pts

    def segment_types(self) -> List[str]:
        """Return the types of all segments."""
        return [type(s).__name__ for s in self.segments]

    def __repr__(self):
        return f"NavPath({self.name}, {len(self.segments)} segments, task={self.task})"


# ---------------------------------------------------------------------------
# TAPE LINE
# ---------------------------------------------------------------------------

@dataclass
class TapeLine:
    """
    White tape segment. Can be straight or curved.
    Curved segments use the waypoints list.
    All tape is assumed at z=0 (floor).
    width = 0.038m (38mm, standard Tesa-4651 tape)

    task     : which competition task this relates to (e.g. 'roundabout', 'shuttle', 'ramp')
               None = general navigation tape
    connects : which zones this segment bridges ['zone_a', 'zone_b']
    """
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
        """Heading of the first segment (degrees)."""
        if len(self.waypoints) < 2:
            return 0.0
        dx = self.waypoints[1].x - self.waypoints[0].x
        dy = self.waypoints[1].y - self.waypoints[0].y
        return math.degrees(math.atan2(dy, dx))

    def __repr__(self):
        return (f"TapeLine({self.name}, "
                f"{self.start.xy()} -> {self.end.xy()}, "
                f"len={self.length:.2f}m)")


# ---------------------------------------------------------------------------
# START AREA
# ---------------------------------------------------------------------------

@dataclass
class StartArea:
    """
    U-shaped starting area.
    open_side: the side the robot exits from ('top', 'bottom', 'left', 'right')
    """
    origin: Point3D       # bottom-left corner
    width: float          # metres
    depth: float          # metres
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
        """The point where the robot exits."""
        if self.open_side == "top":
            return Point3D(self.origin.x + self.width/2,
                           self.origin.y + self.depth,
                           self.origin.z)
        elif self.open_side == "bottom":
            return Point3D(self.origin.x + self.width/2,
                           self.origin.y, self.origin.z)
        elif self.open_side == "right":
            return Point3D(self.origin.x + self.width,
                           self.origin.y + self.depth/2,
                           self.origin.z)
        else:
            return Point3D(self.origin.x,
                           self.origin.y + self.depth/2,
                           self.origin.z)


# ---------------------------------------------------------------------------
# ROUNDABOUT
# ---------------------------------------------------------------------------

@dataclass
class Roundabout:
    """
    Circular loop. Has 3 gates and 2 drones.
    Gates are positioned at the specified angles around the circle.
    """
    center: Point3D
    diameter: float          # metres (approx. 1.21m)
    gate_angles_deg: List[float] = field(default_factory=lambda: [90.0, 210.0, 330.0])
    n_drones: int = 2
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
        """Auto-generate gates."""
        gates = []
        for i, angle in enumerate(self.gate_angles_deg):
            pt = self.point_at_angle(angle, r_scale=1.0)
            pt.z = 0.025
            gates.append(Gate(
                name=f"roundabout_gate_{i+1}",
                center=pt,
                orientation_deg=angle,
                radial_from=(self.center.x, self.center.y),  # centre-to-edge line
                has_satellite=has_satellite,
                points=1,
            ))
        self.gates = gates
        return gates


# ---------------------------------------------------------------------------
# INFINITY PATH (guard robot route)
# ---------------------------------------------------------------------------

@dataclass
class InfinityPath:
    """
    Infinity shaped guard robot route.
    Two circles side by side, crossing at the centre point.
    Touching the guard robot: -1 point (max -2).
    """
    center: Point3D          # crossing point
    loop_radius: float       # radius of each loop (metres)
    guard_speed_cms: float = 30.0   # average speed (cm/s)
    penalty_per_touch: int = -1
    max_penalty: int = -2
    gates: List[Gate] = field(default_factory=list)
    note: str = ""

    @property
    def left_center(self) -> Point3D:
        """Centre of the left loop - offset in -x from the crossing point."""
        return Point3D(self.center.x - self.loop_radius,
                       self.center.y,
                       self.center.z)

    @property
    def right_center(self) -> Point3D:
        """Centre of the right loop - offset in +x from the crossing point."""
        return Point3D(self.center.x + self.loop_radius,
                       self.center.y,
                       self.center.z)

    def build_gates(self) -> List[Gate]:
        """Build left and right gates."""
        self.gates = [
            Gate("infinity_gate_left",
                 self.left_center,
                 orientation_deg=90, points=1,
                 note="Guard route left gate"),
            Gate("infinity_gate_right",
                 self.right_center,
                 orientation_deg=90, points=1,
                 note="Guard route right gate"),
        ]
        return self.gates


# ---------------------------------------------------------------------------
# RAMP
# ---------------------------------------------------------------------------

@dataclass
class Ramp:
    """
    Ramp. Pz = 0 at base, Pz = rise at top.
    pz_at(x, y): returns Pz at the given 2D position (linear interpolation).
    """
    name: str
    base: Point3D        # bottom of ramp (z=0)
    top: Point3D         # top of ramp (z=rise)
    width: float         # metres
    rise: float = 0.55   # height gained (metres)
    gates: List[Gate] = field(default_factory=list)
    note: str = ""

    @property
    def run(self) -> float:
        """Horizontal distance."""
        return self.base.dist_2d(self.top)

    @property
    def slope_deg(self) -> float:
        """Slope angle (degrees)."""
        if self.run < 1e-6:
            return 90.0
        return math.degrees(math.atan2(self.rise, self.run))

    def pz_at(self, x: float, y: float) -> float:
        """
        Returns Pz at the given (x, y) position.
        Returns 0 if outside the ramp (width or length).
        """
        dx = self.top.x - self.base.x
        dy = self.top.y - self.base.y
        run = math.hypot(dx, dy)
        if run < 1e-6:
            return 0.0
        # Unit vectors
        ux, uy = dx / run, dy / run   # along ramp axis
        vx, vy = -uy, ux              # perpendicular to ramp axis
        # Position relative to base
        px = x - self.base.x
        py = y - self.base.y
        along   = px * ux + py * uy   # distance along axis
        lateral = abs(px * vx + py * vy)  # lateral distance
        # Return 0 if outside ramp bounds
        if along < 0 or along > run or lateral > self.width / 2:
            return 0.0
        t = along / run
        return t * self.rise

    def __repr__(self):
        return (f"Ramp({self.name}, "
                f"base={self.base.xy()}, top={self.top.xy()}, "
                f"rise={self.rise}m, slope={self.slope_deg:.1f}deg)")


# ---------------------------------------------------------------------------
# STAIRS
# ---------------------------------------------------------------------------

@dataclass
class Stairs:
    """
    Staircase. Pz increases step by step.
    Gates are placed on the top and second-from-top steps.
    Gates award 2x points when climbing up.
    """
    name: str
    base: Point3D        # bottom of stairs (z=0)
    top: Point3D         # top of stairs (z=n_steps*step_h)
    width: float = 0.40  # metres
    n_steps: int = 4
    step_height: float = 0.11   # metres
    step_depth: float = 0.40    # metres
    gates: List[Gate] = field(default_factory=list)
    note: str = ""

    @property
    def total_rise(self) -> float:
        return self.n_steps * self.step_height

    @property
    def total_run(self) -> float:
        return self.base.dist_2d(self.top)

    def pz_at(self, x: float, y: float) -> float:
        """
        Returns Pz at the given (x, y) position.
        Increases step by step. Returns 0 if outside stairs.
        """
        total = self.total_run
        if total < 1e-6:
            return 0.0
        dx = self.top.x - self.base.x
        dy = self.top.y - self.base.y
        ux, uy = dx / total, dy / total
        vx, vy = -uy, ux
        px = x - self.base.x
        py = y - self.base.y
        along   = px * ux + py * uy
        lateral = abs(px * vx + py * vy)
        # Return 0 if outside stair bounds
        if along < 0 or along > total or lateral > self.width / 2:
            return 0.0
        t = along / total
        step_idx = min(int(t * self.n_steps), self.n_steps - 1)
        return step_idx * self.step_height

    def build_gates(self) -> List[Gate]:
        """Build gates on the 2nd and 4th steps."""
        total = self.total_run
        dx = (self.top.x - self.base.x) / total if total > 0 else 0
        dy = (self.top.y - self.base.y) / total if total > 0 else 0
        # Stairs progress along y axis, x is fixed
        # Gate x centre is at the middle of stair width
        cx_mid = self.base.x + self.width / 2

        gates = []
        for step_idx, pts in [(self.n_steps - 2, 1), (self.n_steps - 1, 2)]:
            t = (step_idx + 0.5) / self.n_steps
            gy = self.base.y + t * total * dy
            gz = step_idx * self.step_height
            gates.append(Gate(
                name=f"{self.name}_gate_{step_idx+1}",
                center=Point3D(cx_mid, gy, gz),
                width=self.width,
                orientation_deg=90,  # passing in y direction
                line_angle_deg=0,    # line along x
                has_satellite=False,
                points=pts,
                note=f"{'2nd from top' if pts==1 else 'Top'} step, up={pts}x points",
            ))
        self.gates = gates
        return gates


# ---------------------------------------------------------------------------
# SEESAW
# ---------------------------------------------------------------------------

@dataclass
class Seesaw:
    """
    Seesaw. Pz is dynamic - read from IMU, not stored in the map.
    Has one golf ball and one gate on it.
    The gate can only be passed by using the seesaw.
    """
    name: str
    pivot: Point3D       # balance point (fixed)
    length: float        # total length (metres)
    width: float = 0.60  # metres
    gate: Optional[Gate] = None
    golf_ball_pos: Optional[Point3D] = None  # initial ball position
    note: str = ""

    @property
    def pz(self) -> str:
        return "dynamic (IMU pitch)"

    def build_gate(self) -> Gate:
        """Build the gate at the end of the seesaw."""
        self.gate = Gate(
            name=f"{self.name}_gate",
            center=Point3D(
                self.pivot.x + self.length / 2,
                self.pivot.y,
                self.pivot.z,
            ),
            orientation_deg=0,   # passing in x direction
            line_angle_deg=0,    # line along y - seesaw extends along y
            has_satellite=False,
            points=1,
            note="Only counts when passed via the seesaw",
        )
        return self.gate


# ---------------------------------------------------------------------------
# PLATFORM
# ---------------------------------------------------------------------------

@dataclass
class Platform:
    """
    Elevated platform (top of ramp/stairs).
    pz = 0.55m (fixed).
    Has a hole (golf ball target) and one golf ball on top.
    Connected to the ramp and stairs.
    """
    name: str
    origin: Point3D      # bottom-left corner (z = platform height)
    width: float         # metres
    depth: float         # metres
    pz: float = 0.55     # height above floor (metres)
    hole: Optional[Point3D] = None          # golf ball hole
    golf_ball: Optional[Point3D] = None     # ball sitting on platform
    connections: List[str] = field(default_factory=list)  # 'ramp', 'stairs'
    note: str = ""

    @property
    def center(self) -> Point3D:
        return Point3D(
            self.origin.x + self.width / 2,
            self.origin.y + self.depth / 2,
            self.pz,
        )

    def contains_2d(self, x: float, y: float) -> bool:
        return (self.origin.x <= x <= self.origin.x + self.width and
                self.origin.y <= y <= self.origin.y + self.depth)


# ---------------------------------------------------------------------------
# SORTING CENTER
# ---------------------------------------------------------------------------

@dataclass
class SortingCenter:
    """
    ABCD sorting area. X-shaped, 4 zones.
    Each zone has two ArUco markers on either side (8 markers total, IDs 10-17).
    Height ~25cm.
    """
    center: Point3D
    zone_size: float = 0.25     # each zone at least 25x25cm
    pz: float = 0.25            # platform height
    zones: Dict[str, Point3D] = field(default_factory=dict)
    aruco_markers: List[ArucoMarker] = field(default_factory=list)
    note: str = ""

    def build_zones(self) -> Dict[str, Point3D]:
        """Build zones A, B, C, D."""
        d = self.zone_size
        self.zones = {
            "A": Point3D(self.center.x + d/2, self.center.y,       self.pz),
            "B": Point3D(self.center.x,       self.center.y + d/2, self.pz),
            "C": Point3D(self.center.x - d/2, self.center.y,       self.pz),
            "D": Point3D(self.center.x,       self.center.y - d/2, self.pz),
        }
        return self.zones

    def build_aruco(self, marker_size: float = 0.10) -> List[ArucoMarker]:
        """
        Build 8 ArUco markers (IDs 10-17).
        Counter-clockwise from A quadrant, at diamond edge midpoints.
        """
        d = self.zone_size * 0.6
        positions = [
            (10, self.center.x + d,    self.center.y,        0.0,   "A right"),
            (11, self.center.x + d/2,  self.center.y + d,   45.0,   "AB top-right"),
            (12, self.center.x - d/2,  self.center.y + d,  135.0,   "B top-left"),
            (13, self.center.x - d,    self.center.y,       180.0,   "BC left"),
            (14, self.center.x - d/2,  self.center.y - d,  225.0,   "C bottom-left"),
            (15, self.center.x + d/2,  self.center.y - d,  315.0,   "CD bottom-right"),
            (16, self.center.x + d,    self.center.y,         0.0,   "D right"),
            (17, self.center.x,        self.center.y + d/2,  90.0,   "DA top"),
        ]
        self.aruco_markers = [
            ArucoMarker(
                id=mid,
                position=Point3D(px, py, self.pz),
                size_m=marker_size,
                facing_deg=fdeg,
                note=note,
            )
            for mid, px, py, fdeg, note in positions
        ]
        return self.aruco_markers

    def get_zone(self, name: str) -> Optional[Point3D]:
        return self.zones.get(name.upper())


# ---------------------------------------------------------------------------
# TIME EXTENSION BUTTON
# ---------------------------------------------------------------------------

@dataclass
class TimeExtButton:
    """
    +90 second button. Extends time when triggered.
    Each button can only be used once.
    """
    name: str
    center: Point3D
    extra_sec: int = 90
    trigger: str = "bump"   # 'bump' or 'optical'
    note: str = ""


# ---------------------------------------------------------------------------
# LUGGAGE
# ---------------------------------------------------------------------------

@dataclass
class Luggage:
    """
    Cargo box. Has an ArUco marker (on all 6 faces).
    Transported on the shuttle.
    """
    aruco_id: int           # 20 or 53
    size_m: float = 0.06    # 6cm cube
    target_zone: str = ""   # target zone ('A' or 'D')
    points_correct: int = 4 # correct zone: 4 or 2 points
    points_any: int = 1     # any zone: 1 point
    note: str = ""

    def __repr__(self):
        return f"Luggage(id={self.aruco_id}, target={self.target_zone}, +{self.points_correct}pt)"


# ---------------------------------------------------------------------------
# LUGGAGE SHUTTLE
# ---------------------------------------------------------------------------

@dataclass
class LuggageShuttle:
    """
    Moving cargo carrier. Travels back and forth.
    Has an ArUco ID5 marker and 2 luggage items on board.
    """
    path_start: Point3D
    path_end: Point3D
    speed_cms: float = 20.0   # cm/s
    surface_height_m: float = 0.13  # height above floor
    aruco_id: int = 5         # shuttle's own marker
    luggage: List[Luggage] = field(default_factory=list)
    note: str = ""

    @property
    def path_length(self) -> float:
        return self.path_start.dist_2d(self.path_end)

    def build_luggage(self) -> List[Luggage]:
        self.luggage = [
            Luggage(aruco_id=20, target_zone="A", points_correct=4,
                    note="Luggage 20 -> zone A"),
            Luggage(aruco_id=53, target_zone="D", points_correct=2,
                    note="Luggage 53 -> zone D"),
        ]
        return self.luggage


# ---------------------------------------------------------------------------
# BALL DISPENSER
# ---------------------------------------------------------------------------

@dataclass
class Ball:
    color: str         # 'red', 'blue', 'white'
    target_zone: str   # 'B', 'C', or any
    points: int
    diameter_m: float = 0.043


@dataclass
class BallDispenser:
    """
    Ball bowl. Pressing it releases 5 balls.
    1 red, 1 blue, 3 white.
    """
    position: Point3D
    trigger_height_m: float = 0.18   # activation height
    balls: List[Ball] = field(default_factory=list)
    note: str = ""

    def build_balls(self) -> List[Ball]:
        self.balls = [
            Ball("blue",  "C", points=4),
            Ball("red",   "B", points=3),
            Ball("white", "",  points=1),
            Ball("white", "",  points=1),
            Ball("white", "",  points=1),
        ]
        return self.balls


# ---------------------------------------------------------------------------
# GOLF BALL
# ---------------------------------------------------------------------------

@dataclass
class GolfBall:
    """
    Golf ball. Awards 2 points when sunk in the hole.
    """
    position: Point3D
    diameter_m: float = 0.043
    color: str = "orange"
    target_hole: Optional[Point3D] = None
    points: int = 2
    note: str = ""