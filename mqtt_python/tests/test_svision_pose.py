"""Self-test: does svision_pose correctly invert known geometry?"""
import sys, math, os
sys.path.insert(0, '/mnt/project')
sys.path.insert(0, '/home/claude/work')

# Stub out the field map with a simple horizontal tape along +X at y=1.0
class FakeMarker:
    def __init__(self, mid, x, y):
        self.id = mid
        class P: pass
        p = P(); p.x = x; p.y = y; p.z = 0
        self.position = p

class FakeField:
    def all_aruco(self):
        return [FakeMarker(25, 5.0, 2.0)]
    def pz_at(self, x, y):
        return 0.0
    def nearest_tape_segment(self, px, py, yaw=None, world_line_points=None,
                              max_yaw_diff_deg=60.0):
        # one segment from (0, 1) to (6, 1) — tape runs along +X at y=1
        return {
            'name':         'test_tape',
            'p0':           (0.0, 1.0),
            'p1':           (6.0, 1.0),
            'dist':         0.0,  # placeholder; we set per-test below
            'heading_deg':  0.0,
            'lateral_error': 0.0,
            'score':        0.0,
        }

# Inject fake module so svision_pose can `from field_map_2026 import FIELD`
import types
fake_mod = types.ModuleType('field_map_2026')
fake_mod.FIELD = FakeField()
sys.modules['field_map_2026'] = fake_mod

from perception.pose.svision_pose import SVisionPose, _wrap_rad

def approx(a, b, tol=0.01):
    return abs(a - b) < tol

def test(name, cond, detail=""):
    mark = "OK  " if cond else "FAIL"
    print(f"  [{mark}] {name}  {detail}")
    return cond


print("=== TAPE FIX TESTS ===")
vp = SVisionPose()
vp.setup()

# Ground truth robot: at (3.0, 0.7), yaw = 0 (facing +X along tape direction)
# Tape is at y=1.0, so tape is 0.3 m to the LEFT of robot (line_offset = +0.3).
# Camera sees the tape parallel to its heading → line_heading = 0.
# Prior (from Kalman) is slightly off: (2.95, 0.6, yaw=0.05).
line_result = {
    'line_valid':   True,
    'line_offset':  +0.30,   # tape is left of robot by 0.3 m
    'line_heading': 0.0,     # tape parallel to robot heading
    'world_line_points': None,
}
# Make the fake field return dist corresponding to this prior
fake_mod.FIELD.nearest_tape_segment = lambda px, py, yaw=None, world_line_points=None, max_yaw_diff_deg=60.0: {
    'name': 'test_tape',
    'p0': (0.0, 1.0), 'p1': (6.0, 1.0),
    'dist': 0.4,  # well within threshold
    'heading_deg': 0.0,
    'lateral_error': 0.0,
    'score': 0.0,
}

out = vp.update(
    aruco_detections=[],
    line_result=line_result,
    px_kalman=2.95, py_kalman=0.60, yaw_kalman=0.05,
    pitch_imu=0.0,
)

print(f"  returned: valid={out['valid']} src={out['source']}")
print(f"  x={out['x']:.3f} y={out['y']:.3f} yaw={out['yaw']:.3f}")

# Expected recovery:
# along = projection of prior onto tape = 2.95 (prior x on tape at y=1)
# anchor = (2.95, 1.0)
# perp world: tape normal is (0, 1) → "left of tape"
#   robot is 0.3 m to the RIGHT → world y = 1.0 + (-0.3)*1 = 0.7  ✓
# yaw: seg_heading 0, cam_heading 0 → yaw_forward = 0 ✓
test("valid=True",              out['valid'])
test("source is tape",          out['source'] == 'tape')
test("along-track x",           approx(out['x'], 2.95))
test("lateral snapped to tape", approx(out['y'], 0.70))
test("yaw recovered",           approx(out['yaw'], 0.0, 0.05))

# --- Second test: robot facing -X (backward along tape) ---
# Ground truth: (3.0, 1.3, yaw=pi). Tape is 0.3 m to the RIGHT of heading now.
# (robot looking -X, tape at y=1.3 → moving to y=1 means moving in robot's LEFT)
# line_offset should be -0.3? Let's reason:
#   robot frame: X forward (world -X), Y left (world +Y from robot view? no,
#   if robot yaw=pi, robot's +Y (left) is world -Y).
#   Tape is at world y=1.0, robot at world y=1.3.
#   Robot's left axis in world = (-sin(pi), cos(pi)) = (0, -1).
#   Tape displacement from robot in world = (_, 1.0-1.3) = (_, -0.3).
#   Lateral in robot frame = projection onto left axis = (-0.3)*(-1) = +0.3.
#   So line_offset = +0.3 (tape is to the LEFT in robot frame).
line_result2 = {
    'line_valid': True, 'line_offset': +0.30,
    'line_heading': 0.0,   # tape appears parallel to heading
    'world_line_points': None,
}
out2 = vp.update(
    aruco_detections=[],
    line_result=line_result2,
    px_kalman=3.05, py_kalman=1.35, yaw_kalman=math.pi - 0.05,
    pitch_imu=0.0,
)
print(f"  [backward] x={out2['x']:.3f} y={out2['y']:.3f} yaw={out2['yaw']:.3f}")
test("backward along-x",   approx(out2['x'], 3.05))
test("backward lateral",   approx(out2['y'], 1.30))
test("backward yaw ≈ pi",  approx(abs(_wrap_rad(out2['yaw'] - math.pi)), 0.0, 0.05))


print("\n=== ARUCO FIX TEST ===")
# Marker is at (5.0, 2.0). Robot is at ground truth (3.0, 2.0, yaw=0).
# Then the marker is at range 2.0, bearing 0 (straight ahead).
vp2 = SVisionPose()
vp2.setup()
out3 = vp2.update(
    aruco_detections=[{'id': 25, 'range': 2.0, 'bearing': 0.0}],
    line_result=None,
    px_kalman=2.95, py_kalman=2.05, yaw_kalman=0.05,
    pitch_imu=0.0,
)
print(f"  x={out3['x']:.3f} y={out3['y']:.3f} yaw={out3['yaw']:.3f} src={out3['source']}")
test("aruco source",        out3['source'] == 'aruco')
test("aruco recovers x",    approx(out3['x'], 3.0, 0.05))
test("aruco recovers y",    approx(out3['y'], 2.0, 0.05))
test("aruco recovers yaw",  approx(out3['yaw'], 0.0, 0.05))

# Marker at range 1.5, bearing +30° (marker is left-front). Ground truth:
# robot at some (x, y, yaw) such that marker is 1.5 m @ +30° from robot.
# Place robot yaw=0, bearing +30° means marker direction in world is +30°.
# Marker position = (5, 2). So robot = (5 - 1.5*cos(30°), 2 - 1.5*sin(30°))
#                        = (5 - 1.299, 2 - 0.75) = (3.701, 1.25)
import math
mx, my = 5.0, 2.0
b = math.radians(30)
r = 1.5
gt_x = mx - r * math.cos(b)
gt_y = my - r * math.sin(b)
print(f"  ground truth: ({gt_x:.3f}, {gt_y:.3f}, yaw=0)")
vp3 = SVisionPose()
vp3.setup()
out4 = vp3.update(
    aruco_detections=[{'id': 25, 'range': 1.5, 'bearing': b}],
    line_result=None,
    px_kalman=gt_x + 0.05, py_kalman=gt_y - 0.04, yaw_kalman=0.02,
    pitch_imu=0.0,
)
print(f"  x={out4['x']:.3f} y={out4['y']:.3f} yaw={out4['yaw']:.3f}")
test("aruco offset x",     approx(out4['x'], gt_x, 0.05))
test("aruco offset y",     approx(out4['y'], gt_y, 0.05))


print("\n=== VELOCITY / OMEGA TEST ===")
import time as t_mod
vp4 = SVisionPose()
vp4.setup()
# Feed 5 successive fixes along the tape moving +X at 0.5 m/s, no rotation.
# Use the tape path to drive fixes.
for i in range(5):
    line_result_i = {
        'line_valid': True, 'line_offset': 0.0,
        'line_heading': 0.0, 'world_line_points': None,
    }
    # Force the fake field's segment dist to be small each time
    prior_x = 1.0 + i * 0.05   # moving in +X at 50 mm per frame
    out_i = vp4.update(
        aruco_detections=[],
        line_result=line_result_i,
        px_kalman=prior_x, py_kalman=1.0, yaw_kalman=0.0,
        pitch_imu=0.0,
    )
    t_mod.sleep(0.1)   # 100 ms between frames → 0.5 m/s

print(f"  final vel={out_i['velocity']:.3f} m/s  omega={out_i['omega']:.3f} rad/s")
test("velocity ~0.5 m/s (loosely)", 0.3 < out_i['velocity'] < 0.7,
     f"got {out_i['velocity']:.3f}")
test("omega ~0",                     abs(out_i['omega']) < 0.1,
     f"got {out_i['omega']:.3f}")

print("\ndone")