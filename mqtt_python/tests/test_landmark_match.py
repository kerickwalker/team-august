#!/usr/bin/env python3
"""
test_landmark_match.py
=============================================================================
Verify the landmark matcher without a camera.

For each test case we hand-craft a line_result dict that mimics what
sline.SLine.process() would produce, plus a Kalman prior, and check that:

    - When the prior is at the right landmark and the signature matches,
      the matcher fires with the correct (x, y, name).
    - When the prior is far from any landmark, no match is produced.
    - When the prior is at a landmark but the line signature doesn't match,
      no match is produced.
    - Cooldown prevents the same landmark from firing twice in quick succession.

Run:
    python test_landmark_match.py
"""
from __future__ import annotations
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, os.pardir)))


def _fork_line_result(angle_sep_deg: float, valid: bool = True) -> dict:
    """Synthesize a line_result that looks like a Y/T fork with the given
    angular separation between branches."""
    half = math.radians(angle_sep_deg / 2.0)
    return {
        "line_valid":     valid,
        "line_points_robot": [(0.5, 0.0), (1.0, 0.0)],
        "line_offset":    0.0,
        "line_heading":   0.0,
        "line_curvature": 0.0,
        "fork_evidence":  20,
        "fork_candidate": True,
        "fork_confirmed": True,
        "fork_detected":  True,
        "active_branch":  0,
        "branches": [
            {"valid": True, "points": [(0.5, +0.05), (1.0, +0.30)],
             "offset": 0.05, "heading": +half, "curvature": 0.0},
            {"valid": True, "points": [(0.5, -0.05), (1.0, -0.30)],
             "offset": -0.05, "heading": -half, "curvature": 0.0},
        ],
    }


def _no_fork_line_result() -> dict:
    return {
        "line_valid":     True,
        "line_points_robot": [(0.5, 0.0), (1.0, 0.0)],
        "line_offset":    0.0,
        "line_heading":   0.0,
        "line_curvature": 0.0,
        "fork_evidence":  0,
        "fork_candidate": False,
        "fork_confirmed": False,
        "fork_detected":  False,
        "active_branch":  0,
        "branches": [
            {"valid": True, "points": [(0.5, 0.0), (1.0, 0.0)],
             "offset": 0.0, "heading": 0.0, "curvature": 0.0},
            {"valid": False, "points": [],
             "offset": 0.0, "heading": 0.0, "curvature": 0.0},
        ],
    }


def _endpoint_line_result(farthest_x: float) -> dict:
    return {
        "line_valid":     True,
        "line_points_robot": [(0.2, 0.0), (farthest_x, 0.0)],
        "line_offset":    0.0,
        "line_heading":   0.0,
        "line_curvature": 0.0,
        "fork_evidence":  0,
        "fork_candidate": False,
        "fork_confirmed": False,
        "fork_detected":  False,
        "active_branch":  0,
        "branches": [
            {"valid": True, "points": [(0.2, 0.0), (farthest_x, 0.0)],
             "offset": 0.0, "heading": 0.0, "curvature": 0.0},
            {"valid": False, "points": [],
             "offset": 0.0, "heading": 0.0, "curvature": 0.0},
        ],
    }


def run() -> int:
    from perception.pose.slandmark_match import SLandmarkMatcher
    from field_map.field_map_2026 import FIELD

    by_name = {lm.name: lm for lm in FIELD.landmarks}
    print(f"FIELD has {len(FIELD.landmarks)} landmarks: {sorted(by_name)}")
    start_fork = by_name["start_fork"]
    rab = by_name["roundabout_north_entry"]
    dispenser = by_name["ball_dispenser"]

    cases = []
    fail = 0
    matcher = SLandmarkMatcher()
    matcher.setup()

    def expect(label, fn, expected_name):
        nonlocal fail
        m = fn()
        ok = (m is None and expected_name is None) or \
             (m is not None and expected_name is not None and m["name"] == expected_name)
        verdict = "PASS" if ok else "FAIL"
        if not ok:
            fail += 1
        got = m["name"] if m else "None"
        conf = f"  conf={m['confidence']:.2f}  why={m['signature_match']!r}" if m else ""
        print(f"[{verdict}] {label:<55}  expected={expected_name}  got={got}{conf}")

    # 1. Prior at start_fork, fork visible with ~90° separation → should match start_fork
    t = time.time()
    expect("at start_fork, 90° fork visible",
           lambda: matcher.process(_fork_line_result(90.0),
                                   (start_fork.position.x, start_fork.position.y, math.radians(90)),
                                   t_now=t),
           "start_fork")

    # 2. Prior at start_fork but no fork → no match
    expect("at start_fork, no fork detected",
           lambda: matcher.process(_no_fork_line_result(),
                                   (start_fork.position.x, start_fork.position.y, math.radians(90)),
                                   t_now=t + 5.0),
           None)

    # 3. Prior far from any landmark, fork visible → no match (gate filters out)
    expect("far from any landmark, fork visible",
           lambda: matcher.process(_fork_line_result(90.0),
                                   (0.0, 0.0, 0.0),
                                   t_now=t + 10.0),
           None)

    # 4. Prior at roundabout_north_entry, 90° T-junction visible → matches that
    expect("at roundabout_north_entry, T-junction",
           lambda: matcher.process(_fork_line_result(90.0),
                                   (rab.position.x, rab.position.y, math.radians(270)),
                                   t_now=t + 15.0),
           "roundabout_north_entry")

    # 5. Cooldown: re-fire same landmark immediately after match → blocked
    matcher2 = SLandmarkMatcher()
    matcher2.setup()
    t2 = time.time()
    matcher2.process(_fork_line_result(90.0),
                     (start_fork.position.x, start_fork.position.y, math.radians(90)),
                     t_now=t2)
    expect("cooldown blocks immediate re-fire of start_fork",
           lambda: matcher2.process(_fork_line_result(90.0),
                                    (start_fork.position.x, start_fork.position.y, math.radians(90)),
                                    t_now=t2 + 0.3),
           None)

    # 6. Prior at ball_dispenser, tape ends short → matches fixture_anchor
    expect("at ball_dispenser, tape ends at 0.4 m",
           lambda: matcher.process(_endpoint_line_result(0.4),
                                   (dispenser.position.x, dispenser.position.y, math.radians(270)),
                                   t_now=t + 30.0),
           "ball_dispenser")

    # 7. Prior at ball_dispenser, tape continues far → no match
    expect("at ball_dispenser, tape continues to 2.0 m",
           lambda: matcher.process(_endpoint_line_result(2.0),
                                   (dispenser.position.x, dispenser.position.y, math.radians(270)),
                                   t_now=t + 35.0),
           None)

    # 8. Prior at start_fork but fork at 30° (wrong angle) → no match
    expect("at start_fork, fork separation wrong (30°)",
           lambda: matcher.process(_fork_line_result(30.0),
                                   (start_fork.position.x, start_fork.position.y, math.radians(90)),
                                   t_now=t + 40.0),
           None)

    print(f"\n{'PASSED' if fail == 0 else 'FAILED'} — {fail} failures")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    sys.exit(run())
