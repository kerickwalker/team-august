#!/usr/bin/env python3
"""
test_aruco_world_fix.py
=============================================================================
Verify the ArUco → robot-world-pose pipeline end-to-end without a camera.

Strategy
--------
1. Place the robot at a known world pose (x, y, yaw).
2. Place an ArUco marker at a known world pose (position + facing).
3. *Synthesize* what the camera would see:
       - compute the marker pose in the robot frame analytically
       - run it through the inverse of the camera extrinsics to get the
         pose the camera would report (tvec, R_marker_cam)
4. Hand the synthetic detection to SVisionPose._aruco_fix and check that
   the recovered (x, y, yaw) matches the ground truth within < 5 mm / 0.1°.

If all three test cases pass, the chain
    detector → extrinsics → world-pose recovery
is mathematically consistent.
"""
from __future__ import annotations
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, os.pardir)))

from perception.camera import scamera_extrinsics
from perception.pose.svision_pose import SVisionPose


def _Rz(theta_rad: float) -> np.ndarray:
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def synth_detection(robot_xy_yaw, marker_xyz, marker_facing_deg, extrinsics):
    """Build a saruco-style detection dict for the given ground-truth scene."""
    rx, ry, ryaw = robot_xy_yaw
    R_world_rob = _Rz(ryaw)
    T_world_rob = np.array([rx, ry, 0.0])

    # Ground-truth marker pose in world frame.
    T_world_marker = np.asarray(marker_xyz, dtype=np.float64)
    R_world_marker = SVisionPose._marker_world_rotation(marker_facing_deg)

    # Marker pose in robot frame:
    T_marker_rob = R_world_rob.T @ (T_world_marker - T_world_rob)
    R_marker_rob = R_world_rob.T @ R_world_marker

    # Marker pose in camera frame (invert the extrinsics):
    Rc = extrinsics.R_cam2rob
    Tc = extrinsics.T_cam2rob
    tvec = Rc.T @ (T_marker_rob - Tc)
    R_marker_cam = Rc.T @ R_marker_rob

    rng_rob = float(np.hypot(T_marker_rob[0], T_marker_rob[1]))
    return {
        "id": 99,
        "range":        rng_rob,
        "bearing":      math.atan2(T_marker_rob[1], T_marker_rob[0]),
        "u": 0.0, "v": 0.0,
        "rvec":         np.zeros(3),     # not used downstream
        "tvec":         tvec,            # camera-frame translation (m)
        "T_marker_rob": T_marker_rob,
        "R_marker_rob": R_marker_rob,
    }


class _StubField:
    """Stand-in for FIELD that exposes a single marker the test cares about."""
    def __init__(self, marker_id, x, y, z, facing_deg):
        from types import SimpleNamespace
        self._m = SimpleNamespace(
            id=marker_id,
            position=SimpleNamespace(x=x, y=y, z=z),
            facing_deg=facing_deg,
        )

    def all_aruco(self):
        return [self._m]

    def pz_at(self, x, y):
        return 0.0


def _check(label, expected, got, tol_pos=0.005, tol_yaw_deg=0.1):
    ex, ey, eyaw_deg = expected
    gx, gy, gyaw = got['x'], got['y'], got['yaw']
    gyaw_deg = math.degrees(gyaw)
    dpos = math.hypot(gx - ex, gy - ey)
    dyaw = abs(((gyaw_deg - eyaw_deg + 180) % 360) - 180)
    ok = dpos < tol_pos and dyaw < tol_yaw_deg
    print(f"[{'PASS' if ok else 'FAIL'}] {label}")
    print(f"        expected ({ex:.3f}, {ey:.3f}, {eyaw_deg:.2f}°)")
    print(f"        got      ({gx:.3f}, {gy:.3f}, {gyaw_deg:.2f}°)")
    print(f"        Δpos={dpos*1000:.2f} mm   Δyaw={dyaw:.3f}°")
    return ok


def run() -> int:
    extrinsics_cases = [
        ("identity-mount (ROS axis swap only)",
         scamera_extrinsics.CameraExtrinsics(
             pitch_deg=0, roll_deg=0, yaw_deg=0,
             tx_m=0.0, ty_m=0.0, tz_m=0.0)),
        ("realistic mount: 12° pitch, 5 cm forward, 22 cm up",
         scamera_extrinsics.CameraExtrinsics(
             pitch_deg=12, roll_deg=0, yaw_deg=0,
             tx_m=0.05, ty_m=0.0, tz_m=0.22)),
    ]

    scene_cases = [
        # robot pose,            marker xyz,          facing_deg, label
        ((0.0, 0.0, 0.0),         (2.0, 0.0, 0.10),    180.0, "robot at origin facing east, marker east facing west"),
        ((1.0, 0.5, math.pi/2),   (1.0, 2.5, 0.10),    270.0, "robot facing north, marker north facing south"),
        ((1.8, 1.10,-math.pi/2),  (1.8, 0.80, 0.10),    90.0, "sorting-zone-like setup, robot facing south"),
    ]

    n_pass = 0
    n_total = 0
    for ext_label, ext in extrinsics_cases:
        print(f"\n=== Extrinsics: {ext_label} ===")
        pose = SVisionPose()
        pose._field = _StubField(99, 0, 0, 0, 0)   # will be overwritten per scene
        pose._ready = True

        for robot_pose, marker_xyz, facing, scene_label in scene_cases:
            pose._field = _StubField(99, *marker_xyz, facing)
            det = synth_detection(robot_pose, marker_xyz, facing, ext)
            fix = pose._aruco_fix(
                [det],
                px_prior=robot_pose[0], py_prior=robot_pose[1],
                yaw_prior=robot_pose[2],
            )
            n_total += 1
            if fix is None:
                print(f"[FAIL] {scene_label}: _aruco_fix returned None")
                continue
            if _check(scene_label,
                      (robot_pose[0], robot_pose[1], math.degrees(robot_pose[2])),
                      fix):
                n_pass += 1

    print(f"\n{n_pass}/{n_total} passed")
    return 0 if n_pass == n_total else 1


if __name__ == "__main__":
    sys.exit(run())
