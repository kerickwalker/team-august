#!/usr/bin/env python3
"""
smeasurement.py
=============================================================================
Vision measurement package for the Kalman filter (EKF).

Takes the output of sline.process() and the robot's current pose estimate,
and produces a structured measurement dict for the EKF update step.

What sline sees (robot frame):
    - line_points_robot : [(X, Y), ...] ground points relative to robot
    - line_offset       : lateral offset (m), + = line is left of robot
    - line_heading      : line angle (rad) relative to robot heading

What smeasurement produces (world frame):
    - line_offset, line_heading : direct error signal for EKF py/yaw correction
    - world_line_points         : line points in world coordinates
    - fork_candidate/confirmed  : fork state for EKF uncertainty management
    - valid                     : whether this frame is reliable

Usage:
    from sensors.fusion.smeasurement import build_measurement

    result = vision.process(frame)       # sline
    meas   = build_measurement(result, pose)

    if meas["valid"]:
        kalman.update(meas)

Coordinate conventions:
    Robot frame : X = forward, Y = left, Z = up
    World frame : same axes, rotated by yaw, translated by (px, py)

Output dict keys:
    valid              bool
    line_offset        float   (m), + = line is left
    line_heading       float   (rad) relative to robot
    line_curvature     float   (1/m)
    world_line_points  list    [(wx, wy), ...] in world frame
    fork_evidence      int
    fork_candidate     bool    increase EKF uncertainty
    fork_confirmed     bool    activate branch selection
    fork_detected      bool    alias for fork_confirmed
    active_branch      int     0 = left, 1 = right
    branches           list    per-branch dicts from sline
=============================================================================
"""

import math
import numpy as np


def _robot_to_world(points_robot, px, py, yaw):
    """
    Transform (X, Y) points from robot frame to world frame.

    Robot frame : X = forward, Y = left
    World frame : rotated by yaw, translated by (px, py)
    """
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    world_pts = []
    for x_r, y_r in points_robot:
        wx = px + cos_y * x_r - sin_y * y_r
        wy = py + sin_y * x_r + cos_y * y_r
        world_pts.append((wx, wy))
    return world_pts


def build_measurement(vision_result: dict, pose) -> dict:
    """
    Build a Kalman measurement dict from sline output + current pose.

    Parameters
    ----------
    vision_result : dict
        Output of sline.process(frame).

    pose : SPose object, tuple/list (px, py, yaw), or dict {"px", "py", "yaw"}

    Returns
    -------
    dict — see module docstring for keys
    """
    # --- Unpack pose ----------------------------------------------------------
    if isinstance(pose, (list, tuple)):
        px  = float(pose[0])
        py  = float(pose[1])
        yaw = float(pose[2]) if len(pose) > 2 else 0.0
    elif isinstance(pose, dict):
        px  = float(pose.get("px",  pose.get("pos_x", 0.0)))
        py  = float(pose.get("py",  pose.get("pos_y", 0.0)))
        yaw = float(pose.get("yaw", 0.0))
    else:
        # SPose object
        px  = float(pose.pose[0])
        py  = float(pose.pose[1])
        yaw = float(pose.pose[2])

    # --- Unpack vision result -------------------------------------------------
    line_valid    = bool(vision_result.get("line_valid",      False))
    line_offset   = float(vision_result.get("line_offset",    0.0))
    line_heading  = float(vision_result.get("line_heading",   0.0))
    line_curv     = float(vision_result.get("line_curvature", 0.0))
    pts_robot     = vision_result.get("line_points_robot",    [])

    fork_evidence = int(vision_result.get("fork_evidence",    0))
    fork_cand     = bool(vision_result.get("fork_candidate",  False))
    fork_conf     = bool(vision_result.get("fork_confirmed",  False))
    fork_det      = bool(vision_result.get("fork_detected",   False))
    active_branch = int(vision_result.get("active_branch",    0))
    branches      = vision_result.get("branches",             [{}, {}])

    # --- Transform line points to world frame --------------------------------
    world_pts = _robot_to_world(pts_robot, px, py, yaw) if pts_robot else []

    # --- Validity check ------------------------------------------------------
    valid = line_valid and len(pts_robot) >= 3

    return {
        "valid":             valid,
        "line_offset":       line_offset,
        "line_heading":      line_heading,
        "line_curvature":    line_curv,
        "world_line_points": world_pts,
        "fork_evidence":     fork_evidence,
        "fork_candidate":    fork_cand,
        "fork_confirmed":    fork_conf,
        "fork_detected":     fork_det,
        "active_branch":     active_branch,
        "branches":          branches,
        "_pose_px":          px,
        "_pose_py":          py,
        "_pose_yaw":         yaw,
    }


def build_measurement_vector(vision_result: dict, pose) -> np.ndarray:
    """
    Compact 4x1 numpy vector for direct EKF use.

        z[0] = line_offset   (m)   -> py correction
        z[1] = line_heading  (rad) -> yaw correction
        z[2] = nearest world X (m) -> absolute X constraint
        z[3] = nearest world Y (m) -> absolute Y constraint

    Returns None if measurement is not valid.
    """
    m = build_measurement(vision_result, pose)
    if not m["valid"]:
        return None

    wx, wy = m["world_line_points"][0] if m["world_line_points"] else (0.0, 0.0)

    return np.array([
        [m["line_offset"]],
        [m["line_heading"]],
        [wx],
        [wy],
    ], dtype=float)