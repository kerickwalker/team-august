#!/usr/bin/env python3
"""
scamera_extrinsics.py
=============================================================================
Camera-to-robot extrinsic transform.

Loads (or constructs) the rigid transform that takes a 3D point expressed in
the camera optical frame and re-expresses it in the robot body frame.

Frame conventions
-----------------
Robot body frame (right-handed):
    +X = forward,  +Y = left,  +Z = up

Camera optical frame (OpenCV / cv2.solvePnP convention, right-handed):
    +X = right,    +Y = down,  +Z = forward

The transform is split into two parts:

    1. AXIS_SWAP — the fixed permutation between the two frame conventions:
           cam +X (right)   →  rob -Y
           cam +Y (down)    →  rob -Z
           cam +Z (forward) →  rob +X

    2. MOUNT — small Euler rotation (pitch / roll / yaw) that accounts for
       how the camera is *physically* tilted on the robot, plus the
       translation offset of the camera optical centre relative to robot
       origin (centre of wheelbase, ground level).

Final transform applied to a marker translation:
    p_rob = R_cam2rob @ p_cam + T_cam2rob,   R_cam2rob = R_mount @ AXIS_SWAP

Source of truth
---------------
The 5 (optionally 6) mount parameters live in
    calibration/camera_extrinsics.py
as plain Python constants — readable, diffable, and *already consumed* by
sground.py. This module imports those constants so we never duplicate the
calibration value across files.

Edit `calibration/camera_extrinsics.py` directly, or use
    calibration/build_camera_extrinsics.py --pitch ... --tz ...
to regenerate it.
"""
from __future__ import annotations
import math
import os
import sys
from typing import Optional

import numpy as np


# Fixed axis swap: OpenCV camera frame  →  robot frame.
AXIS_SWAP: np.ndarray = np.array([
    [ 0.0,  0.0,  1.0],   # rob +X = cam +Z (forward)
    [-1.0,  0.0,  0.0],   # rob +Y = cam -X (left)
    [ 0.0, -1.0,  0.0],   # rob +Z = cam -Y (up)
], dtype=np.float64)


# Hard fall-back used only when calibration/camera_extrinsics.py is missing
# AND no override is supplied. Same numbers sground.py falls back to.
_FALLBACK = dict(pitch_deg=25.0, roll_deg=0.0, yaw_deg=0.0,
                 tx_m=0.02, ty_m=-0.005, tz_m=0.185)


def _euler_to_matrix(pitch_rad: float, roll_rad: float, yaw_rad: float) -> np.ndarray:
    """ZYX intrinsic rotation: R = Rz(yaw) · Ry(pitch) · Rx(roll)."""
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cr, sr = math.cos(roll_rad),  math.sin(roll_rad)
    cy, sy = math.cos(yaw_rad),   math.sin(yaw_rad)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return Rz @ Ry @ Rx


class CameraExtrinsics:
    """Holds R_cam2rob, T_cam2rob and the raw mount parameters that produced them."""
    __slots__ = (
        "pitch_deg", "roll_deg", "yaw_deg",
        "tx_m", "ty_m", "tz_m",
        "R_cam2rob", "T_cam2rob",
        "source",
    )

    def __init__(self, pitch_deg: float = 0.0, roll_deg: float = 0.0, yaw_deg: float = 0.0,
                 tx_m: float = 0.0, ty_m: float = 0.0, tz_m: float = 0.0,
                 source: str = "default"):
        self.pitch_deg = float(pitch_deg)
        self.roll_deg  = float(roll_deg)
        self.yaw_deg   = float(yaw_deg)
        self.tx_m      = float(tx_m)
        self.ty_m      = float(ty_m)
        self.tz_m      = float(tz_m)
        self.source    = source

        R_mount = _euler_to_matrix(
            math.radians(self.pitch_deg),
            math.radians(self.roll_deg),
            math.radians(self.yaw_deg),
        )
        self.R_cam2rob = R_mount @ AXIS_SWAP
        self.T_cam2rob = np.array([self.tx_m, self.ty_m, self.tz_m], dtype=np.float64)

    def transform_point(self, p_cam: np.ndarray) -> np.ndarray:
        """Camera-frame point (3,) [m]  →  robot-frame point (3,) [m]."""
        return self.R_cam2rob @ p_cam + self.T_cam2rob

    def transform_rotation(self, R_in_cam: np.ndarray) -> np.ndarray:
        """Camera-frame rotation matrix  →  robot-frame rotation matrix."""
        return self.R_cam2rob @ R_in_cam

    def __repr__(self) -> str:
        return (
            f"CameraExtrinsics(source={self.source!r}, "
            f"pitch={self.pitch_deg:.2f}°, roll={self.roll_deg:.2f}°, "
            f"yaw={self.yaw_deg:.2f}°, "
            f"t=({self.tx_m:.4f}, {self.ty_m:.4f}, {self.tz_m:.4f}) m)"
        )


def load(_unused_legacy_path: Optional[str] = None) -> CameraExtrinsics:
    """Read calibration/camera_extrinsics.py (the same file sground.py uses).

    The argument is accepted only for backwards compatibility with earlier
    versions of saruco.py that passed an .npz path; it is ignored.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    if here not in sys.path:
        sys.path.insert(0, here)

    try:
        from calibration import camera_extrinsics as ce
        return CameraExtrinsics(
            pitch_deg=getattr(ce, "CAMERA_TILT_DEG"),
            roll_deg =getattr(ce, "CAMERA_ROLL_DEG", 0.0),
            yaw_deg  =getattr(ce, "CAMERA_YAW_DEG",  0.0),
            tx_m     =getattr(ce, "CAMERA_X_OFFSET_M"),
            ty_m     =getattr(ce, "CAMERA_Y_OFFSET_M"),
            tz_m     =getattr(ce, "CAMERA_HEIGHT_M"),
            source   ="calibration/camera_extrinsics.py",
        )
    except (ImportError, AttributeError) as exc:
        print(f"% scamera_extrinsics: could not load calibration/camera_extrinsics.py "
              f"({exc.__class__.__name__}: {exc}) — using fall-back values.")
        return CameraExtrinsics(source="fallback", **_FALLBACK)


if __name__ == "__main__":
    e = load()
    print(e)
    print("R_cam2rob =\n", e.R_cam2rob)
    print("T_cam2rob =", e.T_cam2rob)
