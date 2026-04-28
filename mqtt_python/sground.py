"""
sground.py
=============================================================================
Ground plane projection module.

Converts a pixel (u, v) in the distorted camera image into a 2D point
(X, Y) in the robot's coordinate frame on the ground plane (Z = 0).

Robot coordinate frame (right-hand, Z up):
    X = forward  (robot's driving direction)
    Y = left
    Z = up

Camera frame (OpenCV convention):
    x = right
    y = down
    z = into scene (optical axis)

Three extrinsic angles:
    tilt (pitch) - camera looks downward, positive = more down
    roll         - camera rotates around optical axis, positive = clockwise
                   when viewed from behind. Non-zero roll causes the Y values
                   of centre-column pixels to drift away from zero.

Usage:
    python3 vision/sground.py --params calibration/camera_params.npz
"""

import numpy as np
import cv2
import os
import sys

# --- EXTRINSICS --------------------------------------------------------------------------
try:
    from calibration.camera_extrinsics import (
        CAMERA_HEIGHT_M   as DEFAULT_CAMERA_HEIGHT_M,
        CAMERA_TILT_DEG   as DEFAULT_CAMERA_TILT_DEG,
        CAMERA_X_OFFSET_M as DEFAULT_CAMERA_X_OFFSET_M,
        CAMERA_Y_OFFSET_M as DEFAULT_CAMERA_Y_OFFSET_M,
    )
    try:
        from calibration.camera_extrinsics import CAMERA_ROLL_DEG as DEFAULT_CAMERA_ROLL_DEG
    except ImportError:
        DEFAULT_CAMERA_ROLL_DEG = 0.0
    print("% SGround: extrinsics loaded from calibration/camera_extrinsics.py")
except ImportError:
    DEFAULT_CAMERA_HEIGHT_M   = 0.185
    DEFAULT_CAMERA_TILT_DEG   = 25.0
    DEFAULT_CAMERA_ROLL_DEG   = 0.0
    DEFAULT_CAMERA_X_OFFSET_M = 0.02
    DEFAULT_CAMERA_Y_OFFSET_M = -0.005
    print("% SGround: WARNING - camera_extrinsics.py not found, using hardcoded defaults")


class SGround:
    camera_matrix = None
    dist_coeffs   = None
    img_size      = None

    camera_height = DEFAULT_CAMERA_HEIGHT_M
    camera_tilt   = DEFAULT_CAMERA_TILT_DEG
    camera_roll   = DEFAULT_CAMERA_ROLL_DEG
    x_offset      = DEFAULT_CAMERA_X_OFFSET_M
    y_offset      = DEFAULT_CAMERA_Y_OFFSET_M

    _R_c2r = None   # 3×3 rotation: camera frame --> robot frame (includes roll)
    _t_cam = None   # camera centre in robot frame (3,)

    last_X    = 0.0
    last_Y    = 0.0
    last_dist = 0.0
    ready     = False


    def setup(self, params_path: str = None,
              camera_height: float = None,
              camera_tilt:   float = None,
              camera_roll:   float = None,
              x_offset:      float = None,
              y_offset:      float = None):
        if params_path is None:
            params_path = os.path.join(os.path.dirname(__file__), 'calibration', 'camera_params.npz')
        if not os.path.isfile(params_path):
            print(f"% SGround: ERROR — calibration file not found: {params_path}")
            return

        data = np.load(params_path)
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs   = data['dist_coeffs']
        self.img_size      = tuple(data['img_size'].tolist())

        print(f"% SGround: loaded intrinsics from {params_path}")
        print(f"%           image size : {self.img_size[0]}×{self.img_size[1]} px")
        print(f"%           fx={self.camera_matrix[0,0]:.2f}  "
              f"fy={self.camera_matrix[1,1]:.2f}  "
              f"cx={self.camera_matrix[0,2]:.2f}  "
              f"cy={self.camera_matrix[1,2]:.2f}")

        if camera_height is not None:
            self.camera_height = camera_height
        if camera_tilt is not None:
            self.camera_tilt = camera_tilt
        if camera_roll is not None:
            self.camera_roll = camera_roll
        if x_offset is not None:
            self.x_offset = x_offset
        if y_offset is not None:
            self.y_offset = y_offset

        self._build_geometry()
        self.ready = True

        print(f"% SGround: extrinsics - "
              f"height={self.camera_height*100:.1f}cm  "
              f"tilt={self.camera_tilt:.1f}  "
              f"roll={self.camera_roll:.1f}  "
              f"x_off={self.x_offset*100:.1f}cm  "
              f"y_off={self.y_offset*100:.1f}cm")
        print("% SGround: ready")

    # -----------------------------------------------------------------------------

    def _build_geometry(self):
        """
        Build R_c2r = R_tilt_c2r @ R_roll_cam

        R_tilt_c2r maps camera axes (assuming no roll) to robot frame.
        R_roll_cam rotates rays within the camera frame before that mapping.

        Roll is rotation around the optical axis (z_cam).
        Positive roll = camera rotates clockwise when viewed from behind,
        which shifts the apparent ground Y of centre-column pixels.

        R_roll (around z, in camera frame):
            [ cos γ  -sin γ  0 ]
            [ sin γ   cos γ  0 ]
            [ 0       0      1 ]
        """
        α = np.radians(self.camera_tilt)
        γ = np.radians(self.camera_roll)

        # Tilt: camera axes in robot frame (roll = 0 baseline) 
        x_cam_r = np.array([0.0, -1.0, 0.0])               # camera right  -> robot -Y
        z_cam_r = np.array([np.cos(α), 0.0, -np.sin(α)])   # optical axis  -> forward+down
        y_cam_r = np.cross(z_cam_r, x_cam_r)
        y_cam_r /= np.linalg.norm(y_cam_r)

        R_tilt = np.column_stack([x_cam_r, y_cam_r, z_cam_r])

        # Roll: rotation around optical axis in camera frame 
        R_roll = np.array([
            [ np.cos(γ), -np.sin(γ), 0.0],
            [ np.sin(γ),  np.cos(γ), 0.0],
            [ 0.0,        0.0,       1.0],
        ])

        # Combined, first apply roll in camera frame, then tilt to robot frame
        self._R_c2r = R_tilt @ R_roll

        self._t_cam = np.array([self.x_offset,
                                 self.y_offset,
                                 self.camera_height])

   # -----------------------------------------------------------------------------

    def set_robot_pitch(self, pitch_rad: float):
        """
        Update the effective camera tilt for the current robot pitch.

        On a slope the robot (and camera) pitch forward/back, changing
        the camera's angle to the ground.  Call this each frame with the
        Kalman pitch estimate to get corrected ground projections.

        Parameters
        ----------
        pitch_rad : robot body pitch in radians (positive = nose up)
                    from the Kalman EKF state.
        """
        # Positive pitch (nose up) means the camera tilts more toward the ground,
        # increasing the effective tilt angle.
        effective_tilt = self.camera_tilt + np.degrees(pitch_rad)
        old_tilt = self.camera_tilt
        self.camera_tilt = effective_tilt
        self._build_geometry()
        self.camera_tilt = old_tilt   # restore nominal so next call is relative to mounting

    def pixel_to_ground(self, u: float, v: float):
        """
        Project a single raw (distorted) pixel to the ground plane Z=0.

        Returns
        -------
        ok : bool
        X  : float - forward distance from robot centre (m)
        Y  : float - lateral distance, positive = left (m)
        """
        if not self.ready:
            print("% SGround: pixel_to_ground called before setup()")
            return False, 0.0, 0.0

        pt = np.array([[[float(u), float(v)]]], dtype=np.float32)
        pt_norm = cv2.undistortPoints(pt, self.camera_matrix, self.dist_coeffs)
        xn = float(pt_norm[0, 0, 0])
        yn = float(pt_norm[0, 0, 1])

        d_cam = np.array([xn, yn, 1.0])
        d_rob = self._R_c2r @ d_cam

        if abs(d_rob[2]) < 1e-6:
            return False, 0.0, 0.0

        lam = -self._t_cam[2] / d_rob[2]
        if lam <= 0:
            return False, 0.0, 0.0

        X = self._t_cam[0] + lam * d_rob[0]
        Y = self._t_cam[1] + lam * d_rob[1]

        self.last_X    = X
        self.last_Y    = Y
        self.last_dist = float(np.hypot(X, Y))

        return True, X, Y

    def pixel_to_ground_at_z(self, u: float, v: float, z: float):
        """
        Project a pixel to the horizontal plane at height Z (robot frame, Z up).

        Same ray as pixel_to_ground() but intersects with Z = z instead of Z = 0.
        Use this to correctly place tape features that are elevated on a slope.

        Returns (ok, X, Y) in robot frame.
        """
        if not self.ready:
            return False, 0.0, 0.0

        pt = np.array([[[float(u), float(v)]]], dtype=np.float32)
        pt_norm = cv2.undistortPoints(pt, self.camera_matrix, self.dist_coeffs)
        xn = float(pt_norm[0, 0, 0])
        yn = float(pt_norm[0, 0, 1])

        d_cam = np.array([xn, yn, 1.0])
        d_rob = self._R_c2r @ d_cam

        if abs(d_rob[2]) < 1e-6:
            return False, 0.0, 0.0

        # Intersect ray with plane Z = z:  t_cam[2] + λ * d_rob[2] = z
        lam = (z - self._t_cam[2]) / d_rob[2]
        if lam <= 0:
            return False, 0.0, 0.0

        X = self._t_cam[0] + lam * d_rob[0]
        Y = self._t_cam[1] + lam * d_rob[1]
        return True, X, Y

    def ray_at_distance(self, u: float, v: float, D: float):
        """
        Given a pixel (u, v) and a known 3D distance D from the camera centre,
        return the 3D point in robot frame (X, Y, Z).

        Useful for computing the height of a feature whose actual distance D
        has been estimated from e.g. apparent tape width.

        Returns (ok, X, Y, Z).
        """
        if not self.ready:
            return False, 0.0, 0.0, 0.0

        pt = np.array([[[float(u), float(v)]]], dtype=np.float32)
        pt_norm = cv2.undistortPoints(pt, self.camera_matrix, self.dist_coeffs)
        xn = float(pt_norm[0, 0, 0])
        yn = float(pt_norm[0, 0, 1])

        d_cam = np.array([xn, yn, 1.0])
        d_rob = self._R_c2r @ d_cam
        d_hat = d_rob / np.linalg.norm(d_rob)

        P = self._t_cam + D * d_hat
        return True, float(P[0]), float(P[1]), float(P[2])

    def pixel_ray_robot(self, u: float, v: float):
        """
        Return the unit viewing ray for pixel (u, v) in robot coordinates.

        Returns (ok, rx, ry, rz), where the ray points away from the camera
        centre and is normalized to unit length.
        """
        if not self.ready:
            return False, 0.0, 0.0, 0.0

        pt = np.array([[[float(u), float(v)]]], dtype=np.float32)
        pt_norm = cv2.undistortPoints(pt, self.camera_matrix, self.dist_coeffs)
        xn = float(pt_norm[0, 0, 0])
        yn = float(pt_norm[0, 0, 1])

        d_cam = np.array([xn, yn, 1.0])
        d_rob = self._R_c2r @ d_cam
        norm = np.linalg.norm(d_rob)
        if norm < 1e-9:
            return False, 0.0, 0.0, 0.0

        d_hat = d_rob / norm
        return True, float(d_hat[0]), float(d_hat[1]), float(d_hat[2])


    def pixels_to_ground(self, points):
        """
        Batch version: project a list of (u, v) pixels.

        Returns
        -------------
        ground_pts : list of (X, Y) for valid points only
        valid_mask : bool list, same length as input
        """
        ground_pts = []
        valid_mask = []
        for u, v in points:
            ok, X, Y = self.pixel_to_ground(u, v)
            valid_mask.append(ok)
            if ok:
                ground_pts.append((X, Y))
        return ground_pts, valid_mask


    def undistort_frame(self, frame):
        if not self.ready:
            return frame
        return cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)


    def update_extrinsics(self, camera_height=None, camera_tilt=None,
                          camera_roll=None, x_offset=None, y_offset=None):
        # Update extrinsics at runtime and rebuild geometry
        if camera_height is not None:
            self.camera_height = camera_height
        if camera_tilt is not None:
            self.camera_tilt = camera_tilt
        if camera_roll is not None:
            self.camera_roll = camera_roll
        if x_offset is not None:
            self.x_offset = x_offset
        if y_offset is not None:
            self.y_offset = y_offset
        self._build_geometry()
        print(f"% SGround: extrinsics updated - "
              f"height={self.camera_height*100:.1f}cm  "
              f"tilt={self.camera_tilt:.1f}°  "
              f"roll={self.camera_roll:.1f}°")

    def terminate(self):
        print("% SGround: terminated")


ground = SGround()


# Standalone sanity test
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Test ground plane projection")
    parser.add_argument("--params", default="calibration/camera_params.npz")
    parser.add_argument("--height", type=float, default=None)
    parser.add_argument("--tilt",   type=float, default=None)
    parser.add_argument("--roll",   type=float, default=None)
    args = parser.parse_args()

    ground.setup(args.params, camera_height=args.height,
                 camera_tilt=args.tilt, camera_roll=args.roll)

    if not ground.ready:
        print("% Setup failed.")
        sys.exit(1)

    w, h = ground.img_size
    cx, cy = w / 2, h / 2

    print("\n-- Sanity check --")
    print(f"{'Pixel':<26}  {'X fwd':>10}  {'Y left':>10}  {'dist':>10}")
    print("-" * 62)

    for u, v, label in [
        (cx,      h - 1, "bottom-centre"),
        (cx,      cy,    "image-centre"),
        (cx - 80, h - 1, "bottom-left"),
        (cx + 80, h - 1, "bottom-right"),
    ]:
        ok, X, Y = ground.pixel_to_ground(u, v)
        if ok:
            print(f"  ({u:6.0f},{v:5.0f})  {label:<14}  "
                  f"{X:10.3f}  {Y:10.3f}  {np.hypot(X,Y):10.3f}")
        else:
            print(f"  ({u:6.0f},{v:5.0f})  {label:<14}  [no ground intersection]")

    print("\nHint: if centre-column Y values are not ≈0, adjust --roll")
