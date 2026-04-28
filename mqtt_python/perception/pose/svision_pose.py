#!/usr/bin/env python3
"""
svision_pose.py
=============================================================================
Camera-anchored 6-DOF pose and rates for every frame.

Input  : sline result, saruco detections, IMU pitch, current Kalman prior
Output : x, y, z, yaw, pitch, velocity, omega   (valid / invalid)

Why this works without visual odometry
--------------------------------------
The field is fully mapped in field_map_2026.FIELD. Every tape segment has
known world endpoints, so each tape heading and centreline is known.

This gives us several useful constraints:

1. When the camera sees tape, we can estimate the robot's lateral offset from
   the known tape segment. sline gives this offset in robot frame, and we
   rotate it into world frame.

2. When the camera sees the tape angle, we can estimate the robot yaw relative
   to the known segment heading.

3. Tape alone cannot recover along-track position because a straight tape
   segment is locally featureless. To handle that, we project the Kalman prior
   onto the segment for the along-track coordinate, then snap the perpendicular
   coordinate to the tape. This corrects yaw and lateral drift while leaving
   along-track drift to the prior.

4. ArUco markers, when visible, give an absolute (x, y, yaw) fix without
   depending on the prior. We use this fix whenever the solution passes sanity
   checks.

5. Z comes from FIELD.pz_at(x, y). The map knows the terrain height.

6. Pitch comes from the IMU. A monocular camera looking at the ground cannot
   reliably estimate pitch by itself.

7. Velocity and omega are finite differences of successive vision poses, with
   low-pass filtering. They are intended as low-weight EKF channels; encoders
   and gyro remain the primary rate sources.

Output coordinate conventions
-----------------------------
World frame : X = right, Y = up, Z = height, matching field_map_2026.
=============================================================================
"""
from __future__ import annotations
import math
import time
from collections import deque
from typing import Optional, Dict, Any


def _wrap_rad(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class SVisionPose:
    # ---- tunables ---------------------------------------------------------
    # ArUco sanity gates
    ARUCO_MIN_RANGE_M      = 0.05
    ARUCO_MAX_RANGE_M      = 3.00
    ARUCO_MAX_JUMP_M       = 0.60
    ARUCO_MAX_YAW_JUMP_RAD = math.radians(60.0)

    # ArUco fix mode:
    #   'weighted' combines all gated localization markers using 1/range² weights.
    #   'nearest' uses only the closest marker, which is useful for fallback/debugging.
    ARUCO_FIX_MODE         = 'weighted'
    ARUCO_MIN_RANGE_FOR_WEIGHT_M = 0.10

    # ArUco measurement-noise model.
    # PnP uncertainty grows roughly with range under normal viewing angles.
    # The floor handles very close markers where pixel quantisation dominates.
    ARUCO_SIGMA_XY_PER_M    = 0.015                  # 1.5 cm at 1 m range
    ARUCO_SIGMA_YAW_PER_M   = math.radians(2.0)      # 2° at 1 m range
    ARUCO_SIGMA_XY_FLOOR_M  = 0.005                  # 5 mm minimum
    ARUCO_SIGMA_YAW_FLOOR_RAD = math.radians(0.5)

    # Confidence is derived from XY uncertainty:
    # confidence = REF / (REF + sigma_xy_m)
    # At sigma=REF, confidence is 0.5.
    ARUCO_CONF_SIGMA_REF_M  = 0.010

    # Tape anchoring
    TAPE_MAX_DIST_M        = 0.40     # only trust tape if the prior is close to the segment
    TAPE_MAX_HEAD_ERR_DEG  = 45.0     # reject tape if prior yaw is too far from segment heading

    # Velocity / omega smoothing
    MIN_DT_S               = 0.03     # need at least 30 ms between fixes
    MAX_DT_S               = 0.40     # older than this resets rate estimate
    VEL_LPF_ALPHA          = 0.35     # 0 = smoother, 1 = raw
    HISTORY_N              = 6

    def __init__(self):
        self._field = None
        self._ready = False

        self._velocity = 0.0
        self._omega    = 0.0

        # History entries are (t, x, y, yaw), newest last.
        self._hist = deque(maxlen=self.HISTORY_N)

        # Counters for diagnostics.
        self.update_count    = 0
        self.aruco_fixes     = 0
        self.tape_fixes      = 0
        self.invalid_frames  = 0

        self.last: Dict[str, Any] = self._invalid_output()

    # -------------------------------------------------------------------
    def setup(self) -> bool:
        try:
            from field_map.field_map_2026 import FIELD
            self._field = FIELD
            self._ready = True
            print("% SVisionPose: field map loaded")
        except Exception as e:
            print(f"% SVisionPose: field map load failed: {e}")
            self._ready = False
        return self._ready

    # -------------------------------------------------------------------
    @staticmethod
    def _invalid_output() -> Dict[str, Any]:
        return {
            'valid': False, 'source': None,
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'yaw': 0.0, 'pitch': 0.0,
            'velocity': 0.0, 'omega': 0.0,
            't': 0.0,
        }

    # -------------------------------------------------------------------
    def update(self,
               aruco_detections: list,
               line_result: Optional[dict],
               px_kalman: float,
               py_kalman: float,
               yaw_kalman: float,
               pitch_imu: float = 0.0,
               landmark_fix: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Compute a vision-anchored pose for this frame.

        Always returns a dict. The caller should check result['valid'] before
        publishing.

        `landmark_fix` is the output of slandmark_match.SLandmarkMatcher.process()
        from the same frame, or None. When ArUco is unavailable, this can provide
        an absolute (x, y, yaw) fix from a distinctive map feature.
        """
        if not self._ready:
            return self._invalid_output()

        self.update_count += 1
        t_now = time.time()

        # 1) Prefer an absolute ArUco fix when available and sane.
        fix = self._aruco_fix(aruco_detections,
                              px_kalman, py_kalman, yaw_kalman)
        source = None

        if fix is not None:
            source = 'aruco'
            self.aruco_fixes += 1
        elif landmark_fix is not None:
            # 2) Use a landmark fix from a distinctive map feature.
            fix = {
                'x':        landmark_fix['x'],
                'y':        landmark_fix['y'],
                'yaw':      landmark_fix['yaw'],
                'aruco_id': None,
            }
            source = 'landmark'
            if not hasattr(self, 'landmark_fixes'):
                self.landmark_fixes = 0
            self.landmark_fixes += 1
        else:
            # 3) Fall back to tape-map anchoring.
            # This is relative to the prior and corrects lateral/yaw drift.
            fix = self._tape_fix(line_result,
                                 px_kalman, py_kalman, yaw_kalman)
            if fix is not None:
                source = 'tape'
                self.tape_fixes += 1

        if fix is None:
            self.invalid_frames += 1
            self.last = self._invalid_output()
            return dict(self.last)

        wx, wy, wyaw = fix['x'], fix['y'], fix['yaw']

        # Z comes from the field map. Pitch comes from the IMU.
        wz = float(self._field.pz_at(wx, wy))
        wpitch = float(pitch_imu)

        # Rates come from successive accepted fixes.
        vel, omg = self._update_rates(t_now, wx, wy, wyaw)

        # Add world-frame yaw to detections so task code can consume marker
        # orientation directly without recomputing it from robot yaw.
        # This mutates the detections list in-place.
        aruco_world_yaws = None
        if aruco_detections:
            self.annotate_detections_world_yaw(aruco_detections, wyaw)
            aruco_world_yaws = [
                {
                    'id':        d.get('id'),
                    'role':      d.get('role'),
                    'yaw_world': d.get('yaw_world'),
                }
                for d in aruco_detections
                if 'yaw_world' in d
            ]

        out = {
            'valid':    True,
            'source':   source,
            'x':        wx,
            'y':        wy,
            'z':        wz,
            'yaw':      _wrap_rad(wyaw),
            'pitch':    wpitch,
            'velocity': vel,
            'omega':    omg,
            't':        t_now,
            'aruco_id': fix.get('aruco_id'),
            'aruco_ids': fix.get('aruco_ids'),
            'n_aruco_used': fix.get('n_aruco_used'),
            'aruco_fix_mode': fix.get('aruco_fix_mode'),
            'aruco_world_yaws': aruco_world_yaws,
            'sigma_xy_m':       fix.get('sigma_xy_m'),
            'sigma_yaw_rad':    fix.get('sigma_yaw_rad'),
            'aruco_confidence': fix.get('aruco_confidence'),
            'tape_name': fix.get('tape_name'),
            'landmark': landmark_fix.get('name') if (source == 'landmark' and landmark_fix) else None,
        }
        self.last = out
        return dict(out)

    # -------------------------------------------------------------------
    # ArUco fix: absolute world pose from known marker positions.
    # -------------------------------------------------------------------
    def _aruco_fix(self, detections: list,
                   px_prior: float, py_prior: float,
                   yaw_prior: float) -> Optional[dict]:
        """
        Resolve robot pose in world frame from ArUco observations.

        Transform chain:

            1. Detector gives marker pose in robot frame:
                    T_marker_rob, R_marker_rob

            2. Map gives marker pose in world frame:
                    T_world_marker = (marker.x, marker.y, marker.z)
                    R_world_marker = rotation from marker.facing_deg

            3. Compose:
                    R_world_rob = R_world_marker @ R_marker_rob.T
                    T_world_rob = T_world_marker - R_world_rob @ T_marker_rob

            4. Read yaw from the planar part of R_world_rob.

        The prior is only used for sanity checks, such as rejecting impossible
        position or yaw jumps. It does not determine the output pose.
        """
        if not detections:
            return None

        import numpy as np

        # First pass: build one candidate pose per valid localization marker.
        candidates = []

        for det in detections:
            try:
                aid = int(det.get('id', -1))
                rng = float(det.get('range', 0.0))
            except (TypeError, ValueError):
                continue

            # Only localization markers may update robot pose.
            # Task markers such as luggage and shuttle are ignored here even if
            # they are detected correctly.
            role = det.get('role')
            if role is not None and role != 'localization':
                continue

            if rng < self.ARUCO_MIN_RANGE_M or rng > self.ARUCO_MAX_RANGE_M:
                continue

            T_marker_rob = det.get('T_marker_rob')
            R_marker_rob = det.get('R_marker_rob')
            if T_marker_rob is None or R_marker_rob is None:
                continue

            marker = None
            for m in self._field.all_aruco():
                if m.id == aid:
                    marker = m
                    break
            if marker is None:
                continue

            # Extra safety: the field map role must also allow localization.
            if getattr(marker, 'role', 'localization') != 'localization':
                continue

            T_world_marker = np.array(
                [marker.position.x, marker.position.y, marker.position.z],
                dtype=np.float64,
            )
            R_world_marker = self._marker_world_rotation(marker.facing_deg)

            R_world_rob = R_world_marker @ np.asarray(R_marker_rob).T
            T_world_rob = T_world_marker - R_world_rob @ np.asarray(T_marker_rob)

            rx = float(T_world_rob[0])
            ry = float(T_world_rob[1])
            ryaw = math.atan2(R_world_rob[1, 0], R_world_rob[0, 0])

            # Sanity gate. The prior is not authoritative, but it helps reject
            # bad PnP results, misread IDs, or motion-blurred detections.
            jump = math.hypot(rx - px_prior, ry - py_prior)
            if jump > self.ARUCO_MAX_JUMP_M:
                continue
            if abs(_wrap_rad(ryaw - yaw_prior)) > self.ARUCO_MAX_YAW_JUMP_RAD:
                continue

            candidates.append({
                'x': rx, 'y': ry, 'yaw': ryaw,
                'range': rng, 'aruco_id': aid,
            })

        if not candidates:
            return None

        # Second pass: merge all gated candidates.
        return self._merge_aruco_candidates(candidates)

    def _merge_aruco_candidates(self, candidates: list) -> dict:
        """Combine sanity-gated ArUco candidates into one pose fix.

        Modes:
            'nearest'  : use the closest marker only.
            'weighted' : combine candidates with 1/range² weights.

        Yaw is averaged on the unit circle so wrap-around is handled correctly.

        Each output also includes an uncertainty estimate. Per-marker sigmas
        come from `_per_marker_sigmas()` and are combined with inverse variance:
            sigma_combined² = 1 / sum(1 / sigma_i²)

        `aruco_confidence` is a 0..1 diagnostic value. Kalman should consume
        the sigma values directly.
        """
        candidates_by_range = sorted(candidates, key=lambda c: c['range'])
        nearest = candidates_by_range[0]

        ids_sorted = [c['aruco_id'] for c in candidates_by_range]

        if self.ARUCO_FIX_MODE == 'nearest' or len(candidates_by_range) == 1:
            fix = dict(nearest)
            fix.pop('range', None)
            fix['aruco_ids'] = ids_sorted
            fix['n_aruco_used'] = len(candidates_by_range)
            fix['aruco_fix_mode'] = 'nearest'
            sigma_xy, sigma_yaw = self._per_marker_sigmas(nearest['range'])
            fix['sigma_xy_m']       = sigma_xy
            fix['sigma_yaw_rad']    = sigma_yaw
            fix['aruco_confidence'] = self._confidence_from_sigma(sigma_xy)
            return fix

        # Weighted merge.
        floor = self.ARUCO_MIN_RANGE_FOR_WEIGHT_M
        total_w = 0.0
        wx_sum  = 0.0
        wy_sum  = 0.0
        sin_sum = 0.0
        cos_sum = 0.0
        inv_var_xy_sum  = 0.0
        inv_var_yaw_sum = 0.0

        for c in candidates_by_range:
            r_eff = max(float(c['range']), floor)
            w = 1.0 / (r_eff * r_eff)
            total_w += w
            wx_sum  += w * c['x']
            wy_sum  += w * c['y']
            sin_sum += w * math.sin(c['yaw'])
            cos_sum += w * math.cos(c['yaw'])

            sig_xy_i, sig_yaw_i = self._per_marker_sigmas(c['range'])
            inv_var_xy_sum  += 1.0 / (sig_xy_i  * sig_xy_i)
            inv_var_yaw_sum += 1.0 / (sig_yaw_i * sig_yaw_i)

        wx = wx_sum / total_w
        wy = wy_sum / total_w
        wyaw = math.atan2(sin_sum, cos_sum)

        sigma_xy_combined  = math.sqrt(1.0 / inv_var_xy_sum)
        sigma_yaw_combined = math.sqrt(1.0 / inv_var_yaw_sum)

        return {
            'x': wx,
            'y': wy,
            'yaw': wyaw,
            'aruco_id':         nearest['aruco_id'],
            'aruco_ids':        ids_sorted,
            'n_aruco_used':     len(candidates_by_range),
            'aruco_fix_mode':   'weighted',
            'sigma_xy_m':       sigma_xy_combined,
            'sigma_yaw_rad':    sigma_yaw_combined,
            'aruco_confidence': self._confidence_from_sigma(sigma_xy_combined),
        }

    def _per_marker_sigmas(self, range_m: float) -> tuple:
        """Return per-marker (sigma_xy_m, sigma_yaw_rad) from the range model."""
        r = max(float(range_m), 0.0)
        sigma_xy  = max(self.ARUCO_SIGMA_XY_PER_M  * r, self.ARUCO_SIGMA_XY_FLOOR_M)
        sigma_yaw = max(self.ARUCO_SIGMA_YAW_PER_M * r, self.ARUCO_SIGMA_YAW_FLOOR_RAD)
        return sigma_xy, sigma_yaw

    def _confidence_from_sigma(self, sigma_xy_m: float) -> float:
        """Convert XY uncertainty into a saturating 0..1 confidence score.

        At sigma = 0, confidence is 1.0.
        At sigma = REF, confidence is 0.5.
        As sigma grows, confidence approaches 0.
        """
        ref = self.ARUCO_CONF_SIGMA_REF_M
        return ref / (ref + max(float(sigma_xy_m), 0.0))

    # -------------------------------------------------------------------
    # World-frame marker yaw / rotation.
    # Used by task code for cube grasping, shuttle alignment, and similar tasks.
    # -------------------------------------------------------------------
    @staticmethod
    def annotate_detections_world_yaw(detections: list,
                                       robot_yaw: float) -> None:
        """Add `R_world_marker` and `yaw_world` to detections in-place.

        `yaw_world` is the world-frame direction of the marker's local +Z axis,
        projected into the XY plane. For upright wall markers, this matches the
        marker facing direction. For flat markers, such as a marker on the
        shuttle deck, local +Z points upward, so consumers should use the full
        `R_world_marker` instead of relying on yaw_world.

        Detections without `R_marker_rob` are skipped for compatibility with
        older detector output.
        """
        import numpy as np

        cy = math.cos(robot_yaw)
        sy = math.sin(robot_yaw)

        # Robot-to-world planar yaw rotation.
        R_world_rob = np.array([
            [cy, -sy, 0.0],
            [sy,  cy, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

        for det in detections:
            R_marker_rob = det.get('R_marker_rob')
            if R_marker_rob is None:
                continue
            R_world_marker = R_world_rob @ np.asarray(R_marker_rob)
            det['R_world_marker'] = R_world_marker

            # Marker outward normal, local +Z, projected into world XY.
            det['yaw_world'] = math.atan2(R_world_marker[1, 2],
                                          R_world_marker[0, 2])

    @staticmethod
    def _marker_world_rotation(facing_deg: float):
        """World-frame rotation for an upright wall-mounted marker.

        Marker local axes use the OpenCV ArUco convention:
            +X = right edge
            +Y = down edge
            +Z = outward normal

        World axes use the field map convention:
            +X = right
            +Y = up
            +Z = height

        For a vertical marker with its top edge along world +Z and face
        direction `facing_deg`:
            local +X = world Rz(facing) · (-Y)
            local +Y = world -Z
            local +Z = world Rz(facing) · (+X)
        """
        import numpy as np
        f = math.radians(facing_deg)
        sf, cf = math.sin(f), math.cos(f)
        return np.array([
            [ sf,  0.0,  cf],
            [-cf,  0.0,  sf],
            [0.0, -1.0, 0.0],
        ], dtype=np.float64)

    # -------------------------------------------------------------------
    # Tape fix: snap lateral offset and yaw to the nearest map segment.
    # -------------------------------------------------------------------
    def _tape_fix(self, line_result: Optional[dict],
                  px_prior: float, py_prior: float,
                  yaw_prior: float) -> Optional[dict]:
        if line_result is None:
            return None

        line_valid = (line_result.get('line_valid', False)
                      or line_result.get('valid', False))
        if not line_valid:
            return None

        world_pts = line_result.get('world_line_points') or None
        seg = self._field.nearest_tape_segment(
            px_prior, py_prior, yaw_prior,
            world_line_points=world_pts,
            max_yaw_diff_deg=self.TAPE_MAX_HEAD_ERR_DEG,
        )
        if seg is None:
            return None
        if seg['dist'] > self.TAPE_MAX_DIST_M:
            return None

        # Segment geometry in world frame.
        p0x, p0y = seg['p0']
        p1x, p1y = seg['p1']
        seg_dx = p1x - p0x
        seg_dy = p1y - p0y
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len < 1e-6:
            return None
        seg_heading = math.atan2(seg_dy, seg_dx)
        ux, uy = seg_dx / seg_len, seg_dy / seg_len       # along-tape unit vector
        nx, ny = -uy, ux                                  # left-of-tape normal

        # Tape can be driven in either direction. Choose the direction whose
        # resulting yaw is closer to the prior.
        cam_heading = float(line_result.get('line_heading', 0.0))
        yaw_forward  = _wrap_rad(seg_heading          - cam_heading)
        yaw_backward = _wrap_rad(seg_heading + math.pi - cam_heading)
        going_forward = (abs(_wrap_rad(yaw_forward  - yaw_prior))
                         <= abs(_wrap_rad(yaw_backward - yaw_prior)))
        wyaw = yaw_forward if going_forward else yaw_backward

        # Lateral sign flips when traversing the tape backwards.
        cam_lateral = float(line_result.get('line_offset', 0.0))
        signed_lateral = cam_lateral if going_forward else -cam_lateral

        # If available, use multiple detected tape points. Each camera point is
        # transformed into world frame, snapped to the known segment, and then
        # inverted back to estimate robot position. Averaging gives a stronger
        # constraint than using a single lateral offset.
        world_pts = line_result.get('world_line_points') or []
        robot_pts = line_result.get('line_points_robot',  [])
        cos_y = math.cos(wyaw)
        sin_y = math.sin(wyaw)

        wx, wy = None, None
        if len(world_pts) >= 2 and len(robot_pts) == len(world_pts):
            px_sum, py_sum, count = 0.0, 0.0, 0
            for (wxi, wyi), (xr, yr) in zip(world_pts, robot_pts):
                t = (wxi - p0x) * ux + (wyi - p0y) * uy
                t = max(0.0, min(seg_len, t))
                wx_snap = p0x + t * ux
                wy_snap = p0y + t * uy
                px_sum += wx_snap - (cos_y * xr - sin_y * yr)
                py_sum += wy_snap - (sin_y * xr + cos_y * yr)
                count += 1
            if count > 0:
                wx = px_sum / count
                wy = py_sum / count

        if wx is None:
            # Fallback: keep the prior's along-track coordinate and correct
            # only the perpendicular coordinate using the detected lateral offset.
            along = (px_prior - p0x) * ux + (py_prior - p0y) * uy
            along = max(0.0, min(seg_len, along))
            anchor_x = p0x + along * ux
            anchor_y = p0y + along * uy
            wx = anchor_x + (-signed_lateral) * nx
            wy = anchor_y + (-signed_lateral) * ny

        return {'x': wx, 'y': wy, 'yaw': wyaw, 'tape_name': seg['name']}

    # -------------------------------------------------------------------
    # Velocity and omega: finite difference of accepted fixes with LPF.
    # -------------------------------------------------------------------
    def _update_rates(self, t_now: float,
                      wx: float, wy: float, wyaw: float) -> tuple:
        # Drop stale history after a tracking gap.
        while self._hist and (t_now - self._hist[0][0]) > 2.0:
            self._hist.popleft()

        if self._hist:
            t_prev, x_prev, y_prev, yaw_prev = self._hist[-1]
            dt = t_now - t_prev
            if self.MIN_DT_S <= dt <= self.MAX_DT_S:
                dx = wx - x_prev
                dy = wy - y_prev
                raw_speed = math.hypot(dx, dy) / dt

                # Give speed a sign based on projection onto current heading.
                heading_proj = math.cos(wyaw) * dx + math.sin(wyaw) * dy
                if heading_proj < 0:
                    raw_speed = -raw_speed

                raw_omega = _wrap_rad(wyaw - yaw_prev) / dt

                a = self.VEL_LPF_ALPHA
                self._velocity = a * raw_speed + (1 - a) * self._velocity
                self._omega    = a * raw_omega + (1 - a) * self._omega
            elif dt > self.MAX_DT_S:
                self._velocity = 0.0
                self._omega    = 0.0

        self._hist.append((t_now, wx, wy, wyaw))
        return self._velocity, self._omega


# Singleton instance used by live_perception_overlay.py.
vision_pose = SVisionPose()