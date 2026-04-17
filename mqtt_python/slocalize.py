#!/usr/bin/env python3
"""
slocalize.py
=============================================================================
Field-map aware localization bridge.

Takes sline + saruco outputs, matches them against the field map,
and sends the corrected vision_pose to skalman.

Usage (inside live_perception_overlay.py):
    from slocalize import localizer
    localizer.setup()
    ...
    correction = localizer.update(
        line_result=result,       # output of sline.process()
        aruco_detections=aruco,   # output of saruco.process()
        px=kalman_x, py=kalman_y, yaw=kalman_yaw
    )
    if correction['valid']:
        _send_vision_pose(**correction)
=============================================================================
"""
from __future__ import annotations
import math
import time


def _wrap_deg(a: float) -> float:
    """Wrap angle to [-180, 180] range (degrees)."""
    return (a + 180.0) % 360.0 - 180.0


def _wrap_rad(a: float) -> float:
    """Wrap angle to [-π, π] range (radians)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class SLocalize:
    """
    Combines field map + camera measurements to produce a Kalman correction.

    Main logic:
    1. Take current (px, py, yaw) estimate from encoder + IMU
    2. Take tape/ArUco measurement from camera
    3. Ask field map: "where should the tape/ArUco be at this position?"
    4. Compute difference → send to Kalman as vision_pose
    """

    def __init__(self):
        self._field = None
        self._last_correction_time = 0.0
        self._correction_interval = 0.1   # seconds — max 10 Hz correction
        self._tape_weight = 0.6           # weight given to tape correction
        self._aruco_weight = 1.0          # weight given to ArUco correction
        self._max_lateral_jump = 0.30     # m — sudden jump filter
        self._max_yaw_jump_deg = 30.0     # degrees

        # Last valid correction
        self.last_correction: dict = {}
        self.correction_count = 0

    def setup(self):
        """Load the field map."""
        try:
            from field_map_2026 import FIELD
            self._field = FIELD
            print("% SLocalize: field map loaded OK")
        except Exception as e:
            print(f"% SLocalize: WARNING field map could not be loaded: {e}")

    # ------------------------------------------------------------------
    # MAIN METHOD
    # ------------------------------------------------------------------
    def update(self,
               line_result: dict,
               aruco_detections: list,
               px: float, py: float, yaw: float,
               pitch: float = 0.0) -> dict:
        """
        Combine camera measurements with the field map to produce a correction.

        Returns
        -------
        dict:
            valid    : bool
            x, y, z  : corrected world coordinates
            yaw      : corrected yaw (rad)
            pitch    : pitch (rad)
            source   : 'tape' | 'aruco' | 'tape+aruco'
            context  : result of FIELD.context() (nearby objects)
        """
        if self._field is None:
            return {'valid': False}

        now = time.time()
        if now - self._last_correction_time < self._correction_interval:
            ctx = self._field.context(px, py, yaw)
            return {'valid': False, 'context': ctx}

        # ArUco correction is disabled for the first debug pass.
        # Let tape + map matching stabilise first.
        aruco_detections = []

        # Observed tape points (world frame) — for map matching
        _world_pts = line_result.get('world_line_points', None)

        # Field map context
        ctx = self._field.context(px, py, yaw, world_line_points=_world_pts)

        corrected_x = px
        corrected_y = py
        corrected_yaw = yaw
        corrected_z = ctx['pz']
        source = None
        valid = False

        # ── 1. TAPE CORRECTION ───────────────────────────────────────────────
        line_valid = line_result.get('valid', False) or line_result.get('line_valid', False)
        if line_valid and ctx['nearest_tape'] is not None:
            tape = ctx['nearest_tape']
            tape_dist = tape['dist']

            # Only apply correction when close to tape
            if tape_dist < 0.50:

                # ── Lateral (x/y) correction ──────────────────────────────
                cam_lateral = float(line_result.get('line_offset', 0.0))
                map_lateral = float(tape['lateral_error'])

                # First attempt: use + for sign direction
                net_lateral = cam_lateral + map_lateral

                print(
                    f"% tape debug: cam_lat={cam_lateral:+.3f} "
                    f"map_lat={map_lateral:+.3f} "
                    f"net={net_lateral:+.3f} "
                    f"tape_dist={tape_dist:.3f}"
                )

                if abs(net_lateral) < self._max_lateral_jump:
                    tape_heading_rad = math.radians(tape['heading_deg'])
                    perp_x = -math.sin(tape_heading_rad)
                    perp_y = math.cos(tape_heading_rad)
                    corrected_x += self._tape_weight * net_lateral * perp_x
                    corrected_y += self._tape_weight * net_lateral * perp_y

                # ── Yaw correction (with 180° ambiguity handling) ─────────
                cam_heading_rad = float(line_result.get('line_heading', 0.0))

                seg_heading_deg = tape['heading_deg']
                seg_heading_rad = math.radians(seg_heading_deg)

                # Which direction is closer to the robot's current yaw?
                diff_fwd = abs(_wrap_rad(yaw - seg_heading_rad))
                diff_rev = abs(_wrap_rad(yaw - (seg_heading_rad + math.pi)))
                if diff_rev < diff_fwd:
                    effective_seg_heading_rad = _wrap_rad(seg_heading_rad + math.pi)
                else:
                    effective_seg_heading_rad = seg_heading_rad

                # Expected tape angle in robot frame
                expected_rel_heading_rad = _wrap_rad(effective_seg_heading_rad - yaw)

                # Difference between camera measurement and expectation
                yaw_err = _wrap_rad(cam_heading_rad - expected_rel_heading_rad)

                if abs(math.degrees(yaw_err)) < self._max_yaw_jump_deg:
                    corrected_yaw = _wrap_rad(yaw + self._tape_weight * yaw_err)

                source = 'tape'
                valid = True

        # ── 2. ARUCO CORRECTION ──────────────────────────────────────────────
        # Disabled during first debug phase.
        for det in aruco_detections:
            aruco_id = det.get('id', -1)

            known_marker = None
            for m in self._field.all_aruco():
                if m.id == aruco_id:
                    known_marker = m
                    break

            if known_marker is None:
                continue

            rng = float(det.get('range', 0.0))
            bearing = float(det.get('bearing', 0.0))

            if rng < 0.05 or rng > 3.0:
                continue

            abs_bearing = yaw + bearing
            aruco_x = known_marker.position.x - rng * math.cos(abs_bearing)
            aruco_y = known_marker.position.y - rng * math.sin(abs_bearing)

            jump = math.hypot(aruco_x - px, aruco_y - py)
            if jump > self._max_lateral_jump * 2:
                continue

            corrected_x = aruco_x
            corrected_y = aruco_y
            source = 'aruco' if source is None else 'tape+aruco'
            valid = True
            break

        if not valid:
            return {'valid': False, 'context': ctx}

        corrected_z = self._field.pz_at(corrected_x, corrected_y)

        self._last_correction_time = now
        self.correction_count += 1

        print(
            f"% SLocalize: src={source} "
            f"px={px:.3f} py={py:.3f} yaw={yaw:.3f} -> "
            f"cx={corrected_x:.3f} cy={corrected_y:.3f} cyaw={corrected_yaw:.3f} "
            f"zone={ctx.get('zone','')} "
            f"tape={(ctx['nearest_tape']['name'] if ctx.get('nearest_tape') else 'none')}"
        )

        correction = {
            'valid': True,
            'x': corrected_x,
            'y': corrected_y,
            'z': corrected_z,
            'yaw': corrected_yaw,
            'pitch': pitch,
            'source': source,
            'context': ctx,
        }
        self.last_correction = correction
        return correction

    # ------------------------------------------------------------------
    # HELPERS
    # ------------------------------------------------------------------
    def get_context(self, px: float, py: float, yaw: float = None,
                    world_line_points: list = None) -> dict:
        """Query context only, without computing a correction."""
        if self._field is None:
            return {}
        return self._field.context(px, py, yaw, world_line_points=world_line_points)

    def set_start_pose(self, px: float = None, py: float = None,
                       yaw: float = None) -> dict:
        """
        Set the start position to the Start area defined in the field map.
        Used together with skalman.reset().

        Default yaw is π/2 (90°, +Y direction) — robot exits start area upward.
        """
        if self._field is None:
            return None
        if px is None:
            px = self._field.start_area.center.x
        if py is None:
            py = self._field.start_area.origin.y + self._field.start_area.depth / 2.0
        if yaw is None:
            yaw = math.pi / 2.0   # +Y direction
        pz = self._field.pz_at(px, py)
        return {'x': px, 'y': py, 'z': pz, 'yaw': yaw}


localizer = SLocalize()