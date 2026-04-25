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
               pitch: float = 0.0,
               max_yaw_diff_deg: float = 90.0) -> dict:
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

        # ── Coordinate convention transform ───────────────────────────────
        # Kalman:    x = forward (0-6 m),  y = lateral (0-7 m),  yaw=0 → facing +x
        # Field map: x = lateral (0-7 m),  y = forward (0-6 m),  yaw=0 → facing +x_fmap
        # Transform: fmap_x = kalman_y,  fmap_y = kalman_x,  fmap_yaw = kalman_yaw + π/2
        fmap_px  = py
        fmap_py  = px
        fmap_yaw = _wrap_rad(yaw + math.pi / 2.0)

        if now - self._last_correction_time < self._correction_interval:
            ctx = self._field.context(fmap_px, fmap_py, fmap_yaw)
            return {'valid': False, 'context': ctx}

        # Observed tape points (world frame, Kalman convention) → swap to field-map
        _world_pts = line_result.get('world_line_points', None)
        fmap_world_pts = [(wy, wx) for wx, wy in _world_pts] if _world_pts else None

        # Field map context (all field-map coordinates)
        ctx = self._field.context(fmap_px, fmap_py, fmap_yaw,
                                  world_line_points=fmap_world_pts,
                                  max_yaw_diff_deg=max_yaw_diff_deg)

        corrected_x   = fmap_px    # working in field-map convention until end
        corrected_y   = fmap_py
        corrected_yaw = fmap_yaw
        corrected_z = ctx['pz']
        source = None
        valid = False

        # ── 1. TAPE CORRECTION ───────────────────────────────────────────────
        line_valid = line_result.get('valid', False) or line_result.get('line_valid', False)
        if line_valid and ctx['nearest_tape'] is not None:
            tape = ctx['nearest_tape']
            tape_dist = tape['dist']

            # Only apply correction when close to tape
            if tape_dist < 1.5:

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
                diff_fwd = abs(_wrap_rad(fmap_yaw - seg_heading_rad))
                diff_rev = abs(_wrap_rad(fmap_yaw - (seg_heading_rad + math.pi)))
                if diff_rev < diff_fwd:
                    effective_seg_heading_rad = _wrap_rad(seg_heading_rad + math.pi)
                else:
                    effective_seg_heading_rad = seg_heading_rad

                # Expected tape angle in robot frame
                expected_rel_heading_rad = _wrap_rad(effective_seg_heading_rad - fmap_yaw)

                # Difference between camera measurement and expectation
                yaw_err = _wrap_rad(cam_heading_rad - expected_rel_heading_rad)

                if abs(math.degrees(yaw_err)) < self._max_yaw_jump_deg:
                    corrected_yaw = _wrap_rad(fmap_yaw + self._tape_weight * yaw_err)

                source = 'tape'
                valid = True

        # ── 2. ARUCO CORRECTION ──────────────────────────────────────────────
        # ArUco markers provide precise absolute positioning
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

            abs_bearing = fmap_yaw + bearing
            # All in field-map convention (x=lateral, y=forward)
            aruco_x = known_marker.position.x - rng * math.cos(abs_bearing)
            aruco_y = known_marker.position.y - rng * math.sin(abs_bearing)

            jump = math.hypot(aruco_x - fmap_px, aruco_y - fmap_py)
            # ArUco provides absolute positioning from known marker locations - no jump limit
            print(f"% ArUco correction: marker_id={aruco_id} range={rng:.2f}m bearing={math.degrees(bearing):+.1f}° jump={jump:.2f}m")

            corrected_x = aruco_x
            corrected_y = aruco_y

            # Yaw correction: expected bearing from corrected position to known marker
            expected_abs_bearing = math.atan2(
                known_marker.position.y - aruco_y,   # Δy field-map (forward)
                known_marker.position.x - aruco_x,   # Δx field-map (lateral)
            )
            # measured absolute bearing = fmap_yaw + det bearing
            meas_abs_bearing = _wrap_rad(fmap_yaw + bearing)
            yaw_err_aruco = _wrap_rad(expected_abs_bearing - meas_abs_bearing)
            if abs(math.degrees(yaw_err_aruco)) < self._max_yaw_jump_deg:
                corrected_yaw = _wrap_rad(fmap_yaw + self._aruco_weight * yaw_err_aruco)

            source = 'aruco' if source is None else 'tape+aruco'
            valid = True
            print(
                f"% ArUco correction: id={aruco_id} range={rng:.2f}m bearing={math.degrees(bearing):+.1f}° "
                f"marker_at=({known_marker.position.x:.2f}, {known_marker.position.y:.2f}) "
                f"robot_at=({aruco_x:.2f}, {aruco_y:.2f}) jump={jump:.2f}m"
            )
            break

        if not valid:
            return {'valid': False, 'context': ctx}

        # pz_at takes field-map coordinates
        corrected_z = self._field.pz_at(corrected_x, corrected_y)

        # ── Convert corrected values back to Kalman convention ────────────
        # Kalman x = forward = field-map y
        # Kalman y = lateral = field-map x
        # Kalman yaw = field-map yaw - π/2
        out_x   = corrected_y
        out_y   = corrected_x
        out_yaw = _wrap_rad(corrected_yaw - math.pi / 2.0)

        self._last_correction_time = now
        self.correction_count += 1

        print(
            f"% SLocalize: src={source} "
            f"px={px:.3f} py={py:.3f} yaw={yaw:.3f} -> "
            f"cx={out_x:.3f} cy={out_y:.3f} cyaw={out_yaw:.3f} "
            f"zone={ctx.get('zone','')} "
            f"tape={(ctx['nearest_tape']['name'] if ctx.get('nearest_tape') else 'none')}"
        )

        correction = {
            'valid': True,
            'x': out_x,
            'y': out_y,
            'z': corrected_z,
            'yaw': out_yaw,
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
        fmap_px  = py
        fmap_py  = px
        fmap_yaw = _wrap_rad(yaw + math.pi / 2.0) if yaw is not None else None
        fmap_wpts = [(wy, wx) for wx, wy in world_line_points] if world_line_points else None
        return self._field.context(fmap_px, fmap_py, fmap_yaw, world_line_points=fmap_wpts)

    def set_start_pose(self, px: float = None, py: float = None,
                       yaw: float = None) -> dict:
        """
        Set the start position to the Start area defined in the field map.
        Used together with skalman.reset().

        Default yaw is π/2 (90°, +Y direction) — robot exits start area upward.
        """
        if self._field is None:
            return None
        # Field map: start_area.center.x = lateral (=Kalman y),
        #            start_area.origin.y + depth/2 = forward (=Kalman x)
        if px is None:
            px = self._field.start_area.origin.y + self._field.start_area.depth / 2.0
        if py is None:
            py = self._field.start_area.center.x
        if yaw is None:
            yaw = 0.0   # facing forward (+x in Kalman convention)
        pz = self._field.pz_at(py, px)   # pz_at takes field-map (lateral, forward)
        return {'x': px, 'y': py, 'z': pz, 'yaw': yaw}


localizer = SLocalize()