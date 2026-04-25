#!/usr/bin/env python3
"""
slam_runner.py
=============================================================================
Pure vision EKF SLAM runner for the Robobot.

No AMCL, no field map required.  Builds and uses its own landmark map from:
    - Tape-corner features (Harris on white-tape mask)
    - FAST keypoints in the ground ROI
    - ArUco marker detections (ID-keyed, no Mahalanobis needed)

Two modes
---------
  map       Build a landmark map by driving around the field.
            Press  s  or  q/ESC  to save and exit.

  localize  Load a previously built map and estimate the robot pose.
            Landmark positions are frozen; only the robot pose is updated.

Usage examples
--------------
  # Mapping from a recorded video (recommended first try):
  python slam_runner.py --mode map --source video --path recordings/test2

  # Mapping from the live camera:
  python slam_runner.py --mode map --source live --host 10.197.218.17

  # Localization using a saved map:
  python slam_runner.py --mode localize --map-file slam_map.json \
                        --source live --host 10.197.218.17

  # Replay images and inspect the map:
  python slam_runner.py --mode map --source images \
                        --path recordings/pic/snapshots --no-display

CLI arguments
-------------
  --mode map|localize          [map]
  --source live|video|images|image
  --path PATH                  folder / file for video/images/image sources
  --host HOST                  robot IP for live source           [localhost]
  --cam-port PORT              MJPEG port                         [7123]
  --map-file PATH              map JSON file to save/load         [slam_map.json]
  --init x,y,yaw_deg           initial robot pose                 [0,0,0]
  --speed F                    replay speed multiplier            [1.0]
  --no-display                 headless mode
  --debug                      enable sline / aruco debug overlays
  --pause                      start paused

Window controls
---------------
  SPACE      pause / resume
  s          step one frame (paused)  OR  save map (mapping mode, not paused)
  r          reset SLAM (mapping) / reset pose uncertainty (localize)
  q / ESC    quit  (mapping: saves map before exit)
=============================================================================
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

# ── Perception ────────────────────────────────────────────────────────────────
from sline import vision as sline_vision
from saruco import SArucoDetector
from sfeatures import feature_extractor
from sekf_slam import SEkfSlam
from ssources import FrameInfo, build_source


# =============================================================================
# Visualisation constants
# =============================================================================

_MAP_W = 560
_MAP_H = 480
_FIELD_KX_MAX = 6.0   # Kalman frame: x = forward  (0-6 m)
_FIELD_KY_MAX = 7.0   # Kalman frame: y = lateral  (0-7 m)
_MARGIN = 20


def _field_to_px(kx: float, ky: float) -> Tuple[int, int]:
    """Convert Kalman-frame (kx, ky) to map-panel pixel."""
    px_x = int(_MARGIN + (ky / _FIELD_KY_MAX) * (_MAP_W - 2 * _MARGIN))
    px_y = int((_MAP_H - _MARGIN) - (kx / _FIELD_KX_MAX) * (_MAP_H - 2 * _MARGIN))
    return px_x, px_y


# =============================================================================
# Map panel renderer
# =============================================================================

def draw_map_panel(slam: SEkfSlam,
                   mode: str,
                   ref_pose: Optional[Tuple] = None) -> np.ndarray:
    """
    Render a top-down landmark map panel.

    Colour coding:
      Grid lines      : dim gray
      Visual features : blue  (tentative) / cyan (reliable, >= 3 obs)
      ArUco landmarks : orange diamond + ID label
      Robot pose      : green arrow
      Reference pose  : dim cyan arrow (if available from kalman log)
    """
    panel = np.full((_MAP_H, _MAP_W, 3), 18, dtype=np.uint8)
    font  = cv2.FONT_HERSHEY_PLAIN

    # ── Grid (every 1 m) ─────────────────────────────────────────────────────
    for kx in range(0, int(_FIELD_KX_MAX) + 1):
        p0 = _field_to_px(float(kx), 0.0)
        p1 = _field_to_px(float(kx), _FIELD_KY_MAX)
        cv2.line(panel, p0, p1, (40, 40, 40), 1)
        cv2.putText(panel, f"{kx}", (p1[0] + 3, p1[1]),
                    font, 0.75, (50, 50, 50), 1)
    for ky in range(0, int(_FIELD_KY_MAX) + 1):
        p0 = _field_to_px(0.0,           float(ky))
        p1 = _field_to_px(_FIELD_KX_MAX, float(ky))
        cv2.line(panel, p0, p1, (40, 40, 40), 1)
        cv2.putText(panel, f"{ky}", (p0[0], p0[1] + 10),
                    font, 0.75, (50, 50, 50), 1)

    # ── Field boundary ────────────────────────────────────────────────────────
    tl = _field_to_px(0.0,           0.0)
    br = _field_to_px(_FIELD_KX_MAX, _FIELD_KY_MAX)
    cv2.rectangle(panel, tl, br, (80, 80, 80), 1)

    # ── Landmarks ─────────────────────────────────────────────────────────────
    for lm in slam.get_landmarks():
        lx, ly = lm['x'], lm['y']
        pu, pv = _field_to_px(lx, ly)

        if lm['aruco_id'] is not None:
            # ArUco landmark: orange diamond
            s = 7
            pts = np.array([[pu, pv - s], [pu + s, pv],
                            [pu, pv + s], [pu - s, pv]], np.int32)
            cv2.fillPoly(panel, [pts], (0, 140, 255))
            cv2.polylines(panel, [pts], True, (0, 200, 255), 1)
            cv2.putText(panel, str(lm['aruco_id']),
                        (pu + 9, pv + 4), font, 0.8, (0, 200, 255), 1)
        elif lm['reliable']:
            # Reliable visual feature: filled cyan
            cv2.circle(panel, (pu, pv), 4, (200, 220, 0), -1)
        else:
            # Tentative visual feature: hollow blue
            cv2.circle(panel, (pu, pv), 3, (140, 80, 0), 1)

    # ── Reference pose (kalman log ground truth) ─────────────────────────────
    if ref_pose is not None:
        rx, ry, ryaw = ref_pose
        ru, rv = _field_to_px(rx, ry)
        al = 20
        cv2.circle(panel, (ru, rv), 6, (120, 120, 0), 1)
        cv2.arrowedLine(panel,
                        (ru, rv),
                        (int(ru + al * math.sin(ryaw)),
                         int(rv - al * math.cos(ryaw))),
                        (140, 140, 0), 1, tipLength=0.4)

    # ── Robot pose ────────────────────────────────────────────────────────────
    rx, ry, ryaw, P3 = slam.get_pose()
    ru, rv = _field_to_px(rx, ry)
    al = 30
    cv2.circle(panel, (ru, rv), 8, (0, 230, 70), -1)
    cv2.arrowedLine(panel,
                    (ru, rv),
                    (int(ru + al * math.sin(ryaw)),
                     int(rv - al * math.cos(ryaw))),
                    (0, 255, 90), 2, tipLength=0.35)

    # ── Pose uncertainty ellipse (1-sigma from P) ─────────────────────────────
    p_xy   = P3[:2, :2]
    eigval, eigvec = np.linalg.eigh(p_xy)
    eigval = np.maximum(eigval, 0.0)
    # Scale eigenvectors to pixels (px/m conversion approximation)
    px_per_m = (_MAP_H - 2 * _MARGIN) / _FIELD_KX_MAX
    axes_px  = (max(3, int(math.sqrt(eigval[0]) * px_per_m)),
                max(3, int(math.sqrt(eigval[1]) * px_per_m)))
    angle_deg = math.degrees(math.atan2(float(eigvec[1, 0]), float(eigvec[0, 0])))
    cv2.ellipse(panel, (ru, rv), axes_px, angle_deg, 0, 360, (0, 100, 40), 1)

    # ── HUD text ──────────────────────────────────────────────────────────────
    mode_color = (0, 200, 100) if mode == 'map' else (100, 200, 255)
    n_lm  = slam.get_n_landmarks()
    n_rel = slam.get_n_reliable_landmarks()
    n_aru = slam.get_n_aruco_landmarks()

    cv2.putText(panel, f"MODE: {mode.upper()}",
                (8, 18), font, 1.1, mode_color, 1)
    cv2.putText(panel, f"x={rx:.2f}m  y={ry:.2f}m",
                (8, 34), font, 1.0, (0, 230, 70), 1)
    cv2.putText(panel, f"yaw={math.degrees(ryaw):+.1f} deg",
                (8, 50), font, 1.0, (0, 230, 70), 1)
    cv2.putText(panel, f"LM total={n_lm}  ArUco={n_aru}  reliable={n_rel}",
                (8, _MAP_H - 22), font, 0.9, (160, 160, 160), 1)

    # Legend
    cv2.circle(panel, (_MAP_W - 120, _MAP_H - 38), 4, (200, 220, 0), -1)
    cv2.putText(panel, "visual (reliable)",
                (_MAP_W - 110, _MAP_H - 34), font, 0.8, (200, 220, 0), 1)
    cv2.circle(panel, (_MAP_W - 120, _MAP_H - 22), 3, (140, 80, 0), 1)
    cv2.putText(panel, "visual (tentative)",
                (_MAP_W - 110, _MAP_H - 18), font, 0.8, (140, 80, 0), 1)
    pts = np.array([[_MAP_W - 120, _MAP_H - 10],
                    [_MAP_W - 113, _MAP_H - 4],
                    [_MAP_W - 120, _MAP_H + 2],
                    [_MAP_W - 127, _MAP_H - 4]], np.int32)
    cv2.fillPoly(panel, [pts], (0, 140, 255))
    cv2.putText(panel, "ArUco SLAM",
                (_MAP_W - 110, _MAP_H - 2), font, 0.8, (0, 200, 255), 1)

    return panel


# =============================================================================
# Camera panel renderer
# =============================================================================

def draw_camera_panel(frame: np.ndarray,
                      line_res: dict,
                      aruco_ds: list,
                      feats:    list,
                      debug:    bool = False) -> np.ndarray:
    """
    Render the camera overlay panel.

    Shows:
      - sline debug annotation (if debug=True)
      - ArUco detection overlay
      - Detected feature points as small dots (blue=FAST, cyan=tape_corner)
      - HUD lines for tape and ArUco status
    """
    from sline import vision as sv
    from saruco import aruco_detector as sd

    if debug and sd.debug_frame is not None:
        vis = sd.debug_frame.copy()
    elif debug and sv.debug_frame is not None:
        vis = sv.debug_frame.copy()
    else:
        vis = frame.copy()

    font = cv2.FONT_HERSHEY_PLAIN

    # ── Feature points overlay ────────────────────────────────────────────────
    for f in feats:
        u, v = int(f['px'][0]), int(f['px'][1])
        color = (255, 220, 50) if f['type'] == 'tape_corner' else (200, 80, 50)
        cv2.circle(vis, (u, v), 3, color, -1)

    # ── HUD ───────────────────────────────────────────────────────────────────
    line_valid = line_res.get('valid') or line_res.get('line_valid', False)
    tape_txt   = (f"TAPE  off={line_res.get('line_offset', 0.0):+.3f}m  "
                  f"hdg={math.degrees(line_res.get('line_heading', 0.0)):+.1f}d"
                  if line_valid else "TAPE  not detected")
    cv2.putText(vis, tape_txt, (6, 20), font, 1.0,
                (0, 230, 70) if line_valid else (80, 80, 80), 1)

    if aruco_ds:
        for i, d in enumerate(aruco_ds[:4]):
            cv2.putText(vis,
                        f"ArUco id={d['id']}  r={d['range']:.2f}m  "
                        f"b={math.degrees(d['bearing']):+.1f}d",
                        (6, 38 + i * 16), font, 0.95, (0, 180, 255), 1)
    else:
        cv2.putText(vis, "ArUco  none", (6, 38), font, 0.95, (80, 80, 80), 1)

    cv2.putText(vis, f"feats={len(feats)}", (6, vis.shape[0] - 6),
                font, 0.9, (180, 180, 180), 1)

    return vis


# =============================================================================
# Argument parser
# =============================================================================

def _make_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="EKF SLAM runner (pure vision, no AMCL)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument('--mode',       choices=['map', 'localize'], default='map')
    p.add_argument('--source',     choices=['live', 'video', 'images', 'image'],
                   default='live')
    p.add_argument('--path',       default=None,
                   help='Path to video dir / image folder / image file')
    p.add_argument('--host',       default='localhost',
                   help='Robot IP (live source)')
    p.add_argument('--cam-port',   type=int, default=7123)
    p.add_argument('--map-file',   default='slam_map.json',
                   help='Map JSON file to save (map mode) or load (localize mode)')
    p.add_argument('--init',       default=None,
                   help='Initial pose: x,y,yaw_deg  (Kalman frame)')
    p.add_argument('--speed',      type=float, default=1.0,
                   help='Replay speed multiplier')
    p.add_argument('--no-display', action='store_true')
    p.add_argument('--debug',      action='store_true',
                   help='Enable sline/aruco debug overlays on camera panel')
    p.add_argument('--pause',      action='store_true',
                   help='Start paused')

    # Kalman EKF fusion controls
    p.add_argument('--no-kalman-fusion', dest='no_kalman_fusion',
                   action='store_true',
                   help='Disable on-board Kalman EKF pose fusion '
                        '(use raw odometry only)')
    p.add_argument('--kalman-sigma-xy',  dest='kalman_sigma_xy',
                   type=float, default=0.10,
                   help='Position uncertainty of the Kalman EKF estimate (m)')
    p.add_argument('--kalman-sigma-yaw', dest='kalman_sigma_yaw',
                   type=float, default=0.08,
                   help='Heading uncertainty of the Kalman EKF estimate (rad)')
    return p


# =============================================================================
# Main run function
# =============================================================================

def run(args):
    here       = Path(__file__).parent
    calib_path = str(here / 'calibration' / 'camera_params.npz')

    # ── Setup perception ──────────────────────────────────────────────────────
    sline_vision.setup(params_path=calib_path, debug=args.debug)

    aruco_det = SArucoDetector()
    aruco_det.setup(params_path=calib_path, debug=args.debug)

    feature_extractor.setup(params_path=calib_path)

    # ── Setup EKF SLAM ────────────────────────────────────────────────────────
    slam = SEkfSlam()
    
    # Load known ArUco positions from field map (convert to EKF coordinates)
    known_landmarks = {}
    try:
        from field_map_2026 import FIELD
        for marker in FIELD.all_aruco():
            # Field map: x=lateral, y=forward
            # EKF: x=forward, y=lateral
            # So: ekf_x = fmap_y, ekf_y = fmap_x
            ekf_x = marker.position.y  # forward = field_map y
            ekf_y = marker.position.x  # lateral = field_map x
            known_landmarks[marker.id] = (ekf_x, ekf_y)
        print(f"% slam_runner: loaded {len(known_landmarks)} known ArUco landmarks from field map")
    except Exception as e:
        print(f"% slam_runner: WARNING - could not load field map ArUco positions: {e}")
    
    slam.setup(known_landmarks=known_landmarks)

    if args.mode == 'localize':
        map_path = Path(args.map_file)
        if not map_path.exists():
            print(f"% slam_runner: ERROR -- map file not found: {map_path}")
            print(f"%   Run with --mode map first to build a map.")
            sys.exit(1)
        slam.load_map(str(map_path))
        # localize_only is automatically set by load_map
    else:
        slam.localize_only = False
        print(f"% slam_runner: MAPPING mode  map will be saved to {args.map_file}")

    # ── Parse initial pose ────────────────────────────────────────────────────
    init_x, init_y, init_yaw = 0.0, 0.0, 0.0
    if args.init:
        try:
            parts     = [float(v) for v in args.init.split(',')]
            init_x, init_y = parts[0], parts[1]
            init_yaw  = math.radians(parts[2])
        except (ValueError, IndexError):
            print(f"% slam_runner: bad --init '{args.init}' -- using 0,0,0")

    # ── Build source ──────────────────────────────────────────────────────────
    source = build_source(args)
    source.setup()

    frame_iter = source.frames()
    try:
        first_info = next(frame_iter)
    except StopIteration:
        print("% slam_runner: source is empty -- nothing to do")
        source.release()
        return

    # Initialise pose from first kalman log entry if available
    if first_info.has_ref and args.mode == 'map':
        init_x, init_y, init_yaw = (first_info.ref_x,
                                     first_info.ref_y,
                                     first_info.ref_yaw)
        print(f"% slam_runner: seeding pose from kalman log  "
              f"x={init_x:.2f}  y={init_y:.2f}  "
              f"yaw={math.degrees(init_yaw):.1f}d")

    if not slam.initialized:
        slam.init_pose(init_x, init_y, init_yaw)

    # ── Display window ────────────────────────────────────────────────────────
    win_name = 'SLAM Runner'
    if not args.no_display:
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win_name, 1100, 520)

    paused   = args.pause
    step_req = False

    def process_frame(info: FrameInfo):
        # ── 1. Motion predict (encoder odometry) ─────────────────────────────
        if info.dt > 0:
            slam.predict(info.v, info.omega, info.dt)

        # ── 2. Kalman EKF pose fusion (encoder + IMU, primary estimate) ───────
        # Treat the on-board Kalman EKF as a noisy pose measurement.
        # Visual landmark updates below then act as a slip-correction layer.
        if info.has_ref and not args.no_kalman_fusion:
            slam.update_kalman_pose(info.ref_x, info.ref_y, info.ref_yaw,
                                    sigma_xy=args.kalman_sigma_xy,
                                    sigma_yaw=args.kalman_sigma_yaw)

        # ── 3. Slope correction — update camera tilt from Kalman pitch ────────
        # On a slope the robot pitches, changing the camera's effective angle
        # to the ground.  This corrects pixel_to_ground for each frame.
        if info.has_ref and info.ref_pitch != 0.0:
            from sground import ground as sg
            sg.set_robot_pitch(info.ref_pitch)

        # ── 4. Perception ─────────────────────────────────────────────────────
        line_res = sline_vision.process(info.frame)
        aruco_ds = aruco_det.process(info.frame)
        feats    = feature_extractor.extract(info.frame)

        # ── 4. Visual SLAM updates (slip correction) ──────────────────────────
        # ArUco: known landmarks from field map (precise positioning)
        for det in aruco_ds:
            aruco_id = int(det['id'])
            if aruco_id in known_landmarks:
                # Known ArUco marker - use precise field map position
                slam.update_known(aruco_id,
                                 float(det['range']),
                                 float(det['bearing']))
            else:
                # Unknown ArUco - treat as SLAM landmark
                slam.update_aruco_slam(aruco_id,
                                       float(det['range']),
                                       float(det['bearing']))

        # Visual features: Mahalanobis-gated landmarks
        n_match, n_new = slam.update_features(feats)

        # ── 5. Estimate ───────────────────────────────────────────────────────
        rx, ry, ryaw, _ = slam.get_pose()

        # ── 6. Console ───────────────────────────────────────────────────────
        mode_tag = 'MAP' if not slam.localize_only else 'LOC'
        fuse_tag = '+K' if (info.has_ref and not args.no_kalman_fusion) else '  '
        err_str  = ''
        if info.has_ref:
            err = math.sqrt((rx - info.ref_x) ** 2 + (ry - info.ref_y) ** 2)
            err_str = f'  ref_err={err:.3f}m'
        n_lm  = slam.get_n_landmarks()
        n_aru = slam.get_n_aruco_landmarks()
        print(f"[{info.frame_idx:04d}] {mode_tag}{fuse_tag}  "
              f"x={rx:.3f}m  y={ry:.3f}m  yaw={math.degrees(ryaw):+.1f}d  "
              f"lm={n_lm}(A:{n_aru},V:{n_lm - n_aru})  "
              f"match={n_match}  new={n_new}"
              f"{err_str}")

        # ── 6. Display ────────────────────────────────────────────────────────
        if args.no_display:
            return

        # Camera panel
        cam = draw_camera_panel(info.frame, line_res, aruco_ds, feats, args.debug)
        h, w = cam.shape[:2]
        target_h = _MAP_H
        cam = cv2.resize(cam, (int(w * target_h / h), target_h))

        # Map panel
        ref = (info.ref_x, info.ref_y, info.ref_yaw) if info.has_ref else None
        mp  = draw_map_panel(slam, args.mode, ref_pose=ref)

        # Combine side-by-side
        h1, w1 = cam.shape[:2]
        h2, w2 = mp.shape[:2]
        canvas_h = max(h1, h2)
        canvas   = np.zeros((canvas_h, w1 + w2, 3), dtype=np.uint8)
        canvas[:h1, :w1]      = cam
        canvas[:h2, w1:w1+w2] = mp

        status = (f"{info.source_name}  frame={info.frame_idx}  "
                  f"{'[PAUSED]' if paused else ''}")
        cv2.putText(canvas, status, (6, canvas_h - 5),
                    cv2.FONT_HERSHEY_PLAIN, 0.9, (160, 160, 160), 1)
        cv2.imshow(win_name, canvas)

    # ── Main loop ─────────────────────────────────────────────────────────────
    process_frame(first_info)

    for info in frame_iter:
        if not args.no_display:
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):           # quit
                break
            elif key == ord(' '):
                paused = not paused
                print(f"% {'Paused' if paused else 'Resumed'}")
            elif key == ord('s'):
                if paused:
                    step_req = True             # step one frame
                else:
                    _save_map(slam, args.map_file)
            elif key == ord('r'):
                if slam.localize_only:
                    # Reset pose uncertainty so filter re-localises
                    slam._P[0, 0] = 1.5 ** 2
                    slam._P[1, 1] = 1.5 ** 2
                    slam._P[2, 2] = math.pi ** 2
                    print("% slam_runner: pose uncertainty reset")
                else:
                    slam.__init__()
                    slam.init_pose(init_x, init_y, init_yaw)
                    print("% slam_runner: SLAM reset (map cleared)")

        if paused and not step_req:
            time.sleep(0.05)
            if not args.no_display:
                cv2.waitKey(1)
            continue

        step_req = False
        process_frame(info)

    source.release()

    # In mapping mode: always save on exit
    if args.mode == 'map':
        _save_map(slam, args.map_file)

    # Hold window open until user presses q / ESC
    if not args.no_display:
        print("% Press q or ESC in the window to quit")
        while True:
            if cv2.waitKey(50) & 0xFF in (ord('q'), 27):
                break
        cv2.destroyAllWindows()


def _save_map(slam: SEkfSlam, map_file: str):
    """Save the current SLAM map, printing a warning if no landmarks exist."""
    if slam.get_n_landmarks() == 0:
        print("% slam_runner: no landmarks to save -- drive around first")
        return
    try:
        slam.save_map(map_file)
    except OSError as e:
        print(f"% slam_runner: could not save map: {e}")


# =============================================================================
# Entry point
# =============================================================================

def main():
    parser = _make_parser()
    args   = parser.parse_args()

    if args.source != 'live' and not args.path:
        if args.source in ('video', 'images', 'image'):
            parser.error(f"--source {args.source} requires --path")

    run(args)


if __name__ == '__main__':
    main()
