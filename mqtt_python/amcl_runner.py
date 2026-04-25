#!/usr/bin/env python3
"""
amcl_runner.py
=============================================================================
AMCL test runner – run the particle-filter localiser against:

  --source live    Live MJPEG camera stream from the robot
  --source video   Recorded video (.mp4) + Kalman log (.jsonl)
  --source images  Folder of JPEG snapshots (+ optional Kalman log)
  --source image   Single image file

Usage examples
--------------
# Live (robot must be reachable):
    python3 amcl_runner.py --source live --host 10.197.218.17

# Replay a recorded session (video + Kalman log):
    python3 amcl_runner.py --source video --path recordings/test1

# Replay an image folder:
    python3 amcl_runner.py --source images --path recordings/pic/snapshots

# Single image:
    python3 amcl_runner.py --source image --path path/to/frame.jpg

General options
---------------
  --host HOST          Robot IP (live mode)            [localhost]
  --cam-port PORT      MJPEG port                      [7123]
  --particles N        Number of AMCL particles        [500]
  --init x,y,yaw_deg   Initial pose (Kalman frame)     [uniform]
  --spread-xy M        Position spread for init        [0.30 m]
  --spread-yaw D       Heading spread for init         [20 deg]
  --no-display         Headless mode (no cv2 window)
  --debug              Enable sline/saruco debug frames
  --pause              Start paused (press SPACE to step)
  --speed F            Replay speed multiplier         [1.0]

Controls (cv2 window)
---------------------
  SPACE  toggle pause / resume
  s      step one frame (when paused)
  r      re-initialise particles (uniform)
  q / ESC  quit

Display layout
--------------
  Left panel  : camera frame with sline + saruco overlay
  Right panel : top-down field map with particles + AMCL estimate + ground truth
=============================================================================
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Iterator, Optional, Tuple

import cv2
import numpy as np

# ── Perception modules ────────────────────────────────────────────────────────
from sline import vision as sline_vision
from saruco import SArucoDetector
from samcl import SAMCL
from sfeatures import feature_extractor as slam_features
from sekf_slam import SEkfSlam

# ── Field map ─────────────────────────────────────────────────────────────────
try:
    from field_map_2026 import FIELD
    _FIELD_LOADED = True
except Exception as e:
    print(f"% amcl_runner: WARNING – field map could not be loaded: {e}")
    FIELD = None
    _FIELD_LOADED = False


# =============================================================================
# Source abstractions
# =============================================================================

class FrameInfo:
    """One iteration's worth of data from any source."""
    __slots__ = ('frame', 'v', 'omega', 'dt',
                 'ref_x', 'ref_y', 'ref_yaw',    # ground-truth from kalman log
                 'has_ref', 'frame_idx', 'source_name')

    def __init__(self):
        self.frame:       Optional[np.ndarray] = None
        self.v:           float = 0.0
        self.omega:       float = 0.0
        self.dt:          float = 0.0
        self.ref_x:       float = 0.0
        self.ref_y:       float = 0.0
        self.ref_yaw:     float = 0.0
        self.has_ref:     bool  = False
        self.frame_idx:   int   = 0
        self.source_name: str   = ''


def _load_kalman_log(log_path: Path):
    """
    Load a kalman_log.jsonl and return a list of parsed entries, sorted
    by epoch time.  Supports both v1 format (direct data) and v2 format
    (wrapped with "data" key).
    """
    entries = []
    try:
        with open(log_path, 'r') as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except json.JSONDecodeError:
                    continue
                # Normalise to flat structure
                data = obj.get('data', obj)
                # Extract state
                x_state = data.get('x', {})
                meas    = data.get('measurements', {})
                entries.append({
                    't':       float(obj.get('t', 0.0)),
                    'video_t': obj.get('video_t'),
                    'kx':      float(x_state.get('x',   meas.get('enc_x', 0.0))),
                    'ky':      float(x_state.get('y',   meas.get('enc_y', 0.0))),
                    'kyaw':    float(x_state.get('yaw', meas.get('enc_yaw', 0.0))),
                    'enc_v':   float(meas.get('enc_v', 0.0)),
                    'enc_om':  float(meas.get('enc_om', 0.0)),
                    'dt':      float(meas.get('dt', 0.01)),
                })
    except FileNotFoundError:
        print(f"% amcl_runner: kalman log not found: {log_path}")
    entries.sort(key=lambda e: e['t'])
    return entries


# ---------------------------------------------------------------------------
# Live source
# ---------------------------------------------------------------------------

class LiveSource:
    """
    Yields frames from the robot MJPEG stream.
    Odometry comes from the Kalman MQTT topic if available;
    otherwise motion_update is skipped.
    """

    def __init__(self, host: str, cam_port: int = 7123):
        self._host     = host
        self._cam_port = cam_port
        self._cap      = None
        self._prev_t   = None
        self._mqtt_odom: dict = {}   # latest enc_v, enc_om from MQTT

    def setup(self):
        url = f'http://{self._host}:{self._cam_port}/stream.mjpg'
        print(f"% LiveSource: opening {url}")
        self._cap = cv2.VideoCapture(url)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera stream at {url}")
        print("% LiveSource: camera opened")
        self._prev_t = time.time()
        self._start_mqtt()

    def _start_mqtt(self):
        """Try to subscribe to Kalman state via MQTT for odometry."""
        try:
            import paho.mqtt.client as mqtt_client
            if hasattr(mqtt_client, "CallbackAPIVersion"):
                client = mqtt_client.Client(
                    client_id="amcl-runner-odom",
                    callback_api_version=mqtt_client.CallbackAPIVersion.VERSION1,
                )
            else:
                client = mqtt_client.Client(client_id="amcl-runner-odom")

            def on_connect(c, u, f, rc):
                if rc == 0:
                    c.subscribe("robobot/kalman/state")
                    print("% LiveSource: MQTT odom subscribed")

            def on_message(c, u, msg):
                try:
                    obj  = json.loads(msg.payload)
                    data = obj.get('data', obj)
                    meas = data.get('measurements', {})
                    self._mqtt_odom = {
                        'enc_v':  float(meas.get('enc_v',  0.0)),
                        'enc_om': float(meas.get('enc_om', 0.0)),
                    }
                except Exception:
                    pass

            client.on_connect = on_connect
            client.on_message = on_message
            client.connect(self._host, 1883, keepalive=30)
            client.loop_start()
        except Exception as e:
            print(f"% LiveSource: MQTT not available ({e}) – odometry disabled")

    def frames(self) -> Iterator[FrameInfo]:
        idx = 0
        while True:
            ret, frame = self._cap.read()
            if not ret:
                print("% LiveSource: no frame – retrying")
                time.sleep(0.1)
                continue

            now = time.time()
            dt  = now - self._prev_t if self._prev_t else 0.033
            self._prev_t = now

            info             = FrameInfo()
            info.frame       = frame
            info.v           = self._mqtt_odom.get('enc_v',  0.0)
            info.omega       = self._mqtt_odom.get('enc_om', 0.0)
            info.dt          = float(np.clip(dt, 0.001, 0.2))
            info.has_ref     = False
            info.frame_idx   = idx
            info.source_name = f'live:{self._host}'
            idx += 1
            yield info

    def release(self):
        if self._cap:
            self._cap.release()


# ---------------------------------------------------------------------------
# Video source
# ---------------------------------------------------------------------------

class VideoSource:
    """
    Yields frames from a recorded video.mp4 synchronised with the
    associated kalman_log.jsonl for ground-truth and odometry.

    Sync strategy: use epoch time ('t' field in the log).
    video_t in the log is often null (MQTT logger starts before the video
    writer opens), so we infer the video start epoch as the log's first
    entry epoch and match each frame by  t_frame = t_log0 + frame_idx/fps.
    """

    def __init__(self, session_dir: Path, speed: float = 1.0):
        self._dir         = Path(session_dir)
        self._speed       = max(0.1, speed)
        self._cap         = None
        self._log         = []       # list of log entries sorted by 't'
        self._fps         = 8.0
        self._log_t0      = 0.0     # epoch time of video frame 0 (inferred)

    def setup(self):
        video_path = self._dir / 'video.mp4'
        log_path   = self._dir / 'kalman_log.jsonl'

        if not video_path.exists():
            raise FileNotFoundError(f"No video.mp4 in {self._dir}")

        self._cap = cv2.VideoCapture(str(video_path))
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open {video_path}")

        self._fps = self._cap.get(cv2.CAP_PROP_FPS) or 8.0
        total     = int(self._cap.get(cv2.CAP_PROP_FRAME_COUNT))
        print(f"% VideoSource: {video_path.name}  fps={self._fps:.1f}  frames={total}")

        if log_path.exists():
            self._log = _load_kalman_log(log_path)
            if self._log:
                # Use the epoch time of the first log entry as the video
                # start time.  The recording script starts camera + MQTT
                # logger together, so their timestamps are aligned.
                # video_t in log entries is often null (race condition in
                # record.py), so we always use epoch time matching.
                self._log_t0 = self._log[0]['t']
                t_span = self._log[-1]['t'] - self._log_t0
                print(f"% VideoSource: {len(self._log)} kalman entries  "
                      f"span={t_span:.1f}s  "
                      f"(syncing by epoch time, video_t ignored)")
            else:
                print("% VideoSource: kalman_log.jsonl is empty")
        else:
            print("% VideoSource: no kalman_log.jsonl – ground-truth disabled")

    def frames(self) -> Iterator[FrameInfo]:
        """Yield FrameInfo objects at real-time (scaled by speed)."""
        idx      = 0
        log_idx  = 0          # cursor into sorted log list
        frame_dt = 1.0 / self._fps

        while True:
            ret, frame = self._cap.read()
            if not ret:
                print("% VideoSource: end of video")
                break

            info             = FrameInfo()
            info.frame       = frame
            info.dt          = frame_dt
            info.frame_idx   = idx
            info.source_name = f'video:{self._dir.name}'

            if self._log:
                # Target epoch for this video frame
                target_t = self._log_t0 + idx / self._fps

                # Advance log cursor to the entry closest to target_t
                while (log_idx + 1 < len(self._log) and
                       abs(self._log[log_idx + 1]['t'] - target_t) <
                       abs(self._log[log_idx    ]['t'] - target_t)):
                    log_idx += 1

                entry = self._log[log_idx]

                info.ref_x   = entry['kx']
                info.ref_y   = entry['ky']
                info.ref_yaw = entry['kyaw']
                info.has_ref = True

                # dt and odometry from the log entry itself
                info.v     = entry['enc_v']
                info.omega = entry['enc_om']
                info.dt    = max(entry['dt'], 1.0 / self._fps)

            idx += 1
            yield info

            # Real-time pacing (respect speed multiplier)
            time.sleep(frame_dt / self._speed)

    def release(self):
        if self._cap:
            self._cap.release()


# ---------------------------------------------------------------------------
# Images source  (folder of JPEG files)
# ---------------------------------------------------------------------------

class ImagesSource:
    """
    Yields frames from a folder of JPEG images, sorted by name.
    Optional kalman_log.jsonl or per-image .json sidecars provide ground truth.
    """

    def __init__(self, folder: Path, speed: float = 1.0):
        self._folder = Path(folder)
        self._speed  = max(0.1, speed)
        self._paths  = []
        self._log    = []

    def setup(self):
        exts   = {'.jpg', '.jpeg', '.png'}
        images = sorted(
            p for p in self._folder.iterdir()
            if p.suffix.lower() in exts
        )
        if not images:
            raise FileNotFoundError(f"No images found in {self._folder}")
        self._paths = images
        print(f"% ImagesSource: {len(images)} images in {self._folder}")

        # Look for a kalman_log.jsonl alongside the folder or in its parent
        for candidate in (self._folder / 'kalman_log.jsonl',
                          self._folder.parent / 'kalman_log.jsonl'):
            if candidate.exists():
                self._log = _load_kalman_log(candidate)
                print(f"% ImagesSource: loaded {len(self._log)} kalman entries")
                break

    def frames(self) -> Iterator[FrameInfo]:
        for idx, p in enumerate(self._paths):
            frame = cv2.imread(str(p))
            if frame is None:
                print(f"% ImagesSource: cannot read {p}")
                continue

            info            = FrameInfo()
            info.frame      = frame
            info.v          = 0.0
            info.omega      = 0.0
            info.dt         = 0.0      # no motion between images
            info.frame_idx  = idx
            info.source_name = f'images:{self._folder.name}'

            # Try per-image .json sidecar
            sidecar = p.with_suffix('.json')
            if sidecar.exists():
                try:
                    with open(sidecar) as fh:
                        sj = json.load(fh)
                    ks = sj.get('kalman', {})
                    xs = (ks.get('x', {}) if isinstance(ks, dict) else {})
                    data = ks.get('data', xs) if isinstance(ks, dict) else {}
                    xst  = data.get('x', xs)
                    info.ref_x   = float(xst.get('x',   0.0))
                    info.ref_y   = float(xst.get('y',   0.0))
                    info.ref_yaw = float(xst.get('yaw', 0.0))
                    info.has_ref = True
                except Exception as e:
                    print(f"% ImagesSource: sidecar parse error {sidecar}: {e}")

            yield info
            time.sleep(0.1 / self._speed)   # small delay between frames

    def release(self):
        pass


# ---------------------------------------------------------------------------
# Single-image source
# ---------------------------------------------------------------------------

class SingleImageSource:
    def __init__(self, path: str):
        self._path = Path(path)

    def setup(self):
        if not self._path.exists():
            raise FileNotFoundError(self._path)
        print(f"% SingleImageSource: {self._path}")

    def frames(self) -> Iterator[FrameInfo]:
        ext = self._path.suffix.lower()
        if ext == '.json':
            # JSON sidecar – try to find companion image
            with open(self._path) as fh:
                sj = json.load(fh)
            img_path = self._path.with_suffix('.jpg')
            if not img_path.exists():
                print(f"% SingleImageSource: no companion image for {self._path}")
                return
            frame = cv2.imread(str(img_path))
            ks    = sj.get('kalman', {})
            data  = ks.get('data', ks) if isinstance(ks, dict) else {}
            xst   = data.get('x', {})
            info            = FrameInfo()
            info.frame      = frame
            info.ref_x      = float(xst.get('x',   0.0))
            info.ref_y      = float(xst.get('y',   0.0))
            info.ref_yaw    = float(xst.get('yaw', 0.0))
            info.has_ref    = True
            info.source_name = f'image:{self._path.name}'
        else:
            frame = cv2.imread(str(self._path))
            if frame is None:
                raise FileNotFoundError(f"Cannot read image {self._path}")
            info            = FrameInfo()
            info.frame      = frame
            info.source_name = f'image:{self._path.name}'

        yield info   # single frame

    def release(self):
        pass


# =============================================================================
# Visualisation helpers
# =============================================================================

# Field dimensions in Kalman frame
_FIELD_KX_MAX = 6.0   # forward
_FIELD_KY_MAX = 7.0   # lateral

# Map panel size in pixels
_MAP_W = 560
_MAP_H = 480

# Pre-computed tape-segment pixel endpoints (built once, reused every frame)
_TAPE_PX_LINES: list = []   # list of ((u0,v0),(u1,v1))


def _field_to_px(kx: float, ky: float) -> Tuple[int, int]:
    """Convert Kalman-frame (kx, ky) → pixel in map panel."""
    margin = 20
    px_x = int(margin + (ky / _FIELD_KY_MAX) * (_MAP_W - 2 * margin))
    px_y = int((_MAP_H - margin) - (kx / _FIELD_KX_MAX) * (_MAP_H - 2 * margin))
    return (px_x, px_y)


def _build_tape_px_cache(amcl: SAMCL) -> None:
    """Build _TAPE_PX_LINES once from the AMCL's pre-loaded segments."""
    global _TAPE_PX_LINES
    _TAPE_PX_LINES = []
    for i in range(len(amcl._seg_len)):
        p0 = amcl._seg_p0[i]
        p1 = amcl._seg_p1[i]
        _TAPE_PX_LINES.append((_field_to_px(p0[0], p0[1]),
                                _field_to_px(p1[0], p1[1])))


# Pre-computed field landmark decorations (ArUco, roundabout, platform, …)
_FIELD_DECORATIONS: list = []   # list of ('type', *args) tuples
_PX_PER_M = (_MAP_H - 40) / _FIELD_KX_MAX  # ~73 px/m (roughly isotropic)


def _fp(field_x_lat: float, field_y_fwd: float) -> Tuple[int, int]:
    """Convert field frame (x=lateral, y=forward) → panel pixel."""
    return _field_to_px(field_y_fwd, field_x_lat)  # kx=fwd, ky=lat


def _build_field_decoration_cache(field) -> None:
    """
    Pre-build drawing ops for field landmarks so draw_map_panel is O(1).
    Field frame: x=lateral (0-7m), y=forward (0-6m).
    Kalman frame: kx=forward, ky=lateral  (conversion: kx=field.y, ky=field.x).
    """
    global _FIELD_DECORATIONS
    _FIELD_DECORATIONS = []
    if field is None:
        return

    # Roundabout circle
    if field.roundabout:
        r     = field.roundabout
        cx, cy = _fp(r.center.x, r.center.y)
        r_px  = max(4, int(r.radius * _PX_PER_M))
        _FIELD_DECORATIONS.append(('circle', (cx, cy), r_px, (160, 100,  50), 2))
        _FIELD_DECORATIONS.append(('text',   (cx - 10, cy + 4), 'RB', (160, 100, 50)))

    # Platform rectangle
    if field.platform:
        p  = field.platform
        tl = _fp(p.origin.x,           p.origin.y)
        br = _fp(p.origin.x + p.width, p.origin.y + p.depth)
        _FIELD_DECORATIONS.append(('rect', tl, br, (120, 80, 200), 1))
        mp = _fp(p.origin.x + p.width / 2, p.origin.y + p.depth / 2)
        _FIELD_DECORATIONS.append(('text', (mp[0] - 14, mp[1] + 4), 'PLAT', (120, 80, 200)))

    # Start area
    if field.start_area:
        s  = field.start_area
        tl = _fp(s.origin.x,           s.origin.y)
        br = _fp(s.origin.x + s.width, s.origin.y + s.depth)
        _FIELD_DECORATIONS.append(('rect', tl, br, (0, 200, 200), 1))
        mp = _fp(s.origin.x + s.width / 2, s.origin.y + s.depth / 2)
        _FIELD_DECORATIONS.append(('text', (mp[0] - 4, mp[1] + 4), 'S', (0, 200, 200)))

    # Goal
    if field.goal_point:
        gp = _fp(field.goal_point.x, field.goal_point.y)
        _FIELD_DECORATIONS.append(('circle', gp, 6, (0, 200, 200), -1))
        _FIELD_DECORATIONS.append(('text', (gp[0] + 8, gp[1] + 4), 'GOAL', (0, 200, 200)))

    # Sorting center
    if field.sorting_center:
        sc    = field.sorting_center
        sp    = _fp(sc.center.x, sc.center.y)
        sz_px = max(4, int(sc.zone_size * _PX_PER_M))
        _FIELD_DECORATIONS.append(('rect',
                                   (sp[0] - sz_px, sp[1] - sz_px),
                                   (sp[0] + sz_px, sp[1] + sz_px),
                                   (180, 100, 200), 1))
        _FIELD_DECORATIONS.append(('text', (sp[0] - 8, sp[1] + 4), 'SC', (180, 100, 200)))

    # All ArUco markers
    for m in field.all_aruco():
        mp = _fp(m.position.x, m.position.y)
        _FIELD_DECORATIONS.append(('aruco', mp, str(m.id)))

    print(f"% amcl_runner: field decorations: {len(_FIELD_DECORATIONS)} items")


def draw_slam_overlay(panel: np.ndarray,
                      landmarks: list,
                      slam_pose: Optional[Tuple] = None) -> None:
    """
    Draw EKF SLAM landmarks and pose estimate onto an existing map panel
    (in-place).

    Parameters
    ----------
    panel     : BGR map panel (modified in-place)
    landmarks : list of dicts from SEkfSlam.get_landmarks()
    slam_pose : (x, y, yaw) EKF SLAM robot pose, or None
    """
    font = cv2.FONT_HERSHEY_PLAIN

    for lm in landmarks:
        lx, ly = lm['x'], lm['y']
        pu, pv = _field_to_px(lx, ly)
        if lm['reliable']:
            # Reliable landmark: filled cyan circle
            cv2.circle(panel, (pu, pv), 4, (255, 220, 0), -1)
        else:
            # Tentative landmark: hollow yellow circle
            cv2.circle(panel, (pu, pv), 3, (100, 180, 255), 1)

    if slam_pose is not None:
        sx, sy, syaw = slam_pose
        su, sv = _field_to_px(sx, sy)
        # Magenta arrow for EKF SLAM pose
        al = 24
        cv2.circle(panel, (su, sv), 8, (255, 0, 200), -1)
        cv2.arrowedLine(panel,
                        (su, sv),
                        (int(su + al * math.sin(syaw)),
                         int(sv - al * math.cos(syaw))),
                        (255, 80, 255), 2, tipLength=0.4)
        # Coordinate readout (right-aligned area)
        cv2.putText(panel, "EKF SLAM",
                    (_MAP_W - 110, 18), font, 1.0, (255, 80, 255), 1)
        cv2.putText(panel, f"x={sx:.2f}m",
                    (_MAP_W - 110, 34), font, 0.95, (255, 80, 255), 1)
        cv2.putText(panel, f"y={sy:.2f}m",
                    (_MAP_W - 110, 50), font, 0.95, (255, 80, 255), 1)
        cv2.putText(panel, f"yaw={math.degrees(syaw):.1f}d",
                    (_MAP_W - 110, 66), font, 0.95, (255, 80, 255), 1)

    # Legend
    cv2.circle(panel, (_MAP_W - 110, _MAP_H - 32), 4, (255, 220, 0), -1)
    cv2.putText(panel, "SLAM lm (reliable)",
                (_MAP_W - 100, _MAP_H - 28), font, 0.8, (255, 220, 0), 1)
    cv2.circle(panel, (_MAP_W - 110, _MAP_H - 16), 3, (100, 180, 255), 1)
    cv2.putText(panel, "SLAM lm (tentative)",
                (_MAP_W - 100, _MAP_H - 12), font, 0.8, (100, 180, 255), 1)


def draw_map_panel(particles:    np.ndarray,   # (N, 3) [x, y, yaw]
                   weights:      np.ndarray,   # (N,)
                   estimate:     Tuple,        # (x, y, yaw)
                   n_eff:        float = 0.0,
                   ) -> np.ndarray:
    """
    Render a clean top-down map panel.

    Shows:
      Blue lines : tape segments
      Faint dots : particle cloud (convergence indicator)
      Green arrow: AMCL estimated pose with coordinate text
    """
    panel = np.full((_MAP_H, _MAP_W, 3), 20, dtype=np.uint8)

    # ── Field boundary ──────────────────────────────────────────────────────
    tl = _field_to_px(0.0, 0.0)
    br = _field_to_px(_FIELD_KX_MAX, _FIELD_KY_MAX)
    cv2.rectangle(panel, tl, br, (70, 70, 70), 1)

    # ── Field landmarks (pre-built once) ────────────────────────────────────
    font = cv2.FONT_HERSHEY_PLAIN
    for dec in _FIELD_DECORATIONS:
        if dec[0] == 'circle':
            _, ctr, r_px, col, thick = dec
            cv2.circle(panel, ctr, r_px, col, thick)
        elif dec[0] == 'rect':
            _, pt1, pt2, col, thick = dec
            tl2 = (min(pt1[0], pt2[0]), min(pt1[1], pt2[1]))
            br2 = (max(pt1[0], pt2[0]), max(pt1[1], pt2[1]))
            cv2.rectangle(panel, tl2, br2, col, thick)
        elif dec[0] == 'text':
            _, pos, txt, col = dec
            cv2.putText(panel, txt, pos, font, 0.85, col, 1)
        elif dec[0] == 'aruco':
            _, pos, id_str = dec
            cv2.circle(panel, pos, 4, (200, 160, 60), -1)
            cv2.putText(panel, id_str, (pos[0] + 5, pos[1] + 4),
                        font, 0.75, (200, 160, 60), 1)

    # ── Tape segments ───────────────────────────────────────────────────────
    for (u0, v0), (u1, v1) in _TAPE_PX_LINES:
        cv2.line(panel, (u0, v0), (u1, v1), (0, 90, 200), 1)

    # ── Particle cloud (faint – just shows spread / convergence) ────────────
    for i in range(len(particles)):
        u, v = _field_to_px(particles[i, 0], particles[i, 1])
        cv2.circle(panel, (u, v), 2, (55, 55, 55), -1)

    # ── AMCL estimate (prominent) ────────────────────────────────────────────
    ex, ey, eyaw = estimate
    eu, ev = _field_to_px(ex, ey)
    cv2.circle(panel, (eu, ev), 10, (0, 230, 70), -1)
    al = 28
    cv2.arrowedLine(panel,
                    (eu, ev),
                    (int(eu + al * math.sin(eyaw)),
                     int(ev - al * math.cos(eyaw))),
                    (0, 255, 90), 2, tipLength=0.35)

    # ── Coordinate readout ──────────────────────────────────────────────────
    font = cv2.FONT_HERSHEY_PLAIN
    cv2.putText(panel, "ESTIMATED POSITION", (8, 18),  font, 1.0, (0, 230, 70), 1)
    cv2.putText(panel, f"x  = {ex:.3f} m (fwd)",  (8, 36),  font, 1.1, (0, 230, 70), 1)
    cv2.putText(panel, f"y  = {ey:.3f} m (lat)",  (8, 54),  font, 1.1, (0, 230, 70), 1)
    cv2.putText(panel, f"yaw= {math.degrees(eyaw):.1f} deg",  (8, 72),  font, 1.1, (0, 230, 70), 1)
    cv2.putText(panel, f"N_eff={n_eff:.0f}/{len(particles)}",
                (8, _MAP_H - 8), font, 0.9, (100, 100, 100), 1)

    return panel


def draw_camera_overlay(frame:    np.ndarray,
                        line_res: dict,
                        aruco:    list) -> np.ndarray:
    """
    Draw sline + saruco detection overlays on the camera frame.
    Shows all recognized points/features; no ground-truth info.
    """
    # Prefer the annotated debug frames if available
    from sline import vision as sv
    from saruco import aruco_detector as sd

    if sd.debug and sd.debug_frame is not None:
        vis = sd.debug_frame.copy()
    elif sv.debug and sv.debug_frame is not None:
        vis = sv.debug_frame.copy()
    else:
        vis = frame.copy()

    font  = cv2.FONT_HERSHEY_PLAIN
    line_valid = line_res.get('valid') or line_res.get('line_valid', False)
    offset     = float(line_res.get('line_offset', 0.0))
    heading_d  = math.degrees(float(line_res.get('line_heading', 0.0)))
    n_pts      = len(line_res.get('line_points_robot', []))

    # ── Tape / line detection ─────────────────────────────────────────────
    tape_color = (0, 230, 70) if line_valid else (80, 80, 80)
    tape_txt   = (f"TAPE  off={offset:+.3f}m  hdg={heading_d:+.1f}d  pts={n_pts}"
                  if line_valid else "TAPE  not detected")
    cv2.putText(vis, tape_txt, (6, 20), font, 1.1, tape_color, 1)

    # ── ArUco detections ──────────────────────────────────────────────────
    if aruco:
        for i, d in enumerate(aruco[:4]):
            txt = (f"ArUco id={d['id']}  "
                   f"range={d['range']:.2f}m  "
                   f"bearing={math.degrees(d['bearing']):+.1f}d")
            cv2.putText(vis, txt, (6, 40 + i * 17),
                        font, 1.0, (0, 180, 255), 1)
    else:
        cv2.putText(vis, "ArUco  none detected", (6, 40),
                    font, 1.0, (80, 80, 80), 1)

    return vis


# =============================================================================
# Main runner
# =============================================================================

def build_source(args):
    """Factory that creates the right source object from CLI args."""
    src = args.source.lower()

    if src == 'live':
        s = LiveSource(host=args.host, cam_port=args.cam_port)

    elif src == 'video':
        if not args.path:
            raise ValueError("--source video requires --path <session_dir>")
        s = VideoSource(session_dir=Path(args.path), speed=args.speed)

    elif src == 'images':
        if not args.path:
            raise ValueError("--source images requires --path <folder>")
        s = ImagesSource(folder=Path(args.path), speed=args.speed)

    elif src == 'image':
        if not args.path:
            raise ValueError("--source image requires --path <file>")
        s = SingleImageSource(path=args.path)

    else:
        raise ValueError(f"Unknown --source '{src}'.  "
                         "Choose: live | video | images | image")

    return s


def init_amcl(amcl: SAMCL, args, first_frame_info: Optional[FrameInfo] = None):
    """Initialise AMCL particles according to --init argument."""
    if args.init:
        try:
            parts = [float(v) for v in args.init.split(',')]
            ix, iy, iyaw_deg = parts[0], parts[1], parts[2]
            iyaw = math.radians(iyaw_deg)
            amcl.init_pose(ix, iy, iyaw,
                           spread_xy=args.spread_xy,
                           spread_yaw=math.radians(args.spread_yaw))
        except (ValueError, IndexError):
            print(f"% amcl_runner: bad --init value '{args.init}' – "
                  "expected x,y,yaw_deg")
            amcl.init_uniform()
    elif first_frame_info is not None and first_frame_info.has_ref:
        # Seed from first ground-truth entry with a moderate spread
        amcl.init_pose(first_frame_info.ref_x,
                       first_frame_info.ref_y,
                       first_frame_info.ref_yaw,
                       spread_xy=args.spread_xy,
                       spread_yaw=math.radians(args.spread_yaw))
        print(f"% amcl_runner: seeded from first kalman entry  "
              f"x={first_frame_info.ref_x:.2f}  "
              f"y={first_frame_info.ref_y:.2f}  "
              f"yaw={math.degrees(first_frame_info.ref_yaw):.1f}d")
    else:
        amcl.init_uniform()


def run(args):
    # ── Calibration path ──────────────────────────────────────────────────────
    here         = Path(__file__).parent
    calib_path   = str(here / 'calibration' / 'camera_params.npz')

    # ── Setup perception ──────────────────────────────────────────────────────
    # sline: always debug=True so line-point crosses are drawn on the frame
    sline_vision.setup(params_path=calib_path, debug=True)

    aruco_det = SArucoDetector()
    # aruco: debug=False — we draw detections manually on top of sline frame
    aruco_det.setup(params_path=calib_path, debug=False)

    # ── Setup AMCL ────────────────────────────────────────────────────────
    amcl = SAMCL()
    amcl.setup(field=FIELD if _FIELD_LOADED else None,
               n_particles=args.particles)
    _build_tape_px_cache(amcl)               # pre-compute tape pixels once
    _build_field_decoration_cache(FIELD if _FIELD_LOADED else None)

    # ── Setup EKF SLAM ───────────────────────────────────────────────────
    slam_features.setup(params_path=calib_path)

    # Build known-landmark dict from field ArUco positions (Kalman frame)
    _known_lm: dict = {}
    if _FIELD_LOADED and FIELD is not None:
        for _m in FIELD.all_aruco():
            _known_lm[int(_m.id)] = (float(_m.position.y),   # kx = field.y
                                     float(_m.position.x))   # ky = field.x

    ekf_slam = SEkfSlam()
    ekf_slam.setup(known_landmarks=_known_lm)

    # ── Build source ──────────────────────────────────────────────────────────
    source = build_source(args)
    source.setup()

    # Peek at first frame to optionally seed pose
    frame_iter = source.frames()
    try:
        first_info = next(frame_iter)
    except StopIteration:
        print("% amcl_runner: source is empty – nothing to do")
        source.release()
        return

    init_amcl(amcl, args, first_info)

    # Initialise EKF SLAM pose (mirrors AMCL initialisation)
    if args.init:
        try:
            _ip = [float(v) for v in args.init.split(',')]
            ekf_slam.init_pose(_ip[0], _ip[1], math.radians(_ip[2]),
                               spread_xy=args.spread_xy,
                               spread_yaw=math.radians(args.spread_yaw))
        except (ValueError, IndexError):
            pass
    elif first_info.has_ref:
        ekf_slam.init_pose(first_info.ref_x, first_info.ref_y, first_info.ref_yaw,
                           spread_xy=args.spread_xy,
                           spread_yaw=math.radians(args.spread_yaw))
    else:
        ekf_slam.init_pose(3.0, 3.5, 0.0)   # field centre as fallback

    # ── Display window ────────────────────────────────────────────────────────
    win_name = 'AMCL Runner'
    if not args.no_display:
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win_name, 1100, 520)

    paused   = args.pause
    step_req = False

    def process_frame(info: FrameInfo):
        """Run one full AMCL iteration on a FrameInfo."""
        frame = info.frame
        if frame is None:
            return

        # ── 1. Motion update ─────────────────────────────────────────────────
        if info.dt > 0:
            amcl.motion_update(info.v, info.omega, info.dt)
            ekf_slam.predict(info.v, info.omega, info.dt)

        # ── 2. Perception ────────────────────────────────────────────────────
        line_res = sline_vision.process(frame)
        aruco_ds = aruco_det.process(frame)
        feats    = slam_features.extract(frame)

        # ── 3. Measurement update ─────────────────────────────────────────────
        amcl.measurement_update(line_res, aruco_ds)

        # EKF SLAM: update from ArUco known landmarks
        for _det in aruco_ds:
            ekf_slam.update_known(int(_det['id']),
                                  float(_det['range']),
                                  float(_det['bearing']))
        # EKF SLAM: update / augment from visual features
        n_match, n_new = ekf_slam.update_features(feats)

        # ── 4. Resample ───────────────────────────────────────────────────────
        amcl.resample()

        # ── 5. Estimates ──────────────────────────────────────────────────────
        est          = amcl.get_estimate()
        slam_x, slam_y, slam_yaw, _ = ekf_slam.get_pose()

        # ── 6. Console output ─────────────────────────────────────────────────
        line_valid = line_res.get('valid') or line_res.get('line_valid', False)
        aruco_ids  = [d['id'] for d in aruco_ds]
        print(f"[{info.frame_idx:04d}] "
              f"AMCL x={est[0]:.3f}m  y={est[1]:.3f}m  yaw={math.degrees(est[2]):.1f}d  "
              f"n_eff={amcl.n_eff:.0f}  "
              f"tape={'ok' if line_valid else '--'}  "
              f"aruco={aruco_ids or '--'}  "
              f"| SLAM x={slam_x:.3f}m  y={slam_y:.3f}m  yaw={math.degrees(slam_yaw):.1f}d  "
              f"lm={ekf_slam.get_n_landmarks()}(+{n_new})  feat_match={n_match}")

        # ── 7. Display ────────────────────────────────────────────────────────
        if args.no_display:
            return

        # Camera panel (shows sline + aruco detection overlays)
        cam_panel = draw_camera_overlay(frame, line_res, aruco_ds)
        # Resize camera panel to fixed height
        target_h = _MAP_H
        ch, cw   = cam_panel.shape[:2]
        scale    = target_h / ch
        cam_panel = cv2.resize(cam_panel,
                               (int(cw * scale), target_h))

        # Map panel (estimate only, no ground truth)
        map_panel = draw_map_panel(amcl.particles, amcl.weights, est, amcl.n_eff)
        # EKF SLAM overlay (landmarks + pose)
        draw_slam_overlay(map_panel, ekf_slam.get_landmarks(),
                          slam_pose=(slam_x, slam_y, slam_yaw))

        # Side-by-side
        h1, w1 = cam_panel.shape[:2]
        h2, w2 = map_panel.shape[:2]
        combined_h = max(h1, h2)
        canvas    = np.zeros((combined_h, w1 + w2, 3), dtype=np.uint8)
        canvas[:h1, :w1]       = cam_panel
        canvas[:h2, w1:w1+w2]  = map_panel

        # Status bar
        src_txt = info.source_name
        cv2.putText(canvas, f"{src_txt}  frame={info.frame_idx}  "
                             f"{'[PAUSED]' if paused else ''}",
                    (6, combined_h - 6),
                    cv2.FONT_HERSHEY_PLAIN, 0.9, (180, 180, 180), 1)

        cv2.imshow(win_name, canvas)

    # ── Main loop ─────────────────────────────────────────────────────────────
    process_frame(first_info)

    for info in frame_iter:
        # Keyboard
        if not args.no_display:
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):   # q or ESC
                break
            elif key == ord(' '):
                paused = not paused
                print(f"% {'Paused' if paused else 'Resumed'}")
            elif key == ord('s'):
                step_req = True
            elif key == ord('r'):
                print("% Re-initialising particles (uniform)")
                amcl.init_uniform()
        else:
            key = 0xFF

        if paused and not step_req:
            time.sleep(0.05)
            if not args.no_display:
                cv2.waitKey(1)
            continue

        step_req = False
        process_frame(info)

    source.release()

    # Keep the window open until the user presses q / ESC.
    # This is important for single-image and short image-folder sources
    # where the loop ends before the user has had time to inspect the result.
    if not args.no_display:
        print("% Press q or ESC in the window to quit")
        while True:
            key = cv2.waitKey(50) & 0xFF
            if key in (ord('q'), 27):
                break
        cv2.destroyAllWindows()
    print("% amcl_runner: done")


# =============================================================================
# Entry point
# =============================================================================

def main():
    ap = argparse.ArgumentParser(
        description='AMCL particle-filter localisation runner',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Live camera:
    python3 amcl_runner.py --source live --host 10.197.218.17

  # Replay session:
    python3 amcl_runner.py --source video --path recordings/test1

  # Replay with known starting pose (Kalman frame: x=forward, y=lateral):
    python3 amcl_runner.py --source video --path recordings/test1 --init 0.23,4.79,-4.5

  # Single snapshot:
    python3 amcl_runner.py --source image --path recordings/pic/snapshots/20260424_152301_698.json
""")

    # Source selection
    ap.add_argument('--source', required=True,
                    choices=['live', 'video', 'images', 'image'],
                    help='Input source')
    ap.add_argument('--path', default=None,
                    help='Session dir / file for non-live sources')
    ap.add_argument('--host', default='localhost',
                    help='Robot hostname/IP (live mode)')
    ap.add_argument('--cam-port', type=int, default=7123,
                    help='MJPEG stream port (live mode)')

    # AMCL parameters
    ap.add_argument('--particles', type=int, default=500,
                    help='Number of AMCL particles')
    ap.add_argument('--init', default=None,
                    help='Initial pose "x,y,yaw_deg" (Kalman frame)')
    ap.add_argument('--spread-xy', type=float, default=0.30,
                    help='Position spread for init (m)')
    ap.add_argument('--spread-yaw', type=float, default=20.0,
                    help='Heading spread for init (deg)')

    # Playback
    ap.add_argument('--speed', type=float, default=1.0,
                    help='Replay speed multiplier (video/images)')
    ap.add_argument('--pause', action='store_true',
                    help='Start paused (press SPACE or s to advance)')

    # Display
    ap.add_argument('--no-display', action='store_true',
                    help='Headless mode – no cv2 window')
    ap.add_argument('--debug', action='store_true',
                    help='Enable sline/saruco debug visualisation')

    args = ap.parse_args()
    run(args)


if __name__ == '__main__':
    main()
