#!/usr/bin/env python3
"""
ssources.py
=============================================================================
Shared camera / data source abstractions for SLAM and AMCL runners.

Four source types:
    LiveSource   — live MJPEG stream + MQTT odometry from the robot
    VideoSource  — recorded video.mp4 + kalman_log.jsonl
    ImagesSource — folder of JPEG snapshots + optional kalman log
    SingleImageSource — one image file

All sources yield FrameInfo objects via their .frames() generator.

Usage:
    from ssources import build_source, FrameInfo
    source = build_source(args)   # args from argparse
    source.setup()
    for info in source.frames():
        ...
    source.release()
=============================================================================
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Iterator, Optional

import cv2
import numpy as np


# =============================================================================
# FrameInfo — one frame's worth of data from any source
# =============================================================================

class FrameInfo:
    """One iteration's worth of data from any source."""
    __slots__ = ('frame', 'v', 'omega', 'dt',
                 'ref_x', 'ref_y', 'ref_yaw', 'ref_pitch',
                 'has_ref', 'frame_idx', 'source_name')

    def __init__(self):
        self.frame:       Optional[np.ndarray] = None
        self.v:           float = 0.0
        self.omega:       float = 0.0
        self.dt:          float = 0.0
        self.ref_x:       float = 0.0
        self.ref_y:       float = 0.0
        self.ref_yaw:     float = 0.0
        self.ref_pitch:   float = 0.0   # robot body pitch (rad), for slope correction
        self.has_ref:     bool  = False
        self.frame_idx:   int   = 0
        self.source_name: str   = ''


# =============================================================================
# Kalman log loader
# =============================================================================

def load_kalman_log(log_path: Path) -> list:
    """
    Load a kalman_log.jsonl and return a list of parsed entries sorted
    by epoch time.  Supports v1 (flat) and v2 (wrapped with 'data' key).
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
                data   = obj.get('data', obj)
                x_state = data.get('x', {})
                meas    = data.get('measurements', {})
                entries.append({
                    't':      float(obj.get('t', 0.0)),
                    'kx':     float(x_state.get('x',     meas.get('enc_x',   0.0))),
                    'ky':     float(x_state.get('y',     meas.get('enc_y',   0.0))),
                    'kyaw':   float(x_state.get('yaw',   meas.get('enc_yaw', 0.0))),
                    'kpitch': float(x_state.get('pitch', 0.0)),
                    'enc_v':  float(meas.get('enc_v',  0.0)),
                    'enc_om': float(meas.get('enc_om', 0.0)),
                    'dt':     float(meas.get('dt',     0.01)),
                })
    except FileNotFoundError:
        print(f"% ssources: kalman log not found: {log_path}")
    entries.sort(key=lambda e: e['t'])
    return entries


# =============================================================================
# Live source
# =============================================================================

class LiveSource:
    """
    Yields frames from the robot MJPEG stream.
    Odometry comes from the Kalman MQTT topic when available.
    """

    def __init__(self, host: str, cam_port: int = 7123):
        self._host      = host
        self._cam_port  = cam_port
        self._cap       = None
        self._prev_t    = None
        self._mqtt_odom: dict = {}
        self._mqtt_pose: dict = {}   # latest Kalman EKF pose from MQTT

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
        try:
            import paho.mqtt.client as mqtt_client
            if hasattr(mqtt_client, "CallbackAPIVersion"):
                client = mqtt_client.Client(
                    client_id="slam-runner-odom",
                    callback_api_version=mqtt_client.CallbackAPIVersion.VERSION1,
                )
            else:
                client = mqtt_client.Client(client_id="slam-runner-odom")

            def on_connect(c, u, f, rc):
                if rc == 0:
                    c.subscribe("robobot/kalman/state")
                    print("% LiveSource: MQTT odometry subscribed")

            def on_message(c, u, msg):
                try:
                    obj  = json.loads(msg.payload)
                    data = obj.get('data', obj)
                    meas = data.get('measurements', {})
                    self._mqtt_odom = {
                        'enc_v':  float(meas.get('enc_v',  0.0)),
                        'enc_om': float(meas.get('enc_om', 0.0)),
                    }
                    # Also extract full Kalman pose for SLAM fusion
                    xs = data.get('x', {})
                    if isinstance(xs, dict) and 'x' in xs and 'y' in xs:
                        self._mqtt_pose = {
                            'x':     float(xs['x']),
                            'y':     float(xs['y']),
                            'yaw':   float(xs.get('yaw',   0.0)),
                            'pitch': float(xs.get('pitch', 0.0)),
                        }
                except Exception:
                    pass

            client.on_connect  = on_connect
            client.on_message  = on_message
            client.connect(self._host, 1883, keepalive=30)
            client.loop_start()
        except Exception as e:
            print(f"% LiveSource: MQTT not available ({e}) -- odometry disabled")

    def frames(self) -> Iterator[FrameInfo]:
        idx = 0
        while True:
            ret, frame = self._cap.read()
            if not ret:
                print("% LiveSource: no frame -- retrying")
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
            if self._mqtt_pose:
                info.ref_x     = self._mqtt_pose['x']
                info.ref_y     = self._mqtt_pose['y']
                info.ref_yaw   = self._mqtt_pose['yaw']
                info.ref_pitch = self._mqtt_pose.get('pitch', 0.0)
                info.has_ref   = True
            else:
                info.has_ref = False
            info.frame_idx   = idx
            info.source_name = f'live:{self._host}'
            idx += 1
            yield info

    def release(self):
        if self._cap:
            self._cap.release()


# =============================================================================
# Video source
# =============================================================================

class VideoSource:
    """
    Yields frames from a recorded video.mp4 synchronised with
    kalman_log.jsonl for ground-truth and odometry.
    """

    def __init__(self, session_dir: Path, speed: float = 1.0):
        self._dir    = Path(session_dir)
        self._speed  = max(0.1, speed)
        self._cap    = None
        self._log    = []
        self._fps    = 8.0
        self._log_t0 = 0.0

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
            self._log = load_kalman_log(log_path)
            if self._log:
                self._log_t0 = self._log[0]['t']
                span = self._log[-1]['t'] - self._log_t0
                print(f"% VideoSource: {len(self._log)} kalman entries  span={span:.1f}s")
            else:
                print("% VideoSource: kalman_log.jsonl is empty")
        else:
            print("% VideoSource: no kalman_log.jsonl -- odometry disabled")

    def frames(self) -> Iterator[FrameInfo]:
        idx     = 0
        log_idx = 0
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
                target_t = self._log_t0 + idx / self._fps
                while (log_idx + 1 < len(self._log) and
                       abs(self._log[log_idx + 1]['t'] - target_t) <
                       abs(self._log[log_idx    ]['t'] - target_t)):
                    log_idx += 1
                entry        = self._log[log_idx]
                info.ref_x     = entry['kx']
                info.ref_y     = entry['ky']
                info.ref_yaw   = entry['kyaw']
                info.ref_pitch = entry.get('kpitch', 0.0)
                info.has_ref   = True
                info.v       = entry['enc_v']
                info.omega   = entry['enc_om']
                info.dt      = max(entry['dt'], 1.0 / self._fps)

            idx += 1
            yield info
            time.sleep(frame_dt / self._speed)

    def release(self):
        if self._cap:
            self._cap.release()


# =============================================================================
# Images source
# =============================================================================

class ImagesSource:
    """
    Yields frames from a folder of JPEG images (sorted by name).
    Optional kalman_log.jsonl or per-image .json sidecars for ground truth.
    """

    def __init__(self, folder: Path, speed: float = 1.0):
        self._folder = Path(folder)
        self._speed  = max(0.1, speed)
        self._paths  = []
        self._log    = []

    def setup(self):
        exts = {'.jpg', '.jpeg', '.png'}
        images = sorted(p for p in self._folder.iterdir()
                        if p.suffix.lower() in exts)
        if not images:
            raise FileNotFoundError(f"No images found in {self._folder}")
        self._paths = images
        print(f"% ImagesSource: {len(images)} images in {self._folder}")

        for candidate in (self._folder / 'kalman_log.jsonl',
                          self._folder.parent / 'kalman_log.jsonl'):
            if candidate.exists():
                self._log = load_kalman_log(candidate)
                print(f"% ImagesSource: loaded {len(self._log)} kalman entries")
                break

    def frames(self) -> Iterator[FrameInfo]:
        for idx, p in enumerate(self._paths):
            frame = cv2.imread(str(p))
            if frame is None:
                print(f"% ImagesSource: cannot read {p}")
                continue

            info             = FrameInfo()
            info.frame       = frame
            info.v           = 0.0
            info.omega       = 0.0
            info.dt          = 0.0
            info.frame_idx   = idx
            info.source_name = f'images:{self._folder.name}'

            sidecar = p.with_suffix('.json')
            if sidecar.exists():
                try:
                    with open(sidecar) as fh:
                        sj = json.load(fh)
                    # Sidecar format: {kalman: {x: {x, y, yaw, ...}, ...}}
                    ks = sj.get('kalman', {})
                    xs = ks.get('x', {}) if isinstance(ks, dict) else {}
                    if isinstance(xs, dict) and 'x' in xs and 'y' in xs:
                        info.ref_x     = float(xs['x'])
                        info.ref_y     = float(xs['y'])
                        info.ref_yaw   = float(xs.get('yaw',   0.0))
                        info.ref_pitch = float(xs.get('pitch', 0.0))
                        info.has_ref   = True
                except Exception as e:
                    print(f"% ImagesSource: sidecar parse error {sidecar}: {e}")

            yield info
            time.sleep(0.1 / self._speed)

    def release(self):
        pass


# =============================================================================
# Single-image source
# =============================================================================

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

        yield info

    def release(self):
        pass


# =============================================================================
# Factory
# =============================================================================

def build_source(args):
    """Build the right source from parsed argparse args."""
    src = args.source.lower()
    speed = getattr(args, 'speed', 1.0)

    if src == 'live':
        return LiveSource(host=args.host, cam_port=args.cam_port)
    elif src == 'video':
        if not args.path:
            raise ValueError("--source video requires --path <session_dir>")
        return VideoSource(session_dir=Path(args.path), speed=speed)
    elif src == 'images':
        if not args.path:
            raise ValueError("--source images requires --path <folder>")
        return ImagesSource(folder=Path(args.path), speed=speed)
    elif src == 'image':
        if not args.path:
            raise ValueError("--source image requires --path <file>")
        return SingleImageSource(path=args.path)
    else:
        raise ValueError(f"Unknown --source '{src}'. Choose: live | video | images | image")
