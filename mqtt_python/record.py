#!/usr/bin/env python3
"""
record.py — Record robot camera + Kalman state log simultaneously.

Usage:
  python3 record.py                    # auto-named session
  python3 record.py -s my_session      # named session
  python3 record.py --snapshot         # take single snapshot then exit
  python3 record.py --no-video         # log only, no video recording

Controls (while running):
  p  — save a snapshot JPEG
  q  — quit and finalize files

Output files (in ./recordings/<session>/):
  video.mp4        — H.264 video from robot camera
  kalman_log.jsonl — one JSON line per Kalman state update
  snapshots/       — JPEG snapshots saved with timestamp
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime, timedelta
from pathlib import Path

import cv2
import paho.mqtt.client as mqtt

# ── Config ────────────────────────────────────────────────────────────────────
DEFAULT_HOST        = "localhost"
DEFAULT_CAM_PORT    = 7123
KALMAN_TOPIC        = "robobot/kalman/state"
VIDEO_FPS           = 8
VIDEO_CODEC         = "mp4v"   # fallback; prefer avc1 if available


# ── State ─────────────────────────────────────────────────────────────────────
_stop = threading.Event()
_snapshot_req = threading.Event()
_last_kalman: dict = {}          # latest Kalman state, updated by MQTT logger
_last_kalman_lock = threading.Lock()
_video_start_time: float = None  # set when video writer opens; used for video_t


# ── Helpers ───────────────────────────────────────────────────────────────────
def session_dir(name: str) -> Path:
    base = Path(__file__).parent / "recordings" / name
    base.mkdir(parents=True, exist_ok=True)
    (base / "snapshots").mkdir(exist_ok=True)
    return base


def ts_str():
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]


# ── MQTT Kalman logger ────────────────────────────────────────────────────────
def mqtt_logger(host: str, log_path: Path):
    lines_written = 0

    def on_connect(client, userdata, flags, rc, properties=None):
        if rc == 0:
            client.subscribe(KALMAN_TOPIC)
            print(f"% MQTT connected, subscribed to {KALMAN_TOPIC}")
        else:
            print(f"% MQTT connect failed rc={rc}")

    def on_message(client, userdata, msg):
        nonlocal lines_written
        if _stop.is_set():
            return
        try:
            payload = json.loads(msg.payload.decode())
        except Exception:
            payload = {"raw": msg.payload.decode()}
        with _last_kalman_lock:
            _last_kalman.clear()
            _last_kalman.update(payload)
        now = time.time()
        video_t = str(timedelta(seconds=round(now - _video_start_time, 3)))[:-3] if _video_start_time is not None else None
        record = {"t": now, "datetime": datetime.fromtimestamp(now).isoformat(timespec='milliseconds'), "video_t": video_t, "topic": msg.topic, "data": payload}
        with open(log_path, "a") as f:
            f.write(json.dumps(record) + "\n")
        lines_written += 1
        if lines_written % 100 == 0:
            print(f"% Kalman log: {lines_written} entries")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    try:
        client.connect(host, 1883, keepalive=60)
        client.loop_start()
        _stop.wait()
    finally:
        client.loop_stop()
        client.disconnect()
        print(f"% Kalman log saved: {log_path} ({lines_written} entries)")


# ── Camera recorder ───────────────────────────────────────────────────────────
def camera_recorder(host: str, cam_port: int, out_dir: Path,
                    record_video: bool, snapshot_only: bool):
    stream_url = f"http://{host}:{cam_port}/stream.mjpg"
    print(f"% Connecting to camera: {stream_url}")

    cap = cv2.VideoCapture(stream_url)
    if not cap.isOpened():
        print(f"% ERROR: cannot open camera stream at {stream_url}")
        _stop.set()
        return

    # Grab first frame to get resolution
    ret, frame = cap.read()
    if not ret:
        print("% ERROR: no frame from camera")
        cap.release()
        _stop.set()
        return

    h, w = frame.shape[:2]

    # ── Auto-detect actual camera FPS ────────────────────────────────────────
    # MJPEG streams report 0 from cap.get(); measure it directly.
    _meas_frames = 0
    _meas_start  = time.time()
    while time.time() - _meas_start < 1.0:
        ret, frame = cap.read()
        if not ret:
            break
        _meas_frames += 1
    detected_fps = _meas_frames / max(time.time() - _meas_start, 0.001)
    detected_fps = max(1.0, round(detected_fps))   # at least 1, rounded int
    print(f"% Camera: {w}x{h} @ {detected_fps} fps (measured)")

    global _video_start_time
    writer = None
    if record_video and not snapshot_only:
        video_path = out_dir / "video.mp4"
        for codec in ("avc1", VIDEO_CODEC):
            fourcc = cv2.VideoWriter_fourcc(*codec)
            writer = cv2.VideoWriter(str(video_path), fourcc, detected_fps, (w, h))
            if writer.isOpened():
                _video_start_time = time.time()
                print(f"% Video recording: {video_path} (codec={codec})")
                print(f"% Video start time: {datetime.fromtimestamp(_video_start_time).isoformat(timespec='milliseconds')}")
                break
            writer = None
        if writer is None:
            print("% WARNING: could not open video writer — video disabled")

    frames = 0
    snapshots = 0
    _frame_interval = 1.0 / detected_fps
    _next_frame_t   = time.time()

    if snapshot_only:
        # Save one snapshot and exit
        ts = ts_str()
        snap_path = out_dir / "snapshots" / f"{ts}.jpg"
        cv2.imwrite(str(snap_path), frame)
        with _last_kalman_lock:
            state = dict(_last_kalman)
        if state:
            now = time.time()
            video_t = str(timedelta(seconds=round(now - _video_start_time, 3)))[:-3] if _video_start_time is not None else None
            with open(out_dir / "snapshots" / f"{ts}.json", "w") as f:
                json.dump({"t": now, "datetime": datetime.fromtimestamp(now).isoformat(timespec='milliseconds'), "video_t": video_t, "kalman": state}, f, indent=2)
        print(f"% Snapshot saved: {snap_path}")
        cap.release()
        _stop.set()
        return

    try:
        while not _stop.is_set():
            ret, frame = cap.read()
            if not ret:
                print("% Camera stream ended")
                break

            now = time.time()
            if now < _next_frame_t:
                continue                   # drop frame, not time yet
            _next_frame_t = now + _frame_interval

            if writer is not None:
                writer.write(frame)
            frames += 1

            if _snapshot_req.is_set():
                _snapshot_req.clear()
                ts = ts_str()
                snap_path = out_dir / "snapshots" / f"{ts}.jpg"
                cv2.imwrite(str(snap_path), frame)
                with _last_kalman_lock:
                    state = dict(_last_kalman)
                if state:
                    now = time.time()
                    video_t = str(timedelta(seconds=round(now - _video_start_time, 3)))[:-3] if _video_start_time is not None else None
                    with open(out_dir / "snapshots" / f"{ts}.json", "w") as f:
                        json.dump({"t": now, "datetime": datetime.fromtimestamp(now).isoformat(timespec='milliseconds'), "video_t": video_t, "kalman": state}, f, indent=2)
                snapshots += 1
                print(f"% Snapshot saved: {snap_path} (total {snapshots})")

    finally:
        cap.release()
        if writer is not None:
            writer.release()
        _stop.set()  # ensure logger stops exactly when video stops
        _video_start_time = None
        print(f"% Camera done: {frames} frames, {snapshots} snapshots")


# ── Keyboard input thread ─────────────────────────────────────────────────────
def keyboard_listener():
    import termios, tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while not _stop.is_set():
            ch = sys.stdin.read(1)
            if ch in ("q", "Q"):
                print("\r\n% Quit requested")
                _stop.set()
                break
            elif ch in ("p", "P"):
                _snapshot_req.set()
                print("\r\n% Snapshot requested")
    except Exception:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Record robot camera + Kalman log")
    parser.add_argument("-i", "--host",     default=DEFAULT_HOST,     help="Robot hostname/IP")
    parser.add_argument("-p", "--cam-port", default=DEFAULT_CAM_PORT, type=int, help="Camera stream port")
    parser.add_argument("-s", "--session",  default=None,             help="Session name (default: timestamp)")
    parser.add_argument("--snapshot",       action="store_true",      help="Take single snapshot and exit")
    parser.add_argument("--no-video",       action="store_true",      help="Disable video recording")
    parser.add_argument("--no-log",         action="store_true",      help="Disable Kalman log")
    parser.add_argument("--start-client",    action="store_true",      help="Auto-start mqtt-client.py before recording")
    parser.add_argument("--teleop",           action="store_true",      help="Launch teleop_keyboard.py alongside recording")
    args = parser.parse_args()

    session_name = args.session or datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = session_dir(session_name)
    log_path = out_dir / "kalman_log.jsonl"

    print(f"% Session: {out_dir}")

    # Optionally launch mqtt-client.py
    mqtt_client_proc = None
    if args.start_client:
        client_script = str(Path(__file__).parent / "mqtt-client.py")
        cmd = [sys.executable, client_script, "-i", args.host]
        mqtt_client_proc = subprocess.Popen(cmd)
        print(f"% Started mqtt-client.py (pid {mqtt_client_proc.pid}), waiting 3s to settle...")
        time.sleep(3)

    # Optionally launch teleop_keyboard.py
    teleop_proc = None
    if args.teleop:
        teleop_script = str(Path(__file__).parent / "teleop_keyboard.py")
        cmd = [sys.executable, teleop_script, "-i", args.host]
        teleop_proc = subprocess.Popen(cmd)
        print(f"% Started teleop_keyboard.py (pid {teleop_proc.pid})")

    def _shutdown(*_):
        _stop.set()
        if mqtt_client_proc is not None:
            mqtt_client_proc.terminate()
        if teleop_proc is not None:
            teleop_proc.terminate()

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    threads = []

    # MQTT logger
    if not args.no_log:
        t = threading.Thread(target=mqtt_logger, args=(args.host, log_path), daemon=True)
        t.start()
        threads.append(t)
        if args.snapshot:
            # Wait up to 3s for first Kalman message before snapping
            deadline = time.time() + 3.0
            while time.time() < deadline:
                with _last_kalman_lock:
                    if _last_kalman:
                        break
                time.sleep(0.05)

    # Camera
    t = threading.Thread(
        target=camera_recorder,
        args=(args.host, args.cam_port, out_dir, not args.no_video, args.snapshot),
        daemon=True,
    )
    t.start()
    threads.append(t)

    if not args.snapshot:
        print("% Press  p = snapshot   q = quit")
        kb = threading.Thread(target=keyboard_listener, daemon=True)
        kb.start()
        _stop.wait()
    else:
        for t in threads:
            t.join(timeout=10)

    _stop.set()
    for t in threads:
        t.join(timeout=5)
    print(f"% Done. Files in: {out_dir}")


if __name__ == "__main__":
    main()
