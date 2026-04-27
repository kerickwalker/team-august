#!/usr/bin/env python3
"""
Robobot computer-vision runner.

Self-contained tool to verify two-upright gate detection and a slow gate-align controller.

By default it runs in VISION-ONLY mode: it reads frames, runs detection, computes
the recommended drive command, and prints clean status. Pass --drive to actually
publish those commands to the robot over MQTT.

Examples:
  # Vision only (safe default), using built-in stream host:
  python3 computer_vision.py

  # Same, but also publish drive commands (very slow defaults):
  python3 computer_vision.py --drive

  # Faster preset for once you trust the controller:
  python3 computer_vision.py --drive --preset normal

  # Save annotated frames every second (in addition to the final frame):
  python3 computer_vision.py --save-every 1.0

  # Read from a local video file or a different stream URL instead:
  python3 computer_vision.py --stream-url ./clip.mp4

  # Record annotated MP4 (auto filename in --out-dir):
  python3 computer_vision.py --video-out

  # Live annotated MJPEG stream for browser debugging:
  python3 computer_vision.py --live-stream
"""

import argparse
import sys
import time
import threading
from datetime import datetime
from pathlib import Path

import cv2 as cv

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sgate import gate
try:
    from flask import Flask, Response
    _flask_available = True
except ImportError:
    _flask_available = False


# -----------------------------------------------------------------------------
# Speed presets — tweak here, do not duplicate elsewhere.
# -----------------------------------------------------------------------------
PRESETS = {
    "crawl":  {"min_v": 0.02, "max_v": 0.05, "max_w": 0.25, "kx": 0.5},
    "slow":   {"min_v": 0.03, "max_v": 0.08, "max_w": 0.35, "kx": 0.6},
    "normal": {"min_v": 0.05, "max_v": 0.15, "max_w": 0.50, "kx": 0.8},
}

DEFAULT_TARGET_X      = 0.0     # bar centered horizontally
DEFAULT_TARGET_WIDTH  = 130.0   # px — bar width at the desired distance
DEFAULT_STOP_WIDTH    = 110.0   # px — stop forward when bar is this wide

STATUS_PRINT_EVERY_S  = 0.5
DEADMAN_LOSS_FRAMES   = 5       # stop driving after this many "no-detection" frames
DEFAULT_CONFIRM_FRAMES = 5      # require N consecutive detections for "gate seen"
DEFAULT_STREAM_HOST   = "10.197.219.117"
_stream_frame = None
_stream_lock = threading.Lock()

CTRL_MIN_V = PRESETS["crawl"]["min_v"]
CTRL_MAX_V = PRESETS["crawl"]["max_v"]
CTRL_MAX_W = PRESETS["crawl"]["max_w"]
CTRL_KX = PRESETS["crawl"]["kx"]


# -----------------------------------------------------------------------------
# Argument parsing
# -----------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(description="Robobot computer-vision runner")

    p.add_argument("--stream-url", help="Full video stream URL or local file")

    p.add_argument("--mqtt-host", default="localhost",
                   help="MQTT broker host when --drive is set (default: localhost)")
    p.add_argument("--drive", action="store_true",
                   help="Publish drive commands to robobot/cmd/ti (default: vision only)")

    p.add_argument("--preset", choices=tuple(PRESETS.keys()), default="crawl",
                   help="Speed preset (default: crawl — extremely slow, for first test)")

    p.add_argument("--target-x",     type=float, default=DEFAULT_TARGET_X,
                   help="Desired normalized gate center x in [-1, 1] (default: 0.0 = center)")
    p.add_argument("--target-width", type=float, default=DEFAULT_TARGET_WIDTH,
                   help="Gate pixel width at desired distance (default: 130)")
    p.add_argument("--stop-width",   type=float, default=DEFAULT_STOP_WIDTH,
                   help="Stop forward motion when bar exceeds this width (default: 110)")
    p.add_argument("--confirm-frames", type=int, default=DEFAULT_CONFIRM_FRAMES,
                   help="Consecutive two-bar detections required before gate is valid (default: 5)")

    p.add_argument("--max-time", type=float, default=30.0,
                   help="Max run time in seconds (default: 30)")
    p.add_argument("--save-every", type=float, default=0.0,
                   help="If > 0, save annotated frame every N seconds (default: off)")
    p.add_argument(
        "--video-out",
        nargs="?",
        const="auto",
        default="",
        help=("Record annotated video to MP4. "
              "Optional value = output filename/path; if omitted, auto name in --out-dir."),
    )
    p.add_argument("--out-dir", default="cv_runs",
                   help="Directory for saved frames (default: ./cv_runs)")
    p.add_argument("--live-stream", action="store_true",
                   help="Serve annotated MJPEG stream for live debugging")
    p.add_argument("--live-port", type=int, default=5001,
                   help="Port for --live-stream (default: 5001)")
    p.add_argument("--quiet", action="store_true", help="Suppress periodic status prints")

    return p.parse_args()


# -----------------------------------------------------------------------------
# MQTT publisher (only used when --drive)
# -----------------------------------------------------------------------------
def make_mqtt_publisher(host):
    from paho.mqtt import client as mqtt_client

    try:
        c = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2,
                               client_id="cv-runner")
    except Exception:
        c = mqtt_client.Client("cv-runner")
    c.connect(host, 1883, 60)
    c.loop_start()

    def publish(v, w):
        c.publish("robobot/cmd/ti", f"rc {v:.3f} {w:.3f}")

    def stop():
        c.publish("robobot/cmd/ti", "rc 0 0")
        time.sleep(0.05)
        c.loop_stop()
        c.disconnect()

    return publish, stop


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------
def apply_preset(name):
    global CTRL_MIN_V, CTRL_MAX_V, CTRL_MAX_W, CTRL_KX
    p = PRESETS[name]
    CTRL_MIN_V = p["min_v"]
    CTRL_MAX_V = p["max_v"]
    CTRL_MAX_W = p["max_w"]
    CTRL_KX = p["kx"]


def open_stream(url):
    print(f"% Opening video stream: {url}")
    cap = cv.VideoCapture(url)
    if not cap.isOpened():
        print(f"% ERROR: could not open {url}")
        sys.exit(1)
    return cap


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def compute_gate_command(target_x_norm, target_width_px, stop_width_px, gate_valid):
    """Compute slow alignment command from two-upright gate detection."""
    if not gate_valid:
        return 0.0, 0.0, {"valid": False, "lateral_error": 0.0, "depth_error": 0.0}

    e_x = gate.steeringError() - target_x_norm
    gate_w = float(gate.gate_width_px)
    e_w = (target_width_px - gate_w) / max(target_width_px, 1.0)

    # Steering: opposite sign of lateral error.
    w_cmd = _clamp(-CTRL_KX * e_x, -CTRL_MAX_W, CTRL_MAX_W)

    if gate_w >= stop_width_px:
        v_cmd = 0.0
    else:
        v_cmd = _clamp(
            CTRL_MIN_V + (CTRL_MAX_V - CTRL_MIN_V) * max(e_w, 0.0),
            CTRL_MIN_V,
            CTRL_MAX_V,
        )
        v_cmd *= _clamp(1.0 - 0.5 * abs(e_x), 0.3, 1.0)

    pose = {"valid": True, "lateral_error": float(e_x), "depth_error": float(e_w)}
    return float(v_cmd), float(w_cmd), pose


def annotate(frame, v, w, pose, drive_enabled):
    gate.paint(frame)
    label = f"{'DRIVE' if drive_enabled else 'VIS '} v={v:+.3f}  w={w:+.3f}"
    cv.putText(frame, label, (10, 56),
               cv.FONT_HERSHEY_PLAIN, 1.4, (0, 200, 200), thickness=2)
    if pose["valid"]:
        det = (f"e_x={pose['lateral_error']:+.3f}  "
               f"e_w={pose['depth_error']:+.3f}  "
               f"gate_w={gate.gate_width_px}px")
        cv.putText(frame, det, (10, 80),
                   cv.FONT_HERSHEY_PLAIN, 1.2, (200, 255, 200), thickness=1)

def status_line(pose, v, w, lost_frames, detect_streak):
    if pose["valid"]:
        return ("FOUND  "
                f"cx={gate.gate_cx:4d}  gate_w={gate.gate_width_px:4d}px  "
                f"e_x={pose['lateral_error']:+.3f}  e_w={pose['depth_error']:+.3f}  "
                f"-> v={v:+.3f}  w={w:+.3f}")
    return (f"LOST   ({lost_frames} frames, streak={detect_streak})"
            f" -> v={v:+.3f}  w={w:+.3f}")


def make_video_writer(video_out_value, out_dir: Path, first_frame):
    """Create MP4 writer from first frame dimensions."""
    h, w = first_frame.shape[:2]
    fps = 20.0
    fourcc = cv.VideoWriter_fourcc(*"mp4v")

    if video_out_value == "auto":
        video_path = out_dir / f"cv_run_{datetime.now():%Y%m%d_%H%M%S}.mp4"
    else:
        candidate = Path(video_out_value)
        if candidate.is_absolute() or candidate.parent != Path("."):
            video_path = candidate
        else:
            video_path = out_dir / candidate

    video_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv.VideoWriter(str(video_path), fourcc, fps, (w, h))
    if not writer.isOpened():
        return None, None
    return writer, video_path


def _push_stream_frame(frame):
    global _stream_frame
    with _stream_lock:
        _stream_frame = frame.copy()


def start_live_stream_server(port):
    if not _flask_available:
        print("% WARNING: Flask not installed; live stream disabled (pip install flask)")
        return

    app = Flask(__name__)

    @app.route("/")
    def _index():
        return ('<html><body style="background:#000;margin:0">'
                '<img src="/stream" style="max-width:100%"></body></html>')

    def _mjpeg_generate():
        while True:
            with _stream_lock:
                f = _stream_frame
            if f is not None:
                ok, buf = cv.imencode(".jpg", f, [cv.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                           + buf.tobytes() + b"\r\n")
            time.sleep(0.05)

    @app.route("/stream")
    def _stream():
        return Response(
            _mjpeg_generate(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=port, threaded=True),
        daemon=True,
    ).start()
    print(f"% Live annotated stream: http://localhost:{port}/")


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    args = parse_args()

    if args.stream_url:
        url = args.stream_url
    else:
        url = f"http://{DEFAULT_STREAM_HOST}:7123/stream.mjpg"

    apply_preset(args.preset)

    print("% Robobot computer-vision runner")
    if not args.stream_url:
        print(f"% stream host        : {DEFAULT_STREAM_HOST} (edit DEFAULT_STREAM_HOST in file)")
    print(f"% stream source      : {url}")
    print(f"% mode             : {'DRIVE' if args.drive else 'vision-only'}")
    print(f"% preset            : {args.preset}  "
          f"(min_v={CTRL_MIN_V}, max_v={CTRL_MAX_V}, "
          f"max_w={CTRL_MAX_W}, kx={CTRL_KX})")
    print(f"% target_x_norm     : {args.target_x:+.3f}")
    print(f"% target_width_px   : {args.target_width:.1f}")
    print(f"% stop_width_px     : {args.stop_width:.1f}")
    print(f"% confirm_frames    : {max(1, args.confirm_frames)}")
    print(f"% max-time          : {args.max_time:.1f} s")

    cap = open_stream(url)
    gate.setup()

    publish, stop_pub = (None, None)
    if args.drive:
        print(f"% Connecting MQTT broker at {args.mqtt_host}:1883")
        publish, stop_pub = make_mqtt_publisher(args.mqtt_host)
    if args.live_stream:
        start_live_stream_server(args.live_port)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    save_every = args.save_every
    video_writer = None
    video_path = None

    start_t = time.time()
    next_print = start_t
    next_save  = start_t + save_every if save_every > 0 else None

    frame_count = 0
    lost_frames = 0
    detect_streak = 0
    last_frame  = None

    try:
        while True:
            now = time.time()
            if now - start_t > args.max_time:
                print("% Reached --max-time; stopping")
                break

            ret, frame = cap.read()
            if not ret:
                print("% Lost video stream")
                break

            frame_count += 1

            raw_detected = gate.detect(frame)
            if raw_detected:
                detect_streak += 1
            else:
                detect_streak = 0
            gate_valid = detect_streak >= max(1, args.confirm_frames)
            v, w, pose = compute_gate_command(
                args.target_x,
                args.target_width,
                args.stop_width,
                gate_valid,
            )

            if pose["valid"]:
                lost_frames = 0
            else:
                lost_frames += 1

            # Deadman safety: stop driving entirely if we keep losing the bar.
            if args.drive and lost_frames >= DEADMAN_LOSS_FRAMES:
                v, w = 0.0, 0.0

            if args.drive:
                publish(v, w)

            annotate(frame, v, w, pose, args.drive)
            if args.live_stream:
                _push_stream_frame(frame)
            if args.video_out and video_writer is None:
                video_writer, video_path = make_video_writer(args.video_out, out_dir, frame)
                if video_writer is None:
                    print("% WARNING: could not open video writer; continuing without --video-out")
                else:
                    print(f"% Recording annotated video to {video_path}")
            if video_writer is not None:
                video_writer.write(frame)
            last_frame = frame

            if not args.quiet and now >= next_print:
                print("% " + status_line(pose, v, w, lost_frames, detect_streak))
                next_print = now + STATUS_PRINT_EVERY_S

            if next_save is not None and now >= next_save:
                fn = out_dir / f"cv_{datetime.now():%Y%m%d_%H%M%S}_{frame_count:05d}.jpg"
                cv.imwrite(str(fn), frame)
                next_save = now + save_every

    except KeyboardInterrupt:
        print("\n% Interrupted by user")
    finally:
        if stop_pub is not None:
            stop_pub()
        cap.release()
        if video_writer is not None:
            video_writer.release()
            print(f"% Saved run video to {video_path}")
        gate.terminate()
        if last_frame is not None:
            fn = out_dir / f"cv_final_{datetime.now():%Y%m%d_%H%M%S}.jpg"
            cv.imwrite(str(fn), last_frame)
            print(f"% Saved final frame to {fn}")
        elapsed = time.time() - start_t
        print(f"% Done — processed {frame_count} frames in {elapsed:.1f} s "
              f"({frame_count / max(elapsed, 1e-3):.1f} fps)")


if __name__ == "__main__":
    main()
