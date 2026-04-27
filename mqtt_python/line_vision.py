#!/usr/bin/env python3
"""
Line-vision runner for quick line-detection evaluation.

Uses SVLine detector (camera-based) and provides:
- on-frame overlay (from svline.paint)
- periodic terminal metrics
- optional live MJPEG debug stream
- optional annotated MP4 recording

Examples:
  python3 line_vision.py
  python3 line_vision.py --threshold 170 --roi 0.50 --roi-mode top --live-stream
  python3 line_vision.py --video-out --max-time 45
"""

import argparse
import sys
import time
import threading
from datetime import datetime
from pathlib import Path

import cv2 as cv

sys.path.insert(0, str(Path(__file__).resolve().parent))
from svline import vline

try:
    from flask import Flask, Response
    _flask_available = True
except ImportError:
    _flask_available = False


DEFAULT_STREAM_HOST = "10.197.219.117"
DEFAULT_THRESHOLD = 180
DEFAULT_ROI = 0.50
STATUS_PRINT_EVERY_S = 0.5

_stream_frame = None
_stream_lock = threading.Lock()


def parse_args():
    p = argparse.ArgumentParser(description="Line detection evaluator")
    p.add_argument("--stream-url", help="Override stream URL or use local video file")
    p.add_argument("--threshold", type=int, default=DEFAULT_THRESHOLD,
                   help="Brightness threshold for white line (default: 180)")
    p.add_argument("--roi", type=float, default=DEFAULT_ROI,
                   help="ROI fraction used for line detection (default: 0.50)")
    p.add_argument("--roi-mode", choices=("top", "bottom"), default="top",
                   help="ROI position for line detection (default: top)")
    p.add_argument("--max-time", type=float, default=20.0,
                   help="Max runtime in seconds (default: 20)")
    p.add_argument("--video-out", nargs="?", const="auto", default="",
                   help="Record annotated MP4. Optional value is output filename/path.")
    p.add_argument("--out-dir", default="cv_runs",
                   help="Directory for outputs (default: cv_runs)")
    p.add_argument("--live-stream", action="store_true",
                   help="Serve annotated MJPEG stream for live debugging")
    p.add_argument("--live-port", type=int, default=5001,
                   help="Port for --live-stream (default: 5001)")
    p.add_argument("--quiet", action="store_true", help="Suppress periodic status logs")
    return p.parse_args()


def open_stream(url):
    print(f"% Opening video stream: {url}")
    cap = cv.VideoCapture(url)
    if not cap.isOpened():
        print(f"% ERROR: could not open {url}")
        sys.exit(1)
    return cap


def make_video_writer(video_out_value, out_dir: Path, first_frame):
    h, w = first_frame.shape[:2]
    fps = 20.0
    fourcc = cv.VideoWriter_fourcc(*"mp4v")

    if video_out_value == "auto":
        video_path = out_dir / f"line_run_{datetime.now():%Y%m%d_%H%M%S}.mp4"
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
    print(f"% Live line stream: http://localhost:{port}/")


def status_line():
    return (f"lineValid={vline.lineValid} "
            f"lineValidCnt={vline.lineValidCnt:2d}/20 "
            f"offset={vline.lineOffset:+.3f}")


def main():
    args = parse_args()
    if args.stream_url:
        url = args.stream_url
    else:
        url = f"http://{DEFAULT_STREAM_HOST}:7123/stream.mjpg"

    print("% Line vision runner")
    if not args.stream_url:
        print(f"% stream host   : {DEFAULT_STREAM_HOST} (edit DEFAULT_STREAM_HOST in file)")
    print(f"% stream source : {url}")
    print(f"% threshold     : {args.threshold}")
    print(f"% roi fraction  : {args.roi}")
    print(f"% roi mode      : {args.roi_mode}")
    print(f"% max-time      : {args.max_time:.1f}s")

    cap = open_stream(url)
    vline.setup(
        brightness_threshold=args.threshold,
        roi_fraction=args.roi,
        roi_mode=args.roi_mode,
    )

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    video_writer = None
    video_path = None

    if args.live_stream:
        start_live_stream_server(args.live_port)

    start_t = time.time()
    next_print = start_t
    frame_count = 0
    valid_count = 0
    last_frame = None

    try:
        while True:
            now = time.time()
            if now - start_t > args.max_time:
                print("% Reached --max-time; stopping")
                break

            ok, frame = cap.read()
            if not ok:
                print("% Lost video stream")
                break

            frame_count += 1
            found = vline.detect(frame)
            if found:
                valid_count += 1
            vline.paint(frame)
            cv.putText(frame, f"LINE valid={vline.lineValidCnt}/20 off={vline.lineOffset:+.2f}",
                       (8, 20), cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv.LINE_AA)

            if args.live_stream:
                _push_stream_frame(frame)

            if args.video_out and video_writer is None:
                video_writer, video_path = make_video_writer(args.video_out, out_dir, frame)
                if video_writer is None:
                    print("% WARNING: could not open video writer; continuing without --video-out")
                else:
                    print(f"% Recording video to {video_path}")
            if video_writer is not None:
                video_writer.write(frame)

            last_frame = frame

            if not args.quiet and now >= next_print:
                print("% " + status_line())
                next_print = now + STATUS_PRINT_EVERY_S

    except KeyboardInterrupt:
        print("\n% Interrupted by user")
    finally:
        cap.release()
        if video_writer is not None:
            video_writer.release()
            print(f"% Saved run video to {video_path}")
        vline.terminate()
        if last_frame is not None:
            fn = out_dir / f"line_final_{datetime.now():%Y%m%d_%H%M%S}.jpg"
            cv.imwrite(str(fn), last_frame)
            print(f"% Saved final frame to {fn}")

    elapsed = max(time.time() - start_t, 1e-3)
    ratio = valid_count / max(frame_count, 1)
    print(f"% Done — frames={frame_count}, lineFoundFrames={valid_count} ({ratio:.1%}), fps={frame_count/elapsed:.1f}")


if __name__ == "__main__":
    main()

