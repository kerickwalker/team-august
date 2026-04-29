from __future__ import annotations
import math
import sys
import time as t

import cv2

from control.tasks import aruco_dock
from control.tasks.aruco_dock import (
    CALIB_PATH,
    STREAM_URL_FMT,
    TARGET_IDS,
    detect_target,
    load_camera_params,
    make_detector,
    make_mqtt,
    run_dock,
    stop,
    target_geometry,
)

DEFAULT_IP   = "10.197.219.117"
WATCH_S      = 8.0           
PRINT_PERIOD = 0.25          


def watch(cap, detector, K, D) -> None:
    """Print live detection results for WATCH_S seconds."""
    print(f"\n[watch] looking for ArUco IDs {sorted(TARGET_IDS)} for {WATCH_S:.0f}s")
    print(f"[watch] target stop distance = {aruco_dock.GRIPPER_DISTANCE:.2f} m "
          f"from marker face\n")
    t_end = t.time() + WATCH_S
    last_print = 0.0
    seen_any = False
    while t.time() < t_end:
        _, tvecs = detect_target(cap, detector, K, D)
        now = t.time()
        if now - last_print < PRINT_PERIOD:
            continue
        last_print = now
        if not tvecs:
            print("  [watch]  no markers in frame")
            continue
        seen_any = True
        geom = target_geometry(tvecs)
        if geom is None:
            continue
        rng, bearing, chosen = geom
        ids = sorted(int(i) for i in tvecs.keys())
        print(f"  [watch]  visible={ids}  aligning_to={chosen}  "
              f"range={rng:.2f} m  bearing={math.degrees(bearing):+.1f}°")
    print()
    if not seen_any:
        print("[watch] !! no markers detected at all. Aim the camera and retry.\n")


def confirm(prompt: str) -> bool:
    return input(f"{prompt} [y/N] > ").strip().lower() in ("y", "yes")


def parse_args() -> tuple[str, float | None]:
    ip = DEFAULT_IP
    stop_override = None
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        a = args[i]
        if a in ("--stop", "-s") and i + 1 < len(args):
            stop_override = float(args[i + 1])
            i += 2
        elif a.startswith("--stop="):
            stop_override = float(a.split("=", 1)[1])
            i += 1
        elif not a.startswith("-"):
            ip = a
            i += 1
        else:
            print(f"  ! unknown arg: {a}")
            i += 1
    return ip, stop_override


def main() -> None:
    ip, stop_override = parse_args()

    if stop_override is not None:
        prev = aruco_dock.GRIPPER_DISTANCE
        aruco_dock.GRIPPER_DISTANCE = stop_override
        print(f"[test] GRIPPER_DISTANCE override: {prev:.2f} m → {stop_override:.2f} m")

    K, D = load_camera_params(CALIB_PATH)
    if K is None:
        print("! calibration not found — cannot run.")
        return

    detector = make_detector()
    stream_url = STREAM_URL_FMT.format(ip=ip)
    print(f"[test] opening stream {stream_url}")
    cap = cv2.VideoCapture(stream_url)
    if not cap.isOpened():
        print(f"! cannot open stream {stream_url}")
        return

    print(f"[test] connecting to robot {ip}")
    client = make_mqtt(ip)
    t.sleep(0.5)

    try:
        watch(cap, detector, K, D)

        if not confirm("Drive to the markers now?"):
            print("[test] skipped drive. done.")
            return

        cap.release()
        ok = run_dock(client, ip)
        print(f"\n[test] dock returned ok={ok}")

        cap2 = cv2.VideoCapture(stream_url)
        if cap2.isOpened():
            t.sleep(0.5)
            _, tvecs = detect_target(cap2, detector, K, D)
            geom = target_geometry(tvecs) if tvecs else None
            if geom is not None:
                rng, bearing, chosen = geom
                err = rng - aruco_dock.GRIPPER_DISTANCE
                print(f"[test] final  aligned_to_id={chosen}  "
                      f"range={rng:.2f} m  "
                      f"bearing={math.degrees(bearing):+.1f}°  "
                      f"error={err*100:+.1f} cm vs target "
                      f"{aruco_dock.GRIPPER_DISTANCE:.2f} m")
            else:
                print("[test] final — markers not visible after dock")
            cap2.release()

    except KeyboardInterrupt:
        print("\n[test] ^C — stopping.")
    finally:
        stop(client)
        t.sleep(0.2)
        client.loop_stop()
        client.disconnect()
        try:
            cap.release()
        except Exception:
            pass
        print("[test] done.")


if __name__ == "__main__":
    main()
