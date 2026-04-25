#!/usr/bin/env python3
"""
setup_local_dev.py
=============================================================================
One-shot helper to install the Python packages needed to run the AMCL and
perception pipeline locally on your development machine (Windows / macOS / Linux).

Run once with:
    python setup_local_dev.py

Then test with a recorded session:
    python amcl_runner.py --source video --path recordings/test1
    python amcl_runner.py --source images --path recordings/pic/snapshots
    python amcl_runner.py --source image  --path "recordings/pic/snapshots/20260424_152301_698.json"

NOTE: Live mode (--source live) needs the robot reachable on the network.
      All other modes (video / images / image) work fully offline.
=============================================================================
"""

import subprocess
import sys


REQUIRED = [
    # core vision
    ("cv2",         "opencv-python"),          # main OpenCV
    ("cv2.aruco",   "opencv-contrib-python"),  # ArUco marker support
    ("numpy",       "numpy"),
    # MQTT (only needed for --source live)
    ("paho.mqtt",   "paho-mqtt"),
    # plotting (optional, for plot_kalman_log.py etc.)
    ("matplotlib",  "matplotlib"),
]


def check_and_install(import_name: str, pip_name: str) -> bool:
    """Return True if already importable, else install via pip."""
    # Handle submodule checks like 'cv2.aruco'
    top = import_name.split('.')[0]
    try:
        mod = __import__(top)
        # For sub-attributes, check they exist
        for attr in import_name.split('.')[1:]:
            getattr(mod, attr)
        print(f"  [OK]    {pip_name} already installed")
        return True
    except (ImportError, AttributeError):
        print(f"  [MISS]  {pip_name} – installing …")
        result = subprocess.run(
            [sys.executable, "-m", "pip", "install", pip_name],
            capture_output=True, text=True
        )
        if result.returncode == 0:
            print(f"  [DONE]  {pip_name} installed")
            return True
        else:
            print(f"  [FAIL]  {pip_name} – pip error:\n{result.stderr.strip()}")
            return False


def verify_calibration():
    """Check that the camera calibration files are present."""
    from pathlib import Path
    here  = Path(__file__).parent
    calib = here / "calibration" / "camera_params.npz"
    extr  = here / "calibration" / "camera_extrinsics.py"
    ok    = True
    for f in (calib, extr):
        if f.exists():
            print(f"  [OK]    {f.relative_to(here)}")
        else:
            print(f"  [WARN]  {f.relative_to(here)} NOT FOUND  "
                  "– perception will use default parameters")
            ok = False
    return ok


def verify_field_map():
    """Try importing the field map."""
    try:
        from field_map_2026 import FIELD          # noqa: F401
        print("  [OK]    field_map_2026.py loaded OK")
        return True
    except Exception as e:
        print(f"  [FAIL]  field_map_2026: {e}")
        return False


def verify_amcl():
    """Quick AMCL smoke test."""
    try:
        import math
        from samcl import SAMCL
        from field_map_2026 import FIELD
        a = SAMCL()
        a.setup(field=FIELD, n_particles=50)
        a.init_pose(1.0, 5.0, 0.0)
        a.motion_update(0.1, 0.0, 0.05)
        est = a.get_estimate()
        print(f"  [OK]    AMCL smoke test – est=({est[0]:.2f},{est[1]:.2f},"
              f"{math.degrees(est[2]):.1f}d)  N={a.n_particles}")
        return True
    except Exception as e:
        print(f"  [FAIL]  AMCL: {e}")
        return False


def list_recordings():
    """Print available recordings so the user knows what to pass to --path."""
    from pathlib import Path
    rec_dir = Path(__file__).parent / "recordings"
    if not rec_dir.exists():
        print("  [INFO]  recordings/ folder not found")
        return
    items = sorted(rec_dir.iterdir())
    if not items:
        print("  [INFO]  recordings/ is empty")
        return
    print("\nAvailable recordings:")
    for item in items:
        if item.is_dir():
            contents = [p.name for p in item.iterdir()]
            has_video = "video.mp4" in contents
            has_log   = "kalman_log.jsonl" in contents
            snaps_dir = item / "snapshots"
            n_snaps   = len(list(snaps_dir.glob("*.jpg"))) if snaps_dir.exists() else 0
            kind = "video" if has_video else ("images" in contents or f"{n_snaps} snapshots")
            print(f"  recordings/{item.name}/"
                  f"  [{'video.mp4' if has_video else '      '} "
                  f"{'kalman_log.jsonl' if has_log else '                '} "
                  f"{n_snaps} snapshots]")


def print_quick_start():
    print("""
Quick-start commands
────────────────────
# Replay a full recorded run (video + ground-truth overlay):
  python amcl_runner.py --source video --path recordings/test1

# Replay image snapshots:
  python amcl_runner.py --source images --path recordings/pic/snapshots

# Open a single snapshot with sidecar JSON:
  python amcl_runner.py --source image  --path recordings/pic/snapshots/20260424_152301_698.json

# Seed a known starting pose (x=0.23m forward, y=4.79m lateral, yaw=-4.5deg):
  python amcl_runner.py --source video --path recordings/test1 --init 0.23,4.79,-4.5

# More particles for better accuracy (slower):
  python amcl_runner.py --source video --path recordings/test1 --particles 1000

# Start paused and step one frame at a time:
  python amcl_runner.py --source video --path recordings/test1 --pause

# Headless / no window:
  python amcl_runner.py --source video --path recordings/test1 --no-display

Window key bindings
───────────────────
  SPACE   pause / resume
  s       step one frame (when paused)
  r       reset particles (uniform distribution)
  q / ESC quit
""")


def main():
    print("=" * 60)
    print("AMCL local dev setup")
    print("=" * 60)

    print("\n[1] Checking / installing Python packages …")
    all_ok = True
    for import_name, pip_name in REQUIRED:
        if not check_and_install(import_name, pip_name):
            all_ok = False

    print("\n[2] Checking calibration files …")
    verify_calibration()

    print("\n[3] Checking field map …")
    verify_field_map()

    print("\n[4] AMCL smoke test …")
    verify_amcl()

    list_recordings()
    print_quick_start()

    if all_ok:
        print("Setup complete – ready to run amcl_runner.py\n")
    else:
        print("Some packages failed to install – see errors above.\n"
              "Try manually:  pip install opencv-python opencv-contrib-python "
              "numpy paho-mqtt\n")


if __name__ == "__main__":
    main()
