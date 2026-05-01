#!/usr/bin/env python3


import time as t
import sys
import threading
from pathlib import Path
import numpy as np
from datetime import datetime
from types import SimpleNamespace
from setproctitle import setproctitle
try:
    import termios
    import tty
    import select
    _tty_available = True
except ImportError:
    termios = None
    tty = None
    select = None
    _tty_available = False

OTHER_MQTT_DIR = Path(__file__).resolve().parent / "other-mqtt"
if str(OTHER_MQTT_DIR) not in sys.path:
    sys.path.insert(0, str(OTHER_MQTT_DIR))


from spose import pose
from sedge import edge
from sgpio import gpio
from uservice import service
from uteensy import start_teensy_interface, stop_teensy_interface


DEFAULT_RECORD_STREAM_URL = "http://localhost:7123/stream.mjpg"
DEFAULT_RECORD_OUT_DIR    = Path(__file__).resolve().parent.parent / "cv_runs"
DEFAULT_RECORD_FPS        = 20.0

_telemetry = {"state": "init", "crossing": 0}
_telemetry_lock = threading.Lock()
_mission_abort_evt = threading.Event()
_test_log_file = None
_test_log_stdout = None
_test_log_stderr = None
_test_log_start_mono = None


def _aborted():
    return service.stop or _mission_abort_evt.is_set()


def set_telemetry(**kwargs):
    with _telemetry_lock:
        _telemetry.update(kwargs)


def _read_telemetry():
    with _telemetry_lock:
        return dict(_telemetry)


class TimestampLogStream:
    def __init__(self, original_stream, log_stream, start_mono):
        self.original_stream = original_stream
        self.log_stream = log_stream
        self.start_mono = start_mono
        self._log_line_start = True

    def write(self, data):
        self._write_log(data)
        return len(data)

    def flush(self):
        self.log_stream.flush()

    def isatty(self):
        return False

    @property
    def encoding(self):
        return getattr(self.original_stream, "encoding", "utf-8")

    def _log_prefix(self):
        now = datetime.now()
        elapsed = t.monotonic() - self.start_mono
        return f"[{now:%H:%M:%S}.{now.microsecond // 1000:03d} +{elapsed:8.3f}s] "

    def _write_log(self, data):
        while data:
            if self._log_line_start:
                self.log_stream.write(self._log_prefix())
                self._log_line_start = False
            newline = data.find("\n")
            if newline < 0:
                self.log_stream.write(data)
                return
            self.log_stream.write(data[:newline + 1])
            self._log_line_start = True
            data = data[newline + 1:]


def start_test_log():
    global _test_log_file, _test_log_stdout, _test_log_stderr, _test_log_start_mono
    if _test_log_file is not None:
        return
    log_path = Path(__file__).resolve().parent / "PID_log.txt"
    _test_log_stdout = sys.stdout
    _test_log_stderr = sys.stderr
    _test_log_start_mono = t.monotonic()
    _test_log_file = open(log_path, "w", encoding="utf-8", buffering=1)
    sys.stdout = TimestampLogStream(_test_log_stdout, _test_log_file, _test_log_start_mono)
    sys.stderr = TimestampLogStream(_test_log_stderr, _test_log_file, _test_log_start_mono)
    print(f"% test log: redirecting terminal output to {log_path}")
    print(f"% test log started {datetime.now():%Y-%m-%d %H:%M:%S}")


def stop_test_log():
    global _test_log_file, _test_log_stdout, _test_log_stderr, _test_log_start_mono
    if _test_log_file is None:
        return
    print(f"% test log ended {datetime.now():%Y-%m-%d %H:%M:%S}")
    sys.stdout = _test_log_stdout
    sys.stderr = _test_log_stderr
    _test_log_file.close()
    _test_log_file = None
    _test_log_stdout = None
    _test_log_stderr = None
    _test_log_start_mono = None


class MissionRecorder:
    """Background MJPEG-stream → MP4 recorder with mission-state overlay."""

    def __init__(self, video_out, stream_url=DEFAULT_RECORD_STREAM_URL,
                 out_dir=DEFAULT_RECORD_OUT_DIR, fps=DEFAULT_RECORD_FPS):
        self.video_out = video_out
        self.stream_url = stream_url
        self.out_dir = Path(out_dir)
        self.fps = fps
        self.video_path = None
        self.frame_count = 0
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        try:
            import cv2 as cv
        except ImportError:
            print("% recorder: cv2 not installed — video recording disabled")
            return self
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def _resolve_path(self):
        if self.video_out in ("auto", "", True):
            return self.out_dir / f"mission_{datetime.now():%Y%m%d_%H%M%S}.mp4"
        cand = Path(self.video_out)
        if cand.is_absolute() or cand.parent != Path("."):
            return cand
        return self.out_dir / cand

    def _run(self):
        import cv2 as cv

        cap = cv.VideoCapture(self.stream_url)
        if not cap.isOpened():
            print(f"% recorder: failed to open {self.stream_url} — recording disabled")
            return

        writer = None
        last_warn = 0.0
        try:
            while not self._stop.is_set():
                ret, frame = cap.read()
                if not ret:
                    now = t.time()
                    if now - last_warn > 2.0:
                        print("% recorder: dropped frame from stream")
                        last_warn = now
                    t.sleep(0.05)
                    continue

                if writer is None:
                    h, w = frame.shape[:2]
                    self.video_path = self._resolve_path()
                    self.video_path.parent.mkdir(parents=True, exist_ok=True)
                    fourcc = cv.VideoWriter_fourcc(*"mp4v")
                    writer = cv.VideoWriter(str(self.video_path), fourcc,
                                            self.fps, (w, h))
                    if not writer.isOpened():
                        print(f"% recorder: could not open writer for {self.video_path}")
                        return
                    print(f"% recorder: writing to {self.video_path} ({w}x{h} @ {self.fps:.0f} fps)")

                self._annotate(cv, frame)
                writer.write(frame)
                self.frame_count += 1
        finally:
            cap.release()
            if writer is not None:
                writer.release()
                print(f"% recorder: saved {self.frame_count} frames to {self.video_path}")

    def _annotate(self, cv, frame):
        tel = _read_telemetry()
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line1 = f"{ts}  state={tel.get('state', '?')}  cross={tel.get('crossing', 0)}"
        cv.putText(frame, line1, (10, 24),
                   cv.FONT_HERSHEY_PLAIN, 1.3, (0, 0, 0), thickness=4)
        cv.putText(frame, line1, (10, 24),
                   cv.FONT_HERSHEY_PLAIN, 1.3, (0, 255, 255), thickness=2)

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)


LINE = SimpleNamespace(
    lost_timeout    = 5.0,
)

LINE_SPEEDS = SimpleNamespace(
    normal=float(edge.PARAM_SETS.get("normal", {}).get("lineVelocity", edge.defaultLineVelocity)),
    slow=float(edge.PARAM_SETS.get("slow", {}).get("lineVelocity", edge.defaultLineVelocity)),
)

LINE_SEEK = SimpleNamespace(
    forward_speed = 0.2,
    max_dist_m    = 1.0,
    max_time_s    = 15.0,
)

CROSSINGS = SimpleNamespace(
    sensor_threshold     = 2,
    leave_delay_s        = 0.5,
    go_straight_until    = 3,
    stop_at              = 5,
    hard_turn_cooldown_s = 2.0,
    l_min_sensors        = 4,
    left_rightmost_max   = 6,
    right_leftmost_min   = 1,
)

CROSSING1 = SimpleNamespace(
    turn_deg      = 35.0,
    turn_dir      = "left",
    turn_rate     = 0.5,
    forward_speed = 0.15,
)

HALF = SimpleNamespace(
    second_cross_turn_deg      = 90.0,
    second_cross_turn_dir      = "left",
    second_cross_follow_dist_m = 0.20,
    second_cross_follow_speed  = LINE_SPEEDS.slow,
    second_cross_follow_params = "slow",
)

LINE_FOLLOW_RULES = SimpleNamespace(
    line_detect_valid_cnt     = 4,
    line_reacquire_valid_cnt  = 4,
    line_lost_valid_cnt       = 2,
    line_lost_stop_confirm_cnt= 3,
    post_stop_settle_s        = 0.05,
    hard_turn_deg             = 90.0,
)


ROUNDABOUT_ENTRY = SimpleNamespace(
    drive_dist_m = 0.35,
    drive_speed  = 0.25,
    turn_deg     = 90.0,
    turn_dir     = "right",
)

ROUNDABOUT_ARC = SimpleNamespace(
    diameter_m    = 0.73,
    total_degrees = 440.0,
    speed         = 0.20,
    direction     = "left",
)

ROUNDABOUT_EXIT = SimpleNamespace(
    turn_deg     = 0.0,
    turn_dir     = "right",
    seek_speed   = 0.20,
    seek_timeout = 15.0,
)

LINE_END = SimpleNamespace(
    lost_cnt     = 2,
    lost_confirm = 5,
)

DRIVE_TO_GOAL = SimpleNamespace(
    follow_dist_m = 1.00,
    turn1_deg     = 90.0,
    fwd2_dist_m   = 0.90,
    turn2_deg     = 90.0,
    fwd3_dist_m   = 2.00,
    turn3_deg     = 20.0,
    drive_speed   = 0.20,
    seek_speed    = 0.20,
    seek_timeout  = 30.0,
)

END = SimpleNamespace(
    initial=SimpleNamespace(
        pre_turn_nudge_m     = 0.05,
        pre_turn_nudge_speed = 0.2,
        first_turn_deg       = 90.0,
        first_turn_dir       = "left",
        follow_speed         = 0.15,
        follow_params        = "slow",
    ),
    crossing_maneuver=SimpleNamespace(
        sensor_threshold  = CROSSINGS.sensor_threshold,
        leave_delay_s     = CROSSINGS.leave_delay_s,
        turn180_deg       = 180.0,
        turn180_dir       = "left",
        follow_dist_m     = 1.3,
        follow_speed      = 0.15,
        follow_params     = "slow",
        final_turn_deg    = 90.0,
        final_turn_dir    = "right",
        fwd_dist_m        = 1.45,
        fwd_speed         = 0.2,
        post_fwd_turn_deg = 90.0,
        post_fwd_turn_dir = "left",
    ),
    rejoin=SimpleNamespace(
        forward_before_seek_m     = 1.00,
        forward_before_seek_speed = 0.2,
        seek_speed                = 0.2,
        seek_timeout_s            = 30.0,
        pre_turn_nudge_m          = 0.05,
        pre_turn_nudge_speed      = 0.2,
        turn_deg                  = 90.0,
        turn_dir                  = "left",
        turn_rate                 = 0.3,
        follow_speed              = 0.15,
        follow_params             = "slow",
    ),
    crossing=SimpleNamespace(
        sensor_threshold = CROSSINGS.sensor_threshold,
        leave_delay_s    = CROSSINGS.leave_delay_s,
        turn_deg         = CROSSING1.turn_deg,
        turn_dir         = "right",
        turn_rate        = 0.3,
        forward_speed    = 0.15,
        resume_speed     = 0.15,
        resume_params    = "slow",
    ),
)


SERVO = SimpleNamespace(
    arm_id = 1,
    arm_mirror_id = 2,
    up     = -475,
    down   = 480,
    speed  = 100,
)


def servo_set_arm(position, speed=None):
    """Set both arm servos with mirrored positions (servo2 = -servo1)."""
    if speed is None:
        speed = SERVO.speed
    service.send("robobot/cmd/T0", f"servo {SERVO.arm_id} {position} {speed}")
    service.send("robobot/cmd/T0", f"servo {SERVO.arm_mirror_id} {-position} {speed}")


def servo_arm_up():
    """Drive arm servos to raised position; servo2 mirrors servo1."""
    servo_set_arm(SERVO.up, SERVO.speed)


def servo_arm_down():
    """Drive arm servos to lowered position; servo2 mirrors servo1."""
    servo_set_arm(SERVO.down, SERVO.speed)


def line_detected():
    return edge.lineValidCnt > LINE_FOLLOW_RULES.line_detect_valid_cnt


def line_reacquired():
    return edge.lineValidCnt >= LINE_FOLLOW_RULES.line_reacquire_valid_cnt


def line_lost():
    return edge.lineValidCnt < LINE_FOLLOW_RULES.line_lost_valid_cnt


def is_left_l_crossing():
    return (edge.sensorsAboveCount > CROSSINGS.l_min_sensors
            and edge.leftmostAboveIndex == 0
            and edge.rightmostAboveIndex is not None
            and edge.rightmostAboveIndex <= CROSSINGS.left_rightmost_max)


def is_right_l_crossing():
    return (edge.sensorsAboveCount > CROSSINGS.l_min_sensors
            and edge.rightmostAboveIndex == 7
            and edge.leftmostAboveIndex is not None
            and edge.leftmostAboveIndex >= CROSSINGS.right_leftmost_min)


def speed_for_params(params_name):
    if params_name == "slow":
        return LINE_SPEEDS.slow
    return LINE_SPEEDS.normal


ROUNDABOUT_SEQUENCE = [
    ("drive",     {"dist":  ROUNDABOUT_ENTRY.drive_dist_m,  "speed":   ROUNDABOUT_ENTRY.drive_speed}),
    ("turn",      {"deg":   ROUNDABOUT_ENTRY.turn_deg,       "dir":     ROUNDABOUT_ENTRY.turn_dir}),
    ("arc",       {}),
    ("turn",      {"deg":   ROUNDABOUT_EXIT.turn_deg,        "dir":     ROUNDABOUT_EXIT.turn_dir}),
    ("seek_line", {"speed": ROUNDABOUT_EXIT.seek_speed,      "timeout": ROUNDABOUT_EXIT.seek_timeout}),
]

DRIVE_TO_GOAL_SEQUENCE = [
    ("follow_line", {"dist": DRIVE_TO_GOAL.follow_dist_m, "speed": LINE_SPEEDS.normal}),
    ("turn",        {"deg":  DRIVE_TO_GOAL.turn1_deg,     "dir":   "right"}),
    ("drive",     {"dist":  DRIVE_TO_GOAL.fwd2_dist_m,  "speed":   DRIVE_TO_GOAL.drive_speed}),
    ("turn",      {"deg":   DRIVE_TO_GOAL.turn2_deg,    "dir":     "left"}),
    ("drive",     {"dist":  DRIVE_TO_GOAL.fwd3_dist_m,  "speed":   DRIVE_TO_GOAL.drive_speed}),
    ("turn",      {"deg":   DRIVE_TO_GOAL.turn3_deg,    "dir":     "left"}),
    ("seek_line", {"speed": DRIVE_TO_GOAL.seek_speed,   "timeout": DRIVE_TO_GOAL.seek_timeout}),
]

END_REJOIN_SEQUENCE = [
    ("drive",     {"dist": END.rejoin.forward_before_seek_m, "speed": END.rejoin.forward_before_seek_speed}),
    ("seek_line", {"speed": END.rejoin.seek_speed, "timeout": END.rejoin.seek_timeout_s}),
    ("drive",     {"dist": END.rejoin.pre_turn_nudge_m, "speed": END.rejoin.pre_turn_nudge_speed}),
    ("turn",      {"deg": END.rejoin.turn_deg, "dir": END.rejoin.turn_dir, "rate": END.rejoin.turn_rate}),
]


END_INITIAL_SEQUENCE = [
    ("drive", {"dist": END.initial.pre_turn_nudge_m, "speed": END.initial.pre_turn_nudge_speed}),
    ("turn",  {"deg": END.initial.first_turn_deg, "dir": END.initial.first_turn_dir}),
]


END_CROSSING_MANEUVER_SEQUENCE = [
    ("turn",        {"deg": END.crossing_maneuver.turn180_deg, "dir": END.crossing_maneuver.turn180_dir}),
    ("follow_line", {"dist": END.crossing_maneuver.follow_dist_m,
                     "speed": END.crossing_maneuver.follow_speed,
                     "params": END.crossing_maneuver.follow_params}),
    ("turn",        {"deg": END.crossing_maneuver.final_turn_deg, "dir": END.crossing_maneuver.final_turn_dir}),
    ("drive",       {"dist": END.crossing_maneuver.fwd_dist_m, "speed": END.crossing_maneuver.fwd_speed}),
    ("turn",        {"deg": END.crossing_maneuver.post_fwd_turn_deg, "dir": END.crossing_maneuver.post_fwd_turn_dir}),
]


END_REJOIN_CROSSING_SEQUENCE = [
    ("turn_forward", {"deg": END.crossing.turn_deg,
                      "dir": END.crossing.turn_dir,
                      "speed": END.crossing.forward_speed,
                      "rate": END.crossing.turn_rate,
                      "stop_after": False}),
]

FIRST_CROSSING_SEQUENCE = [
    ("turn_forward", {"deg": CROSSING1.turn_deg,
                      "dir": CROSSING1.turn_dir,
                      "speed": CROSSING1.forward_speed,
                      "rate": CROSSING1.turn_rate,
                      "stop_after": False}),
]

HALF_SECOND_CROSSING_SEQUENCE = [
    ("turn", {"deg": HALF.second_cross_turn_deg, "dir": HALF.second_cross_turn_dir}),
    ("follow_line", {"dist": HALF.second_cross_follow_dist_m,
                     "speed": HALF.second_cross_follow_speed,
                     "params": HALF.second_cross_follow_params}),
]


SUBMISSIONS = {
    "full":            dict(start_state=0),
    "half":            dict(start_state=0,
                            half_mode=True),


    "line_follow":     dict(start_state=0,
                            pure_follow_params="normal",
                            pure_follow_after_seek=True),
    "slow_line":       dict(start_state=0,
                            pure_follow_params="slow",
                            pure_follow_after_seek=True),
    "seesaw":          dict(start_state=0,
                            pure_follow_params="slow",
                            pure_follow_after_seek=True,
                            arm_position="down",
                            arm_settle_before_start_s=1.0,
                            arm_raise_after_s=4.0,
                            arm_raise_stages=[
                                (300, 0.5),
                                (0, 0.5),
                            ],
                            stop_on_line_loss=True,
                            line_loss_forward_s=2.0),
    "after_crossing_1": dict(start_state=10,
                             first_cross_done=True,
                             line_control=("slow", None)),


    "roundabout-entry": dict(start_state=10,
                             first_cross_done=True,
                             line_control=("slow", None),
                             stop_at_roundabout=True),
    "roundabout":      dict(start_state=11,
                            first_cross_done=True),
    "after_roundabout": dict(start_state=10,
                             crossing_count=1,
                             roundabout_done=True,
                             first_cross_done=True,
                             line_control=("normal", None)),
    "drive_to_goal":   dict(start_state=20,
                            crossing_count=4,
                            roundabout_done=True,
                            first_cross_done=True),
    "final_follow":    dict(start_state=21,
                            roundabout_done=True,
                            line_control=("normal", None)),


    "end":             dict(start_state=0,
                            end_mode=True),
}


def driveTurn(deg, direction, turn_rate=0.5, timeout_s=None):
    pose.tripBreset()
    signed_rate = turn_rate if direction == "left" else -turn_rate
    target_rad  = np.radians(deg)
    if timeout_s is None:

        nominal_time = target_rad / max(abs(signed_rate), 0.05)
        timeout_s = max(2.0, nominal_time * 2.0)
    state = 0
    if not service.is_quiet():
        print(f"% driveTurn: {deg:.0f}° {direction} at {turn_rate:.2f} rad/s")
    service.send("robobot/cmd/T0", "leds 16 0 100 0")
    while not _aborted():
        if pose.tripBtimePassed() > timeout_s:
            if not service.is_quiet():
                print(f"% driveTurn: timeout after {timeout_s:.2f} s")
            break
        if state == 0:
            service.send("robobot/cmd/ti", f"rc 0.0 {signed_rate:.2f}")
            state = 1
        elif state == 1:
            if abs(pose.tripBh) >= target_rad:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 2
        elif state == 2:
            if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
                break
        t.sleep(0.05)
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    if not service.is_quiet():
        print(f"# driveTurn: turned {np.degrees(pose.tripBh):.1f}° in {pose.tripBtimePassed():.2f} s")


def driveTurnForward(deg, direction, linvel, turn_rate=0.5, stop_after=True):
    pose.tripBreset()
    signed_rate = turn_rate if direction == "left" else -turn_rate
    target_rad  = np.radians(deg)
    if not service.is_quiet():
        print(f"% driveTurnForward: {deg:.0f}° {direction}, v={linvel:.2f} m/s, w={turn_rate:.2f} rad/s")
    service.send("robobot/cmd/T0", "leds 16 0 100 0")
    service.send("robobot/cmd/ti", f"rc {linvel:.2f} {signed_rate:.2f}")
    while not _aborted():
        if abs(pose.tripBh) >= target_rad:
            break
        t.sleep(0.02)
    if stop_after:
        service.send("robobot/cmd/ti", "rc 0.0 0.0")
        while not _aborted() and (abs(pose.velocity()) > 0.001 or abs(pose.turnrate()) > 0.001):
            t.sleep(0.02)
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    if not service.is_quiet():
        print(f"# driveTurnForward: turned {np.degrees(pose.tripBh):.1f}°")


def driveDistance(dist, speed=0.2):
    actual_speed = speed if dist >= 0 else -speed
    target_dist  = abs(dist)
    pose.tripBreset()
    state = 0
    if not service.is_quiet():
        print(f"% driveDistance: {dist:.2f} m at {actual_speed:.2f} m/s")
    service.send("robobot/cmd/T0", "leds 16 0 100 0")
    while not _aborted():
        if state == 0:
            service.send("robobot/cmd/ti", f"rc {actual_speed:.2f} 0.0")
            state = 1
        elif state == 1:

            if abs(pose.tripB) >= target_dist or pose.tripBtimePassed() > 20:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 2
        elif state == 2:
            if abs(pose.velocity()) < 0.001:
                break
        t.sleep(0.05)
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    if not service.is_quiet():
        print(f"# driveDistance: drove {pose.tripB:.3f} m in {pose.tripBtimePassed():.2f} s")


def driveRoundabout():
    radius_m   = ROUNDABOUT_ARC.diameter_m / 2.0
    sign       = 1.0 if ROUNDABOUT_ARC.direction == "left" else -1.0
    turn_rate  = sign * ROUNDABOUT_ARC.speed / radius_m
    target_rad = np.radians(ROUNDABOUT_ARC.total_degrees)
    pose.tripBreset()
    if not service.is_quiet():
        print(f"% driveRoundabout: ⌀{ROUNDABOUT_ARC.diameter_m:.2f} m, "
              f"v={ROUNDABOUT_ARC.speed:.2f} m/s, w={turn_rate:.3f} rad/s, "
              f"target={ROUNDABOUT_ARC.total_degrees:.0f}°")
    service.send("robobot/cmd/T0", "leds 16 0 0 30")
    service.send("robobot/cmd/ti", f"rc {ROUNDABOUT_ARC.speed:.3f} {turn_rate:.3f}")
    last_print = t.time()
    while not _aborted():
        if abs(pose.tripBh) >= target_rad:
            break
        if not service.is_quiet() and t.time() - last_print >= 0.5:
            print(f"# arc: {np.degrees(abs(pose.tripBh)):.1f}° / {ROUNDABOUT_ARC.total_degrees:.0f}°")
            last_print = t.time()
        t.sleep(0.02)
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    while not _aborted() and abs(pose.velocity()) > 0.001:
        t.sleep(0.02)
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    if not service.is_quiet():
        print(f"% driveRoundabout: done ({np.degrees(abs(pose.tripBh)):.1f}° in {pose.tripBtimePassed():.1f} s)")


def driveLineFollow(dist, speed, params="normal"):
    pose.tripBreset()
    target_dist = abs(dist)
    edge.lineControl(velocity=speed, refPosition=0, params=params)
    if not service.is_quiet():
        print(f"% driveLineFollow: {dist:.2f} m at {speed:.2f} m/s (params={params})")
    while not _aborted():

        if abs(pose.tripB) >= target_dist:
            break
        t.sleep(0.01)
    edge.lineControl(0)
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    while not _aborted() and abs(pose.velocity()) > 0.001:
        t.sleep(0.02)
    if not service.is_quiet():
        print(f"# driveLineFollow: drove {pose.tripB:.3f} m")


def driveForwardTime(seconds, speed):
    """Drive straight for seconds at speed (m/s), then stop."""
    pose.tripBreset()
    service.send("robobot/cmd/ti", f"rc {speed:.2f} 0.0")
    while not _aborted():
        if pose.tripBtimePassed() >= seconds:
            break
        t.sleep(0.01)
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    while not _aborted() and abs(pose.velocity()) > 0.001:
        t.sleep(0.02)
    if not service.is_quiet():
        print(f"# driveForwardTime: ran for {pose.tripBtimePassed():.2f} s at {speed:.2f} m/s")


def seekLine(speed, timeout_s):
    pose.tripBreset()
    service.send("robobot/cmd/ti", f"rc {speed:.2f} 0.0")
    while not _aborted():
        if line_detected():
            break
        if pose.tripBtimePassed() > timeout_s:
            break
        t.sleep(0.01)
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    while not _aborted() and abs(pose.velocity()) > 0.001:
        t.sleep(0.02)
    if not service.is_quiet():
        result = "found" if line_detected() else "timeout"
        print(f"% seekLine: {result} after {pose.tripBtimePassed():.1f} s")


def run_sequence(steps):
    """Execute a list of (action, params) steps in order."""
    for action, params in steps:
        if _aborted():
            break
        if action == "drive":
            driveDistance(params["dist"], params.get("speed", 0.2))
        elif action == "turn":
            driveTurn(params["deg"], params["dir"], params.get("rate", 0.5))
        elif action == "turn_forward":
            speed = params.get("speed", 0.0)
            if speed > 0:
                driveTurnForward(params["deg"], params["dir"],
                                 speed, params.get("rate", 0.5),
                                 params.get("stop_after", True))
            else:
                driveTurn(params["deg"], params["dir"], params.get("rate", 0.5))
        elif action == "arc":
            driveRoundabout()
        elif action == "seek_line":
            seekLine(params["speed"], params["timeout"])
        elif action == "follow_line":
            params_name = params.get("params", "normal")
            driveLineFollow(params["dist"], params.get("speed", speed_for_params(params_name)), params_name)
        else:
            if not service.is_quiet():
                print(f"% run_sequence: unknown action '{action}' — skipped")


class MissionKeyListener:
    def __init__(self):
        self._fd = None
        self._old = None
        self._thread = None
        self._stop_evt = threading.Event()
        self.switch_to_teleop = False
        self.user_quit = False

    def start(self):
        if not _tty_available or not sys.stdin.isatty():
            print("% key listener disabled: no interactive POSIX terminal")
            return self
        try:
            self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            mode = list(termios.tcgetattr(self._fd))
            mode[3] = mode[3] & ~(termios.ICANON | termios.ECHO)
            mode[6] = list(mode[6])
            mode[6][termios.VMIN] = 1
            mode[6][termios.VTIME] = 0
            termios.tcsetattr(self._fd, termios.TCSANOW, mode)
        except Exception as e:
            print(f"% key listener disabled: tty setup failed ({e})")
            self._fd = None
            self._old = None
            return self
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        if not service.is_quiet():
            print("% hotkeys: SPACE = stop + turn 180 + teleop, q/ESC = stop")
        return self

    def _run(self):
        while not self._stop_evt.is_set() and not _aborted():
            try:
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
            except Exception:
                break
            if not r:
                continue
            try:
                ch = sys.stdin.read(1)
            except Exception:
                break
            if ch == " ":
                print("\n% spacebar -> stopping mission, turning 180, switching to teleop")
                self.switch_to_teleop = True
                _mission_abort_evt.set()
                break
            if ch in ("q", "\x1b"):
                label = "ESC" if ch == "\x1b" else "q"
                print(f"\n% {label} -> stopping mission")
                self.user_quit = True
                _mission_abort_evt.set()
                break

    def stop(self):
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._old is not None and self._fd is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
            except Exception:
                pass
        self._old = None
        self._fd = None


def _load_teleop_module():
    import importlib.util
    teleop_path = Path(__file__).resolve().parent / "mqtt-teleop.py"
    spec = importlib.util.spec_from_file_location("_mqtt_teleop_inproc", str(teleop_path))
    if spec is None or spec.loader is None:
        raise ImportError(f"could not load {teleop_path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def safeHandoffTurn180(direction="left",
                       turn_rate=1.0,
                       hard_time_cap_s=4.5,
                       overshoot_factor=1.25):
    target_rad = np.pi
    slow_zone_rad = np.radians(140.0)
    stop_at_rad = np.radians(172.0)
    slow_turn_rate = 0.35
    sign = 1.0 if direction == "left" else -1.0
    pose.tripBreset()

    service.send("robobot/cmd/T0", "leds 16 0 100 0")
    start_t = t.time()
    last_t = start_t
    commanded_angle = 0.0
    reason = "service_stop"

    while not _aborted():
        now = t.time()
        dt = now - last_t
        last_t = now
        elapsed = now - start_t

        if elapsed >= hard_time_cap_s:
            reason = "hard_time_cap"
            break

        progress = abs(pose.tripBh)
        if progress >= stop_at_rad:
            reason = "pose_target_reached"
            break
        if progress >= target_rad * overshoot_factor:
            reason = "pose_overshoot_guard"
            break
        if commanded_angle >= target_rad * overshoot_factor:
            reason = "commanded_overshoot_guard"
            break

        active_rate = turn_rate if progress < slow_zone_rad else slow_turn_rate
        service.send("robobot/cmd/ti", f"rc 0.0 {sign * active_rate:.3f}")
        commanded_angle += active_rate * dt
        t.sleep(0.02)

    for _ in range(3):
        service.send("robobot/cmd/ti", "rc 0.0 0.0")
        t.sleep(0.02)
    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    print(
        f"% handoff turn done: pose={np.degrees(pose.tripBh):.1f} deg, "
        f"cmd={np.degrees(commanded_angle):.1f} deg ({reason})"
    )


def run_teleop_in_process():
    try:
        teleop = _load_teleop_module()
    except Exception as e:
        print(f"% could not load teleop module: {e}")
        return

    edge.lineControl(0)
    service.send("robobot/cmd/ti", "rc 0.0 0.0")
    service.send("robobot/cmd/T0", "leds 16 0 0 30")
    _mission_abort_evt.clear()
    service.stop = False
    print("% teleop active - w/a/s/d to drive, SPACE to halt, q to quit")
    try:
        teleop.loop(step_mode=True)
    finally:
        service.send("robobot/cmd/ti", "rc 0.0 0.0")
        service.send("robobot/cmd/T0", "leds 16 0 0 0")


def apply_pid_overrides():
    """Apply optional CLI overrides to both sedge.PARAM_SETS["normal"/"slow"].

    No implicit defaults: if no flags are given, sedge.py's PARAM_SETS are the
    single source of truth. Each flag overrides ONLY the corresponding field.

    Called once after argparse but before mission start.
    """
    overrides = {}
    for key, attr in (
        ("lineKp", "kp"),
        ("lineKd", "kd"),
        ("lineKi", "ki"),
        ("lineDerivativeAlpha", "alpha"),
    ):
        val = getattr(service.args, attr, None)
        if val is not None:
            overrides[key] = float(val)
    beta_alias = getattr(service.args, "beta", None)
    if beta_alias is not None and "lineDerivativeAlpha" not in overrides:
        overrides["lineDerivativeAlpha"] = float(beta_alias)
    if "lineDerivativeAlpha" in overrides:
        overrides["lineDerivativeAlpha"] = max(0.0, min(1.0, overrides["lineDerivativeAlpha"]))
    if not overrides:
        return
    for params_name in ("normal", "slow"):
        edge.PARAM_SETS[params_name].update(overrides)
    for k, v in overrides.items():
        setattr(edge, k, v)
    if not service.is_quiet():
        print(f"% controller overrides applied to normal+slow: {overrides}")


def driveMission(submission="full"):
    """Full mission state machine.

    States:
      0  — immediate start, drive forward (arm already raised at driveMission entry)
      1  — drive until line found, then start PD controller
      2  — wait for full stop, then exit
      10 — line following + crossing reactions + line-end detection
      11 — roundabout sequence (blocking), then resume line following
      20 — drive-to-goal sequence (blocking), then start final line follow
      21 — follow final line until it ends, then stop
      22 — end-mode rejoin sequence (drive/seek/nudge/turn onto new line)
      23 — end-mode follow line + one crossing-1-like right turn, then continue
      30 — pure line following (no crossing/roundabout/end logic; tuning only)
      99 — mission complete (exits loop)

    submission: name from SUBMISSIONS — selects an entry point and presets
                state, counters and flags accordingly.
    """
    sub = SUBMISSIONS.get(submission, SUBMISSIONS["full"])

    state = sub.get("start_state", 0)
    pose.tripBreset()
    dist_to_line = 0.0

    crossing_count        = sub.get("crossing_count", 0)
    was_at_crossing       = False
    last_time_at_crossing = None

    first_cross_done    = sub.get("first_cross_done", False)
    last_hard_turn_time = None

    roundabout_done     = sub.get("roundabout_done", False)
    stop_at_roundabout  = sub.get("stop_at_roundabout", False)
    half_mode           = bool(sub.get("half_mode", False))
    end_mode            = sub.get("end_mode", False)
    pure_follow_after_seek = sub.get("pure_follow_after_seek", False)
    pure_follow_params  = sub.get("pure_follow_params", "normal")
    arm_settle_before_start_s = sub.get("arm_settle_before_start_s", None)
    arm_raise_after_s   = sub.get("arm_raise_after_s", None)
    arm_raise_stages    = sub.get("arm_raise_stages", None)
    stop_on_line_loss   = bool(sub.get("stop_on_line_loss", False))
    line_loss_forward_s = float(sub.get("line_loss_forward_s", 0.0))
    line_end_lost_count = 0
    pure_follow_lost_count = 0
    pure_follow_stop_pending = False
    arm_raised_after_delay = False
    arm_raise_stage_idx = -1
    arm_raise_stage_deadline = None
    arm_start_settle_done = False

    lost_line_since = None
    goal_line_end_count = 0
    end_rejoin_crossing_done = False
    end_rejoin_was_at_crossing = False
    end_rejoin_last_time_at_crossing = None
    end21_was_at_crossing = False
    end21_last_time_at_crossing = None
    half_second_crossing_cnt = 0

    if not service.is_quiet():
        print(f"% driveMission: starting (submission={submission}, state={state}, "
              f"crossing_count={crossing_count}, first_cross_done={first_cross_done}, "
              f"roundabout_done={roundabout_done})")

    arm_position = sub.get("arm_position", "up")
    if arm_position == "down":
        servo_arm_down()
        if not service.is_quiet():
            print(f"% servo arm -> down (servo {SERVO.arm_id} {SERVO.down} {SERVO.speed}, "
                  f"servo {SERVO.arm_mirror_id} {-SERVO.down} {SERVO.speed})")
    else:
        servo_arm_up()
        if not service.is_quiet():
            print(f"% servo arm -> up (servo {SERVO.arm_id} {SERVO.up} {SERVO.speed}, "
                  f"servo {SERVO.arm_mirror_id} {-SERVO.up} {SERVO.speed})")

    line_ctrl_cfg = sub.get("line_control")
    if line_ctrl_cfg is not None:
        params_name, lc_speed = line_ctrl_cfg
        if lc_speed is None:
            lc_speed = speed_for_params(params_name)
        edge.lineControl(velocity=lc_speed, refPosition=0, params=params_name)

    service.send("robobot/cmd/T0", "leds 16 0 100 0")

    last_state = None
    while not _aborted():

        if state != last_state:
            set_telemetry(state=str(state))
            last_state = state


        if state == 0:
            if arm_settle_before_start_s and not arm_start_settle_done:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                if not service.is_quiet():
                    print(f"% state 0: settling with arm position for {arm_settle_before_start_s:.1f}s")
                t.sleep(float(arm_settle_before_start_s))
                arm_start_settle_done = True
            service.send("robobot/cmd/ti", f"rc {LINE_SEEK.forward_speed:.2f} 0.0")
            service.send("robobot/cmd/T0/", "lognow 3")
            state = 1


        elif state == 1:
            if pose.tripB > LINE_SEEK.max_dist_m or pose.tripBtimePassed() > LINE_SEEK.max_time_s:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                state = 2
            if line_detected():
                dist_to_line = pose.tripB
                pose.tripBreset()
                if end_mode:


                    if not service.is_quiet():
                        print(f"% line found after {dist_to_line:.2f} m → end: hard right 90°")
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    t.sleep(LINE_FOLLOW_RULES.post_stop_settle_s)

                    run_sequence(END_INITIAL_SEQUENCE)
                    edge.lineControl(velocity=END.initial.follow_speed, refPosition=0, params=END.initial.follow_params)
                    goal_line_end_count = 0
                    state = 21
                else:
                    if pure_follow_after_seek:
                        pure_speed = speed_for_params(pure_follow_params)
                        edge.lineControl(velocity=pure_speed, refPosition=0, params=pure_follow_params)
                        if not service.is_quiet():
                            print(f"% line found after {dist_to_line:.2f} m → pure line follow "
                                  f"(state 30, params={pure_follow_params}, v={pure_speed:.2f})")
                        state = 30
                    else:
                        edge.lineControl(velocity=LINE_SPEEDS.normal, refPosition=0, params="normal")
                        if not service.is_quiet():
                            print(f"% line found after {dist_to_line:.2f} m → state 10")
                        state = 10


        elif state == 2:
            if abs(pose.velocity()) < 0.001:
                state = 99


        elif state == 10:
            edge.mission_crossing_count = crossing_count

            if half_mode and roundabout_done and crossing_count == 1:
                if is_left_l_crossing():
                    half_second_crossing_cnt = min(20, half_second_crossing_cnt + 1)
                elif half_second_crossing_cnt > 0:
                    half_second_crossing_cnt -= 1
                at_crossing = half_second_crossing_cnt >= CROSSINGS.sensor_threshold
            else:
                half_second_crossing_cnt = 0
                at_crossing = edge.crossingLineCnt >= CROSSINGS.sensor_threshold
            now = datetime.now()
            if at_crossing:
                last_time_at_crossing = now
                was_at_crossing = True
            elif was_at_crossing and last_time_at_crossing is not None:
                clear_s = (now - last_time_at_crossing).total_seconds()
                if clear_s >= CROSSINGS.leave_delay_s:
                    crossing_count += 1
                    was_at_crossing = False
                    last_time_at_crossing = None
                    set_telemetry(crossing=crossing_count)
                    print(f"% crossing {crossing_count}")

            if at_crossing:
                crossing_number = crossing_count + 1

                if half_mode and roundabout_done and crossing_number == 2:
                    if not service.is_quiet():
                        print("% half crossing 2 -> hard left 90, slow follow 0.20 m, stop")
                    edge.lineControl(0)
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    t.sleep(LINE_FOLLOW_RULES.post_stop_settle_s)
                    run_sequence(HALF_SECOND_CROSSING_SEQUENCE)
                    edge.lineControl(0)
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    state = 2

                elif crossing_number == 1 and not first_cross_done:
                    if not service.is_quiet():
                        print(f"% crossing 1 → turn {CROSSING1.turn_dir} {CROSSING1.turn_deg:.0f}°"
                              f" (fwd={CROSSING1.forward_speed:.2f} m/s)")
                    edge.lineControl(0)
                    run_sequence(FIRST_CROSSING_SEQUENCE)
                    first_cross_done = True
                    edge.lineControl(velocity=LINE_SPEEDS.slow, refPosition=0, params="slow")

                elif crossing_count == CROSSINGS.stop_at - 1:
                    print(f"% crossing {crossing_number}")
                    if not service.is_quiet():
                        print(f"% crossing {crossing_number} → end sequence")
                    edge.lineControl(0)
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    state = 20

                elif crossing_count >= CROSSINGS.go_straight_until:
                    cooldown_ok = (
                        last_hard_turn_time is None or
                        (now - last_hard_turn_time).total_seconds() >= CROSSINGS.hard_turn_cooldown_s
                    )
                    if cooldown_ok:
                        hard_left  = is_left_l_crossing()
                        hard_right = is_right_l_crossing()
                        if hard_left or hard_right:
                            direction = "left" if hard_left else "right"
                            if not service.is_quiet():
                                print(f"% crossing {crossing_number} → hard turn {direction} 90°")
                            edge.lineControl(0)
                            service.send("robobot/cmd/ti", "rc 0.0 0.0")
                            t.sleep(LINE_FOLLOW_RULES.post_stop_settle_s)
                            driveTurn(LINE_FOLLOW_RULES.hard_turn_deg, direction)
                            last_hard_turn_time = datetime.now()
                            edge.lineControl(velocity=LINE_SPEEDS.normal, refPosition=0, params="normal")

            if not roundabout_done and state == 10:
                if line_lost():
                    line_end_lost_count += 1
                else:
                    line_end_lost_count = 0
                if line_end_lost_count >= LINE_END.lost_confirm:
                    if stop_at_roundabout:
                        if not service.is_quiet():
                            print("% line ended → ROUNDABOUT-ENTRY trigger detected; stopping (no roundabout)")
                        edge.lineControl(0)
                        service.send("robobot/cmd/ti", "rc 0.0 0.0")
                        service.send("robobot/cmd/T0", "leds 16 100 100 0")
                        state = 2
                    else:
                        if not service.is_quiet():
                            print("% line ended → starting roundabout sequence")
                        edge.lineControl(0)
                        state = 11

            elif roundabout_done and state == 10:
                if line_reacquired():
                    lost_line_since = None
                elif line_lost():
                    if lost_line_since is None:
                        lost_line_since = datetime.now()
                    elapsed = (datetime.now() - lost_line_since).total_seconds()
                    if elapsed >= LINE.lost_timeout:
                        edge.lineControl(0)
                        service.send("robobot/cmd/ti", "rc 0.0 0.0")
                        if not service.is_quiet():
                            print("% line lost too long → recovery stop")
                        state = 2


        elif state == 11:
            run_sequence(ROUNDABOUT_SEQUENCE)
            crossing_count  = 1
            roundabout_done = True
            lost_line_since = None
            edge.lineControl(velocity=LINE_SPEEDS.normal, refPosition=0, params="normal")
            if not service.is_quiet():
                print("% roundabout done → resuming line follow (crossing_count=1)")
            state = 10


        elif state == 20:
            run_sequence(DRIVE_TO_GOAL_SEQUENCE)
            goal_line_end_count = 0
            edge.lineControl(velocity=LINE_SPEEDS.normal, refPosition=0, params="normal")
            if not service.is_quiet():
                print("% drive-to-goal done → following final line to end")
            state = 21


        elif state == 21:
            if end_mode:
                at_crossing = edge.crossingLineCnt >= END.crossing_maneuver.sensor_threshold
                now = datetime.now()
                if at_crossing:
                    end21_last_time_at_crossing = now
                    end21_was_at_crossing = True
                elif end21_was_at_crossing and end21_last_time_at_crossing is not None:
                    clear_s = (now - end21_last_time_at_crossing).total_seconds()
                    if clear_s >= END.crossing_maneuver.leave_delay_s:
                        end21_was_at_crossing = False
                        end21_last_time_at_crossing = None
                        if not service.is_quiet():
                            print("% end mode: crossing detected → turn 180°, follow 1 m, turn right 90°")
                        edge.lineControl(0)
                        service.send("robobot/cmd/ti", "rc 0.0 0.0")
                        while not _aborted() and abs(pose.velocity()) > 0.001:
                            t.sleep(0.02)
                        run_sequence(END_CROSSING_MANEUVER_SEQUENCE)
                        state = 22
            else:
                if line_lost():
                    goal_line_end_count += 1
                else:
                    goal_line_end_count = 0
                if goal_line_end_count >= LINE_END.lost_confirm:
                    edge.lineControl(0)
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    if not service.is_quiet():
                        print("% final line ended → stopping")
                    state = 2


        elif state == 22:
            if not service.is_quiet():
                print("% end mode: rejoin sequence (drive/seek/nudge/turn)")
            run_sequence(END_REJOIN_SEQUENCE)
            end_rejoin_crossing_done = False
            end_rejoin_was_at_crossing = False
            end_rejoin_last_time_at_crossing = None
            edge.lineControl(velocity=END.rejoin.follow_speed, refPosition=0, params=END.rejoin.follow_params)
            state = 23


        elif state == 23:
            edge.mission_crossing_count = 0 if not end_rejoin_crossing_done else 1
            at_crossing = edge.crossingLineCnt >= END.crossing.sensor_threshold
            now = datetime.now()
            if at_crossing:
                end_rejoin_last_time_at_crossing = now
                end_rejoin_was_at_crossing = True
            elif end_rejoin_was_at_crossing and end_rejoin_last_time_at_crossing is not None:
                clear_s = (now - end_rejoin_last_time_at_crossing).total_seconds()
                if clear_s >= END.crossing.leave_delay_s:
                    end_rejoin_was_at_crossing = False
                    end_rejoin_last_time_at_crossing = None

            if at_crossing and not end_rejoin_crossing_done:
                if not service.is_quiet():
                    print(f"% end mode crossing → turn {END.crossing.turn_dir} "
                          f"{END.crossing.turn_deg:.0f}° (fwd={END.crossing.forward_speed:.2f} m/s)")
                edge.lineControl(0)
                run_sequence(END_REJOIN_CROSSING_SEQUENCE)
                end_rejoin_crossing_done = True
                edge.lineControl(velocity=END.crossing.resume_speed, refPosition=0, params=END.crossing.resume_params)


        elif state == 30:
            edge.mission_crossing_count = 0
            if (arm_raise_after_s is not None and not arm_raised_after_delay
                    and arm_raise_stage_idx < 0
                    and pose.tripBtimePassed() >= arm_raise_after_s):
                if arm_raise_stages:
                    arm_raise_stage_idx = 0
                    p0, hold0 = arm_raise_stages[0]
                    servo_set_arm(int(p0))
                    arm_raise_stage_deadline = t.time() + float(hold0)
                    if not service.is_quiet():
                        print(f"% state 30: arm stage 1/{len(arm_raise_stages)} -> {int(p0)} "
                              f"for {float(hold0):.1f}s after {arm_raise_after_s:.1f}s follow")
                else:
                    servo_arm_up()
                    arm_raised_after_delay = True
                    if not service.is_quiet():
                        print(f"% state 30: arm raised after {arm_raise_after_s:.1f}s follow")

            if arm_raise_stage_idx >= 0 and arm_raise_stage_deadline is not None and t.time() >= arm_raise_stage_deadline:
                nxt = arm_raise_stage_idx + 1
                if arm_raise_stages and nxt < len(arm_raise_stages):
                    pos, hold_s = arm_raise_stages[nxt]
                    servo_set_arm(int(pos))
                    arm_raise_stage_idx = nxt
                    arm_raise_stage_deadline = t.time() + float(hold_s)
                    if not service.is_quiet():
                        print(f"% state 30: arm stage {nxt+1}/{len(arm_raise_stages)} -> {int(pos)} "
                              f"for {float(hold_s):.1f}s")
                else:
                    arm_raised_after_delay = True
                    arm_raise_stage_idx = -1
                    arm_raise_stage_deadline = None
                    if not service.is_quiet():
                        print("% state 30: arm staged raise complete")

            if stop_on_line_loss:
                if line_lost():
                    pure_follow_lost_count += 1
                else:
                    pure_follow_lost_count = 0
                if pure_follow_lost_count >= LINE_FOLLOW_RULES.line_lost_stop_confirm_cnt:
                    pure_follow_stop_pending = True
                    pure_follow_lost_count = 0
                    if not service.is_quiet():
                        print("% state 30: line lost -> stop pending (wait for arm stages)")

            if pure_follow_stop_pending:
                arm_seq_done = (
                    arm_raise_after_s is None
                    or arm_raised_after_delay
                )
                if arm_seq_done:
                    edge.lineControl(0)
                    if line_loss_forward_s > 0.0:
                        fwd_speed = speed_for_params(pure_follow_params)
                        if not service.is_quiet():
                            print(f"% state 30: line lost -> driving forward {line_loss_forward_s:.1f}s "
                                  f"at {fwd_speed:.2f} m/s before stop")
                        driveForwardTime(line_loss_forward_s, fwd_speed)
                    else:
                        service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    if not service.is_quiet():
                        print("% state 30: stopping after arm stages complete")
                    state = 2

        else:
            if not service.is_quiet():
                print(f"% driveMission: done — dist to line {dist_to_line:.2f} m, "
                      f"along line {pose.tripB:.2f} m in {pose.tripBtimePassed():.1f} s")
            service.send("robobot/cmd/ti", "rc 0.0 0.0")
            break

        t.sleep(0.01)

    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    if not service.is_quiet():
        print("% driveMission: end")


def loop(submission="full"):
    from ulog import flog
    oldstate   = -1
    state_time = datetime.now()

    service.send("robobot/cmd/T0", "leds 16 30 30 0")
    edge.lineControl(0)

    state = 103
    while not _aborted():
        if state == 103:
            driveMission(submission=submission)
            state = 99
        else:
            if not service.is_quiet():
                print(f"% loop: finished (state={state})")
            break

        if state != oldstate:
            flog.writeRemark(f"% State change from {oldstate} to {state}")
            if not service.is_quiet():
                print(f"% State change from {oldstate} to {state}")
            oldstate   = state
            state_time = datetime.now()

        t.sleep(0.1)

    service.send("robobot/cmd/T0", "leds 16 0 0 0")
    gpio.set_value(20, 0)
    edge.lineControl(0)
    service.send("robobot/cmd/ti", "rc 0 0")
    t.sleep(0.05)


if __name__ == "__main__":
    if service.process_running("mqtt-client"):
        print("% mqtt-client is already running - terminating")
        print("%   to kill a stuck instance: pkill mqtt-client   (or pkill -9 mqtt-client)")
    else:
        setproctitle("mqtt-client")

        service.parser.add_argument(
            "--video-out",
            nargs="?",
            const="auto",
            default="",
            help=("Record annotated camera stream to an MP4 in cv_runs/. "
                  "Optional value = output filename or path."),
        )
        service.parser.add_argument(
            "--video-stream-url",
            default=DEFAULT_RECORD_STREAM_URL,
            help=f"Camera stream URL for --video-out (default: {DEFAULT_RECORD_STREAM_URL})",
        )
        service.parser.add_argument(
            "--verbose",
            action="store_true",
            help="Enable mission debug prints (default is quiet mode).",
        )
        service.parser.add_argument(
            "--submission",
            default="full",
            choices=sorted(SUBMISSIONS.keys()),
            help=("Mission entry point. Choices: "
                  + ", ".join(sorted(SUBMISSIONS.keys())) + " (default: full)."),
        )
        service.parser.add_argument(
            "--kp", type=float, default=None,
            help="Override line-follow Kp for params='normal' and 'slow' (e.g. 0.4).",
        )
        service.parser.add_argument(
            "--kd", type=float, default=None,
            help="Override line-follow Kd for params='normal' and 'slow' (e.g. 0.10).",
        )
        service.parser.add_argument(
            "--ki", type=float, default=None,
            help="Override line-follow Ki for params='normal' and 'slow' (default 0).",
        )
        service.parser.add_argument(
            "--alpha", type=float, default=None,
            help=("Override derivative smoothing alpha (0..1). "
                  "d = alpha*d_prev + (1-alpha)*d_raw; 0 = raw derivative, 1 = hold previous D. "
                  "Applies to normal+slow."),
        )
        service.parser.add_argument(
            "--beta", type=float, default=None,
            help="Deprecated alias for --alpha.",
        )

        pre_test_mode = any(arg in ("-t", "--test") for arg in sys.argv[1:])
        if pre_test_mode:
            start_test_log()

        start_teensy_interface()
        service.setup('localhost')

        recorder = None
        if service.connected:
            service.args.now = True
            test_mode = bool(getattr(service.args, "test", False))


            if not (getattr(service.args, "verbose", False) or test_mode):
                service.args.silent = True

            apply_pid_overrides()

            submission = getattr(service.args, "submission", "full")
            if submission not in SUBMISSIONS:
                print(f"% unknown submission '{submission}', using 'full'")
                submission = "full"
            if not service.is_quiet():
                print(f"% Starting final mission (submission={submission})")

            video_out = getattr(service.args, "video_out", "")
            if video_out:
                recorder = MissionRecorder(
                    video_out=video_out,
                    stream_url=getattr(service.args, "video_stream_url",
                                       DEFAULT_RECORD_STREAM_URL),
                ).start()

            key_listener = MissionKeyListener().start() if test_mode else None
            try:
                loop(submission=submission)
            except KeyboardInterrupt:
                print("\n% Ctrl+C -> aborting mission")
                service.stop = True
            finally:
                if key_listener is not None:
                    key_listener.stop()
                if recorder is not None:
                    recorder.stop()

            if key_listener is not None and key_listener.switch_to_teleop:
                _mission_abort_evt.clear()
                if _tty_available and sys.stdin.isatty():
                    try:
                        termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
                    except Exception:
                        pass
                print("% test handoff -> turning 180 degrees before teleop")
                safeHandoffTurn180(direction="left",
                                   turn_rate=1.0,
                                   hard_time_cap_s=4.5,
                                   overshoot_factor=1.25)
                run_teleop_in_process()
        service.terminate()
        stop_teensy_interface()
    if not service.is_quiet():
        print("% Main Terminated")
    stop_test_log()
