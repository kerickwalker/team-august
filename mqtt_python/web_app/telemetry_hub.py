#!/usr/bin/env python3
"""
telemetry_hub.py
=============================================================================
Single in-process cache of all sensor + fusion streams, designed to feed the
live telemetry panel in field_web_app.py.

Topics
------
Two prefixes:

    T0/...        — raw hardware streams from the robot (existing convention).
                    The sensor lives on the Teensy / firmware side; we just
                    listen.

    august/...    — our own fusion outputs (Kalman, vision, landmark, aruco).
                    These are published BY OUR code (skalman, svision_pose,
                    saruco wrapper, slandmark_match) and consumed here.

Subscribed topics (actual broker names):
    robobot/drive/T0/pose          encoder pose       "ts ts2 x y heading tilt"
    robobot/drive/T0/vel           wheel velocity     "ts ts2 vL vR"
    robobot/drive/T0/gyro          IMU gyroscope      "ts gx gy gz"
    robobot/drive/T0/acc           IMU accelerometer  "ts ax ay az"
    robobot/kalman/state           fused state        JSON {position,orientation,velocity,...}
    robobot/drive/T0/vision_pose   vision pose        space-delimited "ts x y z yaw pitch source ..."
ArUco / landmark fixes are derived from vision_pose source field; no separate broker topic.

Mock mode
---------
If `--mock` is passed (or hub.start_mock() is called), a background thread
synthesises plausible data so the UI can be developed and demoed without
the robot.
"""
from __future__ import annotations
import json
import math
import random
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, Optional, Tuple


# Topic constants — the UI imports these so a rename touches only this file.
TOPIC_ENC_POSE        = "robobot/drive/T0/pose"
TOPIC_ENC_VEL         = "robobot/drive/T0/vel"
TOPIC_IMU_GYRO        = "robobot/drive/T0/gyro"
TOPIC_IMU_ACC         = "robobot/drive/T0/acc"
TOPIC_KALMAN_STATE    = "robobot/kalman/state"
TOPIC_VISION_POSE     = "robobot/drive/T0/vision_pose"
# Synthetic topics (not on broker) — populated by deriving from vision_pose
# source field so the panel still shows ArUco / landmark fixes when they
# happen.
TOPIC_VISION_ARUCO    = "_synth/vision/aruco"
TOPIC_VISION_LANDMARK = "_synth/vision/landmark"

ALL_TOPICS = (
    TOPIC_ENC_POSE, TOPIC_ENC_VEL,
    TOPIC_IMU_GYRO, TOPIC_IMU_ACC,
    TOPIC_KALMAN_STATE, TOPIC_VISION_POSE,
    TOPIC_VISION_ARUCO, TOPIC_VISION_LANDMARK,
)

# Only topics that actually exist on the broker get subscribed; the synthetic
# ones are populated locally from vision_pose.
SUBSCRIBED_TOPICS = (
    TOPIC_ENC_POSE, TOPIC_ENC_VEL,
    TOPIC_IMU_GYRO, TOPIC_IMU_ACC,
    TOPIC_KALMAN_STATE, TOPIC_VISION_POSE,
)


@dataclass
class StreamCache:
    """Last-known value for one topic, plus when it arrived."""
    topic: str
    value: Optional[Dict[str, Any]] = None
    last_received: float = 0.0
    update_count: int = 0


@dataclass
class TelemetryEvent:
    """One human-readable line for the event log (last N kept)."""
    t: float
    source: str        # "aruco", "landmark", "vision", "kalman", "system"
    text: str


class TelemetryHub:
    """Thread-safe cache of all subscribed topics."""

    EVENT_LOG_SIZE = 30

    def __init__(self):
        self._lock = threading.Lock()
        self._streams: Dict[str, StreamCache] = {
            t: StreamCache(topic=t) for t in ALL_TOPICS
        }
        self._events: Deque[TelemetryEvent] = deque(maxlen=self.EVENT_LOG_SIZE)
        self._mqtt_client = None
        self._mock_thread: Optional[threading.Thread] = None
        self._mock_stop = threading.Event()
        self._mode = "idle"   # "idle" | "mqtt" | "mock"

    # -------------------------------------------------------------------
    # Read API — used by the UI callback.
    # -------------------------------------------------------------------
    def snapshot(self) -> Dict[str, Any]:
        """Thread-safe shallow copy of all streams + recent events."""
        with self._lock:
            streams = {
                t: {
                    "topic":         s.topic,
                    "value":         dict(s.value) if isinstance(s.value, dict) else s.value,
                    "last_received": s.last_received,
                    "update_count":  s.update_count,
                }
                for t, s in self._streams.items()
            }
            events = [
                {"t": e.t, "source": e.source, "text": e.text}
                for e in self._events
            ]
            mode = self._mode
        return {"streams": streams, "events": events, "mode": mode, "now": time.time()}

    # -------------------------------------------------------------------
    # Write API — used by MQTT callback and the mock thread.
    # -------------------------------------------------------------------
    def _ingest(self, topic: str, payload: Any, log_event: Optional[str] = None,
                event_source: str = "system") -> None:
        with self._lock:
            cache = self._streams.get(topic)
            if cache is None:
                return
            cache.value = payload
            cache.last_received = time.time()
            cache.update_count += 1
            if log_event:
                self._events.append(TelemetryEvent(
                    t=cache.last_received, source=event_source, text=log_event,
                ))

    # -------------------------------------------------------------------
    # MQTT mode.
    # -------------------------------------------------------------------
    def start_mqtt(self, broker: str = "localhost", port: int = 1883) -> bool:
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            print("% telemetry_hub: paho-mqtt not installed; cannot start MQTT mode.")
            return False

        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        client.on_connect = self._on_connect
        client.on_message = self._on_message
        try:
            client.connect(broker, port, keepalive=30)
        except Exception as exc:
            print(f"% telemetry_hub: MQTT connect failed ({exc}); not starting.")
            return False
        client.loop_start()
        self._mqtt_client = client
        self._mode = "mqtt"
        print(f"% telemetry_hub: MQTT mode @ {broker}:{port}")
        return True

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        for t in SUBSCRIBED_TOPICS:
            client.subscribe(t)
        with self._lock:
            self._events.append(TelemetryEvent(
                t=time.time(), source="system",
                text=f"connected, subscribed to {len(SUBSCRIBED_TOPICS)} topics",
            ))

    def _on_message(self, client, userdata, msg):
        topic = msg.topic
        raw = msg.payload.decode("utf-8", errors="replace").strip()
        payload = _decode_payload(topic, raw)
        if payload is None:
            return

        log = self._build_event_text(topic, payload)
        src = self._event_source_for(topic)
        self._ingest(topic, payload, log_event=log, event_source=src)

        # Derive synthetic ArUco / landmark cells from vision_pose source
        # (no separate broker topic for those fixes).
        if topic == TOPIC_VISION_POSE and isinstance(payload, dict):
            src_field = payload.get("source")
            if src_field == "aruco":
                self._ingest(TOPIC_VISION_ARUCO, {
                    "id": payload.get("aruco_id", "?"),
                    "range": payload.get("range", 0.0),
                    "bearing": payload.get("bearing", 0.0),
                }, log_event="aruco fix", event_source="aruco")
            elif src_field == "landmark":
                self._ingest(TOPIC_VISION_LANDMARK, {
                    "name": payload.get("landmark", "?"),
                    "confidence": payload.get("conf", 0.0),
                }, log_event=f"landmark {payload.get('landmark', '?')}",
                   event_source="landmark")

    @staticmethod
    def _event_source_for(topic: str) -> str:
        if topic == TOPIC_VISION_ARUCO:
            return "aruco"
        if topic == TOPIC_VISION_LANDMARK:
            return "landmark"
        if topic == TOPIC_VISION_POSE:
            return "vision"
        if topic == TOPIC_KALMAN_STATE:
            return "kalman"
        return "sensor"

    @staticmethod
    def _build_event_text(topic: str, payload: Any) -> Optional[str]:
        # Only log "interesting" events (vision/landmark/aruco fixes), not
        # the firehose of IMU/encoder updates — those are visible in the
        # always-on stream cells.
        if topic == TOPIC_VISION_ARUCO and isinstance(payload, dict):
            mid = payload.get("id")
            return f"aruco ID={mid}" if mid is not None else None
        if topic == TOPIC_VISION_LANDMARK and isinstance(payload, dict):
            return f"landmark {payload.get('name', '?')}"
        if topic == TOPIC_VISION_POSE and isinstance(payload, dict):
            src = payload.get("source")
            if src and src not in ("tape",):  # tape every frame is too noisy
                return f"vision fix ({src})"
        return None

    # -------------------------------------------------------------------
    # Mock mode — for offline UI development.
    # -------------------------------------------------------------------
    def start_mock(self) -> None:
        if self._mock_thread is not None and self._mock_thread.is_alive():
            return
        self._mock_stop.clear()
        self._mock_thread = threading.Thread(target=self._mock_loop, daemon=True)
        self._mock_thread.start()
        self._mode = "mock"
        with self._lock:
            self._events.append(TelemetryEvent(
                t=time.time(), source="system", text="mock mode started",
            ))
        print("% telemetry_hub: mock mode started")

    def stop_mock(self) -> None:
        self._mock_stop.set()

    def _mock_loop(self) -> None:
        """Synthesise sensor + fusion data following a slow circle."""
        t0 = time.time()
        next_aruco = t0 + 5.0
        next_landmark = t0 + 8.0

        while not self._mock_stop.is_set():
            now = time.time()
            t = now - t0

            # Fake trajectory: small circle around (3.5, 2.5).
            cx, cy, R, omg = 3.5, 2.5, 1.2, 0.3
            x   = cx + R * math.cos(omg * t)
            y   = cy + R * math.sin(omg * t)
            yaw = omg * t + math.pi / 2
            vel = R * omg
            pitch = 0.05 * math.sin(0.4 * t)

            # Fast-rate streams: encoder + IMU
            self._ingest(TOPIC_ENC_POSE,
                         {"x": x, "y": y, "yaw": yaw, "tilt": pitch})
            self._ingest(TOPIC_ENC_VEL,
                         {"vL": vel - 0.05, "vR": vel + 0.05})
            self._ingest(TOPIC_IMU_GYRO,
                         {"gx": 0.0, "gy": 0.0, "gz": omg + random.gauss(0, 0.01)})
            self._ingest(TOPIC_IMU_ACC,
                         {"ax": 0.0, "ay": vel * omg, "az": 9.81})
            self._ingest(TOPIC_KALMAN_STATE,
                         {"x": x, "y": y, "z": 0.0, "yaw": yaw,
                          "pitch": pitch, "velocity": vel, "omega": omg})

            # Sporadic: aruco ~every 5 s, landmark ~every 8 s
            if now > next_aruco:
                self._ingest(TOPIC_VISION_ARUCO,
                             {"id": random.choice([10, 11, 12, 25]),
                              "range": round(random.uniform(0.6, 2.0), 2),
                              "bearing": round(random.uniform(-0.5, 0.5), 2)},
                             log_event=None, event_source="aruco")
                self._ingest(TOPIC_VISION_POSE,
                             {"x": x, "y": y, "yaw": yaw, "source": "aruco",
                              "aruco_id": 25, "valid": True})
                self._log_event("aruco", "fix from ID=25")
                next_aruco = now + random.uniform(3.0, 6.0)

            if now > next_landmark:
                name = random.choice(
                    ["start_fork", "roundabout_north_entry", "ball_dispenser"])
                self._ingest(TOPIC_VISION_LANDMARK,
                             {"x": x, "y": y, "yaw": yaw, "name": name,
                              "kind": "fork_y", "confidence": 0.85})
                self._ingest(TOPIC_VISION_POSE,
                             {"x": x, "y": y, "yaw": yaw, "source": "landmark",
                              "landmark": name, "valid": True})
                self._log_event("landmark", f"matched {name}")
                next_landmark = now + random.uniform(7.0, 12.0)

            time.sleep(0.05)   # 20 Hz

    def _log_event(self, source: str, text: str) -> None:
        with self._lock:
            self._events.append(TelemetryEvent(
                t=time.time(), source=source, text=text,
            ))

    # -------------------------------------------------------------------
    def stop(self) -> None:
        self._mock_stop.set()
        if self._mqtt_client is not None:
            try:
                self._mqtt_client.loop_stop()
                self._mqtt_client.disconnect()
            except Exception:
                pass


def _parse_floats(parts):
    out = []
    for p in parts:
        try:
            out.append(float(p))
        except ValueError:
            pass
    return out


def _decode_payload(topic, raw):
    """Convert wire payload to the flat dict shape the UI formatters expect."""
    parts = raw.split()
    if topic == TOPIC_ENC_POSE:
        # "HOST_TS TEENSY_TS x y heading tilt"
        if len(parts) >= 6:
            try:
                return {"x": float(parts[2]), "y": float(parts[3]),
                        "yaw": float(parts[4]), "tilt": float(parts[5])}
            except ValueError:
                return None
        return None
    if topic == TOPIC_ENC_VEL:
        # "HOST_TS TEENSY_TS vL vR"
        if len(parts) >= 4:
            try:
                return {"vL": float(parts[2]), "vR": float(parts[3])}
            except ValueError:
                return None
        return None
    if topic == TOPIC_IMU_GYRO:
        # "HOST_TS gx gy gz"
        if len(parts) >= 4:
            try:
                return {"gx": float(parts[1]), "gy": float(parts[2]),
                        "gz": float(parts[3])}
            except ValueError:
                return None
        return None
    if topic == TOPIC_IMU_ACC:
        # "HOST_TS ax ay az"
        if len(parts) >= 4:
            try:
                return {"ax": float(parts[1]), "ay": float(parts[2]),
                        "az": float(parts[3])}
            except ValueError:
                return None
        return None
    if topic == TOPIC_KALMAN_STATE:
        # JSON published by skalman.publish_state() with nested dicts.
        try:
            obj = json.loads(raw)
        except ValueError:
            return None
        pos = obj.get("position", {})
        ori = obj.get("orientation", {})
        vel = obj.get("velocity", {})
        return {
            "x":        float(pos.get("x", 0.0)),
            "y":        float(pos.get("y", 0.0)),
            "z":        float(pos.get("z", 0.0)),
            "yaw":      float(ori.get("yaw", 0.0)),
            "pitch":    float(ori.get("pitch", 0.0)),
            "velocity": float(vel.get("linear", 0.0)),
            "omega":    float(vel.get("angular", 0.0)),
        }
    if topic == TOPIC_VISION_POSE:
        # "TS x y z yaw pitch source [KEY=VALUE …]"
        if len(parts) < 7:
            return None
        try:
            x = float(parts[1]); y = float(parts[2]); yaw = float(parts[4])
        except ValueError:
            return None
        source = parts[6] if "=" not in parts[6] else "unknown"
        out = {"x": x, "y": y, "yaw": yaw, "source": source}
        for tok in parts[7:]:
            if "=" not in tok:
                continue
            k, _, v = tok.partition("=")
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
        return out
    return None


# Singleton — imported by field_web_app.py
hub = TelemetryHub()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--mock",   action="store_true", help="use synthetic data")
    parser.add_argument("--broker", default="localhost")
    parser.add_argument("--port",   type=int, default=1883)
    args = parser.parse_args()

    if args.mock:
        hub.start_mock()
    else:
        if not hub.start_mqtt(args.broker, args.port):
            print("% MQTT failed; falling back to mock mode for demo.")
            hub.start_mock()

    try:
        while True:
            time.sleep(2.0)
            snap = hub.snapshot()
            ages = {t: round(snap["now"] - s["last_received"], 1)
                    if s["last_received"] else None
                    for t, s in snap["streams"].items()}
            print(f"[{snap['mode']}] ages(s) = {ages}")
    except KeyboardInterrupt:
        hub.stop()
