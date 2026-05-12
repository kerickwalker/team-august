#!/usr/bin/env python3

"""Log selected MQTT topics to console and a JSONL file.

Default topics:
  - robobot/kalman/state
  - robobot/drive/T0/pose
"""

import argparse
import json
import signal
import sys
import time as t
from datetime import datetime
from pathlib import Path

try:
    from paho.mqtt import client as mqtt_client
except ImportError:
    print("% ERROR: Missing paho-mqtt package. Install with:")
    print("%   pip install paho-mqtt")
    sys.exit(1)


DEFAULT_TOPICS = [
    "robobot/kalman/state",
    "robobot/drive/T0/pose",
]


class MQTTTopicLogger:
    def __init__(self, host: str, port: int, topics, output_path: Path) -> None:
        self.host = host
        self.port = port
        self.topics = list(topics)
        self.output_path = Path(output_path)
        self.client = None
        self.connected = False
        self.message_count = 0
        self.log_file = None
        self.running = True
        self._setup_mqtt()

    def _setup_mqtt(self) -> None:
        try:
            if hasattr(mqtt_client, "CallbackAPIVersion"):
                self.client = mqtt_client.Client(
                    client_id="mqtt-topic-logger",
                    callback_api_version=mqtt_client.CallbackAPIVersion.VERSION1,
                )
            else:
                self.client = mqtt_client.Client(client_id="mqtt-topic-logger")
        except TypeError:
            self.client = mqtt_client.Client(client_id="mqtt-topic-logger")

        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            print(f"% Connected to MQTT broker at {self.host}:{self.port}")
            for topic in self.topics:
                client.subscribe(topic, qos=1)
                print(f"% Subscribed to {topic}")
        else:
            print(f"% ERROR: MQTT connection failed with code {rc}")

    def on_disconnect(self, client, userdata, rc):
        self.connected = False
        if rc != 0 and self.running:
            print(f"% WARNING: Unexpected MQTT disconnect with code {rc}")

    def on_message(self, client, userdata, msg):
        payload = msg.payload.decode("utf-8", errors="replace").strip()
        received_at = datetime.now()
        entry = {
            "received_at": received_at.isoformat(),
            "topic": msg.topic,
            "payload": payload,
            "parsed": self._parse_payload(msg.topic, payload),
        }

        self.message_count += 1
        self.log_file.write(json.dumps(entry) + "\n")
        self.log_file.flush()
        self._print_entry(entry)

    def _parse_payload(self, topic: str, payload: str):
        if topic == "robobot/kalman/state":
            try:
                return json.loads(payload)
            except json.JSONDecodeError:
                return {"raw_text": payload}

        if topic == "robobot/drive/T0/pose":
            parts = payload.split()
            if len(parts) >= 6:
                try:
                    return {
                        "unix_time": float(parts[0]),
                        "teensy_time": float(parts[1]),
                        "x": float(parts[2]),
                        "y": float(parts[3]),
                        "heading": float(parts[4]),
                        "tilt": float(parts[5]),
                    }
                except ValueError:
                    return {"raw_text": payload}

        return {"raw_text": payload}

    def _print_entry(self, entry) -> None:
        topic = entry["topic"]
        parsed = entry["parsed"]

        if topic == "robobot/kalman/state" and "position" in parsed:
            pos = parsed.get("position", {})
            vel = parsed.get("velocity", {})
            ori = parsed.get("orientation", {})
            print(
                f"[{entry['received_at']}] kalman "
                f"x={pos.get('x', 0.0):.3f} y={pos.get('y', 0.0):.3f} "
                f"v={vel.get('linear', 0.0):.3f} yaw={ori.get('yaw', 0.0):.3f}"
            )
            return

        if topic == "robobot/drive/T0/pose" and "x" in parsed:
            print(
                f"[{entry['received_at']}] pose "
                f"x={parsed['x']:.3f} y={parsed['y']:.3f} "
                f"heading={parsed['heading']:.3f} tilt={parsed['tilt']:.3f}"
            )
            return

        print(f"[{entry['received_at']}] {topic} {entry['payload']}")

    def run(self) -> int:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_file = self.output_path.open("a", encoding="utf-8")
        print(f"% Logging to {self.output_path}")

        self.client.connect(self.host, self.port, keepalive=60)
        self.client.loop_start()

        t0 = t.time()
        while not self.connected and (t.time() - t0) < 5.0:
            t.sleep(0.05)

        if not self.connected:
            print("% ERROR: Timed out waiting for MQTT connection")
            return 1

        try:
            while self.running:
                t.sleep(0.2)
        finally:
            self.shutdown()
        return 0

    def shutdown(self) -> None:
        if not self.running:
            return
        self.running = False
        print(f"% Stopping logger after {self.message_count} messages")
        try:
            if self.client is not None:
                self.client.loop_stop()
                self.client.disconnect()
        finally:
            if self.log_file is not None:
                self.log_file.close()
                self.log_file = None


def default_output_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("mqtt_python") / "logs" / f"mqtt_log_{stamp}.jsonl"


def main() -> int:
    parser = argparse.ArgumentParser(description="Log selected MQTT topics")
    parser.add_argument("--host", default="localhost", help="MQTT broker host")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    parser.add_argument(
        "--topic",
        action="append",
        dest="topics",
        help="Topic to subscribe to; may be provided multiple times",
    )
    parser.add_argument(
        "--output",
        default=str(default_output_path()),
        help="JSONL output file path",
    )
    args = parser.parse_args()

    topics = args.topics if args.topics else list(DEFAULT_TOPICS)
    logger = MQTTTopicLogger(args.host, args.port, topics, Path(args.output))

    def handle_signal(sig, frame):
        logger.shutdown()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    return logger.run()


if __name__ == "__main__":
    raise SystemExit(main())
