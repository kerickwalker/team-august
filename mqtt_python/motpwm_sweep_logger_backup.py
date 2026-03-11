#!/usr/bin/env python3
"""Sweep motor input commands and log measured outputs to CSV.

This script sends RC commands via MQTT (`robobot/cmd/ti`) and records
corresponding encoder, wheel velocity, and battery voltage from
`robobot/drive/T0/*` topics.

Output:
- One summary CSV with one row per (left_pwm, right_pwm) test.
"""

import argparse
import csv
import itertools
import math
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

from paho.mqtt import client as mqtt_client


@dataclass
class Sample:
    t_unix: float
    vel_left: float
    vel_right: float
    enc_left: float
    enc_right: float
    bat_v: float


def parse_csv_floats(text: str) -> List[float]:
    vals = []
    for part in text.split(","):
        s = part.strip()
        if not s:
            continue
        vals.append(float(s))
    return vals


class SweepLogger:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.client = self._make_client("motpwm-sweep-logger")
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

        self.lock = threading.Lock()
        self.connected = False

        self.last_bat_v = float("nan")
        self.last_enc_left = float("nan")
        self.last_enc_right = float("nan")

        self.test_active = False
        self.samples: List[Sample] = []

    @staticmethod
    def _make_client(client_id: str):
        # Support both older and newer paho-mqtt API variants.
        try:
            return mqtt_client.Client(client_id)
        except Exception:
            return mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2, client_id=client_id)

    def _on_connect(self, client, _userdata, _flags, rc, _properties=None):
        if rc == 0:
            with self.lock:
                self.connected = True
            client.subscribe("robobot/drive/T0/hbt")
            client.subscribe("robobot/drive/T0/vel")
            client.subscribe("robobot/drive/T0/enc")
        else:
            print(f"[mqtt] connect failed rc={rc}")

    def _on_message(self, _client, _userdata, msg):
        payload = msg.payload.decode("utf-8", errors="ignore").strip()
        if not payload:
            return

        topic = msg.topic
        parts = payload.split()

        with self.lock:
            if topic.endswith("/hbt"):
                # Format observed:
                # unix_time teensy_time idx rev battery state hw load supply_current batLowCnt
                if len(parts) >= 5:
                    try:
                        self.last_bat_v = float(parts[4])
                    except ValueError:
                        pass
                return

            if topic.endswith("/enc"):
                # Format observed:
                # unix_time enc_left enc_right
                if len(parts) >= 3:
                    try:
                        self.last_enc_left = float(parts[1])
                        self.last_enc_right = float(parts[2])
                    except ValueError:
                        pass
                return

            if topic.endswith("/vel"):
                # Format observed:
                # unix_time teensy_time vel_left vel_right ...
                if len(parts) >= 4:
                    try:
                        t_unix = float(parts[0])
                        vel_left = float(parts[2])
                        vel_right = float(parts[3])
                    except ValueError:
                        return

                    if self.test_active:
                        self.samples.append(
                            Sample(
                                t_unix=t_unix,
                                vel_left=vel_left,
                                vel_right=vel_right,
                                enc_left=self.last_enc_left,
                                enc_right=self.last_enc_right,
                                bat_v=self.last_bat_v,
                            )
                        )
                return

    def connect(self) -> None:
        self.client.connect(self.host, self.port, keepalive=60)
        self.client.loop_start()

        t0 = time.time()
        while time.time() - t0 < 5.0:
            with self.lock:
                if self.connected:
                    return
            time.sleep(0.05)
        raise RuntimeError("MQTT connection timeout")

    def close(self) -> None:
        try:
            self.client.loop_stop()
        finally:
            self.client.disconnect()

    def send_rc(self, linvel: float, turnrate: float) -> None:
        payload = f"rc {linvel:.4f} {turnrate:.4f}"
        self.client.publish("robobot/cmd/ti", payload)

    def run_test(self, duration_s: float) -> List[Sample]:
        with self.lock:
            self.samples = []
            self.test_active = True

        t0 = time.time()
        while time.time() - t0 < duration_s:
            time.sleep(0.01)

        with self.lock:
            self.test_active = False
            return list(self.samples)


def mean(values: List[float]) -> float:
    if not values:
        return float("nan")
    return sum(values) / len(values)


def finite(values: List[float]) -> List[float]:
    return [v for v in values if math.isfinite(v)]


def main() -> int:
    parser = argparse.ArgumentParser(description="Sweep motor input and log outputs")
    parser.add_argument("--host", default="localhost", help="MQTT host")
    parser.add_argument("--port", type=int, default=1883, help="MQTT port")
    parser.add_argument("--left-values", default="-2000,-1000,0,1000,2000", help="Comma-separated left PWM-like values")
    parser.add_argument("--right-values", default="-2000,-1000,0,1000,2000", help="Comma-separated right PWM-like values")
    parser.add_argument("--duration", type=float, default=1.5, help="Command duration per test [s]")
    parser.add_argument("--settle", type=float, default=0.5, help="Pause after stop between tests [s]")
    parser.add_argument("--measure-tail", type=float, default=0.6, help="Use last N seconds for summary averages")
    parser.add_argument("--max-pwm", type=float, default=4096.0, help="Input full-scale value")
    parser.add_argument("--max-linvel", type=float, default=0.35, help="Linear velocity at full-scale input [m/s]")
    parser.add_argument("--wheelbase", type=float, default=0.22, help="Wheel base [m]")
    parser.add_argument("--output", default="motpwm_sweep_summary.csv", help="Output CSV path")
    args = parser.parse_args()

    left_values = parse_csv_floats(args.left_values)
    right_values = parse_csv_floats(args.right_values)
    if not left_values or not right_values:
        print("No sweep values provided")
        return 1

    logger = SweepLogger(args.host, args.port)
    logger.connect()

    # Stop first to start from a known state.
    logger.send_rc(0.0, 0.0)
    time.sleep(0.2)

    rows = []
    test_id = 0

    try:
        for left_pwm, right_pwm in itertools.product(left_values, right_values):
            left_pwm = max(-args.max_pwm, min(args.max_pwm, left_pwm))
            right_pwm = max(-args.max_pwm, min(args.max_pwm, right_pwm))

            left_v = (left_pwm / args.max_pwm) * args.max_linvel
            right_v = (right_pwm / args.max_pwm) * args.max_linvel
            linvel = 0.5 * (left_v + right_v)
            turnrate = (right_v - left_v) / args.wheelbase

            print(
                f"[test {test_id}] left={left_pwm:.0f}, right={right_pwm:.0f}, "
                f"rc=({linvel:.3f},{turnrate:.3f}), duration={args.duration:.2f}s"
            )

            logger.send_rc(linvel, turnrate)
            samples = logger.run_test(args.duration)

            # Stop between tests.
            logger.send_rc(0.0, 0.0)
            time.sleep(args.settle)

            if not samples:
                rows.append(
                    {
                        "test_id": test_id,
                        "left_pwm": left_pwm,
                        "right_pwm": right_pwm,
                        "linvel_cmd": linvel,
                        "turnrate_cmd": turnrate,
                        "sample_count": 0,
                        "vel_left_mean": float("nan"),
                        "vel_right_mean": float("nan"),
                        "vel_mean": float("nan"),
                        "enc_left_start": float("nan"),
                        "enc_left_end": float("nan"),
                        "enc_left_delta": float("nan"),
                        "enc_right_start": float("nan"),
                        "enc_right_end": float("nan"),
                        "enc_right_delta": float("nan"),
                        "bat_v_mean": float("nan"),
                        "bat_v_min": float("nan"),
                        "bat_v_max": float("nan"),
                        "t_start": float("nan"),
                        "t_end": float("nan"),
                    }
                )
                test_id += 1
                continue

            t_end = samples[-1].t_unix
            t_start = samples[0].t_unix
            t_cut = t_end - max(0.0, args.measure_tail)
            tail = [s for s in samples if s.t_unix >= t_cut]
            if not tail:
                tail = samples

            vls = [s.vel_left for s in tail]
            vrs = [s.vel_right for s in tail]
            bats = finite([s.bat_v for s in tail])
            enc_l = finite([s.enc_left for s in tail])
            enc_r = finite([s.enc_right for s in tail])

            row = {
                "test_id": test_id,
                "left_pwm": left_pwm,
                "right_pwm": right_pwm,
                "linvel_cmd": linvel,
                "turnrate_cmd": turnrate,
                "sample_count": len(tail),
                "vel_left_mean": mean(vls),
                "vel_right_mean": mean(vrs),
                "vel_mean": mean([(a + b) * 0.5 for a, b in zip(vls, vrs)]),
                "enc_left_start": enc_l[0] if enc_l else float("nan"),
                "enc_left_end": enc_l[-1] if enc_l else float("nan"),
                "enc_left_delta": (enc_l[-1] - enc_l[0]) if len(enc_l) >= 2 else float("nan"),
                "enc_right_start": enc_r[0] if enc_r else float("nan"),
                "enc_right_end": enc_r[-1] if enc_r else float("nan"),
                "enc_right_delta": (enc_r[-1] - enc_r[0]) if len(enc_r) >= 2 else float("nan"),
                "bat_v_mean": mean(bats),
                "bat_v_min": min(bats) if bats else float("nan"),
                "bat_v_max": max(bats) if bats else float("nan"),
                "t_start": t_start,
                "t_end": t_end,
            }
            rows.append(row)
            test_id += 1
    finally:
        logger.send_rc(0.0, 0.0)
        logger.close()

    fieldnames = [
        "test_id",
        "left_pwm",
        "right_pwm",
        "linvel_cmd",
        "turnrate_cmd",
        "sample_count",
        "vel_left_mean",
        "vel_right_mean",
        "vel_mean",
        "enc_left_start",
        "enc_left_end",
        "enc_left_delta",
        "enc_right_start",
        "enc_right_end",
        "enc_right_delta",
        "bat_v_mean",
        "bat_v_min",
        "bat_v_max",
        "t_start",
        "t_end",
    ]

    with open(args.output, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Wrote {len(rows)} test rows to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
