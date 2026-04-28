#!/usr/bin/env python3
"""
live_slam.py
=============================================================================
Live EKF SLAM runner for the Robobot.

Runs SLAM in real-time using live camera stream and odometry from MQTT.
Publishes SLAM pose and landmarks to MQTT topics.

Usage:
    python3 live_slam.py --host <robot_ip> [--known-landmarks <file>]

Topics published:
    robobot/slam/pose: x, y, yaw
    robobot/slam/landmarks: JSON list of landmarks
=============================================================================
"""

from __future__ import annotations

import argparse
import json
import math
import time
from typing import Optional

import cv2
import numpy as np

from sekf_slam import SEkfSlam
from sfeatures import feature_extractor
from saruco import aruco_detector
from sground import ground
from uservice import service

# MQTT topics
SLAM_POSE_TOPIC = "robobot/slam/pose"
SLAM_LANDMARKS_TOPIC = "robobot/slam/landmarks"

# Global state
slam: Optional[SEkfSlam] = None
last_pose_time = 0.0
last_x, last_y, last_yaw = 0.0, 0.0, 0.0

def on_pose_msg(client, userdata, msg):
    """Handle pose MQTT messages."""
    global last_pose_time, last_x, last_y, last_yaw
    try:
        parts = msg.payload.decode().split()
        if len(parts) >= 4:
            timestamp = float(parts[0])
            x = float(parts[1])
            y = float(parts[2])
            yaw = float(parts[3])

            if last_pose_time > 0:
                dt = timestamp - last_pose_time
                if dt > 0 and dt < 1.0:  # reasonable dt
                    # Compute velocity from pose difference
                    dx = x - last_x
                    dy = y - last_y
                    dyaw = yaw - last_yaw
                    dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi  # wrap

                    v = math.sqrt(dx**2 + dy**2) / dt
                    omega = dyaw / dt

                    if slam:
                        slam.predict(v, omega, dt)

            last_pose_time = timestamp
            last_x, last_y, last_yaw = x, y, yaw

    except Exception as e:
        print(f"% live_slam: pose parse error: {e}")

def on_vel_msg(client, userdata, msg):
    """Handle velocity MQTT messages for more accurate motion."""
    try:
        parts = msg.payload.decode().split()
        if len(parts) >= 3:
            timestamp = float(parts[0])
            vel_left = float(parts[1])
            vel_right = float(parts[2])

            # Compute v and omega from wheel velocities
            v = (vel_left + vel_right) / 2.0
            omega = (vel_right - vel_left) / 0.22  # wheelbase 0.22m

            if slam and last_pose_time > 0:
                dt = timestamp - last_pose_time
                if 0 < dt < 1.0:
                    slam.predict(v, omega, dt)
                last_pose_time = timestamp

    except Exception as e:
        print(f"% live_slam: vel parse error: {e}")

def process_frame(frame: np.ndarray) -> None:
    """Process a camera frame for SLAM updates."""
    if not slam:
        return

    # Extract features
    feats = feature_extractor.extract(frame)

    # Update SLAM with features
    n_match, n_new = slam.update_features(feats)

    # Detect known landmarks (ArUco)
    aruco_detections = aruco_detector.process(frame)

    # Update known landmarks
    for det in aruco_detections:
        if 'id' in det and 'range' in det and 'bearing' in det:
            slam.update_known(int(det['id']), det['range'], det['bearing'])

    # Publish SLAM state
    x, y, yaw, _ = slam.get_pose()
    pose_msg = f"{time.time():.6f} {x:.4f} {y:.4f} {yaw:.4f}"
    service.clientOut.publish(SLAM_POSE_TOPIC, pose_msg)

    # Publish landmarks
    landmarks = slam.get_landmarks()
    service.clientOut.publish(SLAM_LANDMARKS_TOPIC, json.dumps(landmarks))

    print(f"% SLAM: pose=({x:.2f},{y:.2f},{yaw:.2f}) lm={len(landmarks)} match={n_match} new={n_new}")

def main():
    global slam

    parser = argparse.ArgumentParser(description="Live EKF SLAM for Robobot")
    parser.add_argument("--host", default="localhost", help="Robot IP address")
    parser.add_argument("--cam-port", type=int, default=7123, help="MJPEG port")
    parser.add_argument("--known-landmarks", help="JSON file with known landmarks {id: (x,y)}")
    args = parser.parse_args()

    # Load known landmarks
    known_lm = {}
    if args.known_landmarks:
        try:
            with open(args.known_landmarks, 'r') as f:
                known_lm = json.load(f)
        except Exception as e:
            print(f"% Warning: could not load known landmarks: {e}")

    # Setup service
    service.setup(args.host)

    # Setup SLAM
    slam = SEkfSlam()
    slam.setup(known_landmarks=known_lm, max_landmarks=100)

    # Initialize pose (assume starting at origin)
    slam.init_pose(0.0, 0.0, 0.0)

    # Setup feature extractor
    params_path = 'calibration/camera_params.npz'
    feature_extractor.setup(params_path)
    aruco_detector.setup(params_path)

    # Subscribe to MQTT topics
    service.clientIn.subscribe("robobot/drive/T0/pose")
    service.clientIn.subscribe("robobot/drive/T0/vel")
    service.clientIn.message_callback_add("robobot/drive/T0/pose", on_pose_msg)
    service.clientIn.message_callback_add("robobot/drive/T0/vel", on_vel_msg)

    # Connect to camera stream
    stream_url = f"http://{args.host}:{args.cam_port}/stream.mjpg"
    cap = cv2.VideoCapture(stream_url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"% Error: could not open camera stream {stream_url}")
        return

    print(f"% Live SLAM started on {args.host}")
    print(f"% Camera: {stream_url}")
    print(f"% Known landmarks: {len(known_lm)}")

    try:
        while not service.stop:
            ret, frame = cap.read()
            if ret:
                process_frame(frame)
            else:
                time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        service.terminate()
        print("% Live SLAM stopped")

if __name__ == "__main__":
    main()</content>
<parameter name="filePath">/home/local/svn/robobot/mqtt_python/live_slam.py