#!/usr/bin/env python3
"""
=============================================================================
Manual EKF SLAM runner for the Robobot.

Runs SLAM in real-time using live camera stream and IMU motion estimation.
Publishes SLAM pose and landmarks to MQTT topics.

Usage:
    python3 manual_slam.py --host <robot_ip> [--known-landmarks <file>]

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
from simu import imu
from imu_derived_measurements import ImuDerivedMeasurements
from uservice import service

# MQTT topics
SLAM_POSE_TOPIC = "robobot/slam/pose"
SLAM_LANDMARKS_TOPIC = "robobot/slam/landmarks"

# Global state
slam = None
imu_derived = None
last_predict_time = 0.0
integrated_v = 0.0

def predict_motion():
    """Predict SLAM motion using IMU data."""
    global last_predict_time, integrated_v
    if not slam:
        return

    current_time = time.time()
    if last_predict_time == 0:
        last_predict_time = current_time
        return

    dt = current_time - last_predict_time
    if dt < 0.01 or dt > 1.0:  # reasonable dt
        return

    # Integrate acceleration for velocity (simple dead reckoning)
    if imu.accUpdCnt > 0:
        # Assume forward acceleration is acc[0] (need to check orientation)
        # Subtract gravity component
        acc_forward = imu.acc[0] - math.sin(imu_derived.acc_pitch if imu_derived else 0) * 9.82
        integrated_v += acc_forward * dt
        # Decay velocity over time to prevent drift
        integrated_v *= 0.99

    v = integrated_v
    # Get angular velocity from gyro
    omega = imu.yawgRate if imu.yawgUpdCnt > 0 else 0.0

    slam.predict(v, omega, dt)
    last_predict_time = current_time

def process_frame(frame: np.ndarray) -> None:
    """Process a camera frame for SLAM updates."""
    if not slam:
        return

    # Predict motion using IMU
    predict_motion()

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
    service.clientOut.publish(SLAM_LANDMARKS_TOPIC, str(landmarks))

    print(f"% SLAM: pose=({x:.2f},{y:.2f},{yaw:.2f}) lm={len(landmarks)} match={n_match} new={n_new}")

    # Overlay SLAM info on frame
    cv2.putText(frame, f"Pose: ({x:.2f}, {y:.2f}, {yaw:.2f})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, f"Landmarks: {len(landmarks)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

def main():
    global slam

    parser = argparse.ArgumentParser(description="Manual EKF SLAM for Robobot")
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

    # Setup IMU
    imu.setup()

    # Setup IMU derived measurements
    imu_derived = ImuDerivedMeasurements()
    imu_derived.initialize()

    # Setup SLAM
    slam = SEkfSlam()
    slam.setup(known_landmarks=known_lm, max_landmarks=100)

    # Initialize pose (assume starting at origin)
    slam.init_pose(0.0, 0.0, 0.0)

    # Setup feature extractor
    params_path = 'calibration/camera_params.npz'
    feature_extractor.setup(params_path)
    aruco_detector.setup(params_path)

    # Subscribe to IMU MQTT topics
    service.client.subscribe("T0/gyro")
    service.client.subscribe("T0/acc")
    service.client.subscribe("T0/yawg")
    service.client.subscribe("T0/mag")
    service.client.message_callback_add("T0/gyro", imu.decode)
    service.client.message_callback_add("T0/acc", imu.decode)
    service.client.message_callback_add("T0/yawg", imu.decode)
    service.client.message_callback_add("T0/mag", imu.decode)

    # Connect to camera stream
    stream_url = f"http://{args.host}:{args.cam_port}/stream.mjpg"
    cap = cv2.VideoCapture(stream_url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f"% Error: could not open camera stream {stream_url}")
        return

    print(f"% Manual SLAM started on {args.host}")
    print(f"% Camera: {stream_url}")
    print(f"% Known landmarks: {len(known_lm)}")

    try:
        while not service.stop:
            ret, frame = cap.read()
            if ret:
                process_frame(frame)
                # Removed GUI display for headless operation
                # cv2.imshow('SLAM Camera Feed', frame)
                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #     break
            else:
                time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        # cv2.destroyAllWindows()  # No windows to destroy in headless mode
        service.terminate()
        print("% Manual SLAM stopped")

if __name__ == "__main__":
    main()