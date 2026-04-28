#!/usr/bin/env python3
"""
=============================================================================
SLAM Map Visualizer for Robobot

Subscribes to SLAM MQTT topics and displays a real-time 2D map of landmarks
and robot pose. Shows the map being built as you move the robot.

Usage:
    python3 slam_map_visualizer.py [--host <robot_ip>]

Topics subscribed:
    robobot/slam/pose: x, y, yaw
    robobot/slam/landmarks: JSON list of landmarks
=============================================================================
"""

import json
import time
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import paho.mqtt.client as mqtt
import threading
import numpy as np

# MQTT topics
SLAM_POSE_TOPIC = "robobot/slam/pose"
SLAM_LANDMARKS_TOPIC = "robobot/slam/landmarks"

# Global state
current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
landmarks = []  # List of [x, y] positions
landmarks_lock = threading.Lock()

def on_connect(client, userdata, flags, rc):
    """MQTT connection callback."""
    print(f"% Connected to MQTT broker with result code {rc}")
    client.subscribe(SLAM_POSE_TOPIC)
    client.subscribe(SLAM_LANDMARKS_TOPIC)

def on_message(client, userdata, msg):
    """MQTT message callback."""
    global current_pose, landmarks
    try:
        payload = msg.payload.decode('utf-8')
        if msg.topic == SLAM_POSE_TOPIC:
            # Parse pose: "x,y,yaw"
            parts = payload.split(',')
            if len(parts) == 3:
                with landmarks_lock:
                    current_pose['x'] = float(parts[0])
                    current_pose['y'] = float(parts[1])
                    current_pose['yaw'] = float(parts[2])
        elif msg.topic == SLAM_LANDMARKS_TOPIC:
            # Parse landmarks: JSON list of [id, x, y] or [x, y]
            lm_data = json.loads(payload)
            with landmarks_lock:
                landmarks = lm_data
    except Exception as e:
        print(f"% Error parsing MQTT message: {e}")

def update_plot(frame):
    """Update the matplotlib plot with current SLAM data."""
    with landmarks_lock:
        pose = current_pose.copy()
        lms = landmarks.copy()

    # Clear and redraw
    plt.clf()
    ax = plt.gca()

    # Set up plot
    ax.set_xlim(-2, 5)  # Adjust based on your field size
    ax.set_ylim(-2, 5)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('SLAM Map - Real-time')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # Plot landmarks
    if lms:
        lm_x = [lm[1] if len(lm) > 2 else lm[0] for lm in lms]
        lm_y = [lm[2] if len(lm) > 2 else lm[1] for lm in lms]
        ax.scatter(lm_x, lm_y, c='red', s=50, marker='s', label='Landmarks')
        ax.legend()

    # Plot robot pose
    robot_x, robot_y, robot_yaw = pose['x'], pose['y'], pose['yaw']

    # Robot as arrow
    arrow_length = 0.2
    dx = arrow_length * np.cos(robot_yaw)
    dy = arrow_length * np.sin(robot_yaw)

    ax.arrow(robot_x, robot_y, dx, dy,
             head_width=0.1, head_length=0.1, fc='blue', ec='blue',
             label='Robot Pose')

    # Robot position dot
    ax.scatter([robot_x], [robot_y], c='blue', s=100, marker='o')

    # Info text
    info = f"Pose: ({robot_x:.2f}, {robot_y:.2f}, {np.degrees(robot_yaw):.1f}°)\nLandmarks: {len(lms)}"
    ax.text(0.02, 0.98, info, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

def main():
    parser = argparse.ArgumentParser(description='SLAM Map Visualizer')
    parser.add_argument('--host', default='localhost', help='MQTT broker host')
    args = parser.parse_args()

    # MQTT setup
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(args.host, 1883, 60)
    except Exception as e:
        print(f"% Error connecting to MQTT broker at {args.host}: {e}")
        return

    # Start MQTT loop in background
    client.loop_start()

    # Set up matplotlib
    plt.style.use('default')
    fig, ax = plt.subplots(figsize=(10, 8))

    # Animation
    ani = FuncAnimation(fig, update_plot, interval=500)  # Update every 500ms

    print("% SLAM Map Visualizer started. Close the plot window to exit.")
    print(f"% Connected to MQTT at {args.host}")

    try:
        plt.show()
    except KeyboardInterrupt:
        print("% Exiting...")
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()