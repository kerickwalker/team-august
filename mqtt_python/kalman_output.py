#!/usr/bin/env python3

"""Kalman filter state output reader.

Reads and displays the merged sensor estimates from the Kalman filter.
Shows position, velocity, orientation, and angular velocity in real-time.
"""

import sys
import json
import time as t
from datetime import datetime
from collections import deque

try:
    from paho.mqtt import client as mqtt_client
except ImportError:
    print("% ERROR: Missing paho-mqtt package. Install with:")
    print("%   pip install paho-mqtt")
    sys.exit(1)


class KalmanOutputReader:
    def __init__(self, mqtt_host='localhost', mqtt_port=1883):
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.client = None
        self.connected = False
        
        # State tracking
        self.latest_state = None
        self.latest_state_time = None
        self.state_history = deque(maxlen=100)  # Keep last 100 state updates
        
        # Statistics
        self.update_count = 0
        self.start_time = datetime.now()
        self.last_display_time = datetime.now()
        self.display_interval = 0.2  # Display every 200ms
        
        self.setup_mqtt()
    
    def setup_mqtt(self):
        """Connect to MQTT broker."""
        try:
            self.client = mqtt_client.Client("kalman-output-reader")
            self.client.on_connect = self.on_connect
            self.client.on_message = self.on_message
            self.client.on_disconnect = self.on_disconnect
            
            print(f"% Connecting to MQTT broker {self.mqtt_host}:{self.mqtt_port}...")
            self.client.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
            self.client.loop_start()
            
            # Wait for connection
            for i in range(30):
                if self.connected:
                    break
                t.sleep(0.1)
            
            if not self.connected:
                print("% ERROR: Failed to connect to MQTT broker")
                sys.exit(1)
                
        except Exception as e:
            print(f"% ERROR: MQTT connection failed: {e}")
            sys.exit(1)
    
    def on_connect(self, client, userdata, flags, rc):
        """MQTT connection callback."""
        if rc == 0:
            print(f"% Connected to MQTT Broker at {self.mqtt_host}")
            self.connected = True
            # Subscribe to Kalman state topic
            client.subscribe("robobot/kalman/state", qos=1)
            print("% Subscribed to robobot/kalman/state")
        else:
            print(f"% Connection failed with code {rc}")
    
    def on_disconnect(self, client, userdata, rc):
        """MQTT disconnect callback."""
        if rc != 0:
            print(f"% Unexpected disconnection with code {rc}. Reconnecting...")
            self.connected = False
    
    def on_message(self, client, userdata, msg):
        """Handle incoming Kalman state message."""
        try:
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)
            
            self.latest_state = data
            self.latest_state_time = datetime.now()
            self.update_count += 1
            self.state_history.append((self.latest_state_time, data))
            
            # Display at intervals
            now = datetime.now()
            if (now - self.last_display_time).total_seconds() >= self.display_interval:
                self.display_state()
                self.last_display_time = now
                
        except Exception as e:
            print(f"% Error processing message: {e}")
    
    def display_state(self):
        """Display the current Kalman state estimate."""
        if self.latest_state is None:
            return
        
        state = self.latest_state
        elapsed = (self.latest_state_time - self.start_time).total_seconds()
        
        # Build display string
        print("\033[2J\033[H")  # Clear screen (ANSI escape codes)
        print(f"% Kalman Filter State Estimate (t={elapsed:.2f}s, updates={self.update_count})")
        print("% " + "-" * 70)
        
        # Position
        if 'position' in state:
            pos = state['position']
            print(f"% Position:  X={pos.get('x', 0):.4f} m, Y={pos.get('y', 0):.4f} m, Z={pos.get('z', 0):.4f} m")
        
        # Orientation
        if 'orientation' in state:
            ori = state['orientation']
            print(f"% Angle:     Yaw={ori.get('yaw', 0):.4f} rad, Pitch={ori.get('pitch', 0):.4f} rad")
        
        # Velocity
        if 'velocity' in state:
            vel = state['velocity']
            print(f"% Velocity:  Linear={vel.get('linear', 0):.4f} m/s, Angular={vel.get('angular', 0):.4f} rad/s")
        
        # Covariance (uncertainty)
        if 'covariance_diag' in state:
            cov = state['covariance_diag']
            print(f"% Uncertainty (std dev):")
            print(f"%   Position: σx={cov.get('x_std', 0):.6f}, σy={cov.get('y_std', 0):.6f}, σz={cov.get('z_std', 0):.6f}")
            print(f"%   Velocity: σlin={cov.get('velocity_std', 0):.6f}, σang={cov.get('angular_std', 0):.6f}")
            print(f"%   Angle:    σyaw={cov.get('yaw_std', 0):.6f}, σpitch={cov.get('pitch_std', 0):.6f}")
        
        # Raw state if available
        if 'raw_state' in state:
            raw = state['raw_state']
            print(f"% Raw state: {raw}")
        
        print("% " + "-" * 70)
    
    def print_summary(self):
        """Print summary statistics."""
        elapsed = (datetime.now() - self.start_time).total_seconds()
        update_rate = self.update_count / elapsed if elapsed > 0 else 0
        
        print("\n% Summary Statistics:")
        print(f"%   Total updates: {self.update_count}")
        print(f"%   Elapsed time: {elapsed:.2f}s")
        print(f"%   Update rate: {update_rate:.2f} Hz")
        
        if self.state_history:
            print(f"%   History size: {len(self.state_history)} states")
    
    def run(self):
        """Start reading Kalman state."""
        print("% Kalman Output Reader Started")
        print("% Waiting for Kalman state messages...")
        print("% Press Ctrl+C to exit")
        print("%")
        
        try:
            # Keep running
            while True:
                t.sleep(0.1)
        except KeyboardInterrupt:
            print("\n% Interrupted by user")
            self.print_summary()
        finally:
            self.client.loop_stop()
            print("% Kalman Output Reader Stopped")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Read Kalman filter state from MQTT')
    parser.add_argument('-i', '--host', type=str, default='localhost',
                       help='MQTT broker host (default: localhost)')
    parser.add_argument('-p', '--port', type=int, default=1883,
                       help='MQTT broker port (default: 1883)')
    parser.add_argument('-r', '--rate', type=float, default=0.2,
                       help='Display update rate in seconds (default: 0.2)')
    
    args = parser.parse_args()
    
    reader = KalmanOutputReader(mqtt_host=args.host, mqtt_port=args.port)
    reader.display_interval = args.rate
    
    try:
        reader.run()
    except KeyboardInterrupt:
        print("\n% Interrupted by user")
