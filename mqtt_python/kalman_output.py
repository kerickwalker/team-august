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
        self.display_interval = 0.01  # Display every 100ms for real-time feedback

        
        self.setup_mqtt()
    
    def setup_mqtt(self):
        """Connect to MQTT broker."""
        try:
            if hasattr(mqtt_client, "CallbackAPIVersion"):
                # Paho MQTT v2: avoid positional-arg mismatch and keep v1 callbacks.
                self.client = mqtt_client.Client(
                    client_id="kalman-output-reader",
                    callback_api_version=mqtt_client.CallbackAPIVersion.VERSION1,
                )
            else:
                # Paho MQTT v1.
                self.client = mqtt_client.Client(client_id="kalman-output-reader")
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
        """Display the current Kalman state estimate and predicted state."""
        if self.latest_state is None:
            return
        
        state = self.latest_state
        elapsed = (self.latest_state_time - self.start_time).total_seconds()
        
        # Build display string
        print("\033[2J\033[H")  # Clear screen (ANSI escape codes)
        print(f"% Kalman Filter State (t={elapsed:.2f}s, updates={self.update_count})")
        print("% " + "-" * 70)
        
        #=== ESTIMATED STATE ===
        print("% [ESTIMATED STATE] (after measurement update)")
        # Handle format from uservice.py (with "x" key containing state vector)
        if 'x' in state:
            x_state = state['x']
            print(f"% Position:  X={x_state.get('x', 0):.4f} m, Y={x_state.get('y', 0):.4f} m, Z={x_state.get('z', 0):.4f} m")
            print(f"% Velocity:  Linear={x_state.get('velocity', 0):.4f} m/s, Angular={x_state.get('angular_velocity', 0):.4f} rad/s")
            print(f"% Angle:     Yaw={x_state.get('yaw', 0):.4f} rad, Pitch={x_state.get('pitch', 0):.4f} rad")
        # Handle format from skalman.py (structured format)
        elif 'position' in state:
            pos = state['position']
            print(f"% Position:  X={pos.get('x', 0):.4f} m, Y={pos.get('y', 0):.4f} m, Z={pos.get('z', 0):.4f} m")
            
            if 'velocity' in state:
                vel = state['velocity']
                print(f"% Velocity:  Linear={vel.get('linear', 0):.4f} m/s, Angular={vel.get('angular', 0):.4f} rad/s")
            
            if 'orientation' in state:
                ori = state['orientation']
                print(f"% Angle:     Yaw={ori.get('yaw', 0):.4f} rad, Pitch={ori.get('pitch', 0):.4f} rad")
            
            if 'covariance_diag' in state:
                cov = state['covariance_diag']
                print(f"% Uncertainty (std dev):")
                print(f"%   Position: σx={cov.get('x_std', 0):.6f}, σy={cov.get('y_std', 0):.6f}, σz={cov.get('z_std', 0):.6f}")
                print(f"%   Velocity: σlin={cov.get('velocity_std', 0):.6f}, σang={cov.get('angular_std', 0):.6f}")
                print(f"%   Angle:    σyaw={cov.get('yaw_std', 0):.6f}, σpitch={cov.get('pitch_std', 0):.6f}")
        else:
            print(f"% WARNING: Unknown state format. Keys: {list(state.keys())}")
        
        #=== PREDICTED STATE (MODEL OUTPUT) ===
        if 'x_pred' in state:
            print("% ")
            print("% [PREDICTED STATE] (motion model, before measurement correction)")
            x_pred = state['x_pred']
            print(f"% Position:  X={x_pred.get('x', 0):.4f} m, Y={x_pred.get('y', 0):.4f} m, Z={x_pred.get('z', 0):.4f} m")
            print(f"% Velocity:  Linear={x_pred.get('velocity', 0):.4f} m/s, Angular={x_pred.get('angular_velocity', 0):.4f} rad/s")
            print(f"% Angle:     Yaw={x_pred.get('yaw', 0):.4f} rad, Pitch={x_pred.get('pitch', 0):.4f} rad")
            
            # Show prediction error (innovation)
            if 'x' in state:
                x_est = state['x']
                print("% ")
                print("% [INNOVATION] (measurement correction applied)")
                print(f"% Position delta:  ΔX={x_est.get('x', 0) - x_pred.get('x', 0):+.6f} m, ΔY={x_est.get('y', 0) - x_pred.get('y', 0):+.6f} m")
                print(f"% Velocity delta:  Δlin={x_est.get('velocity', 0) - x_pred.get('velocity', 0):+.6f} m/s")
                print(f"% Angle delta:     ΔYaw={x_est.get('yaw', 0) - x_pred.get('yaw', 0):+.6f} rad")
        
        # Show measurements if available
        if 'measurements' in state and state['measurements']:
            meas = state['measurements']
            print("% ")
            print("% Raw Measurements (all 17 channels):")
            
            # Define measurement names for grouped display
            meas_order = [
                "pose_x_enc", "pose_y_enc", "pose_z_enc",
                "vel_enc", "omega_gyro", "yaw_enc", "omega_enc",
                "pose_x_vision", "pose_y_vision", "pose_z_vision", "yaw_vision",
                "vel_acc_integrated", "yaw_imu_integrated", 
                "yaw_mag_derived", "pitch_acc_derived", "pitch_mag_derived", "pitch_vision"
            ]
            
            # Encoder measurements
            print("%   Encoder (odometry):")
            if 'pose_x_enc' in meas:
                print(f"%     X={meas.get('pose_x_enc', 0):.4f} m, Y={meas.get('pose_y_enc', 0):.4f} m, Z={meas.get('pose_z_enc', 0):.4f} m")
            if 'vel_enc' in meas:
                print(f"%     Velocity={meas.get('vel_enc', 0):.4f} m/s, Omega={meas.get('omega_enc', 0):.4f} rad/s")
            if 'yaw_enc' in meas:
                print(f"%     Yaw={meas.get('yaw_enc', 0):.4f} rad")
            
            # IMU measurements
            print("%   IMU (gyro/acc):")
            if 'omega_gyro' in meas:
                print(f"%     Gyro ω={meas.get('omega_gyro', 0):.4f} rad/s")
            if 'vel_acc_integrated' in meas:
                print(f"%     Acc integrated v={meas.get('vel_acc_integrated', 0):.4f} m/s")
            if 'yaw_imu_integrated' in meas:
                print(f"%     Yaw integrated={meas.get('yaw_imu_integrated', 0):.4f} rad")
            if 'pitch_acc_derived' in meas:
                print(f"%     Pitch (from acc)={meas.get('pitch_acc_derived', 0):.4f} rad")
            
            # Vision measurements
            print("%   Vision:")
            if 'pose_x_vision' in meas:
                print(f"%     X={meas.get('pose_x_vision', 0):.4f} m, Y={meas.get('pose_y_vision', 0):.4f} m, Z={meas.get('pose_z_vision', 0):.4f} m")
                print(f"%     Yaw={meas.get('yaw_vision', 0):.4f} rad, Pitch={meas.get('pitch_vision', 0):.4f} rad")
            
            # Magnetometer measurements
            print("%   Magnetometer (heading):")
            if 'yaw_mag_derived' in meas:
                print(f"%     Yaw={meas.get('yaw_mag_derived', 0):.4f} rad")
                print(f"%     Pitch={meas.get('pitch_mag_derived', 0):.4f} rad")
            
            # Show all raw values if available - dump them all
            print("% ")
            print("%   All raw values (dict):")
            for name in meas_order:
                if name in meas:
                    val = meas.get(name, 0)
                    print(f"%     {name:25s} = {val:10.6f}")
        
        # Show metadata if present
        if 'source' in state:
            print(f"% ")
            print(f"% Source: {state.get('source')}")
        
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
