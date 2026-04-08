#!/usr/bin/env python3

"""Teleoperation input client with prompt-based velocity commands.

Publishes velocity input vectors [linear_velocity, angular_velocity] to MQTT.
Supports both velocity mode (linear/angular) and motor mode (left/right velocities).

These values are consumed by:
  1. Kalman filter (skalman.py) for model-based predictions
  2. Motor control (via teensy_interface) for actual robot movement
  3. uservice.py to send direct RC motor commands

Usage:
    python3 teleop_input.py [options]
    
    Velocity mode (default): enter linear and angular velocities
    Example: 0.5 0.2
    (sends linear_vel=0.5 m/s, angular_vel=0.2 rad/s)
    
    Motor mode (-m): enter left and right motor velocities
    Example: 1.0 1.0
    (sends left=1.0 m/s, right=1.0 m/s)
    
    Type 'help' for commands
    Type 'q' or 'quit' to exit
"""

import sys
import os
import json
import time as t
import threading
from datetime import datetime

try:
    from paho.mqtt import client as mqtt_client
except ImportError:
    print("% ERROR: Missing required packages. Install with:")
    print("%   pip install paho-mqtt")
    sys.exit(1)


class PromptTeleopInput:
    """Teleoperation via prompt-based velocity input."""
    
    def __init__(self, mqtt_host='localhost', mqtt_port=1883, motor_mode=False):
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.client = None
        self.connected = False
        self.motor_mode = motor_mode  # If True, input is left/right motor velocities
        
        # Input limits
        self.max_linear_vel = 1.0   # m/s
        self.max_angular_vel = 2.0  # rad/s
        self.max_motor_vel = 1.0    # m/s for direct motor mode
        self.last_publish_time = datetime.now()
        
        # Continuous resend to keep motors active
        self.last_command = None
        self.last_motor_vel_left = 0.0
        self.last_motor_vel_right = 0.0
        self.resend_active = False
        self.resend_thread = None
        self.resend_interval = 0.1  # Resend every 100ms
        
        self.setup_mqtt()
    
    def setup_mqtt(self):
        """Connect to MQTT broker."""
        try:
            if hasattr(mqtt_client, "CallbackAPIVersion"):
                # Paho MQTT v2: avoid positional-arg mismatch and keep v1 callbacks.
                self.client = mqtt_client.Client(
                    client_id="teleop-input-prompt",
                    callback_api_version=mqtt_client.CallbackAPIVersion.VERSION1,
                )
            else:
                # Paho MQTT v1.
                self.client = mqtt_client.Client(client_id="teleop-input-prompt")
            self.client.on_connect = self.on_connect
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
            print(f"% Connected to MQTT Broker at {self.mqtt_host}:{self.mqtt_port}")
            self.connected = True
        else:
            print(f"% Connection failed with code {rc}")
            self.connected = False
    
    def on_disconnect(self, client, userdata, rc):
        """MQTT disconnect callback."""
        if rc != 0:
            print(f"% Unexpected disconnection with code {rc}")
            self.connected = False
    
    def publish_velocity_input(self, val1, val2):
        """Publish velocity input vector to MQTT.
        
        Args:
            val1 (float): Left motor velocity (motor mode) or linear velocity
            val2 (float): Right motor velocity (motor mode) or angular velocity
        
        Returns:
            tuple: (left_vel, right_vel) motor velocities, or None on error
        """
        if not self.connected:
            print("% ERROR: Not connected to MQTT broker")
            return None
        
        if self.motor_mode:
            # Direct motor velocity mode
            left_vel = max(-self.max_motor_vel, min(self.max_motor_vel, float(val1)))
            right_vel = max(-self.max_motor_vel, min(self.max_motor_vel, float(val2)))
            # Convert to linear/angular for Kalman filter
            linear_vel = (left_vel + right_vel) / 2.0
            angular_vel = (right_vel - left_vel) / 0.23  # wheelbase = 0.23m
        else:
            # Linear/angular mode
            linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, float(val1)))
            angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, float(val2)))
            left_vel = linear_vel - (0.23/2.0) * angular_vel
            right_vel = linear_vel + (0.23/2.0) * angular_vel
        
        # Create input vector message
        cmd = {
            "linear_velocity": linear_vel,
            "angular_velocity": angular_vel,
            "v_left": left_vel,
            "v_right": right_vel,
            "timestamp": datetime.now().isoformat()
        }
        
        # Publish to teleoperation channel
        topic = "robobot/teleop/cmd"
        payload = json.dumps(cmd)
        
        try:
            self.client.publish(topic, payload, qos=1)
            if self.motor_mode:
                print(f"✓ Published motor: L={left_vel:.3f}, R={right_vel:.3f} m/s")
            else:
                print(f"✓ Published: linear={linear_vel:.3f}, angular={angular_vel:.3f}")
            
            # Store for continuous resend
            self.last_command = payload
            self.last_motor_vel_left = left_vel
            self.last_motor_vel_right = right_vel
            
            return (left_vel, right_vel)
        except Exception as e:
            print(f"% ERROR: Failed to publish: {e}")
            return None
    
    def _resend_loop(self):
        """Background thread: continuously resend last command to keep motors active."""
        while self.resend_active:
            if self.last_command is not None:
                try:
                    # Resend with updated timestamp
                    cmd_dict = json.loads(self.last_command)
                    cmd_dict["timestamp"] = datetime.now().isoformat()
                    updated_payload = json.dumps(cmd_dict)
                    
                    self.client.publish("robobot/teleop/cmd", updated_payload, qos=1)
                    # Quietly resend without printing each time
                except Exception as e:
                    print(f"% WARNING: Resend failed: {e}")
            
            t.sleep(self.resend_interval)
    
    def start_resend(self):
        """Start background thread that continuously resends last command."""
        if self.resend_active or self.resend_thread is not None:
            return  # Already running
        
        self.resend_active = True
        self.resend_thread = threading.Thread(target=self._resend_loop, daemon=True)
        self.resend_thread.start()
        print("% Continuous RC resend started (100ms intervals)")
    
    def stop_resend(self):
        """Stop background resend thread."""
        self.resend_active = False
        if self.resend_thread is not None:
            self.resend_thread.join(timeout=1.0)
            self.resend_thread = None
    
    def print_help(self):
        """Print help message."""
        print("\n% Teleoperation Input Commands:")
        print("% ==============================")
        if self.motor_mode:
            print("% MOTOR MODE: Direct left and right motor velocities")
            print("%   <left_vel> <right_vel>")
            print("% ")
            print("% Examples:")
            print("%   1.0 1.0       -> Both motors at 1.0 m/s (forward)")
            print("%   -1.0 -1.0     -> Both motors at -1.0 m/s (backward)")
            print("%   0.5 1.0       -> Left 0.5, Right 1.0 (turn right)")
            print("%   1.0 0.5       -> Left 1.0, Right 0.5 (turn left)")
            print("%   0.0 0.0       -> STOP")
            print("% ")
            print(f"% Limits: motor vel [{-self.max_motor_vel}, {self.max_motor_vel}] m/s")
        else:
            print("% VELOCITY MODE: Linear and angular velocities")
            print("%   <linear_vel> <angular_vel>")
            print("% ")
            print("% Examples:")
            print("%   0.5 0.0      -> Forward at 0.5 m/s")
            print("%   -0.5 0.0     -> Backward at 0.5 m/s")
            print("%   0.0 0.5      -> Turn left at 0.5 rad/s")
            print("%   0.0 -0.5     -> Turn right at 0.5 rad/s")
            print("%   0.3 0.3      -> Forward-left")
            print("%   0.3 -0.3     -> Forward-right")
            print("%   0.0 0.0      -> STOP")
            print("% ")
            print(f"% Limits: linear [{-self.max_linear_vel}, {self.max_linear_vel}] m/s")
            print(f"%         angular [{-self.max_angular_vel}, {self.max_angular_vel}] rad/s")
        print("% ")
        print("% Commands:")
        print("%   help, h, ?   -> Show this help")
        print("%   q, quit      -> Exit")
        print("% ==============================\n")
    
    def run(self):
        """Start teleoperation input loop."""
        mode_str = "Motor (L/R)" if self.motor_mode else "Velocity (lin/ang)"
        print(f"\n% ===== Teleoperation Input ({mode_str}) =====")
        print("% Connected and ready for commands")
        print("% NOTE: Commands are automatically resent every 100ms to keep motors active")
        self.print_help()
        
        self.start_resend()  # Start continuous resend thread
        
        try:
            while True:
                try:
                    prompt = "[L/R m/s]" if self.motor_mode else "[lin ang]"
                    user_input = input(f"Enter velocity {prompt} or command: ").strip()
                    
                    if not user_input:
                        continue
                    
                    # Handle commands
                    if user_input.lower() in ['help', 'h', '?']:
                        self.print_help()
                        continue
                    
                    if user_input.lower() in ['q', 'quit', 'exit']:
                        print("% Exiting teleoperation input")
                        break
                    
                    # Parse velocity input
                    try:
                        parts = user_input.split()
                        if len(parts) != 2:
                            print(f"% ERROR: Expected 2 values, got {len(parts)}")
                            if self.motor_mode:
                                print("% Usage: <left_velocity> <right_velocity>")
                            else:
                                print("% Usage: <linear_velocity> <angular_velocity>")
                            continue
                        
                        val1 = float(parts[0])
                        val2 = float(parts[1])
                        
                        # Publish the velocity vector (will also trigger resend update)
                        self.publish_velocity_input(val1, val2)
                        
                    except ValueError as e:
                        print(f"% ERROR: Invalid input - {e}")
                        print("% Enter two space-separated numbers (e.g., '0.5 0.2')")
                        
                except KeyboardInterrupt:
                    print("\n% Interrupted by user")
                    break
                except Exception as e:
                    print(f"% ERROR: {e}")
                    continue
        
        finally:
            # Cleanup - send stop command and stop resend
            print("% Stopping motors...")
            self.stop_resend()
            self.publish_velocity_input(0.0, 0.0)
            t.sleep(0.2)  # Give one more resend cycle for stop command
            self.client.loop_stop()
        self.client.disconnect()
        print("% Teleoperation Input Stopped")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Teleoperation input via velocity prompts',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                           # Velocity mode (default)
  %(prog)s -m                        # Motor mode (direct L/R velocities)
  %(prog)s -i 192.168.1.100 -p 1883
  %(prog)s --max-vel 0.8 --max-turn 1.5
        """)
    
    parser.add_argument('-i', '--host', type=str, default='localhost',
                       help='MQTT broker host (default: localhost)')
    parser.add_argument('-p', '--port', type=int, default=1883,
                       help='MQTT broker port (default: 1883)')
    parser.add_argument('-m', '--motor', action='store_true',
                       help='Motor mode: input left and right motor velocities directly')
    parser.add_argument('--max-vel', type=float, default=1.0,
                       help='Maximum linear velocity in m/s (default: 1.0)')
    parser.add_argument('--max-turn', type=float, default=2.0,
                       help='Maximum angular velocity in rad/s (default: 2.0)')
    
    args = parser.parse_args()
    
    teleop = PromptTeleopInput(mqtt_host=args.host, mqtt_port=args.port, motor_mode=args.motor)
    teleop.max_linear_vel = args.max_vel
    teleop.max_angular_vel = args.max_turn
    
    try:
        teleop.run()
    except Exception as e:
        print(f"% Fatal error: {e}")
        sys.exit(1)

