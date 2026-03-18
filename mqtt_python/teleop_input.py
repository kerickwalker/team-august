#!/usr/bin/env python3

"""Teleoperation input client with prompt-based velocity commands.

Publishes velocity input vectors [linear_velocity, angular_velocity] to MQTT.
These values are consumed by:
  1. Kalman filter (skalman.py) for model-based predictions
  2. Motor control (via teensy_interface) for actual robot movement

Usage:
    python3 teleop_input.py [options]
    
    When running, enter velocity commands as two space-separated values:
    Example: 0.5 0.2
    (sends linear_vel=0.5 m/s, angular_vel=0.2 rad/s)
    
    Type 'help' for commands
    Type 'q' or 'quit' to exit
"""

import sys
import os
import json
import time as t
from datetime import datetime

try:
    from paho.mqtt import client as mqtt_client
except ImportError:
    print("% ERROR: Missing required packages. Install with:")
    print("%   pip install paho-mqtt")
    sys.exit(1)


class PromptTeleopInput:
    """Teleoperation via prompt-based velocity input."""
    
    def __init__(self, mqtt_host='localhost', mqtt_port=1883):
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.client = None
        self.connected = False
        
        # Input limits
        self.max_linear_vel = 1.0   # m/s
        self.max_angular_vel = 2.0  # rad/s
        self.last_publish_time = datetime.now()
        
        self.setup_mqtt()
    
    def setup_mqtt(self):
        """Connect to MQTT broker."""
        try:
            self.client = mqtt_client.Client("teleop-input-prompt")
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
    
    def publish_velocity_input(self, linear_vel, angular_vel):
        """Publish velocity input vector to MQTT.
        
        Args:
            linear_vel (float): Linear velocity in m/s
            angular_vel (float): Angular velocity in rad/s
        """
        if not self.connected:
            print("% ERROR: Not connected to MQTT broker")
            return False
        
        # Clamp values to safe limits
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, float(linear_vel)))
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, float(angular_vel)))
        
        # Create input vector message
        cmd = {
            "linear_velocity": linear_vel,
            "angular_velocity": angular_vel,
            "timestamp": datetime.now().isoformat()
        }
        
        # Publish to teleoperation channel
        topic = "robobot/teleop/cmd"
        payload = json.dumps(cmd)
        
        try:
            self.client.publish(topic, payload, qos=1)
            print(f"✓ Published: [{linear_vel:.3f}, {angular_vel:.3f}] -> {topic}")
            return True
        except Exception as e:
            print(f"% ERROR: Failed to publish: {e}")
            return False
    
    def print_help(self):
        """Print help message."""
        print("\n% Teleoperation Input Commands:")
        print("% ==============================")
        print("% Enter velocity input as two space-separated values:")
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
        print(f"% Limits: linear_vel [{-self.max_linear_vel}, {self.max_linear_vel}] m/s")
        print(f"%         angular_vel [{-self.max_angular_vel}, {self.max_angular_vel}] rad/s")
        print("% ")
        print("% Commands:")
        print("%   help, h, ?   -> Show this help")
        print("%   q, quit      -> Exit")
        print("% ==============================\n")
    
    def run(self):
        """Start teleoperation input loop."""
        print("\n% ===== Teleoperation Input (Velocity Prompt) =====")
        print("% Connected and ready for velocity commands")
        self.print_help()
        
        while True:
            try:
                user_input = input("Enter velocity [linear angular] or command: ").strip()
                
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
                        print("% Usage: <linear_velocity> <angular_velocity>")
                        continue
                    
                    linear_vel = float(parts[0])
                    angular_vel = float(parts[1])
                    
                    # Publish the velocity vector
                    self.publish_velocity_input(linear_vel, angular_vel)
                    
                except ValueError as e:
                    print(f"% ERROR: Invalid input - {e}")
                    print("% Enter two space-separated numbers (e.g., '0.5 0.2')")
                    
            except KeyboardInterrupt:
                print("\n% Interrupted by user")
                break
            except Exception as e:
                print(f"% ERROR: {e}")
                continue
        
        # Cleanup - send stop command
        print("% Sending stop command...")
        self.publish_velocity_input(0.0, 0.0)
        t.sleep(0.1)
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
  %(prog)s
  %(prog)s -i 192.168.1.100 -p 1883
  %(prog)s --max-vel 0.8 --max-turn 1.5
        """)
    
    parser.add_argument('-i', '--host', type=str, default='localhost',
                       help='MQTT broker host (default: localhost)')
    parser.add_argument('-p', '--port', type=int, default=1883,
                       help='MQTT broker port (default: 1883)')
    parser.add_argument('--max-vel', type=float, default=1.0,
                       help='Maximum linear velocity in m/s (default: 1.0)')
    parser.add_argument('--max-turn', type=float, default=2.0,
                       help='Maximum angular velocity in rad/s (default: 2.0)')
    
    args = parser.parse_args()
    
    teleop = PromptTeleopInput(mqtt_host=args.host, mqtt_port=args.port)
    teleop.max_linear_vel = args.max_vel
    teleop.max_angular_vel = args.max_turn
    
    try:
        teleop.run()
    except Exception as e:
        print(f"% Fatal error: {e}")
        sys.exit(1)

