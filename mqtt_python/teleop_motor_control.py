#!/usr/bin/env python3

"""Motor control integration with teleoperation.

This module provides the interface to convert teleoperation velocity commands
[linear_velocity, angular_velocity] to motor control parameters.

The teleoperation input flows through the following path:
1. teleop_input.py publishes velocity vectors to MQTT
2. Kalman filter (skalman.py) receives velocity input and uses it as control input
3. Motor control applies these velocity commands to drive the robot

Usage:
    Motor control sends velocity commands directly:
    - Teleoperation input: [linear_velocity, angular_velocity]
    - Motor control command: rc {linear_velocity} {angular_velocity}
"""

import json
import time as t
from datetime import datetime

class TeleopMotorController:
    """Handle motor control based on velocity input."""
    
    def __init__(self, wheelbase=0.22):
        """Initialize motor controller.
        
        Args:
            wheelbase (float): Distance between left and right wheels in meters
        """
        self.wheelbase = float(wheelbase)
    
    def velocity_to_wheel_velocities(self, linear_vel, angular_vel):
        """Convert velocity command to wheel velocities.
        
        Differential drive kinematics:
        v_left = linear_vel - (wheelbase/2) * angular_vel
        v_right = linear_vel + (wheelbase/2) * angular_vel
        
        Args:
            linear_vel (float): Linear velocity in m/s
            angular_vel (float): Angular velocity in rad/s
        
        Returns:
            tuple: (v_left, v_right) wheel velocities in m/s
        """
        linear_vel = float(linear_vel)
        angular_vel = float(angular_vel)
        
        half_wheelbase = self.wheelbase / 2.0
        v_left = linear_vel - half_wheelbase * angular_vel
        v_right = linear_vel + half_wheelbase * angular_vel
        
        return (v_left, v_right)
    
    def velocity_to_rc_command(self, linear_vel, angular_vel):
        """Convert velocity command to 'rc' command format for teensy_interface.
        
        The teensy_interface expects 'rc' commands with (linear_velocity, angular_velocity).
        
        Args:
            linear_vel (float): Linear velocity in m/s
            angular_vel (float): Angular velocity in rad/s
        
        Returns:
            str: Command string like "rc 0.5 0.2"
        """
        linear_vel = float(linear_vel)
        angular_vel = float(angular_vel)
        
        return f"rc {linear_vel:.3f} {angular_vel:.3f}"
    
    def velocity_to_motor_speeds(self, linear_vel, angular_vel):
        """Convert velocity command to motor speeds [left, right].
        
        This is useful for PWM-based motor drivers.
        
        Args:
            linear_vel (float): Linear velocity in m/s
            angular_vel (float): Angular velocity in rad/s
        
        Returns:
            list: [left_speed, right_speed] in m/s
        """
        v_left, v_right = self.velocity_to_wheel_velocities(linear_vel, angular_vel)
        return [v_left, v_right]


# Singleton instance
motor_controller = TeleopMotorController()


if __name__ == "__main__":
    # Test the motor controller
    controller = TeleopMotorController()
    
    print("Motor Controller Test")
    print("=" * 60)
    print("Wheelbase: 0.22 m")
    print("")
    
    # Test cases
    test_cases = [
        (0.5, 0.0, "Forward at 0.5 m/s"),
        (-0.5, 0.0, "Backward at 0.5 m/s"),
        (0.0, 0.5, "Turn left at 0.5 rad/s"),
        (0.0, -0.5, "Turn right at 0.5 rad/s"),
        (0.3, 0.3, "Forward-left"),
        (0.3, -0.3, "Forward-right"),
        (0.1, 0.0, "Forward slow"),
        (0.0, 0.0, "Stop"),
    ]
    
    for linear, angular, description in test_cases:
        v_left, v_right = controller.velocity_to_wheel_velocities(linear, angular)
        rc_cmd = controller.velocity_to_rc_command(linear, angular)
        motor_speeds = controller.velocity_to_motor_speeds(linear, angular)
        
        print(f"{description}:")
        print(f"  Input:        linear={linear:.2f} m/s, angular={angular:.2f} rad/s")
        print(f"  Wheel vel:    left={v_left:.3f}, right={v_right:.3f} m/s")
        print(f"  RC command:   {rc_cmd}")
        print(f"  Motor speeds: {motor_speeds}")
        print()

