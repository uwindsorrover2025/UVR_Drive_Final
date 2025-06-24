#!/usr/bin/env python3
"""
Twist to CAN Node

Converts ROS 2 Twist messages to CAN motor commands for differential drive control.
Implements safety features including timeout and acceleration ramping.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import can
import struct
import threading
import time
import os


class TwistToCANNode(Node):
    def __init__(self):
        super().__init__('twist_to_can_node')
        
        # Declare parameters
        self.declare_parameter('wheel_base', 1.22)  # 4 feet in meters
        self.declare_parameter('max_linear_speed', 2.0)  # m/s
        self.declare_parameter('max_angular_speed', 1.64)  # rad/s
        self.declare_parameter('pwm_min', 1000)
        self.declare_parameter('pwm_max', 2000)
        self.declare_parameter('pwm_neutral', 1500)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_id', 0x123)
        self.declare_parameter('can_bitrate', 1000000)
        self.declare_parameter('cmd_vel_timeout', 0.5)  # seconds
        self.declare_parameter('ramp_rate', 5.0)  # PWM units per ms
        self.declare_parameter('debug_mode', False)
        
        # Get parameters
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.pwm_min = self.get_parameter('pwm_min').value
        self.pwm_max = self.get_parameter('pwm_max').value
        self.pwm_neutral = self.get_parameter('pwm_neutral').value
        self.can_interface = self.get_parameter('can_interface').value
        self.can_id = self.get_parameter('can_id').value
        self.can_bitrate = self.get_parameter('can_bitrate').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.ramp_rate = self.get_parameter('ramp_rate').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # State variables
        self.last_cmd_time = self.get_clock().now()
        self.current_left_pwm = self.pwm_neutral
        self.current_right_pwm = self.pwm_neutral
        self.target_left_pwm = self.pwm_neutral
        self.target_right_pwm = self.pwm_neutral
        self.can_bus = None
        self.timeout_active = False
        
        # Initialize CAN interface
        self.init_can_interface()
        
        # Create subscriber
        self.twist_sub = self.create_subscription(
            Twist, '/cmd_vel', self.twist_callback, 10
        )
        
        # Create timer for timeout checking and ramping (20ms = 50Hz)
        self.timer = self.create_timer(0.02, self.update_motors)
        
        self.get_logger().info(f'Twist to CAN Node initialized with:')
        self.get_logger().info(f'  Wheel base: {self.wheel_base} m')
        self.get_logger().info(f'  CAN interface: {self.can_interface}')
        self.get_logger().info(f'  CAN ID: 0x{self.can_id:03X}')
        self.get_logger().info(f'  PWM range: {self.pwm_min}-{self.pwm_max} (neutral: {self.pwm_neutral})')
    
    def init_can_interface(self):
        """Initialize CAN bus interface with error handling"""
        try:
            # Try to bring up the interface if it's down
            os.system(f"sudo ip link set {self.can_interface} type can bitrate {self.can_bitrate} 2>/dev/null")
            os.system(f"sudo ip link set {self.can_interface} up 2>/dev/null")
            
            # Create CAN bus interface
            self.can_bus = can.interface.Bus(
                channel=self.can_interface,
                bustype='socketcan',
                bitrate=self.can_bitrate
            )
            
            self.get_logger().info(f'CAN interface {self.can_interface} initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN interface: {e}')
            self.get_logger().error('Make sure CAN interface is available and you have proper permissions')
            self.can_bus = None
    
    def twist_to_differential(self, linear_vel, angular_vel):
        """Convert Twist to differential drive wheel velocities"""
        # Differential drive kinematics
        left_vel = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_vel = linear_vel + (angular_vel * self.wheel_base / 2.0)
        
        return left_vel, right_vel
    
    def velocity_to_pwm(self, velocity):
        """Convert velocity (m/s) to PWM value"""
        # Normalize velocity to [-1, 1] range
        normalized = max(-1.0, min(1.0, velocity / self.max_linear_speed))
        
        # Convert to PWM range
        # PWM neutral (1000) is stop, 0 is full reverse, 1999 is full forward
        pwm_range = (self.pwm_max - self.pwm_min) / 2.0
        pwm = int(self.pwm_neutral + normalized * pwm_range)
        
        # Clamp to valid range
        return max(self.pwm_min, min(self.pwm_max, pwm))
    
    def twist_callback(self, msg):
        """Handle incoming Twist messages"""
        # Update last command time
        self.last_cmd_time = self.get_clock().now()
        self.timeout_active = False
        
        # Extract linear and angular velocities
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Convert to differential drive
        left_vel, right_vel = self.twist_to_differential(linear_vel, angular_vel)
        
        # Convert to PWM values
        self.target_left_pwm = self.velocity_to_pwm(left_vel)
        self.target_right_pwm = self.velocity_to_pwm(right_vel)
        
        if self.debug_mode:
            self.get_logger().debug(
                f'Twist: linear={linear_vel:.2f}, angular={angular_vel:.2f} -> '
                f'Vel: L={left_vel:.2f}, R={right_vel:.2f} -> '
                f'PWM: L={self.target_left_pwm}, R={self.target_right_pwm}'
            )
    
    def ramp_pwm(self, current, target, max_change):
        """Apply ramping to PWM values for smooth acceleration"""
        if current < target:
            return min(current + max_change, target)
        elif current > target:
            return max(current - max_change, target)
        return target
    
    def update_motors(self):
        """Update motor commands with timeout checking and ramping"""
        # Check for timeout
        time_since_last_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        if time_since_last_cmd > self.cmd_vel_timeout:
            if not self.timeout_active:
                self.get_logger().warn(f'Command timeout! No cmd_vel for {time_since_last_cmd:.1f}s')
                self.timeout_active = True
            # Set target to neutral (stop)
            self.target_left_pwm = self.pwm_neutral
            self.target_right_pwm = self.pwm_neutral
        
        
        if self.timeout_active:
            # Set both current PWM outputs to neutral for safety
            self.current_left_pwm = self.pwm_neutral
            self.current_right_pwm = self.pwm_neutral
# Calculate max PWM change for this update (ramp_rate is per ms, timer is 20ms)
        max_change = int(self.ramp_rate * 20)
        
        # Apply ramping
        self.current_left_pwm = self.ramp_pwm(
            self.current_left_pwm, self.target_left_pwm, max_change
        )
        self.current_right_pwm = self.ramp_pwm(
            self.current_right_pwm, self.target_right_pwm, max_change
        )
        
        # Send CAN message
        self.send_can_message(self.current_left_pwm, self.current_right_pwm)
    
    def send_can_message(self, left_pwm, right_pwm):
        """Pack and send motor commands over CAN"""
        if self.can_bus is None:
            # Try to reinitialize if not connected
            self.init_can_interface()
            if self.can_bus is None:
                return
        
        try:
            # Pack as big-endian int16
            data = struct.pack('>hh', left_pwm, right_pwm)
            
            # Create CAN message
            msg = can.Message(
                arbitration_id=self.can_id,
                data=data,
                is_extended_id=False
            )
            
            # Send message
            self.can_bus.send(msg, timeout=0.1)
            
            # Log periodically in debug mode
            if self.debug_mode and int(self.get_clock().now().nanoseconds / 1e9) % 1 == 0:
                self.get_logger().debug(
                    f'CAN TX: ID=0x{self.can_id:03X} Data={data.hex()} '
                    f'(L={left_pwm}, R={right_pwm})'
                )
                
        except can.CanError as e:
            self.get_logger().error(f'CAN send error: {e}')
            # Try to recover
            self.recover_can_bus()
        except Exception as e:
            self.get_logger().error(f'Unexpected error sending CAN message: {e}')
    
    def recover_can_bus(self):
        """Attempt to recover from CAN bus errors"""
        self.get_logger().info('Attempting CAN bus recovery...')
        
        try:
            # Close existing bus
            if self.can_bus:
                self.can_bus.shutdown()
            
            # Reset the interface
            os.system(f"sudo ip link set {self.can_interface} down")
            time.sleep(0.1)
            os.system(f"sudo ip link set {self.can_interface} up")
            
            # Reinitialize
            self.init_can_interface()
            
        except Exception as e:
            self.get_logger().error(f'CAN recovery failed: {e}')
    
    def check_can_health(self):
        """Monitor CAN bus statistics"""
        try:
            # Get interface statistics
            stats = os.popen(f"ip -s -d link show {self.can_interface}").read()
            
            if "bus-off" in stats:
                self.get_logger().error("CAN bus is in bus-off state!")
                self.recover_can_bus()
                
        except Exception as e:
            self.get_logger().debug(f'Error checking CAN health: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TwistToCANNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        # Make sure to stop motors before exiting
        if 'node' in locals() and node.can_bus:
            try:
                # Send neutral PWM to stop motors
                data = struct.pack('>hh', node.pwm_neutral, node.pwm_neutral)
                msg = can.Message(
                    arbitration_id=node.can_id,
                    data=data,
                    is_extended_id=False
                )
                node.can_bus.send(msg, timeout=0.1)
                node.can_bus.shutdown()
            except:
                pass
        
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
