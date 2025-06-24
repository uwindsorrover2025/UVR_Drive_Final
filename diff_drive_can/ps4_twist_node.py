#!/usr/bin/env python3
"""
PS4 to Twist Node

Converts PS4 controller joystick input to ROS 2 Twist messages for differential drive control.
Includes safety features like emergency stop and deadzone filtering.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math


class PS4TwistNode(Node):
    def __init__(self):
        super().__init__('ps4_twist_node')
        
        # Declare parameters
        self.declare_parameter('linear_axis', 1)  # Left stick Y-axis
        self.declare_parameter('angular_axis', 0)  # Left stick X-axis
        self.declare_parameter('max_linear_speed', 2.0)  # m/s
        self.declare_parameter('max_angular_speed', 1.64)  # rad/s
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('turbo_button', 5)  # R1 button
        self.declare_parameter('turbo_multiplier', 1.5)
        self.declare_parameter('emergency_stop_button', 4)  # L1 button
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('debug_mode', False)
        
        # Get parameters
        self.linear_axis = self.get_parameter('linear_axis').value
        self.angular_axis = self.get_parameter('angular_axis').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.turbo_button = self.get_parameter('turbo_button').value
        self.turbo_multiplier = self.get_parameter('turbo_multiplier').value
        self.emergency_stop_button = self.get_parameter('emergency_stop_button').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # State variables
        self.emergency_stop_active = False
        self.last_emergency_button_state = 0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.turbo_active = False
        
        # Create publisher and subscriber
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        
        # Publish initial zero Twist to prevent forward motion at startup
        initial_twist = Twist()
        initial_twist.linear.x = 0.0
        initial_twist.angular.z = 0.0
        self.twist_pub.publish(initial_twist)
# Create timer for publishing at fixed rate
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_twist)
        
        self.get_logger().info(f'PS4 Twist Node initialized with:')
        self.get_logger().info(f'  Max linear speed: {self.max_linear_speed} m/s')
        self.get_logger().info(f'  Max angular speed: {self.max_angular_speed} rad/s')
        self.get_logger().info(f'  Deadzone: {self.deadzone}')
        self.get_logger().info(f'  Publishing at {self.publish_rate} Hz')
    
    def apply_deadzone(self, value):
        """Apply deadzone to joystick input"""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale the value to maintain full range after deadzone
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def joy_callback(self, msg):
        """Process joystick input"""
        if self.debug_mode:
            self.get_logger().info(f'Joy callback received: {len(msg.axes)} axes, {len(msg.buttons)} buttons')
        
        # Check array bounds
        if len(msg.axes) <= max(self.linear_axis, self.angular_axis):
            self.get_logger().warn(f'Not enough axes in Joy message. Got {len(msg.axes)}, need at least {max(self.linear_axis, self.angular_axis) + 1}')
            return
        
        if len(msg.buttons) <= max(self.turbo_button, self.emergency_stop_button):
            self.get_logger().warn(f'Not enough buttons in Joy message. Got {len(msg.buttons)}, need at least {max(self.turbo_button, self.emergency_stop_button) + 1}')
            return
        
        # Handle emergency stop (L1 button)
        emergency_button = msg.buttons[self.emergency_stop_button]
        if emergency_button and not self.last_emergency_button_state:
            # Button just pressed
            self.emergency_stop_active = not self.emergency_stop_active
            if self.emergency_stop_active:
                self.get_logger().warn('EMERGENCY STOP ACTIVATED!')
                self.current_linear = 0.0
                self.current_angular = 0.0
            else:
                self.get_logger().info('Emergency stop deactivated')
        self.last_emergency_button_state = emergency_button
        
        # If emergency stop is active, ignore other inputs
        if self.emergency_stop_active:
            return
        
        # Get joystick values
        linear_raw = -msg.axes[self.linear_axis]  # Negate for intuitive forward/backward
        angular_raw = msg.axes[self.angular_axis]
        
        if self.debug_mode and (abs(linear_raw) > 0.01 or abs(angular_raw) > 0.01):
            self.get_logger().info(f'Raw axes: linear_raw={linear_raw:.3f} (axis {self.linear_axis}), angular_raw={angular_raw:.3f} (axis {self.angular_axis})')
        
        # Apply deadzone
        linear = self.apply_deadzone(linear_raw)
        angular = self.apply_deadzone(angular_raw)
        
        if self.debug_mode and (abs(linear) > 0.01 or abs(angular) > 0.01):
            self.get_logger().info(f'After deadzone: linear={linear:.3f}, angular={angular:.3f}')
        
        # Check turbo mode (R1 button)
        self.turbo_active = bool(msg.buttons[self.turbo_button])
        
        # Scale to max speeds
        speed_multiplier = self.turbo_multiplier if self.turbo_active else 1.0
        self.current_linear = linear * self.max_linear_speed * speed_multiplier
        self.current_angular = angular * self.max_angular_speed * speed_multiplier
        
        if self.debug_mode and (abs(self.current_linear) > 0.01 or abs(self.current_angular) > 0.01):
            self.get_logger().info(f'Final speeds: linear={self.current_linear:.3f} m/s, angular={self.current_angular:.3f} rad/s')
        
        # Clamp to max speeds (in case turbo exceeds limits)
        max_linear = self.max_linear_speed * self.turbo_multiplier
        max_angular = self.max_angular_speed * self.turbo_multiplier
        self.current_linear = max(-max_linear, min(max_linear, self.current_linear))
        self.current_angular = max(-max_angular, min(max_angular, self.current_angular))
    
    def publish_twist(self):
        """Publish Twist message at fixed rate"""
        twist = Twist()
        twist.linear.x = self.current_linear
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.current_angular
        
        self.twist_pub.publish(twist)
        
        # Log status periodically (every 2 seconds)
        if int(self.get_clock().now().nanoseconds / 1e9) % 2 == 0:
            if self.emergency_stop_active:
                self.get_logger().debug('Emergency stop active - motors stopped')
            elif abs(self.current_linear) > 0.01 or abs(self.current_angular) > 0.01:
                turbo_str = " (TURBO)" if self.turbo_active else ""
                self.get_logger().debug(
                    f'Cmd: linear={self.current_linear:.2f} m/s, '
                    f'angular={self.current_angular:.2f} rad/s{turbo_str}'
                )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PS4TwistNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()