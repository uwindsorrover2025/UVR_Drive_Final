#!/usr/bin/env python3
"""
Debug script to trace PS4 to CAN communication chain
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import subprocess
import time

class PS4CANDebugger(Node):
    def __init__(self):
        super().__init__('ps4_can_debugger')
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )
        
        self.twist_sub = self.create_subscription(
            Twist, '/cmd_vel', self.twist_callback, 10
        )
        
        # State tracking
        self.last_joy_time = None
        self.last_twist_time = None
        self.joy_msg_count = 0
        self.twist_msg_count = 0
        
        # Create timer for status updates
        self.timer = self.create_timer(1.0, self.print_status)
        
        self.get_logger().info('PS4 to CAN Debugger started')
        self.get_logger().info('Monitoring /joy and /cmd_vel topics...')
        
    def joy_callback(self, msg):
        self.last_joy_time = self.get_clock().now()
        self.joy_msg_count += 1
        
        # Log significant joystick movements
        if abs(msg.axes[0]) > 0.1 or abs(msg.axes[1]) > 0.1:
            self.get_logger().info(
                f'Joy input: axes[0]={msg.axes[0]:.2f}, axes[1]={msg.axes[1]:.2f}'
            )
    
    def twist_callback(self, msg):
        self.last_twist_time = self.get_clock().now()
        self.twist_msg_count += 1
        
        # Log twist commands
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.get_logger().info(
                f'Twist cmd: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}'
            )
    
    def print_status(self):
        # Check node status
        nodes_output = subprocess.run(
            ['ros2', 'node', 'list'], 
            capture_output=True, 
            text=True
        ).stdout
        
        ps4_running = '/ps4_twist_node' in nodes_output
        can_running = '/twist_to_can_node' in nodes_output
        
        # Check topic connections
        joy_info = subprocess.run(
            ['ros2', 'topic', 'info', '/joy'], 
            capture_output=True, 
            text=True
        ).stdout
        
        cmd_vel_info = subprocess.run(
            ['ros2', 'topic', 'info', '/cmd_vel'], 
            capture_output=True, 
            text=True
        ).stdout
        
        # Print status
        self.get_logger().info('='*50)
        self.get_logger().info(f'Node Status:')
        self.get_logger().info(f'  ps4_twist_node: {"RUNNING" if ps4_running else "NOT FOUND"}')
        self.get_logger().info(f'  twist_to_can_node: {"RUNNING" if can_running else "NOT FOUND"}')
        
        self.get_logger().info(f'Message Counts:')
        self.get_logger().info(f'  /joy messages: {self.joy_msg_count}')
        self.get_logger().info(f'  /cmd_vel messages: {self.twist_msg_count}')
        
        # Check for issues
        if self.joy_msg_count > 0 and self.twist_msg_count == 0:
            self.get_logger().warn('⚠️  Joy messages received but NO cmd_vel messages!')
            self.get_logger().warn('   ps4_twist_node may not be converting properly')
        
        if self.last_joy_time and self.last_twist_time:
            delay = (self.last_twist_time - self.last_joy_time).nanoseconds / 1e6
            if abs(delay) > 100:
                self.get_logger().warn(f'⚠️  Large delay between joy and twist: {delay:.1f}ms')
        
        # Check CAN interface
        try:
            can_status = subprocess.run(
                ['ip', 'link', 'show', 'can0'], 
                capture_output=True, 
                text=True
            ).stdout
            
            if 'UP' in can_status:
                self.get_logger().info('CAN interface: UP')
            else:
                self.get_logger().error('CAN interface: DOWN')
                
        except Exception as e:
            self.get_logger().error(f'Cannot check CAN status: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PS4CANDebugger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()