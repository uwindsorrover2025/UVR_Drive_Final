#!/usr/bin/env python3
"""
Test script to identify PS4 controller axis mappings
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class PS4AxisTester(Node):
    def __init__(self):
        super().__init__('ps4_axis_tester')
        
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )
        
        self.get_logger().info('PS4 Axis Tester Started')
        self.get_logger().info('Move each joystick to see axis values...')
        self.get_logger().info('-' * 50)
        
        self.last_axes = None
        
    def joy_callback(self, msg):
        # Only print when axes change significantly
        if self.last_axes is None:
            self.last_axes = list(msg.axes)
            self.print_axes(msg)
            return
            
        # Check for significant changes
        changed = False
        for i in range(len(msg.axes)):
            if abs(msg.axes[i] - self.last_axes[i]) > 0.1:
                changed = True
                break
        
        if changed:
            self.print_axes(msg)
            self.last_axes = list(msg.axes)
    
    def print_axes(self, msg):
        self.get_logger().info('Axes values:')
        for i, value in enumerate(msg.axes):
            if abs(value) > 0.1:  # Only show non-zero axes
                self.get_logger().info(f'  Axis {i}: {value:.3f}')
        
        # Also show button presses
        pressed_buttons = []
        for i, value in enumerate(msg.buttons):
            if value == 1:
                pressed_buttons.append(f'Button {i}')
        
        if pressed_buttons:
            self.get_logger().info(f'  Pressed: {", ".join(pressed_buttons)}')
        
        self.get_logger().info('-' * 30)

def main(args=None):
    rclpy.init(args=args)
    
    node = PS4AxisTester()
    
    node.get_logger().info('Common PS4 mappings:')
    node.get_logger().info('  Left stick X: Usually axis 0')
    node.get_logger().info('  Left stick Y: Usually axis 1') 
    node.get_logger().info('  Right stick X: Usually axis 2 or 3')
    node.get_logger().info('  Right stick Y: Usually axis 3 or 4')
    node.get_logger().info('  L2 trigger: Usually axis 2 or 3')
    node.get_logger().info('  R2 trigger: Usually axis 4 or 5')
    node.get_logger().info('')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()