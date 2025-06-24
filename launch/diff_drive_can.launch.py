#!/usr/bin/env python3
"""
Launch file for diff_drive_can package

Launches:
- joy_node: Reads PS4 controller input
- ps4_twist_node: Converts joystick to Twist messages
- twist_to_can_node: Converts Twist to CAN motor commands
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('diff_drive_can')
    
    # Path to config file
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'diff_drive_params.yaml'
    ])
    
    # Launch arguments
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug logging'
    )
    
    # Joy node (from joy package)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )
    
    # PS4 to Twist node
    ps4_twist_node = Node(
        package='diff_drive_can',
        executable='ps4_twist_node',
        name='ps4_twist_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )
    
    # Twist to CAN node
    twist_to_can_node = Node(
        package='diff_drive_can',
        executable='twist_to_can_node',
        name='twist_to_can_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        # Launch arguments
        joy_dev_arg,
        debug_arg,
        
        # Nodes
        joy_node,
        ps4_twist_node,
        twist_to_can_node
    ])