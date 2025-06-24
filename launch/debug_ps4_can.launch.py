#!/usr/bin/env python3
"""
Debug launch file for PS4 to CAN communication
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('diff_drive_can')
    config_file = os.path.join(pkg_dir, 'config', 'diff_drive_params.yaml')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # Launch argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        # Log startup
        LogInfo(msg='Starting PS4 to CAN debug launch...'),
        
        # Joy node for PS4 controller
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
            output='screen'
        ),
        
        # PS4 Twist node with delay to ensure joy is ready
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='diff_drive_can',
                    executable='ps4_twist_node',
                    name='ps4_twist_node',
                    parameters=[config_file],
                    output='screen'
                ),
            ]
        ),
        
        # Twist to CAN node with delay
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='diff_drive_can',
                    executable='twist_to_can_node',
                    name='twist_to_can_node',
                    parameters=[config_file],
                    output='screen'
                ),
            ]
        ),
        
        # Debug node with delay
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='diff_drive_can',
                    executable='debug_ps4_to_can.py',
                    name='ps4_can_debugger',
                    output='screen'
                ),
            ]
        ),
    ])