"""
Steering servo launch file

Starts:
  steering_servo_node — reads /joy axis and controls steering

Usage:
  ros2 launch steering_servo steering.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='steering_servo',
            executable='steering_servo_node',  # adjust to your actual executable
            name='steering_servo_node',
            output='screen',
            parameters=[{
                'servo_pin': 17,
                'center_pulse': 1500,
                'deadzone': 0.1,
            }],
        ),
    ])