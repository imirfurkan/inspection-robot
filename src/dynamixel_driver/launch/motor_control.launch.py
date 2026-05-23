"""
Robot motor control launch file (runs on RPi inside Docker)

Starts:
  1. joy_mapper_node  — converts /joy → /cmd_vel
  2. dynamixel_driver_node — converts /cmd_vel → motor commands

The joy_node runs separately on the operator laptop.

Usage inside Docker:
  ros2 launch dynamixel_driver motor_control.launch.py

To override motor port:
  ros2 launch dynamixel_driver motor_control.launch.py port_name:=/dev/ttyUSB1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('dynamixel_driver')
    params_file = os.path.join(pkg_dir, 'config', 'motor_params.yaml')

    return LaunchDescription([
        # Launch arguments (overridable from command line)
        DeclareLaunchArgument(
            'port_name', default_value='/dev/ttyUSB0',
            description='U2D2 serial port'),

        # Joy Mapper: /joy → /cmd_vel
        Node(
            package='dynamixel_driver',
            executable='joy_mapper_node',
            name='joy_mapper_node',
            output='screen',
            parameters=[{
                'deadzone': 0.15,
                'max_linear_speed': 1.0,
                'max_angular_speed': 1.0,
                'axis_linear': 1,
                'axis_angular': 2,
                'axis_block': 0,
                'block_threshold': 0.5,
                'enable_button': -1,    # set to 0 for trigger deadman switch
            }],
        ),

        # Dynamixel Driver: /cmd_vel → motors
        Node(
            package='dynamixel_driver',
            executable='dynamixel_driver_node',
            name='dynamixel_driver_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
