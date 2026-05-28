"""
Motor control launch file

Starts:
  1. joy_mapper_node       — converts /joy → /cmd_vel
  2. dynamixel_driver_node — converts /cmd_vel linear.x → drive motors

Usage:
  ros2 launch dynamixel_driver motor_control.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('dynamixel_driver')
    robot_params = os.path.join(pkg_dir, 'config', 'robot_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port_name', default_value='/dev/ttyUSB0',
            description='U2D2 serial port'),

        # Joy Mapper: /joy → /cmd_vel
        Node(
            package='dynamixel_driver',
            executable='joy_mapper_node',
            name='joy_mapper_node',
            output='screen',
            parameters=[robot_params],
        ),

        # Dynamixel Driver: /cmd_vel linear.x → motors
        Node(
            package='dynamixel_driver',
            executable='dynamixel_driver_node',
            name='dynamixel_driver_node',
            output='screen',
            parameters=[robot_params],
        ),
    ])