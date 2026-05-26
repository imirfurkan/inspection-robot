"""
OAK-D Pro camera launch file

Starts:
  1. oakd_node — DepthAI pipeline + Flask dashboard + H.265 encoding

Usage inside Docker:
  ros2 launch camera_driver oakd.launch.py

Override parameters:
  ros2 launch camera_driver oakd.launch.py dashboard_port:=9090
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('camera_driver')
    params_file = os.path.join(pkg_dir, 'config', 'oakd_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dashboard_port', default_value='8080',
            description='Flask dashboard HTTP port'),

        Node(
            package='camera_driver',
            executable='oakd_node',
            name='oakd_node',
            output='screen',
            parameters=[params_file],
        ),
    ])