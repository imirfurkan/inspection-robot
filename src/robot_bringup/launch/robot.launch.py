"""
Robot bringup launch file — starts all nodes

Starts:
  1. joy_mapper_node       — converts /joy → /cmd_vel
  2. dynamixel_driver_node — converts /cmd_vel linear.x → drive motors
  3. steering_servo_node   — reads /joy axis 2 → steering servo
  4. oakd_node             — OAK-D Pro pipeline + Flask dashboard + H.265

The joy_node runs separately on the operator laptop.

Usage inside Docker:
  ros2 launch robot_bringup robot.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        # Motors: joy_mapper + dynamixel_driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('dynamixel_driver'),
                             'launch', 'motor_control.launch.py')
            )
        ),

        # Steering servo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('steering_servo'),
                             'launch', 'steering.launch.py')
            )
        ),

        # OAK-D Pro front camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('camera_driver'),
                             'launch', 'oakd.launch.py')
            )
        ),

        # LED strip
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('led_driver'),
                            'launch', 'led.launch.py')
            )
        ),
    ])