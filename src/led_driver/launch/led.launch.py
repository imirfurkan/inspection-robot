from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('led_driver'), 'config', 'led_params.yaml')
    return LaunchDescription([
        Node(package='led_driver', executable='led_node',
             name='led_driver_node', output='screen', parameters=[params]),
    ])