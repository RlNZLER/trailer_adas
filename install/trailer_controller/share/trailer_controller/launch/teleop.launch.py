from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Your simple_controller node
    teleop_controller_node = Node(
        package='trailer_controller',
        executable='teleop_controller.py',
        name='teleop_controller',
        output='screen',
    )

    return LaunchDescription([
        teleop_controller_node,
    ])
