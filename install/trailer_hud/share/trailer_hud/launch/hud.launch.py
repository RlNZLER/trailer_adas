from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trailer_hud',
            executable='trailer_hud',
            name='trailer_hud',
            output='screen',
        ), 
        Node(
            package='trailer_hud',
            executable='articulation_angle_logger',
            name='articulation_angle_logger',
            output='screen'
        )
    ])
