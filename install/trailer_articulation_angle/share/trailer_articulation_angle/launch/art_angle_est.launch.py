from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trailer_articulation_angle',
            executable='articulation_angle_node',
            name='articulation_angle_node',
            output='screen',
        ), 
        Node(
            package='trailer_articulation_angle',
            executable='point_cloud',
            name='point_cloud',
            output='screen'
        )
    ])
