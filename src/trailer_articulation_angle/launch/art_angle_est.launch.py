from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trailer_articulation_angle',
            executable='aruco_marker_node',
            name='aruco_marker_node',
            output='screen',
        ), 
        Node(
            package='trailer_articulation_angle',
            executable='range_processor_node',
            name='range_processor_node',
            output='screen'
        ),
        Node(
            package='trailer_articulation_angle',
            executable='kalman_filter',
            name='kalman_filter',
            output='screen'
        )
    ])
