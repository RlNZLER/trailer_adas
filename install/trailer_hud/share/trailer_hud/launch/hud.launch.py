import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    layout_file = os.path.join(
        get_package_share_directory('trailer_hud'),
        'config',
        'plot_config.xml'
    )

    return LaunchDescription([
        Node(
            package='trailer_hud',
            executable='trailer_hud',
            name='trailer_hud',
            output='screen',
        ), 
        # Node(
        #     package='trailer_hud',
        #     executable='articulation_angle_logger',
        #     name='articulation_angle_logger',
        #     output='screen'
        # ),
        Node(
            package='trailer_hud',
            executable='delay_logger',
            name='delay_logger',
            output='screen'
        ),
        # Node(
        #     package='plotjuggler',
        #     executable='plotjuggler',
        #     name='plotjuggler_with_layout',
        #     arguments=['--layout', layout_file],
        #     output='screen'
        # )
    ])
