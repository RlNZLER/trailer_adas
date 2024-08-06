from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0.28575', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', 
                    '--frame-id', 'base_footprint_ekf', '--child-frame-id', 'imu_link_ekf'],
    )
    
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('trailer_localization'), 'config', 'ekf.yaml')]
    )
    
    imu_republisher_py = Node(
        package='trailer_localization',
        executable='imu_republisher.py',
    )
    
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher"
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('trailer_localization'), 'rviz', 'trailer.rviz')],
    )
    
    return LaunchDescription([
        static_transform_publisher,
        robot_localization,
        imu_republisher_py,
        joint_state_publisher_node,
        rviz_node
    ])