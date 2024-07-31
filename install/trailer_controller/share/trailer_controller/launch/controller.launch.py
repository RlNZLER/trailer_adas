from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", 
            "/controller_manager",
        ],
    )
    
    simple_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    
    steering_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "steering_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    
    # ackermann_steering_controller_spawner  = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=[
    #         "ackermann_steering_controller",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ],
    # )

    ackermann_controller_node = Node(
        package='trailer_controller',
        executable='ackermann_controller.py',
        name='ackermann_controller',
        output='screen',
    )
    
    return LaunchDescription([
        joint_state_broadcaster_spawner,
        simple_velocity_controller_spawner,
        steering_position_controller_spawner,
        ackermann_controller_node

    ])