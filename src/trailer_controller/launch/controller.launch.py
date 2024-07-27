from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    
    use_python_arg = DeclareLaunchArgument(
        'use_python',
        default_value='True',
        description='Whether to use the Python or C++ controller'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.5715',
        description='Radius of the wheels'
    )
    
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='1.225',
        description='Distance between the wheels'
    )
    
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='2.63',
        description='Distance between front and rear axles'
    )
    
    use_python = LaunchConfiguration('use_python')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')
    wheel_base = LaunchConfiguration('wheel_base')
    
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", 
            "/controller_manager",
        ],
    )

    simple_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "simple_velocity_controller",
            "--controller-manager", 
            "/controller_manager",
        ],
    )
    
    steering_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "steering_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    
    ackermann_steering_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "ackermann_steering_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    
    # simple_controller_py = Node(
    #     package='trailer_controller',
    #     executable='simple_controller.py',
    #     output='screen',
    #     parameters=[
    #         {'wheel_radius': wheel_radius, 'wheel_separation': wheel_separation}
    #     ],
    #     condition = IfCondition(use_python)
    # )
    
    ackermann_controller_py = Node(
        package='trailer_controller',
        executable='ackermann_controller.py',
        output='screen',
        parameters=[
            {'wheel_radius': wheel_radius, 'wheel_separation': wheel_separation, 'wheel_base': wheel_base}
        ],
        condition = IfCondition(use_python)
    )
    
    return LaunchDescription([
        use_python_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        wheel_base_arg,
        joint_state_broadcaster_spawner,
        simple_velocity_controller,
        steering_position_controller,
        ackermann_steering_controller,
        ackermann_controller_py,
        # simple_controller_py
    ])