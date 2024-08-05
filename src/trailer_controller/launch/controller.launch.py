from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def noisy_controller(context, *args, **kwargs):
    wheel_radius = float(LaunchConfiguration('wheel_radius').perform(context))
    wheel_separation = float(LaunchConfiguration('wheel_separation').perform(context))
    wheel_base = float(LaunchConfiguration('wheel_base').perform(context))
    
    wheel_radius_error = float(LaunchConfiguration('wheel_radius_error').perform(context))
    wheel_separation_error = float(LaunchConfiguration('wheel_separation_error').perform(context))
    wheel_base_error = float(LaunchConfiguration('wheel_base_error').perform(context))
    
    noisy_controller_py = Node(
        package='trailer_controller',
        executable='noisy_controller.py',
        parameters=[{
            'wheel_radius': wheel_radius + wheel_radius_error,
            'wheel_separation': wheel_separation + wheel_separation_error,
            'wheel_base': wheel_base + wheel_base_error,
        }]
    )
    
    return [noisy_controller_py]

def generate_launch_description():
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.5715',
        description='Wheel radius'
    )
    
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='1.225',
        description='Wheel separation'
    )
    
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='2.63',
        description='Wheel base'
    )
    
    wheel_radius_error = DeclareLaunchArgument(
        'wheel_radius_error',
        default_value='0.005',
        description='Wheel radius error'
    )
    
    wheel_separation_error = DeclareLaunchArgument(
        'wheel_separation_error',
        default_value='0.02',
        description='Wheel separation error'
    )
    
    wheel_base_error = DeclareLaunchArgument(
        'wheel_base_error',
        default_value='0.02',
        description='Wheel base error'
    )
    
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

    ackermann_controller_py = Node(
        package='trailer_controller',
        executable='ackermann_controller.py',
        name='ackermann_controller',
        output='screen',
        parameters=[{"wheel_radius": wheel_radius, "wheel_separation": wheel_separation, "wheel_base": wheel_base}]
    )
    
    noisy_controller_launch = OpaqueFunction(
        function = noisy_controller
    )
    
    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        wheel_base_arg,
        wheel_radius_error,
        wheel_separation_error,
        wheel_base_error,
        joint_state_broadcaster_spawner,
        simple_velocity_controller_spawner,
        steering_position_controller_spawner,
        ackermann_controller_py,
        noisy_controller_launch
    ])