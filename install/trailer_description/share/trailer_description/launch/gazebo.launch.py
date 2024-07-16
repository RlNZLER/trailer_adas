import os
from os import pathsep
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    trailer_description = get_package_share_directory('trailer_description')
    trailer_description_prefix = get_package_prefix('trailer_description')
    
    model_path = os.path.join(trailer_description, 'models')
    model_path += pathsep + os.path.join(trailer_description_prefix, 'share')
    
    env_variable = SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path)
    
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(trailer_description, 'urdf', 'trailer.urdf.xacro'), 
        description='Absolute path to robot urdf file'
    )
    
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration("model")]), value_type=str)
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )
    
    start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')))
    
    start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')))
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'trailer', '-topic', 'robot_description'],
        output='screen'
    )
    
    return LaunchDescription([
        env_variable,
        model_arg,
        robot_state_publisher_node,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot        
    ])