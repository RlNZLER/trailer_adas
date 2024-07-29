# trailer_adas

**Things to do:**
+ Add ackermann to control launch file
+ Add it to agzebo.xacro or ros2_control.xacro
+ Edit the ackermann yaml file and edit the wheel base... etc data.
+ Check if I need to create ackerman.xacro from limo robot


### Step 3: Configure the Controller

You need to create a configuration file for the `ackermann_steering_controller`. This file will specify how the controller interacts with the joints and how it processes commands.

Here’s an example of what this configuration might look like in a YAML file (`ackermann_steering_controller.yaml`):

```yaml
ackermann_steering_controller:
  ros__parameters:
    type: ackermann_steering_controller/AckermannSteeringController
    joint: front_joint
    publish_rate: 50
    command_interface: velocity
    command_timeout: 0.5
    velocity_rolling_window_size: 1000
    allowed_execution_duration_scaling: 1.2
    allowed_goal_duration_margin: 0.5
    state_publish_rate: 75
    stop_trajectory_duration: 0.5
    wheel_separation_h: 0.5
    wheel_separation_w: 0.6
    wheel_radius: 0.3
    wheel_base: 1.0
    input_mode: passthrough
    joint_frame_id: base_link
    left_wheel_name: left_wheel_joint
    right_wheel_name: right_wheel_joint
    left_steering_name: left_steering_joint
    right_steering_name: right_steering_joint
```


### Step 4: Launch the Controller
You need to add this controller to your ROS 2 launch file, ensuring it’s loaded and started with the controller manager. Below is an example of how to include this controller in a launch file:

```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=['/path/to/ackermann_steering_controller.yaml'],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
        ),
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['ackermann_steering_controller', '--controller-manager', '/controller_manager'],
        ),
    ])
```

Launch commands:
ros2 launch trailer_description gazebo.launch.py 
ros2 launch trailer_controller controller.launch.py 

Telop Twist Keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/trailer_controller/cmd_vel

