# trailer_adas

**Things to do:**
+ Add ackermann to control launch file
+ Add it to agzebo.xacro or ros2_control.xacro
+ Edit the ackermann yaml file and edit the wheel base... etc data.
+ Check if I need to create ackerman.xacro from limo robot

## Installation

### Step 1: Copy Material Script to Gazebo materials/scripts

Copy the a custom material script file to the Gazebo materials directory:

```
sudo cp /home/<your_user_name>/trailer_adas/src/trailer_description/materials/scripts/marker.material /usr/share/gazebo-11/media/materials/scripts/
```

### Step 2: Copy the Texture Image to Gazebo materials/textures

Copy the ArUco marker images to the Gazebo textures directory:

```
sudo cp /home/<your_user_name>/trailer_adas/src/trailer_description/materials/textures/m*_marker.png /usr/share/gazebo-11/media/materials/textures/
```