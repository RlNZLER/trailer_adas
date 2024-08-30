Here is the edited version of your README file with a few improvements for clarity and flow:

---

# Trailer Detection and Articulation Angle Estimation

This repository contains the code for a ROS 2-based simulation environment designed to detect trailers and estimate the articulation angle between a towing vehicle and the trailer. The project integrates sensor data from rear-end cameras and proximity sensors to provide accurate, real-time feedback for driving-assistance systems.

## Table of Contents

- [Trailer Detection and Articulation Angle Estimation](#trailer-detection-and-articulation-angle-estimation)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Prerequisites](#prerequisites)
  - [Setup](#setup)
  - [Launching the Simulation](#launching-the-simulation)
    - [Script Details](#script-details)
  - [Troubleshooting](#troubleshooting)
  - [License](#license)

## Overview

This project automates the setup and execution of a robotic simulation environment using ROS 2 on a Linux system. The simulation is designed to:
- Detect trailers using image and point-cloud data.
- Estimate the articulation angle between the towing vehicle and the trailer.
- Provide a visual heads-up display (HUD) for real-time monitoring of the vehicle’s state and the articulation angle.

The simulation environment leverages the ROS 2 framework along with Gazebo for realistic, physics-based simulation.

## Prerequisites

Before running the simulation, ensure that the following software and dependencies are installed:

- **ROS 2 Foxy or later**: Required for running the Robot Operating System 2.
- **Gazebo**: A 3D dynamic simulator compatible with ROS 2.
- **Pygame**: Needed for joystick handling in the simulation. Install using `pip install pygame`.
- **Colcon**: The command-line tool for building and bundling ROS 2 packages.
- **A Linux system with GNOME Terminal**: The script utilizes `gnome-terminal` to open multiple terminal windows.

## Setup

1. **Clone the repository**:
   ```bash
   git clone https://github.com/RlNZLER/trailer_adas.git
   cd <repository_directory>
   ```

2. **Install dependencies**:
   Ensure all necessary ROS 2 and Gazebo dependencies are installed. Refer to the official [ROS 2 installation guide](https://docs.ros.org/en/humble/index.html) for setting up your environment.

3. **Add necessary texture files to Gazebo**:

   Copy the custom material script file to the Gazebo materials directory:

   ```bash
   sudo cp /home/<your_user_name>/trailer_adas/src/trailer_description/materials/scripts/marker.material /usr/share/gazebo-11/media/materials/scripts/
   ```

   Copy the ArUco marker images to the Gazebo textures directory:

   ```bash
   sudo cp /home/<your_user_name>/trailer_adas/src/trailer_description/materials/textures/m*_marker.png /usr/share/gazebo-11/media/materials/textures/
   ```

4. **Build the ROS 2 packages**:
   Navigate to the root of your workspace and run:
   ```bash
   colcon build
   ```

## Launching the Simulation

To start the simulation, run the provided Python launch script. This script will automatically set up and launch the necessary ROS 2 nodes and the Gazebo environment:

```bash
python3 trailer_launch.py
```

### Script Details

- **Joystick Detection**: The script first checks if a joystick is connected to the system, as it is required for controlling the vehicle in the simulation.
- **Building the Workspace**: The script initiates a `colcon build` to ensure all ROS 2 packages are up to date.
- **Launching Simulation and Nodes**: The script sequentially launches the Gazebo simulation, the controller for the trailer, the articulation angle estimation nodes, and the HUD for real-time monitoring.

## Troubleshooting

- **Joystick Not Detected**: Ensure that a joystick is connected before running the script. The simulation will not proceed without detecting a joystick.
- **Simulation Delays**: If the simulation nodes are not starting correctly, try increasing the `time.sleep()` durations in the script to allow more time for each node to initialize.
- **Terminal Issues**: The script uses `gnome-terminal`. If you are using a different terminal or desktop environment, you may need to modify the script to use an appropriate terminal command.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

This revised README provides clear instructions and additional details to help users set up, run, and troubleshoot your ROS 2-based simulation for trailer detection and articulation angle estimation.
