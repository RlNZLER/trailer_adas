#!/usr/bin/env python3
########################################################################################################################################################
# TRAILER LAUNCH
#
# This Python script automates the process of setting up and launching a robotic simulation environment using ROS 2 (Robot Operating System 2) 
# on a Linux system. The script utilizes the gnome-terminal command to open multiple terminal windows and execute different ROS 2 commands sequentially.
#
########################################################################################################################################################

# Time module for introducing delays and subprocess module for running terminal commands from the script.
import time
import subprocess
import pygame

def run_command_in_terminal(command):
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'{command}; exec bash'])

def check_joystick_connected():
    pygame.init()  # Initialize pygame
    joystick_count = pygame.joystick.get_count()  # Get the number of connected joysticks
    if joystick_count == 0:
        print("No joystick detected. Please connect a joystick to continue.")
        pygame.quit()
        exit(1)  # Exit if no joystick is detected
    else:
        print(f"{joystick_count} joystick(s) detected.")

def main():
    check_joystick_connected()  # Check for joystick before proceeding

    # Command 1: colcon build.
    run_command_in_terminal('colcon build')
    time.sleep(5)

    # Command 2: launch the trailer Gazebo simulation.
    run_command_in_terminal('ros2 launch trailer_description gazebo.launch.py')
    time.sleep(15)  # Wait for simulation to start.

    # Command 4: launch the trailer controller.
    run_command_in_terminal('ros2 launch trailer_controller controller.launch.py')
    time.sleep(10)

    # Command 5: launch the trailer articulation angle estimation nodes.
    run_command_in_terminal('ros2 launch trailer_articulation_angle art_angle_est.launch.py')

    # Command 6: launch the trailer HUD.
    run_command_in_terminal('ros2 launch trailer_hud hud.launch.py')

if __name__ == "__main__":
    main()
