# Adeept Robot GPS Tracking Project

Autonomous robot system using an external phone camera and ArUco markers for positioning and object retrieval.

## Architecture

The project is divided into two parts:
* `pc_workspace/`: Code running on the master computer. Handles computer vision, path planning, and sends commands.
* `robot_workspace/`: Code running on the Raspberry Pi. Receives commands, controls hardware, and handles local obstacle avoidance.

## Setup Instructions

Create a virtual environment on your local machine
Install requirements for the PC workspace
Run the calibration script using your phone camera
Start the TCP server on the robot
Run the main control script on the PC