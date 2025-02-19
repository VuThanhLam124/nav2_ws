# Project Description
## Overview:
This project integrates SICNav—a framework providing safe navigation building blocks—with ROS2 NAV2 via Nav2_PYIF. The system is designed for TurtleBot3 in a Gazebo simulation environment, aiming to achieve safe and effective navigation using reinforcement learning techniques.

Key Features:

SICNav Integration:
SICNav is structured as a framework that provides various modules (e.g., RL_nav for reinforcement learning-based methods, crowd_sim_plus for crowd simulation, and sicnav for additional safety functions). The framework is not a complete application on its own. Instead, you must develop a custom controller to integrate these modules based on your needs.

Nav2_PYIF Controller:
Using Nav2_PYIF, you can implement your controller in Python. This controller will receive sensor data (from LiDAR, camera, or both), process it using functions and algorithms from SICNav, and compute safe velocity commands to be used by NAV2.

Simulation Environment:
The project is configured to work with the TurtleBot3 robot in a Gazebo simulation. This setup allows you to test and tune your navigation algorithms without deploying directly on hardware.

Project Objectives:

Develop a safe navigation system for TurtleBot3 in a simulated environment.
Leverage the building blocks provided by SICNav to implement custom control strategies.
Provide a flexible framework where reinforcement learning-based navigation and safe control policies can be integrated.

IMPORTANT:
- You need to create your controller file with your custom algorithm in ~/sicnav_controller/sicnav_controller.
- Currently, the project does not run with the DWA or CAMPc modules from SICNav, nor does it integrate the RL_nav module.
- You are expected to extend and integrate the provided building blocks to meet your navigation requirements.


# 1. Clone the Required Repositories
```bash
git clone https://github.com/your_username/nav2_ws.git
cd nav2_ws
```
# 2. Build the project
```bash
colcon build
source install/setup.bash
```
# 3. Run with your params file(can modify by your self)
```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup navigation_launch.py params_file:=~/nav2_ws/sicnav_controller/params.yaml
```
