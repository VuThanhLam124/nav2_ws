# Project Description
## Make sure that your enviroment is python 3.10, have ROS2 and your Ubuntu version is 22.04
## Overview:
This project integrates SICNav (Safe Interactive Crowd Navigation) with ROS2 NAV2 using Nav2_PYIF to enable safe and reinforcement learning-based navigation for TurtleBot3 in a Gazebo simulation. SICNav provides various modules—such as those for crowd simulation and safety functions—allowing you to build custom controllers. However, the current setup does not yet integrate specific modules like DWA, CAMPc, or RL_nav. You are expected to develop your own controller in ~/sicnav_controller/sicnav_controller that leverages SICNav’s building blocks to process sensor data (from LiDAR and/or camera) and generate safe velocity commands.

## Key Features:

### SICNav Integration:
SICNav is structured as a framework that provides various modules (e.g., RL_nav for reinforcement learning-based methods, crowd_sim_plus for crowd simulation, and sicnav for additional safety functions). The framework is not a complete application on its own. Instead, you must develop a custom controller to integrate these modules based on your needs.

### Nav2_PYIF Controller:
Using Nav2_PYIF, you can implement your controller in Python. This controller will receive sensor data (from LiDAR, camera, or both), process it using functions and algorithms from SICNav, and compute safe velocity commands to be used by NAV2.

### Simulation Environment:
The project is configured to work with the TurtleBot3 robot in a Gazebo simulation. This setup allows you to test and tune your navigation algorithms without deploying directly on hardware.

## Project Objectives:
https://github.com/VuThanhLam124/nav2_ws.git
Develop a safe navigation system for TurtleBot3 in a simulated environment.
Leverage the building blocks provided by SICNav to implement custom control strategies.
Provide a flexible framework where reinforcement learning-based navigation and safe control policies can be integrated.

## IMPORTANT:
- You need to create your controller file with your custom algorithm in ~/sicnav_controller/sicnav_controller.
- Currently, the project does not run with the DWA or CAMPc modules from SICNav, nor does it integrate the RL_nav module.
- You are expected to extend and integrate the provided building blocks to meet your navigation requirements.


# 1. Clone the Required Repositories and setup project step by step
```bash
git clone https://github.com/VuThanhLam124/nav2_ws.git
cd nav2_ws/src
git clone https://github.com/DanelLepp/nav2_pyif.git
git clone https://github.com/sybrenstuvel/Python-RVO2.git
git clone https://github.com/sepsamavi/safe-interactive-crowdnav.git
```
Here, replace requirements.txt file and setup.py to src/safe-interactive-crowdnav, then:
```bash
cd ~/nav2_ws/src/safe-interactive-crowdnav/
pip install -e .
pip install -r requirements.txt
```
```bash
cd ~/nav2_ws/src/Python-RVO2
pip install -e .
```
```bash
cd ~/nav2_ws/
ros2 pkg create --build-type ament_python sicnav_controller
```
# 2. Build the project
```bash
colcon build
source install/setup.bash
```
# 3. Run with your params file(can modify by your self)
```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
source /opt/ros/humble/setup.bash
cd ~/nav2_ws
colcon build --symlink-install
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True \
  params_file:=~/nav2_pyif_ws/src/nav2_pyif/sicnav_python_example/params2.yaml
```
