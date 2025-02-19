
#Introduction
This project integrates SICNav (Safe Interactive Crowd Navigation) with NAV2 using Nav2_PYIF. The goal is to enable reinforcement learning-based navigation for the TurtleBot3 robot in a Gazebo simulation.

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
