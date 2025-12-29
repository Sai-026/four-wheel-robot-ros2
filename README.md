# Four Wheel Robot

ROS2 Humble package for 4-wheel robot simulation.

## Features
- Gazebo Classic 11 simulation
- RViz visualization

## Installation
```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/four_wheel.git
cd ~/ros2_ws
colcon build --packages-select four_wheel
source install/setup.bash
```

## Usage
```bash
# Launch Gazebo only
ros2 launch four_wheel four_wheel_gazebo.launch.py

# Launch Gazebo + RViz
ros2 launch four_wheel four_wheel_combined.launch.py

# Control robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Requirements
- ROS2 Humble
- Gazebo Classic 11
- Ubuntu 22.04
# four-wheel-robot-ros2
