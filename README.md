# OgmenTasks
# Bot Description Package

A simple differential drive robot model with LIDAR and camera sensors.

## Setup

1. Install dependencies:
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-ros-gz
```

2. Build the package:
```bash
cd ~/your_workspace
colcon build --packages-select bot_description
source install/setup.bash
```

## Launch

### RViz Visualization:
```bash
ros2 launch bot_description rviz.launch.py
```
![Screenshot from 2025-04-18 13-46-19](https://github.com/user-attachments/assets/07175f19-296e-41ee-8472-a9868b4842c8)


### Gazebo Simulation:
```bash
ros2 launch bot_description spawn.launch.py
```
Note : The robot will spawn in an empty world currently and I am in the process of setting up the environment.
![Screenshot from 2025-04-18 13-47-41](https://github.com/user-attachments/assets/33fb6059-4a54-4270-857c-ffc27e96a31e)


## Control (IN PROGRESS)

Move the robot with:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.1}}" --once
``


