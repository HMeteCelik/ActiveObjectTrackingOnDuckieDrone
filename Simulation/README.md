# Simulation-Based Vehicle Tracking Using YOLOv5s in ROS-Gazebo

This module implements autonomous vehicle tracking in a simulated environment using YOLOv5s for object detection within the ROS Noetic and Gazebo 11 framework.

---

## Features

- Real-time object detection using YOLOv5s with TensorRT optimization  
- Custom PID controllers for drone navigation  
- Simulation with realistic physics and sensor models  
- Supports static, slow-moving, and fast-moving target scenarios  
- Handles occlusion events with graceful tracking degradation  

---

## Getting Started

### Prerequisites

- ROS Noetic  
- Gazebo 11  
- Python 3 with the following packages:
  - torch, torchvision, torchaudio  
  - ultralytics  
  - opencv-python  
- ArduPilot SITL
- MAVROS

### Usage 🚁


### Running the Simulation

Follow these steps in order to start the complete simulation environment:

#### 1. Terminal: Start ROS Master
```bash
roscore
```
#### 2. Terminal: Launch Gazebo Simulation
```bash
gazebo Simulation/world/simulation_world.world
```
#### 3. Terminal: Start ArduPilot SITL
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```
#### 4. Terminal: Launch MAVROS Communication Node
```bash
roslaunch mavros apm.launch fcu_url:=udp://:14550@
```
#### 5. Terminal: Run the Tracking Script
```bash
rosrun <package> track.py
```
#### 6. Terminal: Start Object Movement to be Tracked
```bash
python Simulation/scripts/test.py
```


