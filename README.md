# Active Object Tracking and Autonomous Positioning Using Duckiedrone

*A comprehensive two-phase approach to autonomous drone systems, bridging the gap between simulation and real-world implementation.*

---

## 🎯 Project Overview

This project addresses autonomous drone systems through a systematic two-phase development strategy:

## Phase 1: Simulation-based Vehicle Tracking

Using YOLOv5s in ROS-Gazebo environment

**Code and documentation available in the `simulation/` folder**

---

## Phase 2: Real-world Hardware Implementation

Using Duckiedrone DD21 platform with AprilTag markers

**Code and documentation available in the `DuckieDrone/` folder**

---

## 🚁 System Architecture

### 🔹 Simulation Phase
- **Environment**: ROS Noetic + Gazebo 11  
- **Drone Model**: Iris ArduPilot quadcopter  
- **Object Detection**: YOLOv5s with TensorRT optimization  
- **Control System**: Custom PID controllers  
- **Performance**: Real-time processing at 30 FPS  

### 🔸 Real-World Phase
- **Platform**: Duckiedrone DD21  
- **Navigation**: AprilTag-based positioning  
- **Computing**: Raspberry Pi 3  
- **Vision**: Real-time marker detection and pose estimation  

---

## 📊 Performance Results

### 🧪 Simulation Results

| Scenario              | FPS | Tracking Success (%) |
|-----------------------|-----|----------------------|
| Static Object         | 32  | 100                  |
| Moving Object (Slow)  | 30  | 95.0                 |
| Moving Object (Fast)  | 28  | 85.0                 |
| Occlusion Scenarios   | 30  | 75.0                 |
| **Average**           | **29** | **88.5**         |

### 🌐 Real-World Results

| Parameter                   | Target   | Achieved |
|----------------------------|----------|----------|
| Hover Stability            | 300s     | 45-60s   |
| AprilTag Detection Range   | 5m       | 3.2m     |
| Processing Frame Rate      | 30 FPS   | 18 FPS   |
| Flight Time                | 30 min   | 20 min   |
| Position Accuracy          | 10 cm    | 35 cm    |

---



