# DuckieDrone Autonomous Control System

This repository contains the autonomous flight software stack developed for the DuckieDrone DD21 platform. The system includes modules for hybrid localization, PID control, and state estimation using both AprilTag-based vision and onboard sensors.

## 📦 Prerequisites

Before using the code in this repository, complete the full DuckieDrone system setup as described in the official Duckietown documentation:

🔗 [Duckiedrone Setup Guide (Daffy)](https://docs.duckietown.com/daffy/opmanual-duckiedrone/intro.html)

---

## ⚙️ CMake Update

DuckieDrone's default CMake version may be too old for building some dependencies. Use the following instructions to update CMake to version `3.25.2`:

```bash
cd ~
wget https://github.com/Kitware/CMake/releases/download/v3.25.2/cmake-3.25.2.tar.gz
tar -zxvf cmake-3.25.2.tar.gz
cd cmake-3.25.2
./bootstrap
make -j4
sudo make install
echo 'export PATH=~/cmake-3.25.2-install/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```

---

## 🧿 AprilTag ROS Installation

AprilTag-based localization is a key component of this system. Install the `apriltag_ros` package as follows:

🔗 [AprilTag ROS Repository](https://github.com/AprilRobotics/apriltag_ros)

```bash
cd ~/catkin_ws/src
git clone https://github.com/AprilRobotics/apriltag_ros.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

> 📌 AprilTag Parameters Used in This Project:
> - Tag Family: `tag36h11`
> - Tag ID: `0`
> - Tag Size: `0.08` meters

---

## 🛫 Deployment on Drone

Once the environment is set up and all dependencies are installed, copy the control and localization modules to the drone:

| File Name                            | Replaces Drone File                 | Description                            |
|-------------------------------------|-------------------------------------|----------------------------------------|
| `flight_controller.py`              | `flight_controller.py`              | Receives RC commands and sends them to motors safely |
| `pid_controller_hybrid.py`          | `pid_controller.py`                 | Computes RC commands using hybrid localization |
| `hybrid_localizator.py`             | `state_estimator.py`                | Hybrid localization (AprilTag + OF)    |
| `pid_class.py`                      | `pid_class.py`                      | PID class definitions and logic        |

Transfer these scripts to the appropriate location on the drone (`~/catkin_ws/src/pidrone_pkg/scripts/`), replacing the existing files with the same names.

---

## 🚀 Running the System

1. Ensure that the drone is connected and powered.
2. Start the necessary ROS core and AprilTag detection node.
3. Run `hybrid_localizator.py` to obtain position estimates (replaces `state_estimator.py`).
4. Run both:
   - `flight_controller.py`: Handles motor command forwarding and safety constraints.
   - `pid_controller_hybrid.py`: Computes control commands (RC) using AprilTag + sensor data.
5. Monitor output topics and control response in RViz or with `rostopic echo`.

**Note:** `pid_controller_hybrid.py` does not control motors directly; it publishes RC commands, which are then interpreted and transmitted to the motors by `flight_controller.py`.

---

## 📌 Notes

- Ensure camera calibration is properly configured before flying.
- AprilTag size and frame names should match between launch files and code.
- For safety, test in simulation or with propeller guards.

---

## 📁 Directory Overview

```text
.
├── flight_controller.py
├── hybrid_localizator.py
├── pid_controller_hybrid.py
├── pid_class.py
├── state_estimator_apriltag.py
└── README.md
```

---

## 📞 Contact

For further assistance or issues, please contact the project maintainers or refer to Duckietown community support.
