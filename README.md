# ROS 2 PID Inverted Pendulum

A simulation and control of an **inverted pendulum** using **ROS 2 Jazzy**, `ros2_control`, and a custom PID controller, running in **Gazebo Harmonic**.  
This project demonstrates real-time control integration in ROS 2 with physics simulation for robotics control applications.

---

## 🚀 Features

- ✅ ROS 2 Jazzy support
- ✅ Custom PID controller node in Python
- ✅ `ros2_control` hardware interface integration
- ✅ URDF description for the pendulum model
- ✅ Gazebo Harmonic simulation support
- ✅ Real-time parameter tuning

---

## 📦 Prerequisites

Before building, make sure you have:

- **ROS 2 Jazzy** installed
- **Gazebo Harmonic** installed
- `ros2_control` and `ros2_controllers` packages
- `colcon` build tool
- Python 3.x

---

## 🔧 Installation

Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/BlackkBeardd/ros2_pid_inverted_pendulum.git
cd ..
colcon build --symlink-install
source install/setup.bash

## 🔧 Usage

ros2 launch pendulum_control_description pendulum_controller.launch.py
ros2 launch pid_controller pid_controller.launch.py

## 🔧 Sending commands
ros2 topic pub /joint_setpoint std_msgs/msg/Float64 'data: {desired_joint_position}'
```
