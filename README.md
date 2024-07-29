# Repulsive force computation package in ROS 2 Humble with Open3D library

## Description

ROS 2 Humble package that computes a repulsive force on a robot arm's end effector, pushing it away from the nearest obstacle using point cloud data.

### Containment of "repulsive_force" package:

3 Nodes:<br>
* f_repulsion.py<br>(computes repulsive force from nearest point in point cloud from .ply file relative to end effector and publishes it)
* f_repulsion_v2.py<br>(same as f_repulsion.py but with point cloud from ZED camera)
* f_repulsion_v3.py<br>(same as f_repulsion_v2.py but without colors from point cloud)

Controller modified ([source](https://github.com/CurdinDeplazes/cartesian_impedance_control)):<br>
* cartesian_impedance_controller.hpp<br>(declares needed functions and variables)
* cartesian_impedance_controller.cpp<br>(subscribes to the f_repulsion topic and includes the repulsive force in the controller loop)

Point Cloud Data:<br>
* pc_workspace.ply<br>(point cloud data as .ply file)

## Installation

### Prerequisites

* Ubuntu (Linux)
* [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* libfranka 0.13.0+
* [franka_ros2 v0.13.1](https://support.franka.de/docs/franka_ros2.html)
* [cartesian impedance controller from pdz](https://github.com/CurdinDeplazes/cartesian_impedance_control)
* Python 3.7+
* setuptools 58.2.0
* [ZED SDK](https://www.stereolabs.com/developers/release)
* [ZED Python API](https://www.stereolabs.com/docs/app-development/python/install)
* [Open3D 0.18.0](https://www.open3d.org/)
* scikit-learn
* NumPy <1.25.0 and >=1.17.3

### Download

```bash
cd ~/franka_ros2_ws/src
git clone https://github.com/anel-b/repulsive_force.git
cd ~/franka_ros2_ws
colcon build --package-select repulsive_force
source ~/franka_ros2_ws/install/setup.bash
```

### Controller

Replace the .hpp and .cpp files from the controller with the modified controller files from this repository

## Extras

[ROS 2 Humble Tutorial in Python](https://www.youtube.com/watch?v=0aPbWsyENA8&list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy)