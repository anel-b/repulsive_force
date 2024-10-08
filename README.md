# Repulsive Force Computation Package in ROS 2 Humble with franka_ros2, ZED Python API, Open3D and scikit-learn

## Description

A ROS 2 Humble package that computes a repulsive force from the Artificial Potential Field Method on the end effector of a Franka Research 3 robot arm, pushing it away from the nearest obstacle in the workspace using point cloud data from a Stereolabs ZED stereo camera.

### Package Contents of `repulsive_force`:

**2 Nodes:**

* **repulsive_force_zed.py**
   * Computes repulsive force from nearest point in point cloud from ZED camera relative to end effector and publishes it.
* **repulsive_force_ply.py**
   * Same as **repulsive_force_zed.py** but with point cloud from .ply file.

**Controller modification ([source](https://github.com/CurdinDeplazes/cartesian_impedance_control)):**

* **cartesian_impedance_controller.hpp**
   * Initialization of required functions and variables.
* **cartesian_impedance_controller.cpp**
   * Subscription to "repulsive_force_topic" topic and including repulsive force in controller loop. Scaling impedance force by 0.5 when repulsive force is not zero and error is greater than 0.1. Function to invert the y coordinate of target position every 5 seconds. Stiffness K was reduced by 20% and damping D was adjusted with D = 2*sqrt(K) accordingly.

**Point cloud data:**

* **point_cloud_data.ply**
   * Point cloud data from workspace as .ply file.

## Installation

### Prerequisites

* Ubuntu 22.04 LTS+ (Linux)
* [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* libfranka 0.13.0+
* [franka_ros2 v0.13.1](https://support.franka.de/docs/franka_ros2.html)
* [cartesian impedance controller from pdz](https://github.com/CurdinDeplazes/cartesian_impedance_control)
* Python 3.7+
* setuptools 58.2.0
* [ZED Python API](https://www.stereolabs.com/docs/app-development/python/install)
* [Open3D 0.18.0](https://www.open3d.org/)
* NumPy <1.25.0 and >=1.17.3
* scikit-learn

### Download

Change your current directory in the terminal to the **src** folder within your **franka_ros2** workspace. Clone the repository into this folder. Then, navigate back to the directory of your **franka_ros2** workspace, build the package with colcon, and source the setup file. These are the commands for the terminal:

```bash
cd ~/franka_ros2_ws/src/
git clone https://github.com/anel-b/repulsive_force.git
cd ~/franka_ros2_ws/
colcon build --packages-select repulsive_force
source ~/franka_ros2_ws/install/setup.bash
```

### Configurations

* **repulsive_force_zed.py**
   * Replace (30635524) with your ZED camera serial number.
   * Replace homogeneous transformation matrices with [camera calibration from pdz](https://github.com/LucasG2001/camera_calibration).
   * Calibrate DBSCAN parameter (eps=0.04) to accurately cluster robot arm silhouette without including obstacles. Parameter scales proportionally with sampling percentage and use add_obstacle function to simulate obstacles.
* **repulsive_force_ply.py**
   * Replace /anyba/ with your PC username.

### Controller

Replace the .hpp and .cpp files from the controller with the modified controller files from this repository. Then build and source the controller package as described in its repository.

Controller modifications are marked with:<br>
//START repulsive_force<br>
\*code\*<br>
//END repulsive_force<br>

## Execution

Start the repulsive force node with a ZED camera:

```bash
ros2 run repulsive_force repulsive_force_zed
```

Start the controller with a Franka Research 3 robot arm:

```bash
ros2 launch cartesian_impedance_control cartesian_impedance_controller.launch.py
```

## Extras

For setting up a ROS 2 package in Python, the following YouTube tutorial playlist can be checked out: [ROS 2 Humble Tutorial in Python](https://www.youtube.com/watch?v=0aPbWsyENA8&list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy)