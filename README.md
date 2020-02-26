# Active inference for robot manipulators. Simulation using Franka Emika Panda 7-DOF

### How to cite this work
This simulation was developed to support the theoretical results of this work:

- Pezzato C., Ferrari, R., Henrandez C., *A Novel Adaptive Controller for Robot Manipulators Based on Active Inference*, IEEE Robotics and Automation Letters, 2020.

If you found this controller useful, please consider citing the paper above.

## News
### Active inference for the real setup
The code for controlling the real Panda using either libfranka or franka_ros is now available at https://github.com/cpezzato/active_inference

## Description package for simulation

Repository for the dynamic simulation of the Franka Emika Panda robot arm in Gazebo using Active Inference. This repository contains 3 packages:

- *franka_description*: it contains the urdf and xacro of the robot and the scene
- *panda_control*: the package contains the active inference controller and a model reference adaptive controller for comparison
- *panda_simulation*: package for launching the simulation loading a jointEffortController for each joint and the scene with the robot(s)

The packages *franka_description* and *panda_simulation* are a modified versions of the following:

- official franka_ros: https://github.com/frankaemika/franka_ros
- panda_simulation: https://github.com/erdalpekel/panda_simulation

Note that the *panda_simulation* package has been modified to allow to spawn multiple robots with different namespaces. Furthermore the *panda_arm.xacro* contains now a model of the robot with approximated dynamics. The file *panda_arm.xacro* has been updated with more accurate inertia tensor and centers of mass obtained using Meshlab and the .stl models.

## Installation

Make sure you installed *ros-kinetic-desktop-full* and that you have all the necessary dependencies. You can do that building a singularity image from this singulatity recipe:

https://github.com/cpezzato/panda-xenial-docker/blob/master/panda-recipe-minimal-xenial

### How to use
Run a shell within the container:
- With Nvidia GPU: `$ singularity shell --nv YOUR_IMAGE_NAME`
- Without Nvidia GPU: `$ singularity shell YOUR_IMAGE_NAME`

Once inside the image:

- Source: `$ source /opt/ros/kinetic/setup.bash`
- Create a folder for your catkin_ws: `$ mkdir -p your_catkin_ws/` <br />
- Move to the folder: $ cd your_catkin_ws/` <br />
- Clone the repository `$ git clone https://github.com/cpezzato/panda_simulation.git src` <br />
- Build the workspace: `$ catkin_make` <br />
- Source: `$ source devel/setup.bash` <br />
- Launch: `$ roslaunch panda_simulation simulation.launch`

The launch file launches a Gazebo simulation in pause with a single robot. The launch file run the node *panda_control_AIC* for the active inference controller (AIC) by default. Alternatively one can run the model reference adaptive controller (MRAC) through the node *panda_control_MRAC*, modifying the launch file accordingly. Just press play and enjoy.

The result with the active inference controller is the following:

![AIC](https://user-images.githubusercontent.com/49310726/56992707-f02b0e80-6b9a-11e9-99fd-58a31f114d0e.gif)
