# Active inference for robot manipulators. Simulation using Franka Emika Panda 7-DOF

### How to cite this work
This simulation was developed to support the theoretical results of this work:

- Pezzato C., Ferrari, R., Henrandez C., A Novel Adaptive Controller for Robot Manipulators Based on Active Inference, IEEE Robotics and Automation Letters, 2020.

- @ARTICLE{Pezzato2019, 
author={C. {Pezzato} and R. {Ferrari} and C. H. {Corbato}}, 
journal={IEEE Robotics and Automation Letters}, 
title={A Novel Adaptive Controller for Robot Manipulators Based on Active Inference}, 
year={2020}, 
volume={5}, 
number={2}, 
pages={2973-2980},}

If you found this controller useful, please consider citing the paper above.

## News

### Video
A video showing the capability of active inference in the real world is now on youtube!

https://www.youtube.com/watch?v=Vsb0MzOp_TY

### Active inference for the real setup
The code for controlling the real Panda using either libfranka or franka_ros is now available at https://github.com/cpezzato/active_inference

## Description package for simulation

Repository for the dynamic simulation of the Franka Emika Panda robot arm in Gazebo using Active Inference. This repository contains 3 packages:

- *franka_description*: it contains the urdf and xacro of the robot and the scene
- *panda_control*: the package contains the active inference controller and a model reference adaptive controller for comparison
- *panda_simulation*: package for launching the simulation loading a jointEffortController for each joint and the scene with the robot(s)

The file *panda_arm.xacro* has been updated with more accurate inertia tensor and centers of mass obtained using Meshlab and the .stl models. See also https://github.com/erdalpekel/panda_simulation

## Installation

Make sure you installed *ros-kinetic-desktop-full* and that you have all the necessary dependencies. 

### How build a container with the necessary dependancies
If you want, you can install Singularity and build a container for this simulation. You can do that building a singularity image from this singulatity recipe:

https://github.com/cpezzato/panda-xenial-docker/blob/master/panda-recipe-minimal-xenial

Run a shell within the container:
- With Nvidia GPU: `$ singularity shell --nv YOUR_IMAGE_NAME`
- Without Nvidia GPU: `$ singularity shell YOUR_IMAGE_NAME`

Once inside the image:

- Source: `$ source /opt/ros/kinetic/setup.bash`

### How to run the simulation
Either in your local machine or inside the image you just created:

- Create a folder for your catkin_ws: `$ mkdir -p your_catkin_ws/` <br />
- Move to the folder: $ cd your_catkin_ws/` <br />
- Clone the repository `$ git clone https://github.com/cpezzato/panda_simulation.git src` <br />
- Build the workspace: `$ catkin_make` <br />
- Source: `$ source devel/setup.bash` <br />
- Launch: `$ roslaunch panda_simulation simulation.launch`

The launch file launches a Gazebo simulation in pause with a single robot. The launch file run the node *panda_control_AIC* for the active inference controller (AIC) by default. Alternatively one can run the model reference adaptive controller (MRAC) through the node *panda_control_MRAC*, modifying the launch file accordingly. Just press play and enjoy.

The result with the active inference controller is the following:

![AIC](https://user-images.githubusercontent.com/49310726/56992707-f02b0e80-6b9a-11e9-99fd-58a31f114d0e.gif)

### Troubleshooting
If you are using ROS Melodic and Gazebo 9, the robot meshes will probably not be showed. To solve this, just remove the "transparent" attribute in the .dae flesin *franka_description/meshes/visual*
