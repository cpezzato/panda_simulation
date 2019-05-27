# panda_simulation (work-in-progress)

## Current progress (DONE)
- Spawn panda robot arm with dynamic description and jointEffortControllers
- Spawn camera Gazebo sensor plugin and ArUco marker 7 for camera pose estimation
- Free-energy minimisation using proprioceptive and visual sensors
- Active inferencve control
- Model reference adaptive control
- Fault detection and recovery for camera faults with online threshold
- More realistic camera simulation from datasheet of ensenso camera + barrel distortion

## TODO
- Clean up the code and run tests

## Description

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
- Launch: `$ roslaunch panda_simulation simulation_single.launch`

The launch file launches a Gazebo simulation in pause with a single robot. You can then run the node *panda_control_AIC_single* for the active inference controller (AIC) and play the simulation to see the robot moving to the set-point. Alternatively one can run the model reference adaptive controller (MRAC) through the node *panda_control_MRAC*. 

- `$ rosrun panda_control panda_control_AIC_single` 
- Then *play* in the Gazebo GUI

The result with the active inference controller is the following:

![AIC](https://user-images.githubusercontent.com/49310726/56992707-f02b0e80-6b9a-11e9-99fd-58a31f114d0e.gif)

