# panda_simulation (work-in-progress)

## Current progress (DONE)
- Spawn panda robot arm with dynamic description and jointEffortControllers
- Spawn camera Gazebo sensor plugin and ArUco marker 7 for camera pose estimation
- Free-energy minimisation using proprioceptive and visual sensors
- Active inferencve control
- Model reference adaptive control
- Fault detection for camera faults with offline threshold

## TODO
- Camera calibration (now the sensor is simulated using noisy data from the DK)
- Online Fault detection (for these we need the dynamic model of the robot)

## Description

Repository for the dynamic simulation of the Franka Emika Panda robot arm in Gazebo using Active Inference. This repository contains 4 packages:

- *franka_description*: it contains the description of the robot
- *panda_control*: the package contains the active inference controller and a model reference adaptive controller for comparison
- *panda_simulation*: package for launching the simulation loading a jointEffortController for each joint
- *aruco_ros*: ROS wrapper for using ArUco libraries

The packages *franka_description* and *panda_simulation* are a modified versions of the following:

- official franka_ros: https://github.com/frankaemika/franka_ros
- panda_simulation: https://github.com/erdalpekel/panda_simulation

The wrapper fot the ArUco library is cloned from:
- aruco_ros ROS Wiki: wiki.ros.org/aruco_ros

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
- Clone the repository `$ git clone https://github.com/cpezzato/panda_simulation.git src` <br /> 
- Build the workspace: `$ catkin_make` <br /> 
- Source: `$ source devel/setup.bash` <br /> 
- Launch: `$ roslaunch panda_simulation simulation.launch`

The launch file launches a Gazebo simulation in pause. You can then run the node *panda_control_AIC* for the active inference controller (AIC) and play the simulation to see the robot moving to the set-point. Alternatively one can run the model reference adaptive controller (MRAC) through the node *panda_control_MRAC*. 

- `$ rosrun panda_control panda_control_AIC` 
- Then *play* in the Gazebo GUI

The result with any of the two controllers is the following:
<p align="center">
<img src="https://user-images.githubusercontent.com/49310726/56719843-cf4d5e00-6741-11e9-8a62-ed898c4ddee4.png" width="326" height="219">
</p>
