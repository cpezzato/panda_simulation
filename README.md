# panda_simulation (work-in-progress)

## Current progress (DONE)
- Spawn panda robot arm with dynamic description and jointEffortControllers
- Spawn camera Gazebo sensor plugin and ArUco marker 7 for camera pose estimation
- Free-energy minimisation using only proprioceptive sensors
- Active inferencve usingonly proprioveptive sensors

## TODO
- Fine tuning of the controller
- Include the camera input for state estimation 
- Add fault detection

## Description

Repository for the dynamic simulation of the Franka Emika Panda robot arm in Gazebo using Active Inference. This repository contains 5 packages:

- *franka_description*: it contains the description of the robot
- *franka_gazebo*: definition of the world with aruco markers
- *panda_control*: the package that will contain the active inference controller (now the AIC_controller contains just free-energy minimization)
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

### Download aruco marker
Download ArUco markers from here (only if you intend to use the *aruco_ros* wrapper):

https://github.com/cpezzato/aruco_marker_gazebo

and put them inside ./gazebo/models (usually in your home directory)

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

The launch file launches a Gazebo simulation in pause. You can then run the node panda_control_AIC and play the simulation to see the robot moving to the set-point. 

- `$ rosrun panda_control panda_control_AIC` 
- Then *play* in the Gazebo GUI

The result is the following:
<p align="center">
<img src="https://user-images.githubusercontent.com/49310726/56133236-3887e100-5f8c-11e9-9ed9-d51f2dd8f834.png" width="410" height="386">
</p>
