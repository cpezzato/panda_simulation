# panda_simulation (work-in-progress)

## Current progress (DONE)
- Spawn panda robot arm with dynamic description and jointEffortControllers
- Spawn camera Gazebo sensor plugin and ArUco marker 7 for camera pose estimation
- Free-energy minimisation using only proprioceptive sensors

## TODO
- Include the camera in the free-energy minimization 
- Develop node for the robot control, at the moment no controller is active and the robot falls on the ground, only free-energy minimization is implemented

Repository for the dynamic simulation of the Franka Emika Panda robot arm in Gazebo. It contains 5 packages:

- franka_description: it contains the description of the robot
- franka_gazebo: definition of the world with aruco markers
- panda_control: the package that will contain the active inference controller (now the AIC_controller contains just free-energy minimization)
- panda_simulation: package for launching the simulation loading a jointEffortController for each joint
- aruco_ros: ROS wrapper for using ArUco libraries

The packages are a modified versions of the following:

- official franka_ros: https://github.com/frankaemika/franka_ros
- panda_simulation: https://github.com/erdalpekel/panda_simulation

## Installation

Make sure you installed ros-kinetic-desktop-full and that you have all the necessary dependencies. You can do that building a singularity image from this singulatity recipe:

https://github.com/cpezzato/panda-xenial-docker/blob/master/panda-recipe-minimal-xenial 

### Download aruco marker
Download ArUco markers from here:

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

The launch file has the GUI disable since this is a development phase. Modified the value to "true" in the launch file so see what is going on. You can then run the node panda_control_AIC to see the free-energy minimization printed out:

-`$ rosrun panda_control panda_control_AIC` 
