# panda_simulation (work-in-progress)

Repository for the dynamic simulation of the Franka Emika Panda robot arm in Gazebo. It contains 4 packages:

- franka_description: it contains the description of the robot
- franka_gazebo: definition of the world with aruco markers
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
Once inside the image, clone this resopitory in *your_catkin_ws/src* and then *catkin_make*. Launch the file *simulation.launch*:

`user@computer:~$ cd your_catkin_ws/` <br /> 
`user@computer:~$ source devel/setup.bash` <br /> 
`user@computer:~$ roslaunch panda_simulation simulation.launch` 
