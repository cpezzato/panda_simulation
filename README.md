# panda_simulation

Repository for the dynamic simulation of the Franka Emika Panda robot arm in Gazebo. It contains two packages:

- franka_description: it contains the description of the robot
- panda_simulation: package for launching the simulation loading a jointEffortController for each joint

The packages are a modified versions of the following:

- official franka_ros:
- panda_simulation: 

## Installation

Make sure you installed ros-melodic-desktop-full and that you have all the necessary packages:

  `user@computer:~$ apt-get install -y ros-melodic-controller-manager* ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-rviz* libboost-filesystem-dev libjsoncpp-dev
` 
Clone this resopitory in *your_catkin_ws/src* and then *catkin_make*. 

## How to use
Launch the file *simulation.launch*:

`user@computer:~$ cd your_catkin_ws/` 
`user@computer:~$ source devel/setup.bash` 
`user@computer:~$ roslaunch panda_similation simulation.launch` 
