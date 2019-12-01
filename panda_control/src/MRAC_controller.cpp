/*
 * File:   MRAC_controller.cpp
 * Author: Corrado Pezzato, TU Delft, DCSC
 *
 * Created on April 14th, 2019
 *
 * This node allows to control the 7DOF Franka Emika Panda robot arm through
 * the new promisin theory called Active Inference proposed by Karl Friston.
 *
 * The robot moves to the desired position specified in desiredPos performing
 * free-energy minimization and actiove inference using gradient descent.
 * The robot is equipped with proprioceptive sensors for joints position and
 * velocity and a camera for the end-effector pose estimation.
 * The control is in joint space.
 *
 */

#include "MRAC.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "MRAC_controller_node");
  // Variables to regulate the flow (Force to read once every 1ms the sensors)
  int count = 0;
  int cycles = 0;
  // Variable for desired position, set here the goal for the Panda for each joint
  std::vector<double> desiredPos1(7), desiredPos2(7), desiredPos3(7);

  desiredPos1[0] = 1;
  desiredPos1[1] = 0.5;
  desiredPos1[2] = 0.0;
  desiredPos1[3] = -2;
  desiredPos1[4] = 0.0;
  desiredPos1[5] = 2.5;
  desiredPos1[6] = 0.0;

  desiredPos2[0] = 0.0;
  desiredPos2[1] = 0.2;
  desiredPos2[2] = 0.0;
  desiredPos2[3] = -1.0;
  desiredPos2[4] = 0.0;
  desiredPos2[5] = 1.2;
  desiredPos2[6] = 0.0;

  desiredPos3[0] = -1;
  desiredPos3[1] = 0.5;
  desiredPos3[2] = 0.0;
  desiredPos3[3] = -1.2;
  desiredPos3[4] = 0.0;
  desiredPos3[5] = 1.6;
  desiredPos3[6] = 0;


  // Object of the class MRAC which will take care of everything
  MRAC MRAC_controller;
  // Set desired position in the MRAC class
  MRAC_controller.setGoal(desiredPos1);

  // Main loop
  // Main loop
  ros::Rate rate(1000);
  while (ros::ok()){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();

    // Skip only first cycle to allow reading the sensory input first
    if ((count!=0)&&(MRAC_controller.dataReady()==1)){
      MRAC_controller.computeControlInput();
      cycles ++;
      if (cycles == 3000){
        MRAC_controller.setGoal(desiredPos2);
      }

      if (cycles == 6000){
        MRAC_controller.setGoal(desiredPos3);
      }

      if (cycles == 9000){
        MRAC_controller.setGoal(desiredPos2);
      }

      if (cycles == 12000){
        MRAC_controller.setGoal(desiredPos1);
        cycles = 0;
      }
    }
    else
      count ++;

    rate.sleep();
  }
  return 0;
}
