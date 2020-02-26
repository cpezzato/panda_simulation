/*
 * File:   AIC_controller.cpp
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

#include "AIC.h"

// Constant for class AIC constructor to define which robot to control
const int robot = 1;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AIC_controller_single_node");
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

  // Object of the class AIC which will take care of everything
  AIC AIC_controller(robot);
  // Set desired position in the AIC class
  AIC_controller.setGoal(desiredPos1);

  // Main loop
  ros::Rate rate(1000);
  while (ros::ok()){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();

    // Skip only first cycle to allow reading the sensory input first
    if ((count!=0)&&(AIC_controller.dataReady()==1)){
      AIC_controller.minimiseF();
      cycles ++;
      if (cycles == 6000){
        //AIC_controller.cameraFaultON();
        AIC_controller.setGoal(desiredPos2);
      }

      if (cycles == 12000){
        //AIC_controller.cameraFaultOFF();
        AIC_controller.setGoal(desiredPos3);
      }

      if (cycles == 18000){
        //AIC_controller.cameraFaultOFF();
        AIC_controller.setGoal(desiredPos2);
      }

      if (cycles == 24000){
        //AIC_controller.cameraFaultOFF();
        AIC_controller.setGoal(desiredPos1);
        cycles = 0;
      }
    }
    else
      count ++;

    rate.sleep();
  }
  return 0;
}
