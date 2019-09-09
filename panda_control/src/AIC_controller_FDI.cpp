/*
 * File:   AIC_realRobot.cpp
 * Author: Corrado Pezzato, TU Delft, DCSC
 *
 * Created on April 14th, 2019
 *
 * This node allows to control the 7DOF Franka Emika Panda robot arm through
 * the new promisin theory called Active Inference proposed by Karl Friston.
 *
 * The robot moves to the desired position specified in desiredPos performing
 * free-energy minimization and active inference using gradient descent.
 * The robot is equipped with proprioceptive sensors for joints position and
 * velocity and a camera for the end-effector pose estimation.
 * The control is in joint space.
 *
 */

#include "AIC.h"

// Constants used by the AIC calss constructor to define the AIC controller
const int realRobot = 1;
const int model = 2;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AIC_controller_FDI_node");
  // Variables to regulate the flow (Force to read once every 1ms the sensors)
  int count = 0;
  int cycles = 0;
  // Variable for fault detection threshold
  double FD_threshold, SPEv;
  // Variable for desired position, set here the goal for the Panda for each joint
  std::vector<double> desiredPos1(7), desiredPos2(7), desiredPos3(7);
  std_msgs::Float64MultiArray SPEreal;
  SPEreal.data.resize(3);

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

  // Objects of the class AIC which will take care of everything
  AIC AIC_realRobot(realRobot);
  AIC AIC_model(model);

  // Set desired position in the AIC class
  AIC_realRobot.setGoal(desiredPos1);
  AIC_model.setGoal(desiredPos1);

  // Main loop
  ros::Rate rate(1000);
  while (ros::ok()){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();

    // Skip only first cycle to allow reading the sensory input first
    if ((count!=0)&&(AIC_realRobot.dataReady()==1)){
      AIC_realRobot.minimiseF();
      AIC_model.minimiseF();

      // Retrieve SPE from model and from real robot for online fault detection
      SPEreal = AIC_realRobot.getSPE();
      FD_threshold = AIC_model.getThreshold();
      // Select the SPE relative to the camera
      SPEv = SPEreal.data[2];
      //std::cout << "Residual" << '\n';
      //std::cout << FD_threshold - SPEv << '\n';

      // Pefrom fault detection
      if(SPEv > FD_threshold){
          ROS_WARN("A fault in the camera has been detected and a recovery action taken");
          //AIC_realRobot.recoveryCameraFault();
      }
      // Cycles counter
      cycles ++;

      if (cycles == 6000){
        //AIC_realRobot.cameraFaultON();
        AIC_realRobot.setGoal(desiredPos2);
        AIC_model.setGoal(desiredPos2);
      }

      if (cycles == 12000){
        //AIC_realRobot.cameraFaultOFF();
        AIC_realRobot.setGoal(desiredPos3);
        AIC_model.setGoal(desiredPos3);
      }

      if (cycles == 18000){
        //AIC_realRobot.cameraFaultOFF();
        AIC_realRobot.setGoal(desiredPos2);
        AIC_model.setGoal(desiredPos2);
      }

      if (cycles == 24000){
        //AIC_realRobot.cameraFaultOFF();
        AIC_realRobot.setGoal(desiredPos1);
        AIC_model.setGoal(desiredPos1);
        cycles = 0;
      }
    }
    else
      count ++;

    rate.sleep();
  }
  return 0;
}
