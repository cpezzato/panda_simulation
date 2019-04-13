/*
 * File:   AIC.h
 * Author: Corrado Pezzato, TU Dleft, DCSC
 *
 * Created on April 13th, 2019
 *
 * Class to perform active inference control of the 7DOF Franka Emika Panda
 *
 */

#ifndef AIC_H
#define AIC_H
#include "ros/ros.h"
#include <vector>
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

// Class AIC to hanle the subscribers and the publishers for the active inference controller
class AIC
{
public:
  // Constructor which takes as argument the publishers and initialises the private ones in the class
  AIC();
  ~AIC();

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

  void cameraCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void initVariables();

  void minimiseF();

  void computeActions();

  int dataReady();

private:
  // Define useful variables for storing joint and end-effector position
  struct eePos
    {
        double x;
        double y;
        double z;
    } eePos;

  // Variances
  double var_q, var_qdot, var_eev, var_mu, var_muprime;
  // Precision matrices
  Eigen::Matrix<double, 7, 7> SigmaP_yq0, SigmaP_yq1, SigmaP_yv0, SigmaP_mu, SigmaP_muprime;
  // Beliefs about the states and their derivatives mu, mu', mu'', arrays of 7 elements
  Eigen::Matrix<double, 7, 1> mu, mu_p, mu_pp, mu_dot, mu_dot_p, mu_dot_pp, jointPos, jointVel;
  // Desired robot's states
  Eigen::Matrix<double, 7, 1> mu_d;
  // Control actions
  Eigen::Matrix<double, 7, 1> u;
  // Learning rates and integration step
  double k_mu, k_a, h;
  // Sensory prediction errors and Free energy
  double SPEq, SPEdq, SPEv, SPEmu_p, SPEmu_pp, F;
  // Support variable
  int dataReceived;
  // ROS related Variables
  ros::NodeHandle nh;
  // Publishers for joint torques
  ros::Publisher tauPub1, tauPub2, tauPub3, tauPub4, tauPub5, tauPub6, tauPub7;
  // Subscriber for proprioceptive sensors (i.e. from joint:states)
  ros::Subscriber sensorSub, cameraSub;
  std_msgs::Float64 tau1, tau2, tau3, tau4, tau5, tau6, tau7;


};

#endif
