/*
 * File:   MRAC.h
 * Author: Corrado Pezzato, TU Delft, DCSC
 *
 * Created on April 14th, 2019
 *
 * Class to perform active inference control of the 7DOF Franka Emika Panda robot.
 *
 * This class takes care of everything, it subscribes to the topics containing
 * the sensory input and it perfoms free-energy minimization using gradient descent
 * updating the beliefs about the rosbot's states (i.e. joint values) and computing
 * the control actions. The control is in joint space.
 *
 */

#ifndef MRAC_H
#define MRAC_H
#define _USE_MATH_DEFINES

#include "ros/ros.h"
#include <vector>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <stdlib.h>

// Class MRAC to hanle the subscribers and the publishers for the active inference controller
class MRAC
{
public:
  // Constructor and destructor
  MRAC();
  ~MRAC();
  // Method to set the necessary variables once the constructor is called
  void initVariables();
  // Callback to handle the proprioceptive sensory data from the topic /joint_states published at 1kHz
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
  // Set desired position
  void setGoal(std::vector<double> desiredPos);
  // Support method to control the program flow. Data ready returns one when the encoders has been read
  int dataReady();
  // Method to send the desider torques to control the robot arm
  void computeControlInput();

private:
  // Variables for sensory information and control input
  Eigen::Matrix<double, 7, 1> jointPos, jointVel, u;
  // Support variables to contain the torques for the joints
  std_msgs::Float64 tau1, tau2, tau3, tau4, tau5, tau6, tau7;
  // Desired robot's states and error, column vector of 7 elements
  Eigen::Matrix<double, 7, 1> qr, dqr, qe, x_qe, qe_integral;
  // States to perform integrals of the error signal simulating a first order system as integrator
  Eigen::Matrix<double, 7, 7> X_qr, X_dqr, X_q, X_dq, qe_q_integral, qe_qr_integral, qe_dq_integral, qe_dqr_integral;
  // Support variable to control the flow of the script
  int dataReceived;
  // ROS related Variables, node handle
  ros::NodeHandle nh;
  // Publishers for joint torques to the topics /panda_joint*_controller/command
  ros::Publisher tauPub1, tauPub2, tauPub3, tauPub4, tauPub5, tauPub6, tauPub7;
  // Subscriber for proprioceptive sensors (i.e. from joint_states) and camera (i.e. aruco_single/pose)
  ros::Subscriber sensorSub;
  // MRAC Variables
  // Natural frequencies and damping ratio
  Eigen::Matrix<double, 7, 1> omega, zeta;
  // Adaptive gains and initial guesses
  Eigen::Matrix<double, 7, 7> K0, K1, Q0, Q1, K0_hat, K1_hat, Q0_hat, Q1_hat;
  // Controller parameters
  Eigen::Matrix<double, 7, 7> ALPHA1, ALPHA2, ALPHA3, E01, E02, E03, E11, E12, E13, F01, F02, F03, F11, F12, F13;
  // Support variables for adaptation law
  Eigen::Matrix<double, 7, 1> f, l1, l2, p2, p3, alpha1, alpha2, alpha3, e01, e02, e03, e11, e12, e13, f01, f02, f03, f11, f12, f13;
  Eigen::Matrix<double, 7, 7> P2, P3;
  // Integration step
  double h;
};

#endif
