/*
 * File:   MRAC.cpp
 * Author: Corrado Pezzato, TU Delft, DCSC
 *
 * Created on April 14th, 2019
 *
 * Class to perform active inference control of the 7DOF Franka Emika Panda robot.
 * Definition of the methods contained in MRAC.h
 *
 */

#include "MRAC.h"

  // Constructor which takes as argument the publishers and initialises the private ones in the class
  MRAC::MRAC(){
    // Initialize the variables for thr MRAC
    MRAC::initVariables();
  }
  MRAC::~MRAC(){}

  void   MRAC::initVariables(){
    // Initialize publishers on the topics /panda_joint*_controller/command for the joint efforts
    tauPub1 = nh.advertise<std_msgs::Float64>("/panda_joint1_controller/command", 20);
    tauPub2 = nh.advertise<std_msgs::Float64>("/panda_joint2_controller/command", 20);
    tauPub3 = nh.advertise<std_msgs::Float64>("/panda_joint3_controller/command", 20);
    tauPub4 = nh.advertise<std_msgs::Float64>("/panda_joint4_controller/command", 20);
    tauPub5 = nh.advertise<std_msgs::Float64>("/panda_joint5_controller/command", 20);
    tauPub6 = nh.advertise<std_msgs::Float64>("/panda_joint6_controller/command", 20);
    tauPub7 = nh.advertise<std_msgs::Float64>("/panda_joint7_controller/command", 20);

    // Subscribers for proprioceptive sensors (i.e. from joint:states) and camera position (i.e. aruco_single/pose)
    sensorSub = nh.subscribe("joint_states", 1, &MRAC::jointStatesCallback, this);

    // Set goal for velocity (always zero), the goal for the position is set by the main node MRAC_controller_node
    qdotGoal << 0, 0, 0, 0, 0, 0, 0;

    // Support variable
    dataReceived = 0;

    // MRAC Variables
    // NAtural pulsation
    omega << 5, 5, 5, 5, 5, 5, 5;
    // Damping
    zeta << 1, 1, 1, 1, 1, 1, 1;

    // Controller parameters
    alpha1 << 1, 1, 1, 1, 1, 1, 1;
    alpha2 << 1, 1, 1, 1, 1, 1, 1;
    alpha3 << 1, 1, 1, 1, 1, 1, 1;
    e01 << 1, 1, 1, 1, 1, 1, 1;
    e02 << 1, 1, 1, 1, 1, 1, 1;
    e03 << 0, 0, 0, 0, 0, 0, 0;
    e11 << 1, 1, 1, 1, 1, 1, 1;
    e12 << 1, 1, 1, 1, 1, 1, 1;
    e13 << 0, 0, 0, 0, 0, 0, 0;
    f01 << 1, 1, 1, 1, 1, 1, 1;
    f02 << 1, 1, 1, 1, 1, 1, 1;
    f03 << 0, 0, 0, 0, 0, 0, 0;
    f11 << 1, 1, 1, 1, 1, 1, 1;
    f12 << 1, 1, 1, 1, 1, 1, 1;
    f13 << 0, 0, 0, 0, 0, 0, 0;
    l1 << 1, 1, 1, 1, 1, 1, 1;
    l2 << 1, 1, 1, 1, 1, 1, 1;

    // From here on a normal user should not modify
    p2 << l1[0]/(2*omega[0]*omega[0]), l1[1]/(2*omega[1]*omega[1]), l1[2]/(2*omega[2]*omega[2]),
          l1[3]/(2*omega[3]*omega[3]), l1[4]/(2*omega[4]*omega[4]), l1[5]/(2*omega[5]*omega[5]), l1[6]/(2*omega[6]*omega[6]);
    p3 << l2[0]/(4*zeta[0]*omega[0])+l1[0]/(4*zeta[0]*pow(omega[0],3)), l2[1]/(4*zeta[1]*omega[1])+l1[1]/(4*zeta[1]*pow(omega[1],3)),
          l2[2]/(4*zeta[2]*omega[2])+l1[2]/(4*zeta[2]*pow(omega[2],3)), l2[3]/(4*zeta[3]*omega[3])+l1[3]/(4*zeta[3]*pow(omega[3],3)),
          l2[4]/(4*zeta[4]*omega[4])+l1[4]/(4*zeta[4]*pow(omega[4],3)), l2[5]/(4*zeta[5]*omega[5])+l1[5]/(4*zeta[5]*pow(omega[5],3)),
          l2[6]/(4*zeta[6]*omega[6])+l1[6]/(4*zeta[6]*pow(omega[6],3));

   // Initial adaptive gains and constant matrices
   K0_hat = Eigen::Matrix<double, 7, 7>::Zero();
   K1_hat = Eigen::Matrix<double, 7, 7>::Zero();
   Q0_hat = Eigen::Matrix<double, 7, 7>::Zero();
   Q1_hat = Eigen::Matrix<double, 7, 7>::Zero();
   ALPHA1 = Eigen::Matrix<double, 7, 7>::Zero();
   ALPHA2 = Eigen::Matrix<double, 7, 7>::Zero();
   ALPHA3 = Eigen::Matrix<double, 7, 7>::Zero();
   E01 = Eigen::Matrix<double, 7, 7>::Zero();
   E02 = Eigen::Matrix<double, 7, 7>::Zero();
   E03 = Eigen::Matrix<double, 7, 7>::Zero();
   E11 = Eigen::Matrix<double, 7, 7>::Zero();
   E12 = Eigen::Matrix<double, 7, 7>::Zero();
   E13 = Eigen::Matrix<double, 7, 7>::Zero();
   F01 = Eigen::Matrix<double, 7, 7>::Zero();
   F02 = Eigen::Matrix<double, 7, 7>::Zero();
   F03 = Eigen::Matrix<double, 7, 7>::Zero();
   F11 = Eigen::Matrix<double, 7, 7>::Zero();
   F12 = Eigen::Matrix<double, 7, 7>::Zero();
   F13 = Eigen::Matrix<double, 7, 7>::Zero();
   P2 = Eigen::Matrix<double, 7, 7>::Zero();
   P3 = Eigen::Matrix<double, 7, 7>::Zero();

   for( int i = 0; i < P2.rows(); i = i + 1 ) {
     ALPHA1(i,i) = alpha1(i);
     ALPHA2(i,i) = alpha2(i);
     ALPHA3(i,i) = alpha3(i);
     E01(i,i) = e01(i);
     E02(i,i) = e02(i);
     E03(i,i) = e03(i);
     E11(i,i) = e11(i);
     E12(i,i) = e12(i);
     E13(i,i) = e13(i);
     F01(i,i) = f01(i);
     F02(i,i) = f02(i);
     F03(i,i) = f03(i);
     F11(i,i) = f11(i);
     F12(i,i) = f12(i);
     F13(i,i) = f13(i);
     P2(i,i) = p2(i);
     P3(i,i) = p3(i);
   }
  }

  void   MRAC::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    // Save joint values
    for( int i = 0; i < 7; i++ ) {
      jointPos(i) = msg->position[i];
      jointVel(i) = msg->velocity[i];
    }
    // If this is the first time we read the joint states then we set the current beliefs
    if (dataReceived == 0){
      // Track the fact that the encoders published
      dataReceived = 1;
    }
    // std::cout << jointPos << '\n';
  }

  void MRAC::setGoal(std::vector<double> desiredPos){
    for(int i=0; i<desiredPos.size(); i++){
      qGoal(i) = desiredPos[i];
    }
  }

  int MRAC::dataReady(){
    // Method to control if the joint states have been received already, used in the main function
    if(dataReceived==1)
      return 1;
    else
      return 0;
  }

  void MRAC::computeControlInput(){
    ROS_INFO("Add MRAC here");
  }
