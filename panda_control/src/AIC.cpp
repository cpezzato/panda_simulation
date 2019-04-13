#include "AIC.h"

// Class AIC to hanle the subscribers and the publishers for the active inference controller

  // Constructor which takes as argument the publishers and initialises the private ones in the class
  AIC::AIC(){
    // Initialize the variables for thr AIC
      AIC::initVariables();
  }
  AIC::~AIC(){}


  void   AIC::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    // set joint values
    for( int i = 0; i < 7; i++ ) {
      jointPos(i) = msg->position[i];
      jointVel(i) = msg->velocity[i];
    }
    // cout << jointPos << "\n\n";
    if (dataReceived == 0){
      dataReceived = 1;
      // The first time we retrieve the position we define the initial beliefs about the states
      mu = jointPos;
      mu_p = jointVel;
    }
  }

  void   AIC::cameraCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    // Whatch out x and y switched from aruco (left hand rule?)
    eePos.x = msg->pose.position.y;
    eePos.y = msg->pose.position.x;
    eePos.z = msg->pose.position.z;
    ROS_INFO("x = %f  y = %f  z = %f ", eePos.x, eePos.y, eePos.z);
  }

  void   AIC::initVariables(){

    // Publishers on the topics /panda_joint*_controller/command for the joint efforts
    tauPub1 = nh.advertise<std_msgs::Float64>("/panda_joint1_controller/command", 20);
    tauPub2 = nh.advertise<std_msgs::Float64>("/panda_joint2_controller/command", 20);
    tauPub3 = nh.advertise<std_msgs::Float64>("/panda_joint3_controller/command", 20);
    tauPub4 = nh.advertise<std_msgs::Float64>("/panda_joint4_controller/command", 20);
    tauPub5 = nh.advertise<std_msgs::Float64>("/panda_joint5_controller/command", 20);
    tauPub6 = nh.advertise<std_msgs::Float64>("/panda_joint6_controller/command", 20);
    tauPub7 = nh.advertise<std_msgs::Float64>("/panda_joint7_controller/command", 20);

    // Subscriber for proprioceptive sensors (i.e. from joint:states)
    sensorSub = nh.subscribe("joint_states", 1, &AIC::jointStatesCallback, this);
    cameraSub = nh.subscribe("aruco_single/pose", 1, &AIC::cameraCallback, this);

    // Support variable
    dataReceived = 0;

    // Variances
    var_mu = 1.0;
    var_muprime = 1.0;
    var_q = 0.1;
    var_qdot = 0.1;
    var_eev = 1.0;

    // Learning rates
    k_mu = 20;
    k_a = 1200;

    // Precision matrices (first set to zero then populate the diagonal)
    SigmaP_yq0 = Eigen::Matrix<double, 7, 7>::Zero();
    SigmaP_yq1 = Eigen::Matrix<double, 7, 7>::Zero();
    SigmaP_yv0 = Eigen::Matrix<double, 7, 7>::Zero();
    SigmaP_mu = Eigen::Matrix<double, 7, 7>::Zero();
    SigmaP_muprime = Eigen::Matrix<double, 7, 7>::Zero();

    for( int i = 0; i < SigmaP_yq0.rows(); i = i + 1 ) {
      SigmaP_yq0(i,i) = 1/var_q;
      SigmaP_yq1(i,i) = 1/var_qdot;
      SigmaP_yv0(i,i) = 1/var_eev;
      SigmaP_mu(i,i) = 1/var_mu;
      SigmaP_muprime(i,i) = 1/var_muprime;
    }

    // Initialize control actions
    u << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // Prior beliefs about the states of the robot
    mu_pp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // Desired final Position for now here since we want to do just state estimation
    //for( int i = 0; i < jointPos.size(); i = i + 1 ) {
    //  mu_d(i)=jointPos[i];
    //}

    mu_d << 0.0, 0.4, 0.0, -0.7, 0.0, 1.6, 0.0;

    // Integration step
    h = 0.001;

    // For now SPEv to zero
    SPEv = 0;
  }

  void   AIC::minimiseF(){

    // Sensory prediction errors
    SPEq = (jointPos.transpose()-mu.transpose())*SigmaP_yq0*(jointPos-mu);
    SPEdq = (jointVel.transpose()-mu_p.transpose())*SigmaP_yq1*(jointVel-mu_p);
    SPEmu_p = (mu_p.transpose()+mu.transpose()-mu_d.transpose())*SigmaP_mu*(mu_p+mu-mu_d);
    SPEmu_pp = (mu_pp.transpose()+mu_p.transpose())*SigmaP_muprime*(mu_pp+mu_p);

    // Free-energy
    F = SPEq + SPEdq + SPEv + SPEmu_p + SPEmu_pp;
    // Monitor the free-energy
    // cout << F <<"\n\n";

    // Minimization using gradient descent
    mu_dot = mu_p - k_mu*(-SigmaP_yq0*(jointPos-mu)+SigmaP_mu*(mu_p+mu-mu_d));
    mu_dot_p = mu_pp - k_mu*(-SigmaP_yq1*(jointVel-mu_p)+SigmaP_mu*(mu_p+mu-mu_d)+SigmaP_muprime*(mu_pp+mu_p));
    mu_dot_pp = - k_mu*(SigmaP_muprime*(mu_pp+mu_p));

    // Belifs update
    mu = mu + h*mu_dot;             // Belief about the position
    mu_p = mu_p + h*mu_dot_p;       // Belief about motion of mu
    mu_pp = mu_pp + h*mu_dot_pp;    // Belief about motion of mu'

    // Calculate and send control actions
      AIC::computeActions();
    //ROS_INFO("JOINT4 = %f", jointPos(6));
  }

  void   AIC::computeActions(){
    u = u-h*k_a*(SigmaP_yq1*(jointVel-mu_p)+SigmaP_yq0*(jointPos-mu));
    // cout << u <<"\n\n";

    // Set the toques
    tau1.data = u(0); tau2.data = u(1); tau3.data = u(2); tau4.data = u(3);
    tau5.data = u(4); tau6.data = u(5); tau7.data = u(6);
    //tau1.data = 0; tau2.data = 0; tau3.data = 0; tau4.data = 0;
    //tau5.data = 0; tau6.data = 0; tau7.data = 0;
    // Publishing
    tauPub1.publish(tau1); tauPub2.publish(tau2); tauPub3.publish(tau3);
    tauPub4.publish(tau4); tauPub5.publish(tau5); tauPub6.publish(tau6);
    tauPub7.publish(tau7);
  }

  int AIC::dataReady(){
    if(dataReceived==1)
      return 1;
    else
      return 0;
  }
