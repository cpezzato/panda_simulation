#include "ros/ros.h"
#include <vector>
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

using  namespace  std;
using  namespace  Eigen;

// Class AIC to hanle the subscribers and the publishers for the active inference controller
class AIC
{
public:
  // Constructor which takes as argument the publishers and initialises the private ones in the class
  AIC(const ros::Publisher& tauPub1, const ros::Publisher& tauPub2,
      const ros::Publisher& tauPub3, const ros::Publisher& tauPub4,
      const ros::Publisher& tauPub5, const ros::Publisher& tauPub6,const ros::Publisher& tauPub7){

    // Initialize all the publishers
    tauPub1_ = tauPub1; tauPub2_ = tauPub2; tauPub3_ = tauPub3; tauPub4_ = tauPub4;
    tauPub5_ = tauPub5; tauPub6_ = tauPub6; tauPub7_ = tauPub7;

    // Initialize the variables for thr AIC
    initVariables();
  }

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    // set joint values
    for( int i = 0; i < 7; i++ ) {
      jointPos(i) = msg->position[i];
      jointVel(i) = msg->velocity[i];
    }
    // cout << jointPos << "\n\n";

    // Desired final Position for now here since we want to do just state estimation
    for( int i = 0; i < jointPos.size(); i = i + 1 ) {
      mu_d(i)=jointPos[i];
    }
   }

  void cameraCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    // Whatch out x and y switched from aruco (left hand rule?)
    eePos.x = msg->pose.position.y;
    eePos.y = msg->pose.position.x;
    eePos.z = msg->pose.position.z;
    ROS_INFO("x = %f  y = %f  z = %f ", eePos.x, eePos.y, eePos.z);
  }

  void initVariables(){

    // Variances
    var_mu = 1.0;
    var_muprime = 1.0;
    var_q = 1.0;
    var_qdot = 1.0;
    var_eev = 1.0;

    // Precision matrices (first set to zero then populate the diagonal)
    SigmaP_yq0 = Matrix<double, 7, 7>::Zero();
    SigmaP_yq1 = Matrix<double, 7, 7>::Zero();
    SigmaP_yv0 = Matrix<double, 7, 7>::Zero();
    SigmaP_mu = Matrix<double, 7, 7>::Zero();
    SigmaP_muprime = Matrix<double, 7, 7>::Zero();

    for( int i = 0; i < SigmaP_yq0.rows(); i = i + 1 ) {
      SigmaP_yq0(i,i) = 1/var_q;
      SigmaP_yq1(i,i) = 1/var_qdot;
      SigmaP_yv0(i,i) = 1/var_eev;
      SigmaP_mu(i,i) = 1/var_mu;
      SigmaP_muprime(i,i) = 1/var_muprime;
    }

    // Prior beliefs about the states of the robot
    mu << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    mu_p << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    mu_pp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // Learning rates
    k_mu =50;
    k_a = 1;

    // Integration step
    h = 0.001;
  }

  void minimiseF(){

    // Sensory prediction errors
    SPEq = (jointPos.transpose()-mu.transpose())*SigmaP_yq0*(jointPos-mu);
    SPEmu = (mu_p.transpose()+mu.transpose()-mu_d.transpose())*SigmaP_mu*(mu_p+mu-mu_d);
    // Free-energy
    F = SPEq + SPEmu;
    // Monitor the free-energy
    cout << F <<"\n\n";

    // Minimization using gradient descent
    mu_dot = mu_p - k_mu*(-SigmaP_yq0*(jointPos-mu)+SigmaP_mu*(mu_p+mu-mu_d));
    // Belifs about the position
    mu = mu + h*mu_dot;
  }

  void sendTorques(){
    // Set the toques
    tau1.data = 10;
    tau2.data = -30;
    tau3.data = 1;
    tau4.data = 30;
    tau5.data = 1;
    tau6.data = 1;
    tau7.data = 1;

    // Publishing
    tauPub1_.publish(tau1); tauPub2_.publish(tau2); tauPub3_.publish(tau3);
    tauPub4_.publish(tau4); tauPub5_.publish(tau5); tauPub6_.publish(tau6);
    tauPub7_.publish(tau7);
  }

private:
  // Define useful variables for storing joint and end-effector position
  struct eePos
    {
        double x;
        double y;
        double z;
    } eePos;

  ros::Publisher tauPub1_, tauPub2_, tauPub3_, tauPub4_, tauPub5_, tauPub6_, tauPub7_;
  std_msgs::Float64 tau1, tau2, tau3, tau4, tau5, tau6, tau7;
  // Variances
  double var_q, var_qdot, var_eev, var_mu, var_muprime;
  // Precision matrices
  Matrix<double, 7, 7> SigmaP_yq0, SigmaP_yq1, SigmaP_yv0, SigmaP_mu, SigmaP_muprime;
  // Beliefs about the states and their derivatives mu, mu', mu'', arrays of 7 elements
  Matrix<double, 7, 1> mu, mu_p, mu_pp, mu_dot, mu_dot_p, mu_dot_pp, jointPos, jointVel;
  // Desired robot's states
  Matrix<double, 7, 1> mu_d;
  // Learning rates and integration step
  double k_mu, k_a, h;
  // Sensory prediction errors and Free energy
  double SPEq, SPEmu, F;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AIC_controller_node");
  ros::NodeHandle nh;
  // Variables to regulate the flow (Force to read once every 1ms the sensors)
  int count = 0;

  // Publishers on the topics /panda_joint*_controller/command for the joint efforts
  ros::Publisher tauPub1 = nh.advertise<std_msgs::Float64>("/panda_joint1_controller/command", 20);
  ros::Publisher tauPub2 = nh.advertise<std_msgs::Float64>("/panda_joint2_controller/command", 20);
  ros::Publisher tauPub3 = nh.advertise<std_msgs::Float64>("/panda_joint3_controller/command", 20);
  ros::Publisher tauPub4 = nh.advertise<std_msgs::Float64>("/panda_joint4_controller/command", 20);
  ros::Publisher tauPub5 = nh.advertise<std_msgs::Float64>("/panda_joint5_controller/command", 20);
  ros::Publisher tauPub6 = nh.advertise<std_msgs::Float64>("/panda_joint6_controller/command", 20);
  ros::Publisher tauPub7 = nh.advertise<std_msgs::Float64>("/panda_joint7_controller/command", 20);

  // Object of the class AIC which will take care of everything
  AIC AIC_controller(tauPub1, tauPub2, tauPub3, tauPub4, tauPub5, tauPub6, tauPub7);
  // Subscriber for proprioceptive sensors (i.e. from joint:states)
  ros::Subscriber sensorSub = nh.subscribe("joint_states", 1, &AIC::jointStatesCallback, &AIC_controller);
  ros::Subscriber cameraSub = nh.subscribe("aruco_single/pose", 1, &AIC::cameraCallback, &AIC_controller);

  ros::Rate rate(1000);

  // Main loop
  while (ros::ok()){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();

    // Skip only first cycle to allow reading the sensory input first
    if (count!=0){
      AIC_controller.minimiseF();
    }
    else
      count ++;

    rate.sleep();
  }

  return 0;
}
