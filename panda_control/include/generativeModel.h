/*
 * File:   AIC.h
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

#ifndef GENERATIVEMODEL_H
#define GENERATIVEMODEL_H
#define _USE_MATH_DEFINES

#include <vector>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "ros/ros.h"

// Class AIC to hanle the subscribers and the publishers for the active inference controller
class generativeModel
{
public:
  // Constructor and destructor
  generativeModel();
  ~generativeModel();
  // Methods to retrieve the generative model of the robot and its derivatives
  Eigen::Matrix<double, 3, 1> getG(Eigen::Matrix<double, 7, 1> theta);
  Eigen::Matrix<double, 7, 1> getGxprime(Eigen::Matrix<double, 7, 1> theta);
  Eigen::Matrix<double, 7, 1> getGyprime(Eigen::Matrix<double, 7, 1> theta);
  Eigen::Matrix<double, 7, 1> getGzprime(Eigen::Matrix<double, 7, 1> theta);

private:
  // Variables for storing the generative model and the derivatives
  Eigen::Matrix<double, 3, 1> g;
  Eigen::Matrix<double, 7, 1> gxprime, gyprime, gzprime;
};

#endif
