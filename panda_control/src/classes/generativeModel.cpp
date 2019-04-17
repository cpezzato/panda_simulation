/*
 * File:   AIC.cpp
 * Author: Corrado Pezzato, TU Delft, DCSC
 *
 * Created on April 14th, 2019
 *
 * Class to perform active inference control of the 7DOF Franka Emika Panda robot.
 * Definition of the methods contained in AIC.h
 *
 */

#include "generativeModel.h"

  // Constructor which takes as argument the publishers and initialises the private ones in the class
  generativeModel::generativeModel(){}
  generativeModel::~generativeModel(){}

  Eigen::Matrix<double, 3, 1> generativeModel::getG(Eigen::Matrix<double, 7, 1> theta){

    g[0] = sin(theta[0]) * (cos(theta[2]) * sin(theta[4]) * (-0.088
           * cos(theta[5]) + -0.107 * sin(theta[5])) + sin(theta[2]) *
           ((-0.0825 + sin(theta[3]) * ((0.384 + -0.107 * cos(theta[5])) +
           0.088 * sin(theta[5]))) + cos(theta[3]) * ((0.0825 + -0.088 *
           cos(theta[4]) * cos(theta[5])) + -0.107 * cos(theta[4]) * sin
           (theta[5])))) + cos(theta[0]) * (sin(theta[1]) * ((0.316 +
           cos(theta[3]) * ((0.384 + -0.107 * cos(theta[5])) + 0.088 * sin
           (theta[5]))) + sin(theta[3]) * ((-0.0825 + 0.088 *
           cos(theta[4]) * cos(theta[5])) + 0.107 * cos(theta[4]) * sin
           (theta[5]))) + cos(theta[1]) * (sin(theta[2]) * sin(theta[4])
           * (-0.088 * cos(theta[5]) + -0.107 * sin(theta[5])) + cos
           (theta[2]) * ((0.0825 + sin(theta[3]) * ((-0.384 + 0.107 * cos
           (theta[5])) + -0.088 * sin(theta[5]))) + cos(theta[3]) * ((-0.0825
           + 0.088 * cos(theta[4]) * cos(theta[5])) + 0.107 * cos(theta
           [4]) * sin(theta[5])))));
    g[1] = (sin(theta[0]) * sin(theta[1]) * ((0.316 + cos(theta[3])
           * ((0.384 + -0.107 * cos(theta[5])) + 0.088 * sin(theta[5])))
           + sin(theta[3]) * ((-0.0825 + 0.088 * cos(theta[4]) *
           cos(theta[5])) + 0.107 * cos(theta[4]) * sin(theta[5])))
           + cos(theta[1]) * sin(theta[0]) * (sin(theta[2]) *
           sin(theta[4]) * (-0.088 * cos(theta[5]) + -0.107 * sin
           (theta[5])) + cos(theta[2]) * ((0.0825 + sin(theta[3]) *
           ((-0.384 + 0.107 * cos(theta[5])) + -0.088 * sin(theta[5])))
           + cos(theta[3]) * ((-0.0825 + 0.088 * cos(theta[4]) *
           cos(theta[5])) + 0.107 * cos(theta[4]) * sin(theta[5])))))
           + cos(theta[0]) * (cos(theta[2]) * sin(theta[4]) * (0.088 *
           cos(theta[5]) + 0.107 * sin(theta[5])) + sin(theta[2]) *
           ((0.0825 + sin(theta[3]) * ((-0.384 + 0.107 * cos(theta[5])) +
           -0.088 * sin(theta[5]))) + cos(theta[3]) * ((-0.0825 + 0.088 *
           cos(theta[4]) * cos(theta[5])) + 0.107 * cos(theta[4]) * sin
           (theta[5]))));
    g[2] = ((0.333 + sin(theta[1]) * sin(theta[2]) * sin(theta[4]) *
           (0.088 * cos(theta[5]) + 0.107 * sin(theta[5]))) + cos
           (theta[2]) * sin(theta[1]) * ((-0.0825 + sin(theta[3]) *
           ((0.384 + -0.107 * cos(theta[5])) + 0.088 * sin(theta[5])))
           + cos(theta[3]) * ((0.0825 + -0.088 * cos(theta[4]) *
           cos(theta[5])) + -0.107 * cos(theta[4]) * sin(theta[5]))))
           + cos(theta[1]) * ((0.316 + cos(theta[3]) * ((0.384 + -0.107 *
           cos(theta[5])) + 0.088 * sin(theta[5]))) + sin(theta[3]) *
           ((-0.0825 + 0.088 * cos(theta[4]) * cos(theta[5])) + 0.107 *
           cos(theta[4]) * sin(theta[5])));

    return(g);
  }

  Eigen::Matrix<double, 7, 1> generativeModel::getGxprime(Eigen::Matrix<double, 7, 1> theta){

    gxprime[0] = (sin(theta[0]) * sin(theta[1]) * ((-0.316 + cos
    (theta[3]) * ((-0.384 + 0.107 * cos(theta[5])) + -0.088 * sin
                  (theta[5]))) + sin(theta[3]) * ((0.0825 + -0.088 *
    cos(theta[4]) * cos(theta[5])) + -0.107 * cos(theta[4]) * sin
    (theta[5]))) + cos(theta[1]) * sin(theta[0]) * (sin(theta[2])
    * sin(theta[4]) * (0.088 * cos(theta[5]) + 0.107 * sin(theta
    [5])) + cos(theta[2]) * ((-0.0825 + sin(theta[3]) * ((0.384 +
    -0.107 * cos(theta[5])) + 0.088 * sin(theta[5]))) + cos
    (theta[3]) * ((0.0825 + -0.088 * cos(theta[4]) * cos(theta[5])) +
                  -0.107 * cos(theta[4]) * sin(theta[5]))))) +
    cos(theta[0]) * (cos(theta[2]) * sin(theta[4]) * (-0.088 *
    cos(theta[5]) + -0.107 * sin(theta[5])) + sin(theta[2]) *
                     ((-0.0825 + sin(theta[3]) * ((0.384 + -0.107 *
    cos(theta[5])) + 0.088 * sin(theta[5]))) + cos(theta[3]) *
                      ((0.0825 + -0.088 * cos(theta[4]) * cos(theta[5]))
                       + -0.107 * cos(theta[4]) * sin(theta[5]))));
  gxprime[1] = cos(theta[0]) * (cos(theta[1]) * ((0.316 + cos
    (theta[3]) * ((0.384 + -0.107 * cos(theta[5])) + 0.088 * sin
                  (theta[5]))) + sin(theta[3]) * ((-0.0825 + 0.088 *
    cos(theta[4]) * cos(theta[5])) + 0.107 * cos(theta[4]) * sin
    (theta[5]))) + sin(theta[1]) * (sin(theta[2]) * sin(theta[4])
    * (0.088 * cos(theta[5]) + 0.107 * sin(theta[5])) + cos
    (theta[2]) * ((-0.0825 + sin(theta[3]) * ((0.384 + -0.107 * cos
    (theta[5])) + 0.088 * sin(theta[5]))) + cos(theta[3]) * ((0.0825 +
    -0.088 * cos(theta[4]) * cos(theta[5])) + -0.107 * cos(theta
    [4]) * sin(theta[5])))));
  gxprime[2] = sin(theta[0]) * (sin(theta[2]) * sin(theta[4]) *
    (0.088 * cos(theta[5]) + 0.107 * sin(theta[5])) + cos(theta[2])
    * ((-0.0825 + sin(theta[3]) * ((0.384 + -0.107 * cos(theta[5])) +
    0.088 * sin(theta[5]))) + cos(theta[3]) * ((0.0825 + -0.088 *
    cos(theta[4]) * cos(theta[5])) + -0.107 * cos(theta[4]) * sin
    (theta[5])))) + cos(theta[0]) * cos(theta[1]) * (cos(theta[2])
    * sin(theta[4]) * (-0.088 * cos(theta[5]) + -0.107 * sin
    (theta[5])) + sin(theta[2]) * ((-0.0825 + sin(theta[3]) * ((0.384
    + -0.107 * cos(theta[5])) + 0.088 * sin(theta[5]))) + cos
    (theta[3]) * ((0.0825 + -0.088 * cos(theta[4]) * cos(theta[5])) +
                  -0.107 * cos(theta[4]) * sin(theta[5]))));
  gxprime[3] = sin(theta[0]) * sin(theta[2]) * (cos(theta[3]) *
    ((0.384 + -0.107 * cos(theta[5])) + 0.088 * sin(theta[5])) +
    sin(theta[3]) * ((-0.0825 + 0.088 * cos(theta[4]) * cos(theta[5]))
                     + 0.107 * cos(theta[4]) * sin(theta[5]))) +
    cos(theta[0]) * (cos(theta[1]) * cos(theta[2]) * (cos(theta[3])
    * ((-0.384 + 0.107 * cos(theta[5])) + -0.088 * sin(theta[5])) +
    sin(theta[3]) * ((0.0825 + -0.088 * cos(theta[4]) * cos
    (theta[5])) + -0.107 * cos(theta[4]) * sin(theta[5]))) + sin
                     (theta[1]) * (sin(theta[3]) * ((-0.384 + 0.107 *
    cos(theta[5])) + -0.088 * sin(theta[5])) + cos(theta[3]) *
    ((-0.0825 + 0.088 * cos(theta[4]) * cos(theta[5])) + 0.107 *
     cos(theta[4]) * sin(theta[5]))));
  gxprime[4] = sin(theta[0]) * (cos(theta[2]) * cos(theta[4]) * (
    -0.088 * cos(theta[5]) + -0.107 * sin(theta[5])) + cos(theta
    [3]) * sin(theta[2]) * sin(theta[4]) * (0.088 * cos(theta[5])
    + 0.107 * sin(theta[5]))) + cos(theta[0]) * (sin(theta[1]) *
    sin(theta[3]) * sin(theta[4]) + cos(theta[1]) * (cos
    (theta[4]) * sin(theta[2]) + cos(theta[2]) * cos(theta[3]) *
    sin(theta[4]))) * (-0.088 * cos(theta[5]) + -0.107 * sin
    (theta[5]));
  gxprime[5] = cos(theta[0]) * (cos(theta[1]) * (cos(theta[2]) *
    (sin(theta[3]) * (-0.088 * cos(theta[5]) + -0.107 * sin
    (theta[5])) + cos(theta[3]) * cos(theta[4]) * (0.107 * cos
    (theta[5]) + -0.088 * sin(theta[5]))) + sin(theta[2]) * sin
    (theta[4]) * (-0.107 * cos(theta[5]) + 0.088 * sin(theta[5]))) +
    sin(theta[1]) * (cos(theta[4]) * sin(theta[3]) * (0.107 *
    cos(theta[5]) + -0.088 * sin(theta[5])) + cos(theta[3]) * (0.088 *
    cos(theta[5]) + 0.107 * sin(theta[5])))) + sin(theta[0]) *
    (sin(theta[2]) * (cos(theta[3]) * cos(theta[4]) * (-0.107 *
       cos(theta[5]) + 0.088 * sin(theta[5])) + sin(theta[3]) *
      (0.088 * cos(theta[5]) + 0.107 * sin(theta[5]))) + cos
     (theta[2]) * sin(theta[4]) * (-0.107 * cos(theta[5]) + 0.088 *
      sin(theta[5])));
gxprime[6] = 0.0;
    return(gxprime);
  }

  Eigen::Matrix<double, 7, 1> generativeModel::getGyprime(Eigen::Matrix<double, 7, 1> theta){

    gyprime[0] = sin(theta[0]) * (cos(theta[2]) * sin(theta[4]) * (
      -0.088 * cos(theta[5]) + -0.107 * sin(theta[5])) + sin(theta
      [2]) * ((-0.0825 + sin(theta[3]) * ((0.384 + -0.107 * cos(theta[5]))
      + 0.088 * sin(theta[5]))) + cos(theta[3]) * ((0.0825 + -0.088 *
      cos(theta[4]) * cos(theta[5])) + -0.107 * cos(theta[4]) *
      sin(theta[5])))) + cos(theta[0]) * (sin(theta[1]) * ((0.316 +
      cos(theta[3]) * ((0.384 + -0.107 * cos(theta[5])) + 0.088 * sin
                       (theta[5]))) + sin(theta[3]) * ((-0.0825 + 0.088 *
      cos(theta[4]) * cos(theta[5])) + 0.107 * cos(theta[4]) * sin
      (theta[5]))) + cos(theta[1]) * (sin(theta[2]) * sin(theta[4])
      * (-0.088 * cos(theta[5]) + -0.107 * sin(theta[5])) + cos
      (theta[2]) * ((0.0825 + sin(theta[3]) * ((-0.384 + 0.107 * cos
      (theta[5])) + -0.088 * sin(theta[5]))) + cos(theta[3]) * ((-0.0825
      + 0.088 * cos(theta[4]) * cos(theta[5])) + 0.107 * cos(theta
      [4]) * sin(theta[5])))));
    gyprime[1] = sin(theta[0]) * (cos(theta[1]) * ((0.316 + cos
      (theta[3]) * ((0.384 + -0.107 * cos(theta[5])) + 0.088 * sin
                    (theta[5]))) + sin(theta[3]) * ((-0.0825 + 0.088 *
      cos(theta[4]) * cos(theta[5])) + 0.107 * cos(theta[4]) * sin
      (theta[5]))) + sin(theta[1]) * (sin(theta[2]) * sin(theta[4])
      * (0.088 * cos(theta[5]) + 0.107 * sin(theta[5])) + cos
      (theta[2]) * ((-0.0825 + sin(theta[3]) * ((0.384 + -0.107 * cos
      (theta[5])) + 0.088 * sin(theta[5]))) + cos(theta[3]) * ((0.0825 +
      -0.088 * cos(theta[4]) * cos(theta[5])) + -0.107 * cos(theta
      [4]) * sin(theta[5])))));
    gyprime[2] = cos(theta[1]) * sin(theta[0]) * (cos(theta[2]) *
      sin(theta[4]) * (-0.088 * cos(theta[5]) + -0.107 * sin(theta
      [5])) + sin(theta[2]) * ((-0.0825 + sin(theta[3]) * ((0.384 +
      -0.107 * cos(theta[5])) + 0.088 * sin(theta[5]))) + cos
      (theta[3]) * ((0.0825 + -0.088 * cos(theta[4]) * cos(theta[5])) +
                    -0.107 * cos(theta[4]) * sin(theta[5])))) + cos
      (theta[0]) * (sin(theta[2]) * sin(theta[4]) * (-0.088 * cos
      (theta[5]) + -0.107 * sin(theta[5])) + cos(theta[2]) * ((0.0825 +
      sin(theta[3]) * ((-0.384 + 0.107 * cos(theta[5])) + -0.088 *
      sin(theta[5]))) + cos(theta[3]) * ((-0.0825 + 0.088 * cos(theta[4])
      * cos(theta[5])) + 0.107 * cos(theta[4]) * sin(theta[5]))));
    gyprime[3] = (cos(theta[1]) * cos(theta[2]) * sin(theta[0]) *
                  (sin(theta[3]) * (0.0825 + cos(theta[4]) * (-0.088 *
      cos(theta[5]) + -0.107 * sin(theta[5]))) + cos(theta[3]) * ((
      -0.384 + 0.107 * cos(theta[5])) + -0.088 * sin(theta[5]))) +
                  cos(theta[0]) * sin(theta[2]) * (sin(theta[3]) *
      (0.0825 + cos(theta[4]) * (-0.088 * cos(theta[5]) + -0.107 *
      sin(theta[5]))) + cos(theta[3]) * ((-0.384 + 0.107 * cos(theta[5]))
      + -0.088 * sin(theta[5])))) + sin(theta[0]) * sin(theta[1]) *
      (cos(theta[3]) * (-0.0825 + cos(theta[4]) * (0.088 * cos
         (theta[5]) + 0.107 * sin(theta[5]))) + sin(theta[3]) * ((-0.384
         + 0.107 * cos(theta[5])) + -0.088 * sin(theta[5])));
    gyprime[4] = cos(theta[0]) * (cos(theta[3]) * sin(theta[2]) *
      sin(theta[4]) * (-0.088 * cos(theta[5]) + -0.107 * sin(theta
      [5])) + cos(theta[2]) * cos(theta[4]) * (0.088 * cos(theta[5])
      + 0.107 * sin(theta[5]))) + sin(theta[0]) * (sin(theta[1]) *
      sin(theta[3]) * sin(theta[4]) + cos(theta[1]) * (cos
      (theta[4]) * sin(theta[2]) + cos(theta[2]) * cos(theta[3]) *
      sin(theta[4]))) * (-0.088 * cos(theta[5]) + -0.107 * sin
      (theta[5]));
    gyprime[5] = (cos(theta[0]) * (sin(theta[2]) * (sin(theta[3]) *
      (-0.088 * cos(theta[5]) + -0.107 * sin(theta[5])) + cos
      (theta[3]) * cos(theta[4]) * (0.107 * cos(theta[5]) + -0.088 *
      sin(theta[5]))) + cos(theta[2]) * sin(theta[4]) * (0.107 *
      cos(theta[5]) + -0.088 * sin(theta[5]))) + cos(theta[1]) *
                  sin(theta[0]) * (cos(theta[2]) * (sin(theta[3]) *
      (-0.088 * cos(theta[5]) + -0.107 * sin(theta[5])) + cos
      (theta[3]) * cos(theta[4]) * (0.107 * cos(theta[5]) + -0.088 *
      sin(theta[5]))) + sin(theta[2]) * sin(theta[4]) * (-0.107 *
      cos(theta[5]) + 0.088 * sin(theta[5])))) + sin(theta[0]) *
      sin(theta[1]) * (cos(theta[4]) * sin(theta[3]) * (0.107 * cos
      (theta[5]) + -0.088 * sin(theta[5])) + cos(theta[3]) * (0.088 *
      cos(theta[5]) + 0.107 * sin(theta[5])));
    gyprime[6] = 0.0;

    return(gyprime);
  }

  Eigen::Matrix<double, 7, 1> generativeModel::getGzprime(Eigen::Matrix<double, 7, 1> theta){

    gzprime[0] = 0.0;
    gzprime[1] = sin(theta[1]) * ((-0.316 + cos(theta[3]) * ((-0.384 +
      0.107 * cos(theta[5])) + -0.088 * sin(theta[5]))) + sin
      (theta[3]) * ((0.0825 + -0.088 * cos(theta[4]) * cos(theta[5])) +
                    -0.107 * cos(theta[4]) * sin(theta[5]))) + cos
      (theta[1]) * (sin(theta[2]) * sin(theta[4]) * (0.088 * cos
      (theta[5]) + 0.107 * sin(theta[5])) + cos(theta[2]) * ((-0.0825 +
      sin(theta[3]) * ((0.384 + -0.107 * cos(theta[5])) + 0.088 *
      sin(theta[5]))) + cos(theta[3]) * ((0.0825 + -0.088 * cos(theta[4])
      * cos(theta[5])) + -0.107 * cos(theta[4]) * sin(theta[5]))));
    gzprime[2] = sin(theta[1]) * (cos(theta[2]) * sin(theta[4]) *
      (0.088 * cos(theta[5]) + 0.107 * sin(theta[5])) + sin(theta[2])
      * ((0.0825 + sin(theta[3]) * ((-0.384 + 0.107 * cos(theta[5])) +
      -0.088 * sin(theta[5]))) + cos(theta[3]) * ((-0.0825 + 0.088 *
      cos(theta[4]) * cos(theta[5])) + 0.107 * cos(theta[4]) * sin
      (theta[5]))));
    gzprime[3] = cos(theta[1]) * (sin(theta[3]) * ((-0.384 + 0.107 *
      cos(theta[5])) + -0.088 * sin(theta[5])) + cos(theta[3]) *
      ((-0.0825 + 0.088 * cos(theta[4]) * cos(theta[5])) + 0.107 *
       cos(theta[4]) * sin(theta[5]))) + cos(theta[2]) * sin(theta
      [1]) * (cos(theta[3]) * ((0.384 + -0.107 * cos(theta[5])) + 0.088 *
               sin(theta[5])) + sin(theta[3]) * ((-0.0825 + 0.088 *
                cos(theta[4]) * cos(theta[5])) + 0.107 * cos(theta[4]) *
               sin(theta[5])));
    gzprime[4] = sin(theta[4]) * (cos(theta[1]) * sin(theta[3]) * (
      -0.088 * cos(theta[5]) + -0.107 * sin(theta[5])) + cos(theta
      [2]) * cos(theta[3]) * sin(theta[1]) * (0.088 * cos(theta[5])
      + 0.107 * sin(theta[5]))) + cos(theta[4]) * sin(theta[1]) *
      sin(theta[2]) * (0.088 * cos(theta[5]) + 0.107 * sin(theta[5]));
    gzprime[5] = sin(theta[1]) * (cos(theta[2]) * (cos(theta[3]) *
      cos(theta[4]) * (-0.107 * cos(theta[5]) + 0.088 * sin(theta[5]))
      + sin(theta[3]) * (0.088 * cos(theta[5]) + 0.107 * sin(theta
      [5]))) + sin(theta[2]) * sin(theta[4]) * (0.107 * cos(theta[5])
      + -0.088 * sin(theta[5]))) + cos(theta[1]) * (cos(theta[4]) *
      sin(theta[3]) * (0.107 * cos(theta[5]) + -0.088 * sin(theta[5]))
      + cos(theta[3]) * (0.088 * cos(theta[5]) + 0.107 * sin(theta
      [5])));
  gzprime[6] = 0.0;
    return(gzprime);
  }
