#include "AIC.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AIC_controller_node");

  // Variables to regulate the flow (Force to read once every 1ms the sensors)
  int count = 0;

  // Object of the class AIC which will take care of everything
  AIC AIC_controller;

  ros::Rate rate(1000);

  // Main loop
  while (ros::ok()){
    // Manage all the callbacks and so read sensors
    ros::spinOnce();

    // Skip only first cycle to allow reading the sensory input first
    if ((count!=0)&&(AIC_controller.dataReady()==1)){
      AIC_controller.minimiseF();
    }
    else
      count ++;

    rate.sleep();
  }

  return 0;
}
