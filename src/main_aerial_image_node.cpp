
// ROS lib (roscpp)
#include <ros/ros.h>

#include "AerialImageNode.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "aerial_img_node");
  AerialImageNode ain;
  ain.start();
}
