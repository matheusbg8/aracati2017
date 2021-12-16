
// ROS lib (roscpp)
#include <ros/ros.h>
#include "Odom.h"

/**
 * @brief main - This node integrates the
 * cmd_vel msgs to obtain the vehicle pose.
 */
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "odom_node");
  Odom oNode;
  oNode.start();
}
