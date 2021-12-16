#ifndef ODOM_H
#define ODOM_H

// ROS lib (roscpp)
#include <ros/ros.h>

// Pose msg
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// ROS Transform
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>

using namespace std;

#include "Odometry.h"

class Odom
{
  // ROS stuff
  ros::NodeHandle n;

  ros::Subscriber subCmdVel;
  ros::Publisher pubOdom;
  ros::Publisher pubIniPose;
  tf2_ros::TransformBroadcaster tfBroadcastOdom;

  ros::Duration tfBroadcastPeriod;
  ros::Timer tfBroadcastTimer;

  string worldFrameId="map";
  string odomFrameId="odom";
  string baseLinkFrameId="son";

  Odometry odom;

  void publishInitialPose(const ros::Time &time,
                          double x, double y, double yaw);

  void CmdVelCallback(const geometry_msgs::TwistStamped &msg);

  void initROS();

public:
  Odom();

  void start();

};

#endif // ODOM_H
