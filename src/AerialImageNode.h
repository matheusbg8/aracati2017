#ifndef AERIALIMAGENODE_H
#define AERIALIMAGENODE_H

#include "AerialImage.h"
#include "SonShape.h"

// ROS lib (roscpp)
#include <ros/ros.h>

// ROS Img transport plugin
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Pose msg
#include <geometry_msgs/PoseStamped.h>


class AerialImageNode
{
  AerialImage ai; // Aerial Image (ai)
  SonShape sonShape; // Manipulate sonar field of view on aerial image

  // ROS stuff
  ros::NodeHandle n;
  ros::Subscriber subGtPose;
  ros::Subscriber subOdomPose;

  image_transport::ImageTransport it;
  image_transport::Publisher pubAerialImgs;
  image_transport::Publisher pubSonAerial;

  // UTM reference of vehicle position (Offset)
  bool hasUTMRef=false;
  Point2d utmRef;

  bool getUTMRef();
  bool initROS();

  void odomPoseCallback(const geometry_msgs::PoseStamped &msg);
  void gtPoseCallback(const geometry_msgs::PoseStamped &msg);

public:
  AerialImageNode();

  void start();

};

#endif // AERIALIMAGENODE_H
