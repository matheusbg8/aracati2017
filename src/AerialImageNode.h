#ifndef AERIALIMAGENODE_H
#define AERIALIMAGENODE_H

#include "AerialImage.h"
#include "SonShape.h"
#include "CircularVector.h"

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

  std_msgs::Header aerialImgHeader;

  // UTM reference of vehicle position (Offset)
  bool hasUTMRef=false;
  Point2d utmRef;

  // Paths on aerial image
  CircularVector odomPts;
  CircularVector gtPts;
  double lastHeading=0.0;

  bool getUTMRef();
  bool initROS();

  void odomPoseCallback(const geometry_msgs::PoseStamped &msg);
  void gtPoseCallback(const geometry_msgs::PoseStamped &msg);

  void publishAerialImg();

public:
  AerialImageNode();

  void start();

};

#endif // AERIALIMAGENODE_H
