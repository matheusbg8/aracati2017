#include "AerialImageNode.h"
#include <tf2/utils.h>

bool AerialImageNode::getUTMRef()
{
  ROS_INFO("Waiting first GPS msg to estimate UTM ref.");
  sensor_msgs::NavSatFix::ConstPtr msg =
    ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/dgps");

  if (msg != nullptr)
  {
    // Get UTM Ref
    Point2d latLong(msg->latitude,msg->longitude);
    utmRef = latLon2UTM(latLong);
    hasUTMRef = true;
    return true;
  }
  return false;
}

bool AerialImageNode::initROS()
{
  ros::NodeHandle nh("~");

  // Get parameters:
  string aerial_img_path;
  if(!nh.getParam("aerial_image_path",aerial_img_path))
  {
    ROS_ERROR("Missing parameter aerial_image_path point to the .yaml file.");
    return false;
  }

  // Load aerial image
  if(!ai.loadMap(aerial_img_path))
  {
    ROS_ERROR("It could not load the aerial image: %s",
              aerial_img_path.c_str());
    return false;
  }
//  ai.resizeToMaxCols(800);

  // Load sonar FoV shape
  double sonOpenning, sonMinRange, sonMaxRange;
  nh.param<double>("son_fov_openning", sonOpenning, 130.0); // BlueView P900-130
  nh.param<double>("son_min_range", sonMinRange, 0.4); // BlueView P900 can be set from 0.4m to 100m
  nh.param<double>("son_max_range", sonMaxRange, 50.0); // BlueView P900 can be set from 0.4m to 100m (aracati2017 use 50m)

  sonShape.initShape(sonOpenning,
                     sonMaxRange,
                     sonMinRange);

  // Topic Subscriptions
  subGtPose = n.subscribe("/gt_pose",5,
        &AerialImageNode::gtPoseCallback,this);

  subOdomPose = n.subscribe("/odom",5,
        &AerialImageNode::odomPoseCallback,this);

  // Topic Advertisements
  pubAerialImgs = it.advertise("/aerial_img",1);
  pubSonAerial = it.advertise("/son_aerial",1);

  // Get a UTM reference to the vehicle position
  // We assume the first DGPS msg (Ground Truth)
  // match the first vehicle position (It is true
  // on dataset aracati2017).

  // Otherwise the time difference between gps
  // and first vehicle position should be take
  // in account and the UTMRef should be interpolated
  // to get an approximated UTMRef.

  // It is not a good idea to use UTM in ROS, especially
  // on TF tree. We had some problems working
  // with high values, so we decided to remove the
  // UTM offset in the dataset.
  getUTMRef(); // This may wait forever (No timeout)

  return true;
}

void AerialImageNode::odomPoseCallback(const geometry_msgs::PoseStamped &msg)
{

}

void AerialImageNode::gtPoseCallback(const geometry_msgs::PoseStamped &msg)
{
  Mat aerialImg = ai.getMapImg().clone();

  if(hasUTMRef)
  {
    Point2d UTMSonPosition = utmRef + Point2d(msg.pose.position.x,
                                           msg.pose.position.y);

    double heading= tf2::getYaw(msg.pose.orientation);

    sonShape.drawPoly(aerialImg,ai,
                      UTMSonPosition,heading,
                      Scalar(255,0,255),
                      2);

    // Publish sonar aerial image
    Mat sonAerialImg = sonShape.cropSonShape(ai,UTMSonPosition,heading);
    sensor_msgs::ImagePtr imgMsg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", sonAerialImg).toImageMsg();

    imgMsg->header = msg.header;
    pubSonAerial.publish(imgMsg);
  }

  // Publish aerial image
  sensor_msgs::ImagePtr imgMsg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", aerialImg).toImageMsg();

  imgMsg->header = msg.header;
  pubAerialImgs.publish(imgMsg);
}

AerialImageNode::AerialImageNode():
  it(n)
{

}

void AerialImageNode::start()
{
  initROS();
  ros::spin();
}
