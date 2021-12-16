#include "Odom.h"

void Odom::publishInitialPose(const ros::Time &time,
                                  double x, double y, double yaw)
{
  geometry_msgs::PoseWithCovarianceStamped msg;

  msg.header.stamp = time;
  msg.header.frame_id = worldFrameId;

  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = 0.0;

  tf2::Quaternion quat_tf;
  quat_tf.setRPY(0.0,0.0,yaw);
  tf2::convert(quat_tf,msg.pose.pose.orientation);

  double *m = msg.pose.covariance.elems;
  for(uint i = 0; i < 36;i++)
    m[i]=0.0;

  m[6*0+0] = 0.5*0.5;   // CovX
  m[6*1+1] = 0.5*0.5;   // CovY
  m[6*3+3] = 0.4*0.4;   // CovYaw

  // Publish Initial Pose
  pubIniPose.publish(msg);
}

void Odom::CmdVelCallback(const geometry_msgs::TwistStamped &msg)
{
  double time = msg.header.stamp.toSec();

  // Update odometry with fake local velocity
  odom.updateVelocity(time,
                      msg.twist.linear.x,
                      msg.twist.linear.y,
                      msg.twist.angular.z);

  double odomX,odomY,odomYaw;

  // Predict odometry on time
  if(odom.predict(time,odomX,odomY,odomYaw))
  {
    // Pose msg
    geometry_msgs::PoseStamped msg;

    msg.header.stamp = ros::Time(time);
    msg.header.frame_id = odomFrameId;
    msg.pose.position.x = odomX;
    msg.pose.position.y = odomY;
    msg.pose.position.z = 0.0;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0,0.0,odomYaw);
    tf2::convert(quat_tf,msg.pose.orientation);

    cout << "Odom yaw: " << (odomYaw *180/M_PI) << endl;

    // Publish odom
    pubOdom.publish(msg);

    // Publish TF
    geometry_msgs::TransformStamped tf_odom;
    tf_odom.header.stamp = msg.header.stamp;
    tf_odom.header.frame_id = odomFrameId;
    tf_odom.child_frame_id = baseLinkFrameId;

    tf_odom.transform.translation.x = odomX;
    tf_odom.transform.translation.y = odomY;
    tf_odom.transform.translation.z = 0.0;

    tf_odom.transform.rotation = tf2::toMsg(quat_tf);

    tfBroadcastOdom.sendTransform(tf_odom);
  }
}

void Odom::initROS()
{
  subCmdVel = n.subscribe("/cmd_vel",5,
        &Odom::CmdVelCallback,this);

  pubOdom = n.advertise<geometry_msgs::PoseStamped>("/odom_pose",1);
  pubIniPose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);

  ros::NodeHandle nh("~");

  nh.param<string>("world_frame_id", worldFrameId, "map");
  nh.param<string>("odom_frame_id", odomFrameId, "odom");
  nh.param<string>("base_frame_id", baseLinkFrameId, "son");

  // We assume the initial vehicle position
  // is know in order to generate odometry
  // so the referential of our ground truth
  // and the odom are the same.

  // Receiving first vehicle pose
  ROS_INFO("Waiting first vehicle pose.");
  geometry_msgs::PoseStamped::ConstPtr msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/pose_gt");

  if (msg != nullptr)
  {
    // Get yaw
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.orientation,q);

    tf::Matrix3x3 mat(q);
    double yaw=tf2::getYaw(msg->pose.orientation),
        time = msg->header.stamp.toSec(),
        x = msg->pose.position.x,
        y = msg->pose.position.y;

    odom.setInitialPose(time,x,y,yaw);
    publishInitialPose(msg->header.stamp,
                       x,y,yaw);

  }
  else
  {
    ROS_ERROR("No initial position received!");
    ros::Time time = ros::Time::now();
    odom.setInitialPose(time.toSec(),
                        0.0, 0.0,0.0);
    publishInitialPose(msg->header.stamp,
                       0.0,0.0,0.0);
  }
}

Odom::Odom()
{
  initROS();
}

void Odom::start()
{
  ros::spin();
}
