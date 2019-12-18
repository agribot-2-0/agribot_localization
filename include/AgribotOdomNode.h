#include <AgribotOdometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <agribot_msgs/AgribotCaterpillar.h>

class AgribotOdomNode
{
private:
  ros::NodeHandle _nh;
  ros::Publisher _odometryPub;
  ros::Subscriber _caterpillarSub;
  ros::Subscriber _frontCartSub;
  ros::Subscriber _rearCartSub;

  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener _tfListener;

  AgribotOdometry _agribotOdom;

  void caterpillarCallback(const agribot_msgs::AgribotCaterpillar msg);
  void frontCartCallback(const std_msgs::Float32 msg);
  void rearCartCallback(const std_msgs::Float32 msg);

  bool updateTheta0FromLookupTransform();

public:
  AgribotOdomNode();
  void run();
};
