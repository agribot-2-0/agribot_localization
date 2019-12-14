# pragma once
#include <agribot_common/AgribotComponent.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <agribot_msgs/AgribotHeading.h>

class HeadingImuTranslator: public AgribotComponent
{
private:
  ros::NodeHandle _nh;
  ros::Publisher _headingImuPublisher;
  ros::Subscriber _headingSubscriber;
  void headingCallback(const agribot_msgs::AgribotHeading msg);

public:
  HeadingImuTranslator(const ros::NodeHandle &node_handle);
  virtual int configure();
  virtual bool startup();
  virtual bool update();
  virtual bool stop();
  virtual bool cleanup();
};
