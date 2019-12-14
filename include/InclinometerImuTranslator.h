# pragma once
#include <agribot_common/AgribotComponent.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <agribot_msgs/AgribotInclinometer.h>

class InclinometerImuTranslator: public AgribotComponent
{
private:
  ros::NodeHandle _nh;
  ros::Publisher _inclinometerImuPublisher;
  ros::Subscriber _inclinometerSubscriber;
  void inclinometerCallback(const agribot_msgs::AgribotInclinometer msg);

public:
  InclinometerImuTranslator(const ros::NodeHandle &node_handle);
  virtual int configure();
  virtual bool startup();
  virtual bool update();
  virtual bool stop();
  virtual bool cleanup();
};
