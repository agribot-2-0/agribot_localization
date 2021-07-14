/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <tf2/utils.h>
#pragma GCC diagnostic pop

#include <agribot_localization/AgribotOdometry.h>
#include <agribot_msgs/AgribotCaterpillar.h>
#include <agribot_msgs/AgribotCaterpillarWheelsRotation.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_listener.h>

class AgribotOdometryNode
{
private:
  ros::NodeHandle _nh;
  ros::Subscriber _caterpillarSub;
  ros::Subscriber _frontCartSub;
  ros::Subscriber _backCartSub;
  ros::Subscriber _caterpillarWheelsSub;
  ros::Publisher _odometryPub;
  ros::Publisher _odometryWheelsPub;
  nav_msgs::Odometry _odometryMsg;
  nav_msgs::Odometry _odometryWheelMsg;

  AgribotOdometry _agribotOdom;
  double _dPreviousTimestamp;
  bool _bTimestampSet;

  float _fOrientationCovariance;
  float _fPositionCovariance;
  float _fLinearVelocityXCovariance;
  float _fLinearVelocityYCovariance;
  float _fAngularVelocityZCovariance;

  agribot_msgs::AgribotCaterpillarWheelsRotation _xWheelsRotationLast;
  bool _bFirstMessage = true;

  void caterpillarCallback(const agribot_msgs::AgribotCaterpillar msg);
  void caterpillarWheelsCallback(const agribot_msgs::AgribotCaterpillarWheelsRotation msg);
  void frontCartCallback(const std_msgs::Float32 msg);
  void backCartCallback(const std_msgs::Float32 msg);

  void publishOdometry();
  bool updateDt(double newTimestamp);

public:
  AgribotOdometryNode();
  void run();
};
