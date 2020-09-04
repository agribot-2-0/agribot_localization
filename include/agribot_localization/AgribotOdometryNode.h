/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_localization/AgribotOdometry.h>
#include <agribot_msgs/AgribotCaterpillar.h>
#include <agribot_msgs/AgribotCaterpillarWheels.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf2/utils.h>
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

  double _wheelDiameter;  // m
  double _wheelbase;      // m

  double x = 0;
  double y = 0;
  double theta = 0;
  double front_left_last = 0;
  double front_right_last = 0;
  bool firstMessage = true;

  void caterpillarCallback(const agribot_msgs::AgribotCaterpillar msg);
  void caterpillarWheelsCallback(const agribot_msgs::AgribotCaterpillarWheels msg);
  void frontCartCallback(const std_msgs::Float32 msg);
  void backCartCallback(const std_msgs::Float32 msg);

  void publishOdometry();
  bool updateDt(double newTimestamp);

public:
  AgribotOdometryNode();
  void run();
};
