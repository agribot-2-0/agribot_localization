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

  double x = 0;
  double y = 0;
  double theta = 0;
  double front_left_last = 0;
  double front_right_last = 0;
  bool firstMessage = true;

  static constexpr double wheel_diameter = 0.32;  // m
  // static constexpr double impulses_per_wheel = 1024;
  static constexpr double wheelbase = 0.8;
  static constexpr double impulses_max = 4294967296;

  void caterpillarCallback(const agribot_msgs::AgribotCaterpillar msg);
  void caterpillarWheelsCallback(const agribot_msgs::AgribotCaterpillarWheels msg);
  void frontCartCallback(const std_msgs::Float32 msg);
  void backCartCallback(const std_msgs::Float32 msg);

  float limitDiff(float encoder_diff);

  void publishOdometry();
  bool updateDt(double newTimestamp);

public:
  AgribotOdometryNode();
  void run();
};
