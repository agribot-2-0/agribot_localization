/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <robot_localization/navsat_conversions.h>
#include <tf2/utils.h>
#pragma GCC diagnostic pop

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <string>

class OdomToNavSatFixNode
{
private:
  ros::NodeHandle* _nh;
  ros::Publisher _navSatFixPub;
  ros::Subscriber _odometrySub;
  ros::Subscriber _realNavSub;
  tf2_ros::Buffer _tf2Buffer;
  tf2_ros::TransformListener _tf2Listener;

  std::string _utmZone;

  void odometryCallback(const nav_msgs::Odometry odomMsg);
  void realNavSatFixCallback(const sensor_msgs::NavSatFix navSatFixMsg);

public:
  explicit OdomToNavSatFixNode(ros::NodeHandle* nh);
  void run();
};
