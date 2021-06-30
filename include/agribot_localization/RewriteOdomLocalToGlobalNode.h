/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <agribot_common/CommonFunctions/ros_common_functions.h>

#include <string>

class RewriteOdomLocalToGlobalNode
{
private:
  ros::NodeHandle* nh_;
  ros::Publisher global_odom_pub_;
  ros::Subscriber local_odom_pub_;
  std::string target_frame_id_;

  void localOdometryCallback(const nav_msgs::Odometry& odomMsg);

public:
  void run();

  explicit RewriteOdomLocalToGlobalNode(ros::NodeHandle* nh);
  RewriteOdomLocalToGlobalNode() = delete;
  RewriteOdomLocalToGlobalNode(const RewriteOdomLocalToGlobalNode&) = delete;
  RewriteOdomLocalToGlobalNode(RewriteOdomLocalToGlobalNode&&) = delete;
  RewriteOdomLocalToGlobalNode& operator=(const RewriteOdomLocalToGlobalNode&) = delete;
  RewriteOdomLocalToGlobalNode& operator=(RewriteOdomLocalToGlobalNode&&) = delete;
};
