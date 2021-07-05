/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <agribot_localization/RewriteOdomLocalToGlobalNode.h>

RewriteOdomLocalToGlobalNode::RewriteOdomLocalToGlobalNode(ros::NodeHandle* nh) : nh_(nh)
{
  target_frame_id_ = ros_common_functions::getRosParam<std::string>("~target_frame_id");
  global_odom_pub_ = nh_->advertise<nav_msgs::Odometry>("global_odom", 1, false);
  local_odom_pub_ = nh_->subscribe("local_odom", 1, &RewriteOdomLocalToGlobalNode::localOdometryCallback, this);
}

void RewriteOdomLocalToGlobalNode::localOdometryCallback(const nav_msgs::Odometry& in_msg)
{
  auto out_msg = in_msg;
  out_msg.header.frame_id = target_frame_id_;
  global_odom_pub_.publish(out_msg);
}

void RewriteOdomLocalToGlobalNode::run()
{
  ros::spin();
}
