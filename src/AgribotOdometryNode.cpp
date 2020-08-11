/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_localization/AgribotOdometryNode.h>

AgribotOdometryNode::AgribotOdometryNode() : _bTimestampSet(false)
{
  float wheelRadius, chassisLength;

  if (_nh.getParam("wheelRadius", wheelRadius))
  {
    _agribotOdom.vSetWheelRadius(wheelRadius);
  }

  if (_nh.getParam("chassisLength", chassisLength))
  {
    _agribotOdom.vSetChassisLength(chassisLength);
  }

  _odometryPub = _nh.advertise<nav_msgs::Odometry>("agribot/odom/raw", 10);
  _odometryMsg.header.frame_id = "odom";
  _odometryMsg.child_frame_id = "base_footprint";

  _caterpillarSub = _nh.subscribe("agribot/encoder/caterpillar", 1, &AgribotOdometryNode::caterpillarCallback, this);
  _frontCartSub = _nh.subscribe("agribot/encoder/cart/front", 1, &AgribotOdometryNode::frontCartCallback, this);
  _backCartSub = _nh.subscribe("agribot/encoder/cart/back", 1, &AgribotOdometryNode::backCartCallback, this);
}

void AgribotOdometryNode::caterpillarCallback(const agribot_msgs::AgribotCaterpillar msg)
{
  _agribotOdom.vSetEta1(msg.front_linear);

  if (!updateDt(msg.header.stamp.toSec()))
  {
    // First message only sets timestamp for reference
    return;
  }

  if (_agribotOdom.update())
  {
    // Update header
    _odometryMsg.header.seq = msg.header.seq;
    _odometryMsg.header.stamp = msg.header.stamp;
    // Publish odometry
    publishOdometry();
  }
  else
  {
    ROS_WARN("One of theta angles not published yet");
  }
}

bool AgribotOdometryNode::updateDt(double newTimestamp)
{
  // First message only sets timestamp for reference
  if (!_bTimestampSet)
  {
    _dPreviousTimestamp = newTimestamp;
    _bTimestampSet = true;
    return false;
  }
  else
  {
    _agribotOdom.vSetDt(newTimestamp - _dPreviousTimestamp);
    _dPreviousTimestamp = newTimestamp;
    return true;
  }
}

void AgribotOdometryNode::frontCartCallback(const std_msgs::Float32 msg)
{
  _agribotOdom.vSetTheta1(msg.data);
}

void AgribotOdometryNode::backCartCallback(const std_msgs::Float32 msg)
{
  _agribotOdom.vSetTheta2(msg.data);
}

void AgribotOdometryNode::publishOdometry()
{
  // X,Y
  _odometryMsg.pose.pose.position.x = _agribotOdom.fGetX();
  _odometryMsg.pose.pose.position.y = _agribotOdom.fGetY();
  // Yaw
  tf2::Quaternion quat;
  quat.setRPY(0, 0, _agribotOdom.fGetTheta0());
  _odometryMsg.pose.pose.orientation = tf2::toMsg(quat);
  // Pose covariance
  _odometryMsg.pose.covariance[0] = POSITION_COVARIANCE;
  _odometryMsg.pose.covariance[7] = POSITION_COVARIANCE;
  _odometryMsg.pose.covariance[35] = ORIENTATION_COVARIANCE;

  // X,Y linear velocities (in child_frame_id)
  _odometryMsg.twist.twist.linear.x = _agribotOdom.fGetEta1();
  _odometryMsg.twist.twist.linear.y = 0.0;
  // Yaw angular velocity
  _odometryMsg.twist.twist.angular.z = _agribotOdom.fGetAngularVelocityZ();
  // Twist covariances
  _odometryMsg.twist.covariance[0] = LINEAR_VEL_X_COVARIANCE;
  _odometryMsg.twist.covariance[7] = LINEAR_VEL_Y_COVARIANCE;
  _odometryMsg.twist.covariance[35] = ANGULAR_VEL_Z_COVARIANCE;

  _odometryPub.publish(_odometryMsg);
}

void AgribotOdometryNode::run()
{
  ros::spin();
}
