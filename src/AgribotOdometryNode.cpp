/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_localization/AgribotOdometryNode.h>

AgribotOdometryNode::AgribotOdometryNode() : _bTimestampSet(false), _nh("~")
{
  float wheelRadius, chassisLength;

  if (_nh.hasParam("wheel_radius"))
  {
    _nh.getParam("wheel_radius", wheelRadius);
    _agribotOdom.vSetWheelRadius(wheelRadius);
  }
  else
  {
    ROS_FATAL_STREAM("[Agribot Odometry Node] Missing wheel_radius parameter. Exiting...");
    _nh.shutdown();
    exit(0);
  }

  if (_nh.hasParam("chassis_length"))
  {
    _nh.getParam("chassis_length", chassisLength);
    _agribotOdom.vSetChassisLength(chassisLength);
  }
  else
  {
    ROS_FATAL_STREAM("[Agribot Odometry Node] Missing chassis_length parameter. Exiting...");
    _nh.shutdown();
    exit(0);
  }

  if (_nh.hasParam("orientation_covariance"))
  {
    _nh.getParam("orientation_covariance", _fOrientationCovariance);
  }
  else
  {
    ROS_FATAL_STREAM("[Agribot Odometry Node] Missing orientation_covariance parameter. Exiting...");
    _nh.shutdown();
    exit(0);
  }

  if (_nh.hasParam("position_covariance"))
  {
    _nh.getParam("position_covariance", _fPositionCovariance);
  }
  else
  {
    ROS_FATAL_STREAM("[Agribot Odometry Node] Missing position_covariance parameter. Exiting...");
    _nh.shutdown();
    exit(0);
  }

  if (_nh.hasParam("linear_velocity_x_covariance"))
  {
    _nh.getParam("linear_velocity_x_covariance", _fLinearVelocityXCovariance);
  }
  else
  {
    ROS_FATAL_STREAM("[Agribot Odometry Node] Missing linear_velocity_x_covariance parameter. Exiting...");
    _nh.shutdown();
    exit(0);
  }

  if (_nh.hasParam("linear_velocity_y_covariance"))
  {
    _nh.getParam("linear_velocity_y_covariance", _fLinearVelocityYCovariance);
  }
  else
  {
    ROS_FATAL_STREAM("[Agribot Odometry Node] Missing linear_velocity_y_covariance parameter. Exiting...");
    _nh.shutdown();
    exit(0);
  }

  if (_nh.hasParam("angular_velocity_z_covariance"))
  {
    _nh.getParam("angular_velocity_z_covariance", _fAngularVelocityZCovariance);
  }
  else
  {
    ROS_FATAL_STREAM("[Agribot Odometry Node] Missing angular_velocity_z_covariance parameter. Exiting...");
    _nh.shutdown();
    exit(0);
  }

  _odometryPub = _nh.advertise<nav_msgs::Odometry>("odom/raw", 10);
  _odometryWheelsPub = _nh.advertise<nav_msgs::Odometry>("odom/raw/wheels", 10);
  _odometryMsg.header.frame_id = "odom";
  _odometryMsg.child_frame_id = "base_footprint";

  _caterpillarSub = _nh.subscribe("encoder/caterpillar", 1, &AgribotOdometryNode::caterpillarCallback, this);
  _frontCartSub = _nh.subscribe("encoder/cart/front", 1, &AgribotOdometryNode::frontCartCallback, this);
  _backCartSub = _nh.subscribe("encoder/cart/back", 1, &AgribotOdometryNode::backCartCallback, this);
  _caterpillarWheelsSub =
      _nh.subscribe("encoder/caterpillar/wheels", 1, &AgribotOdometryNode::caterpillarWheelsCallback, this);
}

void AgribotOdometryNode::caterpillarWheelsCallback(const agribot_msgs::AgribotCaterpillarWheels msg)
{
  if (firstMessage)
  {
    front_right_last = msg.front_right;
    front_left_last = msg.front_left;
    firstMessage = false;
  }
  double front_right_diff = msg.front_right - front_right_last;
  double front_left_diff = msg.front_left - front_left_last;

  // front_right_diff = limitDiff(front_right_diff);
  // front_left_diff = limitDiff(front_left_diff);

  front_right_last = msg.front_right;
  front_left_last = msg.front_left;

  double dfr = (wheel_diameter / 2 * front_right_diff);
  double dfl = (wheel_diameter / 2 * front_left_diff);

  double dfc = (dfr + dfl) / 2;
  double dtheta = (dfr - dfl) / wheelbase;

  // ROS_WARN_STREAM("dfr " << dfr << " dfl " << dfl);
  // ROS_WARN_STREAM("dfc " << dfc << " dtheta " << dtheta);

  // x += dfc * cos(theta);
  // y += dfc * sin(theta);
  if (abs(dtheta) < 1e-6)
  {
    x += dfl * cos(theta);
    y += dfr * sin(theta);
  }
  else
  {
    float icc = dfc / dtheta;
    x += icc * sin(dtheta + theta) - icc * sin(theta);
    y += -icc * cos(dtheta + theta) + icc * cos(theta);
    theta += dtheta;
    // ROS_WARN_STREAM("icc " << icc);
  }

  _odometryMsg.header.stamp = ros::Time::now();

  _odometryWheelMsg.header.frame_id = "odom";
  _odometryWheelMsg.child_frame_id = "base_footprint";
  // X,Y
  _odometryWheelMsg.pose.pose.position.x = x;
  _odometryWheelMsg.pose.pose.position.y = y;
  // Yaw
  tf2::Quaternion quat;
  quat.setRPY(0, 0, theta);
  _odometryWheelMsg.pose.pose.orientation = tf2::toMsg(quat);
  // Pose covariance
  _odometryWheelMsg.pose.covariance[0] = _fPositionCovariance;
  _odometryWheelMsg.pose.covariance[7] = _fPositionCovariance;
  _odometryWheelMsg.pose.covariance[35] = _fOrientationCovariance;

  _odometryWheelsPub.publish(_odometryWheelMsg);
}

float AgribotOdometryNode::limitDiff(float encoder_diff)
{
  if (abs(encoder_diff) > (impulses_max / 2))
  {
    return abs(encoder_diff) - impulses_max;
  }
  else
  {
    return encoder_diff;
  }
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
  _odometryMsg.pose.covariance[0] = _fPositionCovariance;
  _odometryMsg.pose.covariance[7] = _fPositionCovariance;
  _odometryMsg.pose.covariance[35] = _fOrientationCovariance;

  // X,Y linear velocities (in child_frame_id)
  _odometryMsg.twist.twist.linear.x = _agribotOdom.fGetEta1();
  _odometryMsg.twist.twist.linear.y = 0.0;
  // Yaw angular velocity
  _odometryMsg.twist.twist.angular.z = _agribotOdom.fGetAngularVelocityZ();
  // Twist covariances
  _odometryMsg.twist.covariance[0] = _fLinearVelocityXCovariance;
  _odometryMsg.twist.covariance[7] = _fLinearVelocityYCovariance;
  _odometryMsg.twist.covariance[35] = _fAngularVelocityZCovariance;

  _odometryPub.publish(_odometryMsg);
}

void AgribotOdometryNode::run()
{
  ros::spin();
}
