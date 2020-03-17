#pragma once

#include <AgribotOdometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <agribot_msgs/AgribotCaterpillar.h>

static const float ORIENTATION_COVARIANCE   = 0.01;
static const float POSITION_COVARIANCE      = 0.01;
static const float LINEAR_VEL_X_COVARIANCE  = 0.0001;
static const float LINEAR_VEL_Y_COVARIANCE  = 0.0001;
static const float ANGULAR_VEL_Z_COVARIANCE = 0.0001;

class AgribotOdometryNode
{
 private:
    ros::NodeHandle _nh;
    ros::Subscriber _caterpillarSub;
    ros::Subscriber _frontCartSub;
    ros::Subscriber _backCartSub;
    ros::Publisher _odometryPub;
    nav_msgs::Odometry _odometryMsg;

    AgribotOdometry _agribotOdom;
    double _dPreviousTimestamp;
    bool _bTimestampSet;

    void caterpillarCallback(const agribot_msgs::AgribotCaterpillar msg);
    void frontCartCallback(const std_msgs::Float32 msg);
    void backCartCallback(const std_msgs::Float32 msg);

    void publishOdometry();
    bool updateDt(double newTimestamp);

 public:
    AgribotOdometryNode();
    void run();
};
