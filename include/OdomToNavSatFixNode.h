#pragma once

#include <string>

#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <robot_localization/navsat_conversions.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>

class OdomToNavSatFixNode
{
 private:
    ros::NodeHandle *_nh;
    ros::Publisher _navSatFixPub;
    ros::Subscriber _odometrySub;
    ros::Subscriber _realNavSub;
    tf2_ros::Buffer _tf2Buffer;
    tf2_ros::TransformListener _tf2Listener;

    std::string _utmZone;

    void odometryCallback(const nav_msgs::Odometry odomMsg);
    void realNavSatFixCallback(const sensor_msgs::NavSatFix navSatFixMsg);

 public:
    explicit OdomToNavSatFixNode(ros::NodeHandle *nh);
    void run();
};
