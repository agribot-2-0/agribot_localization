/*
* Copyright (C) 2020 Agribot. All rights reserved.
*/

#include <agribot_localization/OdomToNavSatFixNode.h>


OdomToNavSatFixNode::OdomToNavSatFixNode(ros::NodeHandle *nh):
    _nh(nh),
    _tf2Listener(_tf2Buffer),
    _utmZone("")
{
    // Create NavSatFix publisher
    _navSatFixPub = _nh->advertise<sensor_msgs::NavSatFix>
                    ("translated_navsatfix",
                     1,
                     true);

    // Create odometry subscriber
    _odometrySub = _nh->subscribe("input_odometry",
                                  1,
                                  &OdomToNavSatFixNode::odometryCallback,
                                  this);

    // Create "real" NavSatFix subscriber
    _realNavSub = _nh->subscribe("input_navsatfix",
                                  1,
                                  &OdomToNavSatFixNode::realNavSatFixCallback,
                                  this);
}

void OdomToNavSatFixNode::odometryCallback(const nav_msgs::Odometry odomMsg)
{
    sensor_msgs::NavSatFix navSatFixMsg;
    geometry_msgs::PointStamped odomPointMsg;
    geometry_msgs::PointStamped utmPointMsg;

    // We must wait for _utmZone being set in realNavSatFixCallback
    if (_utmZone == "")
    {
        ROS_WARN_STREAM("[OdomToNavSatFixNode] No utmZone set with GPS data."
                        << " Skipping odometry message...");
        return;
    }

    // Rewrite odom to point
    odomPointMsg.header.frame_id = odomMsg.header.frame_id;
    odomPointMsg.point = odomMsg.pose.pose.position;

    // Transform odometry from odom frame to utm frame
    _tf2Buffer.transform(odomPointMsg, utmPointMsg, "utm", ros::Duration(0.5));

    // Convert from UTM to latitude and longitude
    double utm_x = utmPointMsg.point.x;
    double utm_y = utmPointMsg.point.y;

    RobotLocalization::NavsatConversions::UTMtoLL(utm_y,
                                                  utm_x,
                                                  _utmZone,
                                                  navSatFixMsg.latitude,
                                                  navSatFixMsg.longitude);

    // Fill-in header of navSatFixMsg
    navSatFixMsg.header.stamp = odomMsg.header.stamp;
    navSatFixMsg.header.frame_id = "ll";

    // Publish NavSatFix message
    _navSatFixPub.publish(navSatFixMsg);
}

void OdomToNavSatFixNode::realNavSatFixCallback(
    const sensor_msgs::NavSatFix navSatFixMsg)
{
    double utmNorthing, utmEasting;

    RobotLocalization::NavsatConversions::LLtoUTM(navSatFixMsg.latitude,
                                                  navSatFixMsg.longitude,
                                                  utmNorthing,
                                                  utmEasting,
                                                  _utmZone);
}

void OdomToNavSatFixNode::run()
{
  ros::spin();
}
