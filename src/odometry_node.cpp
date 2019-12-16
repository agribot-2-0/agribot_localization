#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <agribot_msgs/AgribotCaterpillar.h>

ros::Publisher odometryPub;

void caterpillarCallback(const agribot_msgs::AgribotCaterpillar msg)
{
  nav_msgs::Odometry odomMsg = nav_msgs::Odometry();

  odomMsg.header = msg.header;

  // TODO - calculate odom velocities (3 first equations from 2.16?)
  // odomMsg.child_frame_id = "front_cart_link";
  // odomMsg.twist.twist.linear.x = msg.front_linear;
  // odomMsg.twist.twist.angular.z = msg.front_angular;

  odometryPub.publish(odomMsg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_node");
  ros::NodeHandle nh;

  odometryPub = nh.advertise<nav_msgs::Odometry>("agribot/odom/raw", 1);
  ros::Subscriber caterpillarSub = nh.subscribe("agribot/encoder/caterpillar", 1, caterpillarCallback);

  ros::spin();

  return 0;
}
