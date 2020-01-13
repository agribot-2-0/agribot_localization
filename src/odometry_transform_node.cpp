#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"

class OdomFilterNode
{
public:
  OdomFilterNode(std::string inOdomTopic, std::string outOdomTopic, std::string targetFrame) :
    _tf2Listener(_tf2Buffer),  _targetFrame(targetFrame),
    _tf2Filter(_inOdomSub, _tf2Buffer, _targetFrame, 10, 0)
  {
    _inOdomSub.subscribe(_nh, inOdomTopic, 10);
    _outOdomPub = _nh.advertise<nav_msgs::Odometry>(outOdomTopic, 5);
    _tf2Filter.registerCallback(boost::bind(&OdomFilterNode::msgCallback, this, _1));
  }

  void msgCallback(const nav_msgs::OdometryConstPtr& odomInMsgConstPtr)
  {
    geometry_msgs::TransformStamped targetFrameTransform;
    geometry_msgs::TransformStamped childFrameTransform;
    targetFrameTransform = _tf2Buffer.lookupTransform("odom",
                                                      _targetFrame,
                                                      ros::Time(0),
                                                      ros::Duration(0.1));

    childFrameTransform = _tf2Buffer.lookupTransform("odom",
                                                      odomInMsgConstPtr->child_frame_id,
                                                      ros::Time(0),
                                                      ros::Duration(0.1));

    nav_msgs::Odometry odomOutMsg;
    odomOutMsg.header = odomInMsgConstPtr->header;
    odomOutMsg.child_frame_id = _targetFrame;
    odomOutMsg.pose.pose = odomInMsgConstPtr->pose.pose;
    odomOutMsg.pose.covariance = odomInMsgConstPtr->pose.covariance;

    float dx, dy, dz;
    dx = targetFrameTransform.transform.translation.x - childFrameTransform.transform.translation.x;
    dy = targetFrameTransform.transform.translation.y - childFrameTransform.transform.translation.y;
    dz = targetFrameTransform.transform.translation.z - childFrameTransform.transform.translation.z;
    odomOutMsg.pose.pose.position.x = odomInMsgConstPtr->pose.pose.position.x + dx;
    odomOutMsg.pose.pose.position.y = odomInMsgConstPtr->pose.pose.position.y + dy;
    odomOutMsg.pose.pose.position.z = odomInMsgConstPtr->pose.pose.position.z + dz;

    // TODO - transform velocities

    _outOdomPub.publish(odomOutMsg);
  }

private:
  std::string _targetFrame;
  ros::NodeHandle _nh;
  tf2_ros::Buffer _tf2Buffer;
  tf2_ros::TransformListener _tf2Listener;
  message_filters::Subscriber<nav_msgs::Odometry> _inOdomSub;
  tf2_ros::MessageFilter<nav_msgs::Odometry> _tf2Filter;
  ros::Publisher _outOdomPub;
};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "odom_filter_node");

  // TODO - add topics and targetFrame as parameters
  OdomFilterNode pd("agribot/odom/raw_front_cart", "agribot/odom/raw", "base_footprint");

  // TODO - remove transform broadcast (and associated includes)
  tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base_footprint";
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);

  ros::spin();
  return 0;
};
