#include "ros/ros.h"
// #include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class OdomFilterNode
{
public:
  OdomFilterNode() :
    _tf2Listener(_tf2Buffer),  _targetFrame("base_footprint"),
    _tf2Filter(_inOdomSub, _tf2Buffer, _targetFrame, 10, 0)
  {
    _inOdomSub.subscribe(_nh, "agribot/odom/raw_front_cart", 10);
    _outOdomPub = _nh.advertise<nav_msgs::Odometry>("agribot/odom/raw", 5);
    _tf2Filter.registerCallback( boost::bind(&OdomFilterNode::msgCallback, this, _1) );
  }

  //  Callback to register with tf2_ros::MessageFilter to be called when transforms are available
  // void msgCallback(const nav_msgs::OdometryConstPtr& odom_in)
  // {
  //   ROS_INFO("Callback");
  //
  //   geometry_msgs::PoseStamped pose_in;
  //   geometry_msgs::PoseStamped pose_out;
  //
  //   nav_msgs::Odometry odom_out;
  //
  //   pose_in.pose = odom_in->pose.pose;
  //   pose_in.header.frame_id = odom_in->child_frame_id;
  //
  //   // TODO - recalculate odom
  //   _tf2Buffer.transform(pose_in, pose_out, _targetFrame);
  //
  //
  //   odom_out.header = odom_in->header;
  //   odom_out.child_frame_id = _targetFrame;
  //   odom_out.pose.pose = pose_out.pose;
  //   odom_out.pose.covariance = odom_in->pose.covariance;
  //
  //   _outOdomPub.publish(odom_out);
  // }

  void msgCallback(const nav_msgs::OdometryConstPtr& odom_in)
  {
    ROS_INFO("Callback");

    nav_msgs::Odometry odom_out;

    // TODO - recalculate odom
    geometry_msgs::TransformStamped odomBaseTransform;
    geometry_msgs::TransformStamped odomCartTransform;
    odomBaseTransform = _tf2Buffer.lookupTransform(
                                                  "odom",
                                                  _targetFrame,
                                                  ros::Time(0),
                                                  ros::Duration(0.1));

    odomCartTransform = _tf2Buffer.lookupTransform(
                                                  "odom",
                                                  odom_in->child_frame_id,
                                                  ros::Time(0),
                                                  ros::Duration(0.1));

    float dx, dy, dz;
    dx = odomBaseTransform.transform.translation.x - odomCartTransform.transform.translation.x;
    dy = odomBaseTransform.transform.translation.y - odomCartTransform.transform.translation.y;
    // dz = odomBaseTransform.transform.translation.z - odomCartTransform.transform.translation.z;
    dz = 0;

    // TODO, add rotation
    odom_out.header = odom_in->header;
    odom_out.child_frame_id = _targetFrame;
    odom_out.pose.pose = odom_in->pose.pose;
    odom_out.pose.covariance = odom_in->pose.covariance;

    // Translate
    odom_out.pose.pose.position.x += dx;
    odom_out.pose.pose.position.y += dy;
    odom_out.pose.pose.position.z = 0.0;

    ROS_INFO("%f\t%f\t%f", dx, dy, dz);

    // // Rotate
    // tf2::Quaternion q_orig, q_rot, q_new;
    //
    // // Get original quaternion
    // tf2::convert(odom_out.pose.pose.orientation , q_orig);
    // // Get rotation quaternion
    // tf2::convert(transformStamped.transform.rotation , q_rot);
    //
    // ROS_INFO("%f\t%f\t%f", transformStamped.transform.translation.x,
    //                       transformStamped.transform.translation.y,
    //                       transformStamped.transform.translation.z);
    //
    // // Calculate the new orientation
    // q_new = q_rot*q_orig;
    // q_new.normalize();
    // // Put new orientation into the pose
    // tf2::convert(q_new, odom_out.pose.pose.orientation);

    _outOdomPub.publish(odom_out);
  }

private:
  std::string _targetFrame;
  tf2_ros::Buffer _tf2Buffer;
  tf2_ros::TransformListener _tf2Listener;
  ros::NodeHandle _nh;
  message_filters::Subscriber<nav_msgs::Odometry> _inOdomSub;
  tf2_ros::MessageFilter<nav_msgs::Odometry> _tf2Filter;
  ros::Publisher _outOdomPub;
};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "odom_filter_node"); //Init ROS
  OdomFilterNode pd; //Construct class

  tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base_footprint";
  transformStamped.transform.translation.x = -1.05;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);

  ros::spin(); // Run until interupted
  return 0;
};
