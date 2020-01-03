#include "ros/ros.h"
// #include <boost/bind.hpp>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


class OdomFilterNode
{
public:
  OdomFilterNode():
      // _tf2Listener(_tf2Buffer), _targetFrame("base_footprint"),
      _tf2Listener(_tf2Buffer), _targetFrame("turtle1"),
      _tf2Filter(_inOdomSub, _tf2Buffer, _targetFrame, 10, 0)
  {
    // _inOdomSub.subscribe(_nh, "agribot/odom/raw_front_cart", 1);
    _inOdomSub.subscribe(_nh, "turtle_point_stamped", 1);
    _tf2Filter.registerCallback(boost::bind(&OdomFilterNode::msgCallback, this, _1));
    ROS_ERROR("Done");
  }

  void msgCallback(const geometry_msgs::PointStampedConstPtr& point_ptr)
  {
    ROS_ERROR("Odom bind callback");
  }

  void run()
  {
    ros::spin();
  }

private:
  std::string _targetFrame;
  ros::NodeHandle _nh;
  tf2_ros::Buffer _tf2Buffer;
  tf2_ros::TransformListener _tf2Listener;
  tf2_ros::MessageFilter<geometry_msgs::PointStamped> _tf2Filter;
  message_filters::Subscriber<geometry_msgs::PointStamped> _inOdomSub;
  ros::Publisher _outOdomPub;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_filter_node");
  OdomFilterNode _odomFilterNode;
  // _odomFilterNode.run();
  ros::spin();
  return 0;
}
