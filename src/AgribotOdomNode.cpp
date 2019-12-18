#include <AgribotOdomNode.h>

AgribotOdomNode::AgribotOdomNode():
  _tfListener(_tfBuffer)
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

  _odometryPub = _nh.advertise<nav_msgs::Odometry>("agribot/odom/raw", 1);
  _caterpillarSub = _nh.subscribe("agribot/encoder/caterpillar",
                                  1,
                                  &AgribotOdomNode::caterpillarCallback,
                                  this);
  _frontCartSub = _nh.subscribe("agribot/encoder/cart/front",
                                1,
                                &AgribotOdomNode::frontCartCallback,
                                this);
  _rearCartSub = _nh.subscribe("agribot/encoder/cart/rear",
                              1,
                              &AgribotOdomNode::rearCartCallback,
                              this);
}

void AgribotOdomNode::caterpillarCallback(const agribot_msgs::AgribotCaterpillar msg)
{
  _agribotOdom.vSetEta1(msg.front_linear);

  try
  {
    updateTheta0FromLookupTransform();
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("Skipping publication of odometry - %s",ex.what());
    return;
  }

  if(!_agribotOdom.update())
  {
    ROS_WARN("Skipping publication of odometry - one of theta angles not published yet");
    return;
  }

  nav_msgs::Odometry odomMsg = nav_msgs::Odometry();
  odomMsg.twist.twist.linear.x = _agribotOdom.fGetLinearVelocityX();
  odomMsg.twist.twist.linear.y = _agribotOdom.fGetLinearVelocityY();
  odomMsg.twist.twist.angular.z = _agribotOdom.fGetAngularVelocityZ();
  odomMsg.header = msg.header;
  odomMsg.header.frame_id = "odom";
  odomMsg.child_frame_id = "front_cart_link";

  odomMsg.twist.covariance[0] = LINEAR_VEL_X_COVARIANCE;
  odomMsg.twist.covariance[7] = LINEAR_VEL_Y_COVARIANCE;
  odomMsg.twist.covariance[35] = ANGULAR_VEL_Z_COVARIANCE;

  _odometryPub.publish(odomMsg);
}

void AgribotOdomNode::frontCartCallback(const std_msgs::Float32 msg)
{
  _agribotOdom.vSetTheta1(msg.data);
}

void AgribotOdomNode::rearCartCallback(const std_msgs::Float32 msg)
{
  _agribotOdom.vSetTheta2(msg.data);
}

bool AgribotOdomNode::updateTheta0FromLookupTransform()
{
  geometry_msgs::TransformStamped transformStamped;
  transformStamped = _tfBuffer.lookupTransform("base_footprint",
                                                "odom",
                                                ros::Time(0),
                                                ros::Duration(0.1));
  float theta0 = tf2::getYaw(transformStamped.transform.rotation);
  _agribotOdom.vSetTheta0(theta0);
  return true;
}

void AgribotOdomNode::run()
{
  ros::spin();
}
