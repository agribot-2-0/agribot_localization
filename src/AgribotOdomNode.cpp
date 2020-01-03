#include <AgribotOdomNode.h>

AgribotOdomNode::AgribotOdomNode():
  _tfListener(_tfBuffer),
  _bTimestampSet(false)
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

  _nh.param<bool>("publish_tf", _bPublishTf, "false");

  _odometryPub = _nh.advertise<nav_msgs::Odometry>("agribot/odom/raw_front_cart", 1);
  _odometryMsg.header.frame_id = "odom";
  _odometryMsg.child_frame_id = "front_cart_footprint";


  _caterpillarSub = _nh.subscribe("agribot/encoder/caterpillar",
                                  1,
                                  &AgribotOdomNode::caterpillarCallback,
                                  this);
  _frontCartSub = _nh.subscribe("agribot/encoder/cart/front",
                                1,
                                &AgribotOdomNode::frontCartCallback,
                                this);
  _backCartSub = _nh.subscribe("agribot/encoder/cart/back",
                              1,
                              &AgribotOdomNode::backCartCallback,
                              this);
}

void AgribotOdomNode::caterpillarCallback(const agribot_msgs::AgribotCaterpillar msg)
{
  _agribotOdom.vSetEta1(msg.front_linear);

  // if(!updateTheta0FromLookupTransform())
  // {
  //   // odom->base_footprint transform not available - use Yaw from odometry
  //   // Most likely will happen only at the beginning of localization
  //   ROS_WARN("odom->base_footprint transform not available - using odometry Yaw as Theta0");
  //   _agribotOdom.vSetTheta0(_agribotOdom.fGetYaw());
  // }

  _agribotOdom.vSetTheta0(_agribotOdom.fGetYaw());

  // First message only sets timestamp for reference
  if(!_bTimestampSet)
  {
    _dPreviousTimestamp = msg.header.stamp.toSec();
    _bTimestampSet = true;
    return;
  }

  double _dCurrentTimestamp = msg.header.stamp.toSec();
  _agribotOdom.vSetDt(_dCurrentTimestamp - _dPreviousTimestamp);
  _dPreviousTimestamp = _dCurrentTimestamp;

  if(_agribotOdom.update())
  {
    _odometryMsg.header.seq = msg.header.seq;
    _odometryMsg.header.stamp = msg.header.stamp;
    publishOdometry();
    if (_bPublishTf)
    {
      publishTf();
    }
  }
  else
  {
    ROS_WARN("Skipping publication of odometry - one of theta angles not published yet");
  }
}

void AgribotOdomNode::frontCartCallback(const std_msgs::Float32 msg)
{
  float angleRadians = msg.data*M_PI/180.0;
  _agribotOdom.vSetTheta1(angleRadians);
}

void AgribotOdomNode::backCartCallback(const std_msgs::Float32 msg)
{
  float angleRadians = msg.data*M_PI/180.0;
  _agribotOdom.vSetTheta2(angleRadians);
}

bool AgribotOdomNode::updateTheta0FromLookupTransform()
{
  try
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
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }
}

void AgribotOdomNode::publishOdometry()
{
  _odometryMsg.pose.pose.position.x = _agribotOdom.fGetX();
  _odometryMsg.pose.pose.position.y = _agribotOdom.fGetY();
  tf2::Quaternion quat;
  quat.setRPY(0, 0, _agribotOdom.fGetYaw());
  _odometryMsg.pose.pose.orientation = tf2::toMsg(quat);
  _odometryMsg.pose.covariance[0] = POSITION_COVARIANCE;
  _odometryMsg.pose.covariance[7] = POSITION_COVARIANCE;
  _odometryMsg.pose.covariance[35] = ORIENTATION_COVARIANCE;

  _odometryMsg.twist.twist.linear.x = _agribotOdom.fGetLinearVelocityX();
  _odometryMsg.twist.twist.linear.y = _agribotOdom.fGetLinearVelocityY();
  _odometryMsg.twist.twist.angular.z = _agribotOdom.fGetAngularVelocityZ();
  _odometryMsg.twist.covariance[0] = LINEAR_VEL_X_COVARIANCE;
  _odometryMsg.twist.covariance[7] = LINEAR_VEL_Y_COVARIANCE;
  _odometryMsg.twist.covariance[35] = ANGULAR_VEL_Z_COVARIANCE;

  _odometryPub.publish(_odometryMsg);
}

void AgribotOdomNode::publishTf()
{

}

void AgribotOdomNode::run()
{
  ros::spin();
}
