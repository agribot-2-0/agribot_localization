#include <HeadingImuTranslator.h>

HeadingImuTranslator::HeadingImuTranslator(const ros::NodeHandle &node_handle)
:_nh(node_handle)
{

}

int HeadingImuTranslator::configure()
{
  _headingImuPublisher = _nh.advertise<sensor_msgs::Imu>("agribot/heading_imu", 1);
  _headingSubscriber = _nh.subscribe("agribot/heading", 1, &HeadingImuTranslator::headingCallback, this);
  return 0;
}
bool HeadingImuTranslator::startup()
{
  return false;
}
bool HeadingImuTranslator::update()
{
  return false;
}
bool HeadingImuTranslator::stop()
{
  return false;
}
bool HeadingImuTranslator::cleanup()
{
  _headingSubscriber.shutdown();
  _headingImuPublisher.shutdown();
  return true;
}

void HeadingImuTranslator::headingCallback(const agribot_msgs::AgribotHeading msg)
{
  sensor_msgs::Imu imu_msg;
  imu_msg.header = msg.header;
  imu_msg.orientation = tf::createQuaternionMsgFromYaw(msg.heading);
  imu_msg.orientation_covariance[8] = msg.heading_accuracy;

  _headingImuPublisher.publish(imu_msg);
}
