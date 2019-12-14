#include <InclinometerImuTranslator.h>

InclinometerImuTranslator::InclinometerImuTranslator(const ros::NodeHandle &node_handle)
:_nh(node_handle)
{

}

int InclinometerImuTranslator::configure()
{
  _inclinometerImuPublisher = _nh.advertise<sensor_msgs::Imu>("agribot/inclinometer_imu", 1);
  _inclinometerSubscriber = _nh.subscribe("agribot/inclinometer", 1, &InclinometerImuTranslator::inclinometerCallback, this);
  return 0;
}
bool InclinometerImuTranslator::startup()
{
  return false;
}
bool InclinometerImuTranslator::update()
{
  return false;
}
bool InclinometerImuTranslator::stop()
{
  return false;
}
bool InclinometerImuTranslator::cleanup()
{
  _inclinometerSubscriber.shutdown();
  _inclinometerImuPublisher.shutdown();
  return true;
}

void InclinometerImuTranslator::inclinometerCallback(const agribot_msgs::AgribotInclinometer msg)
{
  sensor_msgs::Imu imu_msg;
  imu_msg.header = msg.header;
  imu_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(msg.roll, msg.pitch, 0.0);

  // TODO - fill in covariance matrix for roll & pitch
  // imu_msg.orientation_covariance[0] = 0.0;
  // imu_msg.orientation_covariance[4] = 0.0;

  _inclinometerImuPublisher.publish(imu_msg);
}
