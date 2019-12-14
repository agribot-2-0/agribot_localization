#include <ros/ros.h>
#include <InclinometerImuTranslator.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inclinometer_imu_translator_node");
  ros::NodeHandle nh;
  InclinometerImuTranslator inclinometer_imu_translator(nh);
  inclinometer_imu_translator.configure();
  ros::spin();

  return 0;
}
