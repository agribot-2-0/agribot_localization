#include <ros/ros.h>
#include <HeadingImuTranslator.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "heading_imu_translator_node");
  ros::NodeHandle nh;
  HeadingImuTranslator heading_imu_translator(nh);
  heading_imu_translator.configure();
  ros::spin();

  return 0;
}
