#include <AgribotOdometryNode.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agribot_odometry_node");
  AgribotOdometryNode _agribotOdometryNode;
  _agribotOdometryNode.run();
  return 0;
}
