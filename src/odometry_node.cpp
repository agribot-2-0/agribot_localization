#include <AgribotOdomNode.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_node");
  AgribotOdomNode _agribotOdomNode;
  _agribotOdomNode.run();
  return 0;
}
