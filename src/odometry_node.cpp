/*
* Copyright (C) 2020 Agribot. All rights reserved.
*/

#include <agribot_localization/AgribotOdometryNode.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agribot_odometry_node");
  AgribotOdometryNode _agribotOdometryNode;
  _agribotOdometryNode.run();
  return 0;
}
