/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <agribot_localization/RewriteOdomLocalToGlobalNode.h>

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "rewrite_local_odom_to_global_node");
    ros::NodeHandle nh("~");
    RewriteOdomLocalToGlobalNode rewrite_odom_local_to_global_node(&nh);
    rewrite_odom_local_to_global_node.run();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM(e.what());
    exit(-1);
  }
}
