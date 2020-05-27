#############################################################################
## Set minimum required version of cmake, project name and compile options ##
#############################################################################
cmake_minimum_required(VERSION 2.8.3)
project(agribot_localization)
add_compile_options(-std=c++14)

#############################################################################
## Find catkin packages and libraries for catkin and system dependencies   ##
#############################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  message_filters
  robot_localization
  imu_complementary_filter
  std_msgs
  nav_msgs
  sensor_msgs
  agribot_msgs
  agribot_common
)

find_package(Boost REQUIRED COMPONENTS thread)
################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hal_agribot_control
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

#############################################################################
## Build                                                                   ##
#############################################################################

include_directories(
 include
 ${Boost_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
 ${agribot_common_SOURCE_DIR}/include
)

link_directories(
 ${PROJECT_SOURCE_DIR}
)

add_executable(odometry_node src/odometry_node.cpp src/AgribotOdometry.cpp src/AgribotOdometryNode.cpp)
target_link_libraries(odometry_node ${catkin_LIBRARIES})
add_dependencies(odometry_node ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS
  odometry_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############
