#############################################################################
## Set minimum required version of cmake, project name and compile options ##
#############################################################################
cmake_minimum_required(VERSION 2.8.3)
project(agribot_localization)

#############################################################################
## Find catkin packages and libraries for catkin and system dependencies   ##
#############################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  robot_localization
  tf
  imu_complementary_filter
  std_msgs
  nav_msgs
  sensor_msgs
  agribot_msgs
  agribot_common
)

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
 ${catkin_INCLUDE_DIRS}
 ${agribot_common_SOURCE_DIR}/include
)

link_directories("${PROJECT_SOURCE_DIR}")

add_executable(heading_imu_translator_node src/heading_imu_translator_node.cpp src/HeadingImuTranslator.cpp)
add_executable(inclinometer_imu_translator_node src/inclinometer_imu_translator_node.cpp src/InclinometerImuTranslator.cpp)
add_executable(forward_kinematics_node src/forward_kinematics_node.cpp)

target_link_libraries(heading_imu_translator_node ${catkin_LIBRARIES})
target_link_libraries(inclinometer_imu_translator_node ${catkin_LIBRARIES})
target_link_libraries(forward_kinematics_node ${catkin_LIBRARIES})

add_dependencies(heading_imu_translator_node ${catkin_EXPORTED_TARGETS})
add_dependencies(inclinometer_imu_translator_node ${catkin_EXPORTED_TARGETS})
add_dependencies(forward_kinematics_node ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

#############
## Testing ##
#############
