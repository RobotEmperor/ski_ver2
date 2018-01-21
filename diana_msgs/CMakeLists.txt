################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(diana_msgs)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  BalanceParam.msg
  JointFeedBackGain.msg
  ForceTorque.msg
  CenterChange.msg
  BalanceParamWaist.msg
  BalanceParamArm.msg
  RegionOfInterestArray.msg
  FlagData.msg
  FlagDataArray.msg
  GateData.msg
  GateDataArray.msg
  DesiredPoseCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs geometry_msgs
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
