################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(pose_module)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  cmake_modules
  robotis_math
  robotis_framework_common
  robotis_controller_msgs
  heroehs_math
  diana_balance_control
  message_generation
)

find_package(Eigen REQUIRED)

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")
find_package(PkgConfig)
pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)

add_service_files(FILES command.srv)


generate_messages(DEPENDENCIES std_msgs)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_module
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${yaml_cpp_INCLUDE_DIRS}
)

add_library(pose_module
  src/pose_module.cpp
  src/pose_module_state.cpp
)

add_dependencies(pose_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(pose_module pose_module_generate_messages_cpp)
target_link_libraries(pose_module ${yaml_cpp_LIBRARIES} ${catkin_LIBRARIES})


