cmake_minimum_required(VERSION 2.8.3)
project(ski_main_manager)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  robotis_framework_common
  robotis_math
  robotis_controller
  robotis_controller_msgs
  cmake_modules
  heroehs_math 
  #add module package
  offset_module
  base_module
  pose_module
  motion_module
  diana_balance_control
  diana_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ski_manager
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/ski_main_manager_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
