cmake_minimum_required(VERSION 2.8.3)
project(heroehs_math)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  robotis_math
  robotis_controller_msgs
  robotis_framework_common
)


find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES heroehs_math
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(heroehs_math
   src/fifth_order_trajectory_generate.cpp
   src/kinematics.cpp
   src/end_point_to_rad_cal.cpp
 )
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(heroehs_math
   ${catkin_LIBRARIES}
)
