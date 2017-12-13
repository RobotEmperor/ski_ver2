cmake_minimum_required(VERSION 2.8.3)
project(offset_module)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  robotis_math
  robotis_controller_msgs
  robotis_framework_common
  message_generation
)

find_package(Eigen REQUIRED)

add_message_files(FILES dynamixel_info.msg)
add_service_files(FILES command.srv)


generate_messages(DEPENDENCIES std_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES offset_module
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


 add_library(offset_module
   src/offset_module.cpp
 )
 
 add_dependencies(offset_module ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(offset_module offset_module_generate_messages_cpp)
 target_link_libraries(offset_module
   ${catkin_LIBRARIES}
 )