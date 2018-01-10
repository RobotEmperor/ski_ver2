cmake_minimum_required(VERSION 2.8.3)
project(diana_sensors)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  diana_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES diana_sensors
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

set(SOEM_INCLUDE_INSTALL_DIR include/diana_sensors)

set(OS "linux")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)
include_directories(include/${PROJECT_NAME}/soem)
include_directories(include/${PROJECT_NAME}/osal)
include_directories(include/${PROJECT_NAME}/osal/${OS})
include_directories(include/${PROJECT_NAME}/oshw/${OS})

#add_subdirectory(include/${PROJECT_NAME}/osal)
#add_subdirectory(include/${PROJECT_NAME}/oshw)
#add_subdirectory(include/${PROJECT_NAME}/soem)
#add_subdirectory(include/${PROJECT_NAME}/test)

file(GLOB SOEM_SOURCES include/${PROJECT_NAME}/soem/*.c)
file(GLOB OSAL_SOURCES include/${PROJECT_NAME}/osal/${OS}/*.c)
file(GLOB OSHW_SOURCES include/${PROJECT_NAME}/oshw/${OS}/*.c)

file(GLOB SOEM_HEADERS include/${PROJECT_NAME}/soem/*.h)
file(GLOB OSAL_HEADERS include/${PROJECT_NAME}/osal/osal.h include/${PROJECT_NAME}/osal/${OS}/*.h)
file(GLOB OSHW_HEADERS include/${PROJECT_NAME}/oshw/${OS}/*.h)

add_library(soem STATIC
  ${SOEM_HEADERS}
  ${OSAL_HEADERS}
  ${OSHW_HEADERS}
  ${SOEM_SOURCES}
  ${OSAL_SOURCES}
  ${OSHW_SOURCES}
#  ${OSHW_EXTRA_SOURCES}
)
target_link_libraries(soem pthread rt)

#install(TARGETS soem_test DESTINATION lib)
#install(FILES
#  ${SOEM_HEADERS}
#  ${OSAL_HEADERS}
#  ${OSHW_HEADERS}
#  DESTINATION ${SOEM_INCLUDE_INSTALL_DIR}
#)

add_executable(ft_node src/ft_sensor.cpp src/get_time.cpp)
add_dependencies(ft_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ft_node ${catkin_LIBRARIES} soem)



