##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(atest_gui)

##############################################################################
# Catkin
##############################################################################


# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS 
  qt_build 
  roscpp 
  roslib
  message_generation
  std_msgs
  geometry_msgs
  cmake_modules
  robotis_math
  robotis_framework_common
  robotis_controller_msgs
  heroehs_math
  diana_msgs
  )

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CATKIN_DEVEL_PREFIX}/lib/pkgconfig")
find_package(PkgConfig)

pkg_check_modules(yaml_cpp yaml-cpp REQUIRED)

add_message_files(FILES dynamixel_info.msg)
add_service_files(FILES command.srv)

generate_messages(DEPENDENCIES std_msgs)
include_directories(
include/atest_gui
${catkin_INCLUDE_DIRS} 
${yaml_cpp_INCLUDE_DIRS}
)

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/atest_gui/*.hpp)
file(GLOB_RECURSE QT_MOCH RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/atest_gui/*.h)



QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
QT4_WRAP_CPP(QT_MOCH_H ${QT_MOCH})



##############################################################################
# Sources
##############################################################################


file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)


##############################################################################
# Binaries
##############################################################################


add_executable(atest_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${CMAKE_CURRENT_SOURCE_DIR}/src/qcustomplot/qcustomplot.cpp  ${QT_MOCH_H})

add_dependencies(atest_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(atest_gui ${QT_LIBRARIES} ${catkin_LIBRARIES} ${yaml_cpp_LIBRARIES})

install(TARGETS atest_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

