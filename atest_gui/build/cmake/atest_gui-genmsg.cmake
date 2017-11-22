# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "atest_gui: 1 messages, 1 services")

set(MSG_I_FLAGS "-Iatest_gui:/home/robot11/catkin_ws/src/atest_gui/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(atest_gui_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/robot11/catkin_ws/src/atest_gui/msg/dynamixel_info.msg" NAME_WE)
add_custom_target(_atest_gui_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "atest_gui" "/home/robot11/catkin_ws/src/atest_gui/msg/dynamixel_info.msg" ""
)

get_filename_component(_filename "/home/robot11/catkin_ws/src/atest_gui/srv/command.srv" NAME_WE)
add_custom_target(_atest_gui_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "atest_gui" "/home/robot11/catkin_ws/src/atest_gui/srv/command.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(atest_gui
  "/home/robot11/catkin_ws/src/atest_gui/msg/dynamixel_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/atest_gui
)

### Generating Services
_generate_srv_cpp(atest_gui
  "/home/robot11/catkin_ws/src/atest_gui/srv/command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/atest_gui
)

### Generating Module File
_generate_module_cpp(atest_gui
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/atest_gui
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(atest_gui_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(atest_gui_generate_messages atest_gui_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot11/catkin_ws/src/atest_gui/msg/dynamixel_info.msg" NAME_WE)
add_dependencies(atest_gui_generate_messages_cpp _atest_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot11/catkin_ws/src/atest_gui/srv/command.srv" NAME_WE)
add_dependencies(atest_gui_generate_messages_cpp _atest_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(atest_gui_gencpp)
add_dependencies(atest_gui_gencpp atest_gui_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS atest_gui_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(atest_gui
  "/home/robot11/catkin_ws/src/atest_gui/msg/dynamixel_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/atest_gui
)

### Generating Services
_generate_srv_eus(atest_gui
  "/home/robot11/catkin_ws/src/atest_gui/srv/command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/atest_gui
)

### Generating Module File
_generate_module_eus(atest_gui
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/atest_gui
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(atest_gui_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(atest_gui_generate_messages atest_gui_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot11/catkin_ws/src/atest_gui/msg/dynamixel_info.msg" NAME_WE)
add_dependencies(atest_gui_generate_messages_eus _atest_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot11/catkin_ws/src/atest_gui/srv/command.srv" NAME_WE)
add_dependencies(atest_gui_generate_messages_eus _atest_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(atest_gui_geneus)
add_dependencies(atest_gui_geneus atest_gui_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS atest_gui_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(atest_gui
  "/home/robot11/catkin_ws/src/atest_gui/msg/dynamixel_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/atest_gui
)

### Generating Services
_generate_srv_lisp(atest_gui
  "/home/robot11/catkin_ws/src/atest_gui/srv/command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/atest_gui
)

### Generating Module File
_generate_module_lisp(atest_gui
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/atest_gui
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(atest_gui_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(atest_gui_generate_messages atest_gui_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot11/catkin_ws/src/atest_gui/msg/dynamixel_info.msg" NAME_WE)
add_dependencies(atest_gui_generate_messages_lisp _atest_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot11/catkin_ws/src/atest_gui/srv/command.srv" NAME_WE)
add_dependencies(atest_gui_generate_messages_lisp _atest_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(atest_gui_genlisp)
add_dependencies(atest_gui_genlisp atest_gui_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS atest_gui_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(atest_gui
  "/home/robot11/catkin_ws/src/atest_gui/msg/dynamixel_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/atest_gui
)

### Generating Services
_generate_srv_nodejs(atest_gui
  "/home/robot11/catkin_ws/src/atest_gui/srv/command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/atest_gui
)

### Generating Module File
_generate_module_nodejs(atest_gui
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/atest_gui
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(atest_gui_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(atest_gui_generate_messages atest_gui_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot11/catkin_ws/src/atest_gui/msg/dynamixel_info.msg" NAME_WE)
add_dependencies(atest_gui_generate_messages_nodejs _atest_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot11/catkin_ws/src/atest_gui/srv/command.srv" NAME_WE)
add_dependencies(atest_gui_generate_messages_nodejs _atest_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(atest_gui_gennodejs)
add_dependencies(atest_gui_gennodejs atest_gui_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS atest_gui_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(atest_gui
  "/home/robot11/catkin_ws/src/atest_gui/msg/dynamixel_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/atest_gui
)

### Generating Services
_generate_srv_py(atest_gui
  "/home/robot11/catkin_ws/src/atest_gui/srv/command.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/atest_gui
)

### Generating Module File
_generate_module_py(atest_gui
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/atest_gui
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(atest_gui_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(atest_gui_generate_messages atest_gui_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/robot11/catkin_ws/src/atest_gui/msg/dynamixel_info.msg" NAME_WE)
add_dependencies(atest_gui_generate_messages_py _atest_gui_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/robot11/catkin_ws/src/atest_gui/srv/command.srv" NAME_WE)
add_dependencies(atest_gui_generate_messages_py _atest_gui_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(atest_gui_genpy)
add_dependencies(atest_gui_genpy atest_gui_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS atest_gui_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/atest_gui)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/atest_gui
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(atest_gui_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/atest_gui)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/atest_gui
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(atest_gui_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/atest_gui)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/atest_gui
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(atest_gui_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/atest_gui)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/atest_gui
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(atest_gui_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/atest_gui)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/atest_gui\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/atest_gui
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(atest_gui_generate_messages_py std_msgs_generate_messages_py)
endif()
