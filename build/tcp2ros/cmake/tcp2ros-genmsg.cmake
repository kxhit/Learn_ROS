# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tcp2ros: 4 messages, 0 services")

set(MSG_I_FLAGS "-Itcp2ros:/home/kx/catkin_ws/src/tcp2ros/msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tcp2ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/rtkGPSmessage.msg" NAME_WE)
add_custom_target(_tcp2ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tcp2ros" "/home/kx/catkin_ws/src/tcp2ros/msg/rtkGPSmessage.msg" ""
)

get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/reach.msg" NAME_WE)
add_custom_target(_tcp2ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tcp2ros" "/home/kx/catkin_ws/src/tcp2ros/msg/reach.msg" ""
)

get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/readDataAll.msg" NAME_WE)
add_custom_target(_tcp2ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tcp2ros" "/home/kx/catkin_ws/src/tcp2ros/msg/readDataAll.msg" ""
)

get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/cmd.msg" NAME_WE)
add_custom_target(_tcp2ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tcp2ros" "/home/kx/catkin_ws/src/tcp2ros/msg/cmd.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/rtkGPSmessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tcp2ros
)
_generate_msg_cpp(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/reach.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tcp2ros
)
_generate_msg_cpp(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/readDataAll.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tcp2ros
)
_generate_msg_cpp(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/cmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tcp2ros
)

### Generating Services

### Generating Module File
_generate_module_cpp(tcp2ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tcp2ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tcp2ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tcp2ros_generate_messages tcp2ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/rtkGPSmessage.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_cpp _tcp2ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/reach.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_cpp _tcp2ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/readDataAll.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_cpp _tcp2ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/cmd.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_cpp _tcp2ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tcp2ros_gencpp)
add_dependencies(tcp2ros_gencpp tcp2ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tcp2ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/rtkGPSmessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tcp2ros
)
_generate_msg_eus(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/reach.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tcp2ros
)
_generate_msg_eus(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/readDataAll.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tcp2ros
)
_generate_msg_eus(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/cmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tcp2ros
)

### Generating Services

### Generating Module File
_generate_module_eus(tcp2ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tcp2ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tcp2ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tcp2ros_generate_messages tcp2ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/rtkGPSmessage.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_eus _tcp2ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/reach.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_eus _tcp2ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/readDataAll.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_eus _tcp2ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/cmd.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_eus _tcp2ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tcp2ros_geneus)
add_dependencies(tcp2ros_geneus tcp2ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tcp2ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/rtkGPSmessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tcp2ros
)
_generate_msg_lisp(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/reach.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tcp2ros
)
_generate_msg_lisp(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/readDataAll.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tcp2ros
)
_generate_msg_lisp(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/cmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tcp2ros
)

### Generating Services

### Generating Module File
_generate_module_lisp(tcp2ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tcp2ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tcp2ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tcp2ros_generate_messages tcp2ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/rtkGPSmessage.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_lisp _tcp2ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/reach.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_lisp _tcp2ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/readDataAll.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_lisp _tcp2ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/cmd.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_lisp _tcp2ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tcp2ros_genlisp)
add_dependencies(tcp2ros_genlisp tcp2ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tcp2ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/rtkGPSmessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tcp2ros
)
_generate_msg_nodejs(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/reach.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tcp2ros
)
_generate_msg_nodejs(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/readDataAll.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tcp2ros
)
_generate_msg_nodejs(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/cmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tcp2ros
)

### Generating Services

### Generating Module File
_generate_module_nodejs(tcp2ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tcp2ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tcp2ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tcp2ros_generate_messages tcp2ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/rtkGPSmessage.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_nodejs _tcp2ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/reach.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_nodejs _tcp2ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/readDataAll.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_nodejs _tcp2ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/cmd.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_nodejs _tcp2ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tcp2ros_gennodejs)
add_dependencies(tcp2ros_gennodejs tcp2ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tcp2ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/rtkGPSmessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tcp2ros
)
_generate_msg_py(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/reach.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tcp2ros
)
_generate_msg_py(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/readDataAll.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tcp2ros
)
_generate_msg_py(tcp2ros
  "/home/kx/catkin_ws/src/tcp2ros/msg/cmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tcp2ros
)

### Generating Services

### Generating Module File
_generate_module_py(tcp2ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tcp2ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tcp2ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tcp2ros_generate_messages tcp2ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/rtkGPSmessage.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_py _tcp2ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/reach.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_py _tcp2ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/readDataAll.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_py _tcp2ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kx/catkin_ws/src/tcp2ros/msg/cmd.msg" NAME_WE)
add_dependencies(tcp2ros_generate_messages_py _tcp2ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tcp2ros_genpy)
add_dependencies(tcp2ros_genpy tcp2ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tcp2ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tcp2ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tcp2ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(tcp2ros_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tcp2ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tcp2ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(tcp2ros_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tcp2ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tcp2ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(tcp2ros_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tcp2ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tcp2ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(tcp2ros_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tcp2ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tcp2ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tcp2ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(tcp2ros_generate_messages_py sensor_msgs_generate_messages_py)
endif()
