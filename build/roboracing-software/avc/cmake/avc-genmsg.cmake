# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "avc: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(avc_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/brian/catkin_ws/src/roboracing-software/avc/srv/transform_image.srv" NAME_WE)
add_custom_target(_avc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "avc" "/home/brian/catkin_ws/src/roboracing-software/avc/srv/transform_image.srv" "sensor_msgs/Image:std_msgs/Header"
)

get_filename_component(_filename "/home/brian/catkin_ws/src/roboracing-software/avc/srv/calibrate_image.srv" NAME_WE)
add_custom_target(_avc_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "avc" "/home/brian/catkin_ws/src/roboracing-software/avc/srv/calibrate_image.srv" "sensor_msgs/Image:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(avc
  "/home/brian/catkin_ws/src/roboracing-software/avc/srv/transform_image.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avc
)
_generate_srv_cpp(avc
  "/home/brian/catkin_ws/src/roboracing-software/avc/srv/calibrate_image.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avc
)

### Generating Module File
_generate_module_cpp(avc
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avc
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(avc_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(avc_generate_messages avc_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/brian/catkin_ws/src/roboracing-software/avc/srv/transform_image.srv" NAME_WE)
add_dependencies(avc_generate_messages_cpp _avc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/brian/catkin_ws/src/roboracing-software/avc/srv/calibrate_image.srv" NAME_WE)
add_dependencies(avc_generate_messages_cpp _avc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(avc_gencpp)
add_dependencies(avc_gencpp avc_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS avc_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(avc
  "/home/brian/catkin_ws/src/roboracing-software/avc/srv/transform_image.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avc
)
_generate_srv_eus(avc
  "/home/brian/catkin_ws/src/roboracing-software/avc/srv/calibrate_image.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avc
)

### Generating Module File
_generate_module_eus(avc
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avc
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(avc_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(avc_generate_messages avc_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/brian/catkin_ws/src/roboracing-software/avc/srv/transform_image.srv" NAME_WE)
add_dependencies(avc_generate_messages_eus _avc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/brian/catkin_ws/src/roboracing-software/avc/srv/calibrate_image.srv" NAME_WE)
add_dependencies(avc_generate_messages_eus _avc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(avc_geneus)
add_dependencies(avc_geneus avc_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS avc_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(avc
  "/home/brian/catkin_ws/src/roboracing-software/avc/srv/transform_image.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avc
)
_generate_srv_lisp(avc
  "/home/brian/catkin_ws/src/roboracing-software/avc/srv/calibrate_image.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avc
)

### Generating Module File
_generate_module_lisp(avc
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avc
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(avc_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(avc_generate_messages avc_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/brian/catkin_ws/src/roboracing-software/avc/srv/transform_image.srv" NAME_WE)
add_dependencies(avc_generate_messages_lisp _avc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/brian/catkin_ws/src/roboracing-software/avc/srv/calibrate_image.srv" NAME_WE)
add_dependencies(avc_generate_messages_lisp _avc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(avc_genlisp)
add_dependencies(avc_genlisp avc_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS avc_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(avc
  "/home/brian/catkin_ws/src/roboracing-software/avc/srv/transform_image.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avc
)
_generate_srv_nodejs(avc
  "/home/brian/catkin_ws/src/roboracing-software/avc/srv/calibrate_image.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avc
)

### Generating Module File
_generate_module_nodejs(avc
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avc
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(avc_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(avc_generate_messages avc_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/brian/catkin_ws/src/roboracing-software/avc/srv/transform_image.srv" NAME_WE)
add_dependencies(avc_generate_messages_nodejs _avc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/brian/catkin_ws/src/roboracing-software/avc/srv/calibrate_image.srv" NAME_WE)
add_dependencies(avc_generate_messages_nodejs _avc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(avc_gennodejs)
add_dependencies(avc_gennodejs avc_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS avc_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(avc
  "/home/brian/catkin_ws/src/roboracing-software/avc/srv/transform_image.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avc
)
_generate_srv_py(avc
  "/home/brian/catkin_ws/src/roboracing-software/avc/srv/calibrate_image.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avc
)

### Generating Module File
_generate_module_py(avc
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avc
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(avc_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(avc_generate_messages avc_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/brian/catkin_ws/src/roboracing-software/avc/srv/transform_image.srv" NAME_WE)
add_dependencies(avc_generate_messages_py _avc_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/brian/catkin_ws/src/roboracing-software/avc/srv/calibrate_image.srv" NAME_WE)
add_dependencies(avc_generate_messages_py _avc_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(avc_genpy)
add_dependencies(avc_genpy avc_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS avc_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/avc
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(avc_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(avc_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/avc
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(avc_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(avc_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/avc
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(avc_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(avc_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avc)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/avc
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(avc_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(avc_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avc)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avc\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/avc
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(avc_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(avc_generate_messages_py sensor_msgs_generate_messages_py)
endif()
