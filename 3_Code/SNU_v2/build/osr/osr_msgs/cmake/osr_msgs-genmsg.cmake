# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "osr_msgs: 3 messages, 0 services")

set(MSG_I_FLAGS "-Iosr_msgs:/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/kinetic/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(osr_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Tracks.msg" NAME_WE)
add_custom_target(_osr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "osr_msgs" "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Tracks.msg" "geometry_msgs/Twist:std_msgs/Header:osr_msgs/BoundingBox:geometry_msgs/Quaternion:geometry_msgs/Vector3:geometry_msgs/Point:geometry_msgs/TwistWithCovariance:osr_msgs/Track:geometry_msgs/PoseWithCovariance:geometry_msgs/Pose:nav_msgs/Odometry"
)

get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg" NAME_WE)
add_custom_target(_osr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "osr_msgs" "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg" ""
)

get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg" NAME_WE)
add_custom_target(_osr_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "osr_msgs" "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg" "geometry_msgs/Twist:osr_msgs/BoundingBox:geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/Vector3:geometry_msgs/Pose"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(osr_msgs
  "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Tracks.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/osr_msgs
)
_generate_msg_cpp(osr_msgs
  "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/osr_msgs
)
_generate_msg_cpp(osr_msgs
  "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/osr_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(osr_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/osr_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(osr_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(osr_msgs_generate_messages osr_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Tracks.msg" NAME_WE)
add_dependencies(osr_msgs_generate_messages_cpp _osr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(osr_msgs_generate_messages_cpp _osr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg" NAME_WE)
add_dependencies(osr_msgs_generate_messages_cpp _osr_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(osr_msgs_gencpp)
add_dependencies(osr_msgs_gencpp osr_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS osr_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(osr_msgs
  "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Tracks.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/osr_msgs
)
_generate_msg_eus(osr_msgs
  "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/osr_msgs
)
_generate_msg_eus(osr_msgs
  "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/osr_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(osr_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/osr_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(osr_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(osr_msgs_generate_messages osr_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Tracks.msg" NAME_WE)
add_dependencies(osr_msgs_generate_messages_eus _osr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(osr_msgs_generate_messages_eus _osr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg" NAME_WE)
add_dependencies(osr_msgs_generate_messages_eus _osr_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(osr_msgs_geneus)
add_dependencies(osr_msgs_geneus osr_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS osr_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(osr_msgs
  "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Tracks.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/osr_msgs
)
_generate_msg_lisp(osr_msgs
  "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/osr_msgs
)
_generate_msg_lisp(osr_msgs
  "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/osr_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(osr_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/osr_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(osr_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(osr_msgs_generate_messages osr_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Tracks.msg" NAME_WE)
add_dependencies(osr_msgs_generate_messages_lisp _osr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(osr_msgs_generate_messages_lisp _osr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg" NAME_WE)
add_dependencies(osr_msgs_generate_messages_lisp _osr_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(osr_msgs_genlisp)
add_dependencies(osr_msgs_genlisp osr_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS osr_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(osr_msgs
  "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Tracks.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/osr_msgs
)
_generate_msg_nodejs(osr_msgs
  "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/osr_msgs
)
_generate_msg_nodejs(osr_msgs
  "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/osr_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(osr_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/osr_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(osr_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(osr_msgs_generate_messages osr_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Tracks.msg" NAME_WE)
add_dependencies(osr_msgs_generate_messages_nodejs _osr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(osr_msgs_generate_messages_nodejs _osr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg" NAME_WE)
add_dependencies(osr_msgs_generate_messages_nodejs _osr_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(osr_msgs_gennodejs)
add_dependencies(osr_msgs_gennodejs osr_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS osr_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(osr_msgs
  "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Tracks.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/nav_msgs/cmake/../msg/Odometry.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/osr_msgs
)
_generate_msg_py(osr_msgs
  "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/osr_msgs
)
_generate_msg_py(osr_msgs
  "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Twist.msg;/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/osr_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(osr_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/osr_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(osr_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(osr_msgs_generate_messages osr_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Tracks.msg" NAME_WE)
add_dependencies(osr_msgs_generate_messages_py _osr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/BoundingBox.msg" NAME_WE)
add_dependencies(osr_msgs_generate_messages_py _osr_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/kyle/USR_SNU_MODULE/SNU_Integrated_v2/src/osr/osr_msgs/msg/Track.msg" NAME_WE)
add_dependencies(osr_msgs_generate_messages_py _osr_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(osr_msgs_genpy)
add_dependencies(osr_msgs_genpy osr_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS osr_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/osr_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/osr_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(osr_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(osr_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(osr_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/osr_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/osr_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(osr_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(osr_msgs_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(osr_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/osr_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/osr_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(osr_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(osr_msgs_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(osr_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/osr_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/osr_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(osr_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(osr_msgs_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(osr_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/osr_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/osr_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/osr_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(osr_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(osr_msgs_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(osr_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
