# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "TicTac: 0 messages, 1 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(TicTac_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ian/Documents/School/Robotics/Labs/FinalProject/Robot/catkin_ws/src/TicTac/srv/CalculateBestMove.srv" NAME_WE)
add_custom_target(_TicTac_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "TicTac" "/home/ian/Documents/School/Robotics/Labs/FinalProject/Robot/catkin_ws/src/TicTac/srv/CalculateBestMove.srv" "std_msgs/String"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(TicTac
  "/home/ian/Documents/School/Robotics/Labs/FinalProject/Robot/catkin_ws/src/TicTac/srv/CalculateBestMove.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/TicTac
)

### Generating Module File
_generate_module_cpp(TicTac
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/TicTac
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(TicTac_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(TicTac_generate_messages TicTac_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ian/Documents/School/Robotics/Labs/FinalProject/Robot/catkin_ws/src/TicTac/srv/CalculateBestMove.srv" NAME_WE)
add_dependencies(TicTac_generate_messages_cpp _TicTac_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(TicTac_gencpp)
add_dependencies(TicTac_gencpp TicTac_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS TicTac_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(TicTac
  "/home/ian/Documents/School/Robotics/Labs/FinalProject/Robot/catkin_ws/src/TicTac/srv/CalculateBestMove.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/TicTac
)

### Generating Module File
_generate_module_eus(TicTac
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/TicTac
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(TicTac_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(TicTac_generate_messages TicTac_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ian/Documents/School/Robotics/Labs/FinalProject/Robot/catkin_ws/src/TicTac/srv/CalculateBestMove.srv" NAME_WE)
add_dependencies(TicTac_generate_messages_eus _TicTac_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(TicTac_geneus)
add_dependencies(TicTac_geneus TicTac_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS TicTac_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(TicTac
  "/home/ian/Documents/School/Robotics/Labs/FinalProject/Robot/catkin_ws/src/TicTac/srv/CalculateBestMove.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/TicTac
)

### Generating Module File
_generate_module_lisp(TicTac
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/TicTac
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(TicTac_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(TicTac_generate_messages TicTac_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ian/Documents/School/Robotics/Labs/FinalProject/Robot/catkin_ws/src/TicTac/srv/CalculateBestMove.srv" NAME_WE)
add_dependencies(TicTac_generate_messages_lisp _TicTac_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(TicTac_genlisp)
add_dependencies(TicTac_genlisp TicTac_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS TicTac_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(TicTac
  "/home/ian/Documents/School/Robotics/Labs/FinalProject/Robot/catkin_ws/src/TicTac/srv/CalculateBestMove.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/TicTac
)

### Generating Module File
_generate_module_nodejs(TicTac
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/TicTac
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(TicTac_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(TicTac_generate_messages TicTac_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ian/Documents/School/Robotics/Labs/FinalProject/Robot/catkin_ws/src/TicTac/srv/CalculateBestMove.srv" NAME_WE)
add_dependencies(TicTac_generate_messages_nodejs _TicTac_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(TicTac_gennodejs)
add_dependencies(TicTac_gennodejs TicTac_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS TicTac_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(TicTac
  "/home/ian/Documents/School/Robotics/Labs/FinalProject/Robot/catkin_ws/src/TicTac/srv/CalculateBestMove.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/TicTac
)

### Generating Module File
_generate_module_py(TicTac
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/TicTac
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(TicTac_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(TicTac_generate_messages TicTac_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ian/Documents/School/Robotics/Labs/FinalProject/Robot/catkin_ws/src/TicTac/srv/CalculateBestMove.srv" NAME_WE)
add_dependencies(TicTac_generate_messages_py _TicTac_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(TicTac_genpy)
add_dependencies(TicTac_genpy TicTac_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS TicTac_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/TicTac)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/TicTac
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(TicTac_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(TicTac_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/TicTac)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/TicTac
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(TicTac_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(TicTac_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/TicTac)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/TicTac
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(TicTac_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(TicTac_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/TicTac)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/TicTac
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(TicTac_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(TicTac_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/TicTac)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/TicTac\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/TicTac
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(TicTac_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(TicTac_generate_messages_py std_msgs_generate_messages_py)
endif()
