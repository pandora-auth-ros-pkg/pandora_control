cmake_minimum_required(VERSION 2.8.3)
project(pandora_linear_movement_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  message_generation
  tf
  urdf
  roslint
)

catkin_python_setup()

add_action_files(
  FILES
  MoveLinear.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rrbot_plugin
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    actionlib
    actionlib_msgs
    tf
    urdf
    message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#~ set(CMAKE_BUILD_TYPE Debug)


add_executable(
  linear_movement_controller
  src/linear_movement_controller.cpp)
add_dependencies(
  linear_movement_controller
  ${PROJECT_NAME}_gencpp)
target_link_libraries(
  linear_movement_controller 
  ${catkin_LIBRARIES} )

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_PYTHON
     ${PROJECT_SOURCE_DIR}
    scripts/pandora_linear_movement_controller/move_linear_script.py
    )
roslint_python(${${PROJECT_NAME}_LINT_PYTHON})

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  include/*
  src/*
)

roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
