cmake_minimum_required(VERSION 2.8.3)
project(pandora_end_effector_planner)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  message_generation
  tf
  roslint
)

add_action_files(
  FILES
  MoveSensor.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rrbot_plugin
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    sensor_msgs
    actionlib
    actionlib_msgs
    tf
    roslint
    message_runtime
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

#~ set(CMAKE_BUILD_TYPE Debug)
  
add_executable(
  sensor_orientation_planner
  src/sensor_orientation_planner.cpp)
add_dependencies(
  sensor_orientation_planner
  ${PROJECT_NAME}_gencpp)
target_link_libraries(
  sensor_orientation_planner 
  ${catkin_LIBRARIES} )

# To check lint : catkin_make roslint_pandora_end_effector_planner
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  include/pandora_end_effector_planner/sensor_orientation_planner.h
  src/sensor_orientation_planner.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
