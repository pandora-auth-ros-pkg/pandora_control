cmake_minimum_required(VERSION 2.8.3)
project(pandora_kinect_control)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs
  sensor_msgs
  pandora_control_communications
  actionlib
  actionlib_msgs
  roslint
)


catkin_package(
  #~ INCLUDE_DIRS include
#  LIBRARIES rrbot_plugin
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    sensor_msgs
    pandora_control_communications
    roslint
    actionlib
    actionlib_msgs
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)
  
add_executable(
  pandora_kinect_control_node
  src/pandora_kinect_control.cpp
  src/pandora_kinect_control_node.cpp)
target_link_libraries(
  pandora_kinect_control_node 
  ${catkin_LIBRARIES} )

# To check lint : catkin_make roslint_pandora_kinect_control
set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use")

FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}
  src/kinect_control_node.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
