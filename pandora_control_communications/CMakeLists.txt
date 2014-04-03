# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(pandora_control_communications)

find_package(
  catkin REQUIRED COMPONENTS
  message_generation 
  std_msgs 
  actionlib_msgs 
  )


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_action_files(
  FILES
  MoveKinect.action
  )

include_directories(${Boost_INCLUDE_DIRS})

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime 
  #~ INCLUDE_DIRS # TODO include
  #~ LIBRARIES # TODO
)
