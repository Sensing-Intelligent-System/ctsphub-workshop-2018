cmake_minimum_required(VERSION 2.8.3)
project(tutorial_0905)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  message_generation
  tf
  actionlib
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  moveit_visual_tools
)

find_package(gazebo REQUIRED)
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
  FILES
  SetTargetPose.srv
)

add_message_files(
  FILES
  MyPose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tutorial_0905
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library

## Declare a C++ executable
add_executable(ur3_control
  src/ur3_control_gazebo.cpp)
add_dependencies(ur3_control 
  ${catkin_EXPORTED_TARGETS}
  tutorial_0905_gencpp)
target_link_libraries(ur3_control
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ur3_kin)

add_executable(ur3_control_real
  src/ur3_control_real.cpp)
add_dependencies(ur3_control_real
  ${catkin_EXPORTED_TARGETS}
  tutorial_0905_gencpp)
target_link_libraries(ur3_control_real
  ${catkin_LIBRARIES}
  ur3_kin)

add_executable(pick_and_place
  src/pick_and_place.cpp
  src/widget.hpp)
add_dependencies(pick_and_place 
  ${catkin_EXPORTED_TARGETS}
  tutorial_0905_gencpp)
target_link_libraries(pick_and_place
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES})

add_executable(pick_and_place_real
  src/pick_and_place_real.cpp
  src/widget.hpp)
add_dependencies(pick_and_place_real
  ${catkin_EXPORTED_TARGETS}
  tutorial_0905_gencpp)
target_link_libraries(pick_and_place_real
  ${catkin_LIBRARIES})

add_executable(path_planning
  src/path_planning.cpp
  src/widget.hpp)
add_dependencies(path_planning 
  ${catkin_EXPORTED_TARGETS}
  tutorial_0905_gencpp)
target_link_libraries(path_planning
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES})

add_executable(path_planning_real
  src/path_planning_real.cpp
  src/widget.hpp)
add_dependencies(path_planning_real
  ${catkin_EXPORTED_TARGETS}
  tutorial_0905_gencpp)
target_link_libraries(path_planning_real
  ${catkin_LIBRARIES})



