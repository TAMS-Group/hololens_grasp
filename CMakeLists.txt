cmake_minimum_required(VERSION 2.8.3)
project(hololens_grasp)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  actionlib
  moveit_msgs
  moveit_ros_planning_interface
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pick_place src/pick_place.cpp)
target_link_libraries(pick_place ${catkin_LIBRARIES})
