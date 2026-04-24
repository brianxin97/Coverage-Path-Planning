cmake_minimum_required(VERSION 3.0.2)
project(coverage_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  explore_lite
  slam_karto
  amcl
  move_base
  map_server
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs nav_msgs geometry_msgs tf explore_lite slam_karto amcl move_base map_server
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/coverage_path_planner.py
  scripts/goal_publisher.py
  scripts/coordinator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

