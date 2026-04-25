cmake_minimum_required(VERSION 3.0.2)
project(navegacion_semantica)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  move_base_msgs
  visualization_msgs
  actionlib
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/semantic_navigation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
