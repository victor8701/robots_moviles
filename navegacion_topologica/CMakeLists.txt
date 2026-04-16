cmake_minimum_required(VERSION 2.8.3)
project(navegacion_topologica)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  move_base_msgs
  tf2
  tf2_ros
  actionlib
)

## Python setup
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    move_base_msgs
    tf2
    tf2_ros
    actionlib
)

###########
## Build ##
###########

include_directories()

#############
## Install ##
#############

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/topological_navigation.py
  scripts/generate_all_maps.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install Python modules
install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install map files
install(DIRECTORY mapas/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mapas
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()
