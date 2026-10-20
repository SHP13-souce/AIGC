cmake_minimum_required(VERSION 3.0.2)
project(aigc_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  move_base_msgs
  geometry_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS rospy actionlib actionlib_msgs move_base_msgs geometry_msgs tf
)

###########
## Build ##
###########

# No compiled code: this package ships configuration, launch files and a
# Python script only.

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/send_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config maps rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
