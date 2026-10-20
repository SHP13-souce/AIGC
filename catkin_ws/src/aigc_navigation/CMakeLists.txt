cmake_minimum_required(VERSION 3.0.2)
project(aigc_navigation)

# This package ships only Python scripts, config and resources: it needs the
# catkin build tool but no build/export dependencies (declared as exec_depend
# in package.xml). No C++ targets are built.
find_package(catkin REQUIRED)

catkin_package()

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
