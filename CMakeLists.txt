cmake_minimum_required(VERSION 3.0.2)
project(path_extraction)

find_package(catkin REQUIRED
    rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_extraction
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/path_extraction.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)