cmake_minimum_required(VERSION 3.0.2)
project(shelfmatch_task)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/shelf_follower
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
