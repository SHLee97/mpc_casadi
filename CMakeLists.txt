cmake_minimum_required(VERSION 3.0.2)
project(mpc_casadi)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  laser_geometry
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  src/nmpc_node.py
  scripts/odom_noise.py
  plotting/plot_tracking.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)