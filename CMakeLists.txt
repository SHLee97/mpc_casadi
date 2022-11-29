cmake_minimum_required(VERSION 3.0.2)
project(mpc_casadi)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  laser_geometry
  decomp_util
  decomp_ros_msgs
  decomp_ros_utils
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS} ${DECOMP_UTIL_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  src/nmpc_node.py
  scripts/odom_noise.py
  scripts/scan2points.py
  plotting/plot_tracking.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(sfc_node src/sfc.cpp)
target_link_libraries(sfc_node ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${OpenCV_LIBS})
