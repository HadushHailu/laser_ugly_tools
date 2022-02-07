cmake_minimum_required(VERSION 3.0.2)
project(laser_ugly_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  laser_geometry
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laser_ugly_tools
   CATKIN_DEPENDS roscpp rospy std_msgs laser_geometry
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_fidelity_projection src/scan_fidelity_projection.cpp)
target_link_libraries(scan_fidelity_projection ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/laser_info.py scripts/laser_pointcloud_etc.py scripts/laser_geometry_check.py scripts/pointcloud_info.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

