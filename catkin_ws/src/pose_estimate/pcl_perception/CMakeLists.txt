cmake_minimum_required(VERSION 2.8.3)
project(pcl_perception)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geographic_msgs
  geodesy
  tf
  tf_conversions
  message_generation
)

catkin_package(

)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-std=c++0x")
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
# add_executable(orb_process_base src/orb_process_base.h src/orb_process_base.cpp)
# target_link_libraries(orb_process_base ${catkin_LIBRARIES})