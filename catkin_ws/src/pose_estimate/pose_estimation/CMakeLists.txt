cmake_minimum_required(VERSION 2.8.3)
project(pose_estimation)

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
  text_msgs
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
add_executable(object_pose_node src/object_pose_node.h src/object_pose_node.cpp)
target_link_libraries(object_pose_node ${catkin_LIBRARIES})
add_dependencies(object_pose_node src/object_pose_node.cpp ${catkin_EXPORTED_TARGETS})

add_executable(bn_pose_node src/bn_pose_node.h src/bn_pose_node.cpp)
target_link_libraries(bn_pose_node ${catkin_LIBRARIES})
add_dependencies(bn_pose_node src/bn_pose_node.cpp ${catkin_EXPORTED_TARGETS})
