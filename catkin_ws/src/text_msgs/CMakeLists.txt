cmake_minimum_required(VERSION 2.8.12)

project(text_msgs)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    sensor_msgs
    std_msgs
    message_generation
)

add_message_files(
  FILES
  text_detection_msg.msg
  text_detection_array.msg
  bb_box.msg
  int_arr.msg
)

add_service_files(
  FILES
  int32.srv
  text_recognize_srv.srv
  text_detection_srv.srv
  predict_switch.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    sensor_msgs
    message_runtime
    std_msgs
)
