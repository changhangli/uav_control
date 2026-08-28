cmake_minimum_required(VERSION 3.0.2)
project(video_sender)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    cv_bridge
    image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(
  video_sender_node
  src/video_sender_node.cpp
)

add_dependencies(
  video_sender_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  video_sender_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
