cmake_minimum_required(VERSION 2.8.3)
project(sunray_tutorial)

# 本机适配版：仅编译官方自动降落节点（auto_land / auto_land_by_pose）
# 完整上游见 official_upstream/ 与 /tmp/Sunray_official

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  mavros_msgs
  swarm_msgs
  detection_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS
    roscpp rospy std_msgs geometry_msgs
    swarm_msgs detection_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /home/uav/catkin_ws/src/uav_common/common_lib
)

add_executable(auto_land_by_pose advanced/auto_land_by_pose.cpp)
add_dependencies(auto_land_by_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(auto_land_by_pose ${catkin_LIBRARIES})

add_executable(vision_cmd_mux advanced/vision_cmd_mux.cpp)
add_dependencies(vision_cmd_mux ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_cmd_mux ${catkin_LIBRARIES})

add_executable(auto_land advanced/auto_land.cpp)
add_dependencies(auto_land ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(auto_land ${catkin_LIBRARIES})
