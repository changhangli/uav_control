cmake_minimum_required(VERSION 2.8.3)
project(uav_control)

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
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include include/pos_controller
  CATKIN_DEPENDS
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
)

include_directories(
  include
  include/pos_controller
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../uav_common/common_lib
)

add_library(uav_control src/uav_control.cpp)
add_dependencies(uav_control ${swarm_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav_control ${catkin_LIBRARIES})

add_executable(uav_control_node src/uav_control_node.cpp)
add_dependencies(uav_control_node ${swarm_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav_control_node uav_control ${catkin_LIBRARIES})

add_library(externalFusion src/externalFusion.cpp)
add_dependencies(externalFusion ${swarm_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(externalFusion ${catkin_LIBRARIES})

add_executable(external_fusion_node src/external_fusion_node.cpp)
add_dependencies(external_fusion_node ${swarm_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(external_fusion_node externalFusion ${catkin_LIBRARIES})

add_executable(control_evaluation_node utils/control_evaluation_node.cpp)
add_dependencies(control_evaluation_node ${swarm_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_evaluation_node ${catkin_LIBRARIES})

add_executable(uav_command_pub utils/uav_command_pub.cpp)
add_dependencies(uav_command_pub ${swarm_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav_command_pub ${catkin_LIBRARIES})

add_executable(waypoint_pub utils/waypoint_pub.cpp)
add_dependencies(waypoint_pub ${swarm_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_pub ${catkin_LIBRARIES})

add_executable(positioning_accuracy utils/positioning_accuracy.cpp)
add_dependencies(positioning_accuracy ${swarm_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(positioning_accuracy ${catkin_LIBRARIES})

add_executable(joy_node utils/joy_node.cpp)
add_dependencies(joy_node ${swarm_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_node ${catkin_LIBRARIES})

add_executable(fake_vision_pose_node utils/fake_vision_pose_node.cpp)
add_dependencies(fake_vision_pose_node ${swarm_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fake_vision_pose_node ${catkin_LIBRARIES})

add_executable(rc_control_node src/rc_control_node.cpp)
add_dependencies(rc_control_node ${swarm_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rc_control_node ${catkin_LIBRARIES})

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
