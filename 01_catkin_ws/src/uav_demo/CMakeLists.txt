cmake_minimum_required(VERSION 2.8.3)
project(uav_demo)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
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
  swarm_msgs
  tf2_geometry_msgs
  mavros_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs 
  nav_msgs
  sensor_msgs
  std_msgs

)

catkin_package(
  CATKIN_DEPENDS  message_runtime
)
###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../uav_common/common_lib
)


###############################
##      声明可执行cpp文件     ##
###############################

add_executable(circle_vel src/circle_vel.cpp)
add_dependencies(circle_vel uav_demo_gencpp)
target_link_libraries(circle_vel ${catkin_LIBRARIES})

add_executable(square_trajectory src/square_trajectory.cpp)
add_dependencies(square_trajectory uav_demo_gencpp)
target_link_libraries(square_trajectory ${catkin_LIBRARIES})

add_executable(hexagon_trajectory src/hexagon_trajectory.cpp)
add_dependencies(hexagon_trajectory uav_demo_gencpp)
target_link_libraries(hexagon_trajectory ${catkin_LIBRARIES})


## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)