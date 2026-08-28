cmake_minimum_required(VERSION 3.12)
project(ellipse_detection_ros)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  message_runtime
  actionlib_msgs
  geometry_msgs
  image_transport
  message_generation
  nodelet
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  detection_msgs
)

# Declare a catkin package
catkin_package()


# Find OpenCV package
set(OpenCV_DIR "/usr/local/opencv")
find_package(OpenCV REQUIRED)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# We default to 'Release' if none was specified
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  MESSAGE(STATUS "Setting build type to 'Release' as none was specified.")
  SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the build type" FORCE)
  SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Coverage" "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

# BUILD  TYPE
if(CMAKE_BUILD_TYPE AND(CMAKE_BUILD_TYPE STREQUAL "Debug"))
  message(STATUS "Debug mode:")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS} -Wall -O0  -g")
  message(STATUS "CMAKE_C_FLAGS_DEBUG: ${CMAKE_C_FLAGS_DEBUG}")

  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -O0  -g")
  message(STATUS "CMAKE_CXX_FLAGS_DEBUG: ${CMAKE_CXX_FLAGS_DEBUG}")

  add_definitions(-DDEBUG)
elseif(CMAKE_BUILD_TYPE AND(CMAKE_BUILD_TYPE STREQUAL "Release"))
  message(STATUS "Release mode:")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} -Wall -O3")
  message(STATUS "CMAKE_C_FLAGS_RELEASE: ${CMAKE_C_FLAGS_RELEASE}")

  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Wall -O3")
  message(STATUS "CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")

  add_definitions(-DNDEBUG)
else()
  message(STATUS "Debug mode:")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS} -Wall -O0  -g")
  message(STATUS "CMAKE_C_FLAGS_DEBUG: ${CMAKE_C_FLAGS_DEBUG}")

  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -O0  -g")
  message(STATUS "CMAKE_CXX_FLAGS_DEBUG: ${CMAKE_CXX_FLAGS_DEBUG}")

  add_definitions(-DDEBUG)
endif()

add_compile_options("-O3" "-funsafe-loop-optimizations" "-fsee" "-funroll-loops" "-fno-math-errno" "-funsafe-math-optimizations" "-ffinite-math-only" "-fno-signed-zeros")

include_directories(
  include
  ../detection_libs/include)
include_directories(${catkin_INCLUDE_DIRS})

# Add the source files
file(GLOB SOURCES "src/*.cpp")

add_library(${PROJECT_NAME}_detector 
            src/ellipse_detector.cpp
            ../detection_libs/src/common/common.cpp
            ../detection_libs/src/common/allocator.cpp
            ../detection_libs/src/common/box.cpp
            ../detection_libs/src/common/target.cpp
            ../detection_libs/src/common/targets_in_frame.cpp
            ../detection_libs/src/inference_backend/aruco_detecor.cpp
            ../detection_libs/src/utils/gason.cpp)
target_link_libraries(${PROJECT_NAME}_detector ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_node src/ellipse_detection_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
                ${catkin_LIBRARIES}
                ${OpenCV_LIBS}
                ${PROJECT_NAME}_detector
)

add_dependencies(${PROJECT_NAME}_detector ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
