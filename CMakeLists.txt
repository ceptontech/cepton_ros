cmake_minimum_required(VERSION 2.8.3)
project(cepton)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  angles
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  std_msgs
  tf)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cepton
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include)

# Cepton SDK
set(CEPTON_SDK_DIR "${PROJECT_SOURCE_DIR}/third_party/cepton_sdk")
include_directories(SYSTEM "${CEPTON_SDK_DIR}/include")
link_directories("${CEPTON_SDK_DIR}/lib")

# Cepton
set(CEPTON_DIR "")

# PCL
find_package(PCL REQUIRED)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CEPTON_SOURCES
  src/cepton_node.cpp
  src/point_cloud_publisher.cpp)
add_executable(cepton-node ${CEPTON_SOURCES})

target_link_libraries(cepton-node
  ${catkin_LIBRARIES}
  cepton_sdk
  ${PCL_LIBRARIES}
)
