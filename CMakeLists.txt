cmake_minimum_required(VERSION 2.8.3)
project(cepton)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pcl_conversions
  pcl_ros
  pluginlib
  roscpp
  roslib
  std_msgs
  tf)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES cepton
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include)

# Cepton SDK
set(CEPTON_SDK_DIR "${PROJECT_SOURCE_DIR}/third_party/cepton_sdk")
include_directories(SYSTEM "${CEPTON_SDK_DIR}/include")
link_directories("${CEPTON_SDK_DIR}/lib")

# PCL
find_package(PCL REQUIRED)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Driver
set(CEPTON_ROS_DRIVER_SOURCES
  src/driver.cpp)
add_library(cepton_ros_driver ${CEPTON_ROS_DRIVER_SOURCES})
target_link_libraries(cepton_ros_driver
  ${catkin_LIBRARIES}
  cepton_sdk
  ${PCL_LIBRARIES}
)

# Driver nodelet
set(CEPTON_ROS_DRIVER_NODELET_SOURCES
  src/driver_nodelet.cpp)
add_library(cepton_ros_driver_nodelet ${CEPTON_ROS_DRIVER_NODELET_SOURCES})
target_link_libraries(cepton_ros_driver_nodelet
  ${catkin_LIBRARIES}
  cepton_ros_driver
)

# Cepton node
set(CEPTON_ROS_NODE_SOURCES
  src/cepton_node.cpp)
add_executable(cepton_ros_node ${CEPTON_ROS_NODE_SOURCES})
target_link_libraries(cepton_ros_node
  ${catkin_LIBRARIES}
  cepton_ros_driver
)
