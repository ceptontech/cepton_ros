cmake_minimum_required(VERSION 2.8)
project(cepton_ros)

# ------------------------------------------------------------------------------
# Variables
# ------------------------------------------------------------------------------
execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINE_ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)
set(OS_NAME "linux-${MACHINE_ARCH}")

# ------------------------------------------------------------------------------
# Compiler Flags
# ------------------------------------------------------------------------------
add_definitions(-std=c++11)

# ------------------------------------------------------------------------------
# External libraries
# ------------------------------------------------------------------------------
# catkin
set(CEPTON_ROS_CATKIN_DEPENDS
  nodelet
  pcl_conversions
  pcl_ros
  pluginlib
  roscpp
  roslib
  rospy
  std_msgs
  tf
)
find_package(catkin REQUIRED COMPONENTS ${CEPTON_ROS_CATKIN_DEPENDS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

# cepton_sdk
set(CEPTON_SDK_SOURCE_DIR "${PROJECT_SOURCE_DIR}/third_party/cepton_sdk")
include_directories(SYSTEM "${CEPTON_SDK_SOURCE_DIR}/include")
list(APPEND CEPTON_ROS_EXTERNAL_LIBRARIES 
  "${CEPTON_SDK_SOURCE_DIR}/lib/${OS_NAME}/libcepton_sdk.a")

# ------------------------------------------------------------------------------
# Targets
# ------------------------------------------------------------------------------
include_directories("${PROJECT_SOURCE_DIR}/include")

add_message_files(FILES
  SensorInformation.msg
  SensorInformationStamped.msg
)
generate_messages(DEPENDENCIES
  std_msgs
)

file(GLOB_RECURSE CEPTON_ROS_SOURCES
  "${PROJECT_SOURCE_DIR}/src/*.cpp")
add_library(cepton_ros ${CEPTON_ROS_SOURCES})
target_link_libraries(cepton_ros
  ${catkin_LIBRARIES}
  ${CEPTON_ROS_EXTERNAL_LIBRARIES}
)
add_dependencies(cepton_ros cepton_ros_generate_messages_cpp)
set(CEPTON_ROS_LIBRARIES
  cepton_ros
)

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
catkin_package(
  CATKIN_DEPENDS ${CEPTON_ROS_CATKIN_DEPENDS}
  DEPENDS
  INCLUDE_DIRS include
  LIBRARIES ${CEPTON_ROS_EXTERNAL_LIBRARIES} ${CEPTON_ROS_LIBRARIES}
)