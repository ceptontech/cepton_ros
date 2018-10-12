cmake_minimum_required(VERSION 2.8)
project(cepton_ros)

add_definitions(-std=c++11)

# ------------------------------------------------------------------------------
# Variables
# ------------------------------------------------------------------------------
execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINE_ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)
set(OS_NAME "linux-${MACHINE_ARCH}")

# Variables
set(CEPTON_ROS_VERSION_MAJOR 1)
set(CEPTON_ROS_VERSION_MINOR 8)
set(CEPTON_ROS_VERSION_STRING "${CEPTON_ROS_VERSION_MAJOR}.${CEPTON_ROS_VERSION_MINOR}")
add_definitions(-DCEPTON_ROS_VERSION_MAJOR=${CEPTON_ROS_VERSION_MAJOR})
add_definitions(-DCEPTON_ROS_VERSION_MINOR=${CEPTON_ROS_VERSION_MINOR})
add_definitions(-DCEPTON_ROS_VERSION_STRING="${CEPTON_ROS_VERSION_STRING}")

# ------------------------------------------------------------------------------
# External libraries
# ------------------------------------------------------------------------------
# Catkin
set(CEPTON_CATKIN_DEPENDS
  message_generation
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
find_package(catkin REQUIRED COMPONENTS ${CEPTON_CATKIN_DEPENDS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

# Cepton SDK
set(CEPTON_SDK_DIR "${PROJECT_SOURCE_DIR}/third_party/cepton_sdk")
include_directories(SYSTEM "${CEPTON_SDK_DIR}/include")
set(CEPTON_SDK_LIBRARIES "${CEPTON_SDK_DIR}/lib/${OS_NAME}/libcepton_sdk.a")

set(CEPTON_ROS_EXTERNAL_LIBRARIES
  ${CEPTON_SDK_LIBRARIES})

# ------------------------------------------------------------------------------
# Targets
# ------------------------------------------------------------------------------
include_directories(include)

# Cepton ROS
add_message_files(
  FILES
  SensorInformation.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

set(CEPTON_ROS_SOURCES
  src/driver_nodelet.cpp
)
add_library(cepton_ros ${CEPTON_ROS_SOURCES})
target_link_libraries(cepton_ros
  ${catkin_LIBRARIES}
  ${CEPTON_SDK_LIBRARIES}
)
add_dependencies(cepton_ros cepton_ros_generate_messages_cpp)
set(CEPTON_ROS_LIBRARIES
  cepton_ros
)

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
catkin_package(
  CATKIN_DEPENDS ${CEPTON_CATKIN_DEPENDS}
  DEPENDS
  INCLUDE_DIRS include
  LIBRARIES ${CEPTON_ROS_EXTERNAL_LIBRARIES} ${CEPTON_ROS_LIBRARIES}
)

# Libraries
foreach(LIB ${CEPTON_ROS_LIBRARIES})
  install(TARGETS ${LIB}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
endforeach(LIB)

foreach(LIB ${CEPTON_ROS_EXTERNAL_LIBRARIES})
  install(FILES ${LIB}
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
endforeach(LIB)

# Configurations
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Includes
install(DIRECTORY include/cepton_ros/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Plugins
install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Launchers
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
