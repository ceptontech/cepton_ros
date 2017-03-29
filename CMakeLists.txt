cmake_minimum_required(VERSION 2.8.3)
project(cepton)

add_definitions(-std=c++11)

# External libraries
# ------------------
# Catkin
set(CEPTON_CATKIN_DEPENDS
  nodelet
  pcl_conversions
  pcl_ros
  pluginlib
  roscpp
  roslib
  std_msgs
  tf
)
find_package(catkin REQUIRED COMPONENTS ${CEPTON_CATKIN_DEPENDS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

# Cepton SDK
set(CEPTON_SDK_DIR "${PROJECT_SOURCE_DIR}/third_party/cepton_sdk")
include_directories(SYSTEM "${CEPTON_SDK_DIR}/include")
link_directories("${CEPTON_SDK_DIR}/lib")

# Targets
# -------
include_directories(include)

# Driver
set(CEPTON_ROS_DRIVER_SOURCES
  src/driver.cpp
)
add_library(cepton_ros_driver ${CEPTON_ROS_DRIVER_SOURCES})
target_link_libraries(cepton_ros_driver
  ${catkin_LIBRARIES}
  cepton_sdk
)

# Driver nodelet
set(CEPTON_ROS_DRIVER_NODELET_SOURCES
  src/driver_nodelet.cpp
)
add_library(cepton_ros_driver_nodelet ${CEPTON_ROS_DRIVER_NODELET_SOURCES})
target_link_libraries(cepton_ros_driver_nodelet
  ${catkin_LIBRARIES}
  cepton_ros_driver
)

# Cepton node
set(CEPTON_ROS_NODE_SOURCES
  src/cepton_node.cpp
)
add_executable(cepton_ros_node ${CEPTON_ROS_NODE_SOURCES})
target_link_libraries(cepton_ros_node
  ${catkin_LIBRARIES}
  cepton_ros_driver
)

set(CEPTON_ROS_LIBRARIES
  cepton_ros_driver
cepton_ros_driver_nodelet)

# Install
# -------
catkin_package(
  CATKIN_DEPENDS ${CEPTON_CATKIN_DEPENDS}
  DEPENDS
  INCLUDE_DIRS include
  LIBRARIES ${CEPTON_ROS_LIBRARIES}
)

# Libraries
foreach(LIB ${CEPTON_ROS_LIBRARIES})
  install(TARGETS ${LIB}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
endforeach(LIB)

# Includes
install(DIRECTORY include/cepton/
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
