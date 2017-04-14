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
  rospy
  std_msgs
  tf
)
find_package(catkin REQUIRED COMPONENTS ${CEPTON_CATKIN_DEPENDS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

# Cepton SDK
set(CEPTON_SDK_DIR "${PROJECT_SOURCE_DIR}/third_party/cepton_sdk")
include_directories(SYSTEM "${CEPTON_SDK_DIR}/include")
find_library(CEPTON_SDK_LIBRARIES NAMES cepton_sdk PATHS "${CEPTON_SDK_DIR}/lib")

set(CEPTON_ROS_EXTERNAL_LIBRARIES
  ${CEPTON_SDK_LIBRARIES})

# Targets
# -------
include_directories(include)

# Cepton ROS
set(CEPTON_ROS_SOURCES
  src/driver.cpp
  src/driver_nodelet.cpp
)
add_library(cepton_ros ${CEPTON_ROS_SOURCES})
target_link_libraries(cepton_ros
  ${catkin_LIBRARIES}
  ${CEPTON_SDK_LIBRARIES}
)

set(CEPTON_ROS_LIBRARIES
  cepton_ros
)

# Install
# -------
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
