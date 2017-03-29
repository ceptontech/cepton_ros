#include "cepton/ros/driver_nodelet.hpp"

#include <pluginlib/class_list_macros.h>

#include "cepton/ros/driver.hpp"

PLUGINLIB_EXPORT_CLASS(cepton_ros::DriverNodelet, nodelet::Nodelet)

namespace cepton_ros {

void DriverNodelet::onInit() {
  auto& driver =
      cepton_ros::Driver::initialize(getNodeHandle(), getPrivateNodeHandle());
}
}
