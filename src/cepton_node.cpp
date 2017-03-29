#include <ros/ros.h>

#include "cepton/ros/driver.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "cepton_node");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  cepton_ros::Driver::initialize(node_handle, private_node_handle);

  ros::spin();
}
