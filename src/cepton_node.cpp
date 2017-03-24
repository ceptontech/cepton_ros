#include <ros/ros.h>

#include "cepton_ros/point_cloud_publisher.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "cepton");
  ros::NodeHandle node_handle;

  auto& point_cloud_publisher =
      cepton_ros::PointCloudPublisher::initialize(node_handle);

  ros::spin();

  return 0;
}
