#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include <cepton_sdk.h>
#include "cepton_pcl/point.hpp"

namespace cepton_ros {

class PointCloudPublisher {
 public:
  ros::Publisher publisher;
  tf::TransformBroadcaster transform_broadcaster;

 private:
  std::shared_ptr<PointCloudPublisher> instance_ptr_;

  std::atomic<bool> running{true};
  std::size_t frame_idx = 0;

public:
  PointCloudPublisher() = default;

 public:
  ~PointCloudPublisher() { close(); }
  PointCloudPublisher(const PointCloudPublisher&) = delete;
  PointCloudPublisher& operator=(const PointCloudPublisher&) = delete;

  static PointCloudPublisher& get_instance();
  static PointCloudPublisher& initialize(
      ros::NodeHandle &node_handle);

  void close();
  void on_receive(int error_code, CeptonSensorHandle sensor_handle,
                  std::size_t n_points, CeptonSensorPoint const *points);
  void on_event(int error_code, CeptonSensorHandle sensor_handle,
                CeptonSensorInformation const *sensor_information_ptr,
                int sensor_event);
};
}
