#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include <cepton_sdk.h>
#include "cepton_pcl/point.hpp"

namespace cepton_ros {

class PointCloudPublisher {
 private:
  std::shared_ptr<PointCloudPublisher> instance_ptr_;

  std::atomic<bool> running_{true};
  std::mutex sdk_mutex_;

  ros::NodeHandle node_handle_;
  std::map<std::string, ros::Publisher> sensor_publishers_;
  tf::TransformBroadcaster transform_broadcaster_;

 public:
  PointCloudPublisher() = default;

 public:
  ~PointCloudPublisher() { close(); }
  PointCloudPublisher(const PointCloudPublisher &) = delete;
  PointCloudPublisher &operator=(const PointCloudPublisher &) = delete;

  static PointCloudPublisher &get_instance();
  static PointCloudPublisher &initialize(ros::NodeHandle &node_handle);

  void close();
  ros::Publisher &get_sensor_publisher(const std::string &sensor_name);
  void on_receive(int error_code, CeptonSensorHandle sensor_handle,
                  std::size_t n_points, CeptonSensorPoint const *points);
  void on_event(int error_code, CeptonSensorHandle sensor_handle,
                CeptonSensorInformation const *sensor_information_ptr,
                int sensor_event);
  void add_sensor(CeptonSensorHandle sensor_handle,
                  const CeptonSensorInformation &sensor_information);
};
}
