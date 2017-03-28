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

class Driver {
 private:
  std::shared_ptr<Driver> instance_ptr_;

  std::atomic<bool> initialized_{false};
  std::atomic<bool> running_{true};
  std::mutex sdk_mutex_;

  std::string name_prefix_ = "cepton";
  bool combine_sensors_ = false;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;

  ros::Publisher combined_publisher_;
  std::map<std::string, ros::Publisher> sensor_publishers_;
  tf::TransformBroadcaster transform_broadcaster_;

 public:
  Driver() = default;
  ~Driver() { close(); }
  Driver(const Driver &) = delete;
  Driver &operator=(const Driver &) = delete;

  static Driver &get_instance();
  static Driver &initialize(ros::NodeHandle &node_handle,
                            ros::NodeHandle &private_node_handle);
  void close();

  std::string get_sensor_topic_id(const std::string &sensor_name) const;
  std::string get_sensor_frame_id(const std::string &sensor_name) const;
  ros::Publisher &get_sensor_publisher(const std::string &sensor_name);

  void on_receive_(int error_code, CeptonSensorHandle sensor_handle,
                   std::size_t n_points, CeptonSensorPoint const *points);
  void on_event_(int error_code, CeptonSensorHandle sensor_handle,
                 CeptonSensorInformation const *sensor_information_ptr,
                 int sensor_event);

 private:
  void initialize_impl_(ros::NodeHandle &node_handle,
                        ros::NodeHandle &private_node_handle);
};
}
