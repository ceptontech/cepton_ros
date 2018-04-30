#pragma once

#include <memory>
#include <string>

#include <nodelet/nodelet.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include <cepton_sdk.hpp>
#include <cepton_sdk_util.hpp>

#include "cepton_ros/SensorInformation.h"
#include "cepton_ros/point.hpp"

namespace cepton_ros {

class DriverNodelet : public nodelet::Nodelet {
 public:
  ~DriverNodelet();

  void on_error(cepton_sdk::SensorHandle sensor_handle, int error_code,
                const char *const error_msg, const void *const error_data,
                size_t error_data_size);
  void on_image_points(
      cepton_sdk::SensorHandle sensor_handle, std::size_t n_points,
      const cepton_sdk::SensorImagePoint *const p_image_points);

 protected:
  void onInit() override;

 private:
  std::string get_image_points_topic_id(uint64_t sensor_serial_number) const;
  std::string get_points_topic_id(uint64_t sensor_serial_number) const;
  std::string get_frame_id(uint64_t sensor_serial_number) const;
  ros::Publisher &get_image_points_publisher(uint64_t sensor_serial_number);
  ros::Publisher &get_points_publisher(uint64_t sensor_serial_number);

  void publish_sensor_information(
      const cepton_sdk::SensorInformation &sensor_information);

  void publish_image_points(uint64_t sensor_serial_number,
                            uint64_t message_timestamp);
  void publish_points(uint64_t sensor_serial_number,
                      uint64_t message_timestamp);

 private:
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle;

  bool combine_sensors = false;
  std::string output_namespace = "cepton";

  ros::Publisher sensor_information_publisher;
  ros::Publisher combined_image_points_publisher;
  ros::Publisher combined_points_publisher;
  std::map<uint64_t, ros::Publisher> image_points_publishers;
  std::map<uint64_t, ros::Publisher> points_publishers;

  std::vector<cepton_sdk::SensorImagePoint> image_points;
  std::vector<cepton_sdk::SensorPoint> points;
};
}  // namespace cepton_ros
