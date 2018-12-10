#pragma once

#include <string>

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <cepton_sdk_api.hpp>

namespace cepton_ros {

class DriverNodelet : public nodelet::Nodelet {
 public:
  ~DriverNodelet();

  void on_image_points(cepton_sdk::SensorHandle sensor_handle,
                       std::size_t n_points,
                       const cepton_sdk::SensorImagePoint *const image_points);

 protected:
  void onInit() override;

 private:
  std::string get_points_topic_id(uint64_t sensor_serial_number) const;
  std::string get_frame_id(uint64_t sensor_serial_number) const;
  ros::Publisher &get_points_publisher(uint64_t sensor_serial_number);

  void publish_sensor_information(
      const cepton_sdk::SensorInformation &sensor_information);
  void publish_points(uint64_t sensor_serial_number,
                      uint64_t message_timestamp);

 private:
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle;

  bool combine_sensors = false;
  std::string output_namespace = "cepton";

  cepton_sdk::api::SensorErrorCallback error_callback;
  cepton_sdk::api::SensorImageFrameCallback image_frame_callback;

  ros::Timer timer;
  ros::Publisher sensor_information_publisher;
  ros::Publisher combined_image_points_publisher;
  ros::Publisher combined_points_publisher;
  std::map<uint64_t, ros::Publisher> points_publishers;

  std::vector<cepton_sdk::SensorImagePoint> image_points;
  std::vector<cepton_sdk::util::SensorPoint> points;
};
}  // namespace cepton_ros
