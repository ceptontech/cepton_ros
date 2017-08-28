#pragma once

#include <memory>
#include <string>

#include <nodelet/nodelet.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include "cepton_ros/SensorInformation.h"
#include "cepton_ros/driver.hpp"
#include "cepton_ros/point.hpp"

namespace cepton_ros {

class DriverNodelet : public nodelet::Nodelet {
 public:
  ~DriverNodelet();

  void event_callback(int error_code, CeptonSensorHandle sensor_handle,
                      CeptonSensorInformation const *sensor_information_ptr,
                      int sensor_event);
  void image_points_callback(int error_code, CeptonSensorHandle sensor_handle,
                             std::size_t n_points,
                             CeptonSensorImagePoint const *image_points);

 protected:
  void onInit() override;

 private:
  std::string get_sensor_image_points_topic_id(
      const std::string &sensor_name) const;
  std::string get_sensor_points_topic_id(const std::string &sensor_name) const;
  std::string get_sensor_frame_id(const std::string &sensor_name) const;
  ros::Publisher &get_sensor_image_points_publisher(
      const std::string &sensor_name);
  ros::Publisher &get_sensor_points_publisher(const std::string &sensor_name);

  void convert_image_to_points(const CeptonSensorImagePoint &image_point,
                               CeptonSensorPoint &point);
  void publish_sensor_information(
      const CeptonSensorInformation &sensor_information);

  void publish_image_points(const std::string &sensor_name,
                            uint64_t message_timestamp, std::size_t n_points,
                            CeptonSensorImagePoint const *image_points);
  void publish_points(const std::string &sensor_name,
                      uint64_t message_timestamp, std::size_t n_points,
                      CeptonSensorPoint const *points);

 private:
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle;

  bool combine_sensors = false;
  int n_frames_per_message = 2;
  std::string output_namespace = "cepton";

  ros::Publisher sensor_information_publisher;
  ros::Publisher combined_image_points_publisher;
  ros::Publisher combined_points_publisher;
  std::map<std::string, ros::Publisher> sensor_image_points_publishers;
  std::map<std::string, ros::Publisher> sensor_points_publishers;

  std::size_t n_cached_frames = 0;
  std::vector<CeptonSensorImagePoint> image_points_cache;
  std::vector<CeptonSensorPoint> points_cache;
};
}  // namespace cepton_ros
