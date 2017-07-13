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
 private:
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle;

  std::string capture_path;
  bool combine_sensors = false;
  std::string output_namespace = "cepton";
  bool output_scanlines = false;
  bool use_sensor_time = false;

  ros::Publisher sensor_information_publisher;
  ros::Publisher combined_points_publisher;
  std::map<std::string, ros::Publisher> sensor_points_publishers;

 public:
  ~DriverNodelet();

  void event_callback(int error_code, CeptonSensorHandle sensor_handle,
                      CeptonSensorInformation const *sensor_information_ptr,
                      int sensor_event);
  void image_points_callback(int error_code, CeptonSensorHandle sensor_handle,
                             std::size_t n_points,
                             CeptonSensorImagePoint const *image_points);
  void points_callback(int error_code, CeptonSensorHandle sensor_handle,
                       std::size_t n_points, CeptonSensorPoint const *points);

 protected:
  void onInit() override;

 private:
  std::string get_sensor_points_topic_id(const std::string &sensor_name) const;
  std::string get_sensor_frame_id(const std::string &sensor_name) const;
  ros::Publisher &get_sensor_points_publisher(const std::string &sensor_name);

  void publish_sensor_information(
      const CeptonSensorInformation &sensor_information);
};
}  // namespace cepton_ros
