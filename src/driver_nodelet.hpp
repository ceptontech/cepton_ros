#pragma once

#include <memory>
#include <string>

#include <nodelet/nodelet.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include "cepton/pcl/point.hpp"
#include "cepton/ros/driver.hpp"

namespace cepton_ros {

class DriverNodelet : public nodelet::Nodelet {
private:
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;

  std::string ros_namespace_ = "cepton";
  bool combine_sensors_ = false;

  ros::Publisher combined_publisher_;
  std::map<std::string, ros::Publisher> sensor_publishers_;
  tf::TransformBroadcaster transform_broadcaster_;

public:
  ~DriverNodelet();

  std::string get_sensor_topic_id(const std::string &sensor_name) const;
  std::string get_sensor_frame_id(const std::string &sensor_name) const;
  ros::Publisher &get_sensor_publisher(const std::string &sensor_name);
  bool is_point_valid(const CeptonSensorPoint &point) const;

  void on_receive_(int error_code, CeptonSensorHandle sensor_handle,
                   std::size_t n_points, CeptonSensorPoint const *points);
  void on_event_(int error_code, CeptonSensorHandle sensor_handle,
                 CeptonSensorInformation const *sensor_information_ptr,
                 int sensor_event);

protected:
  void onInit() override;
};
} // namespace cepton_ros
