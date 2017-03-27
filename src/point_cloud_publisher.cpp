#include "cepton_ros/point_cloud_publisher.hpp"

#include <memory>

namespace {

std::string get_sensor_name(const CeptonSensorInformation &sensor_information) {
  return std::to_string(sensor_information.serial_number);
}

void on_receive_global_(int error_code, CeptonSensorHandle sensor_handle,
                        std::size_t n_points, CeptonSensorPoint const *points) {
  auto &point_cloud_publisher = cepton_ros::PointCloudPublisher::get_instance();
  point_cloud_publisher.on_receive(error_code, sensor_handle, n_points, points);
}

void on_event_global_(int error_code, CeptonSensorHandle sensor_handle,
                      CeptonSensorInformation const *sensor_information_ptr,
                      int sensor_event) {
  auto &point_cloud_publisher = cepton_ros::PointCloudPublisher::get_instance();
  point_cloud_publisher.on_event(error_code, sensor_handle,
                                 sensor_information_ptr, sensor_event);
}
}

namespace cepton_ros {

PointCloudPublisher &PointCloudPublisher::get_instance() {
  static PointCloudPublisher instance;

  return instance;
}

PointCloudPublisher &PointCloudPublisher::initialize(
    ros::NodeHandle &node_handle) {
  auto &point_cloud_publisher = PointCloudPublisher::get_instance();

  point_cloud_publisher.node_handle_ = node_handle;

  int error_code;

  error_code = cepton_sdk_initialize(CEPTON_SDK_VERSION, 0, on_event_global_);
  if (error_code < 0) {
    ROS_WARN("cepton_sdk_initialize failed: %i", error_code);
  }

  error_code = cepton_sdk_listen_frames(on_receive_global_);
  if (error_code < 0) {
    ROS_WARN("cepton_sdk_listen_frames failed: %i", error_code);
  }
}

void PointCloudPublisher::close() {
  if (running_) {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    running_.store(false);
    cepton_sdk_deinitialize();
  }
}

ros::Publisher &PointCloudPublisher::get_sensor_publisher(
    const std::string &sensor_name) {
  if (!sensor_publishers_.count(sensor_name)) {
    const std::string publisher_name = "cepton_point_cloud_" + sensor_name;
    const std::size_t message_buffer_size = 10;
    sensor_publishers_[sensor_name] =
        node_handle_.advertise<sensor_msgs::PointCloud2>(publisher_name,
                                                         message_buffer_size);
  }

  return sensor_publishers_.at(sensor_name);
}

void PointCloudPublisher::on_receive(int error_code,
                                     CeptonSensorHandle sensor_handle,
                                     std::size_t n_points,
                                     CeptonSensorPoint const *points) {
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  if (!running_) {
    return;
  }

  if (error_code < 0) {
    ROS_WARN("on_receive failed: %i", error_code);
  }

  CeptonSensorInformation const *sensor_information_ptr =
      cepton_sdk_get_sensor_information(sensor_handle);
  std::string sensor_name = get_sensor_name(*sensor_information_ptr);

  // Convert to point cloud
  cepton_pcl::CeptonPointCloud::Ptr point_cloud_ptr(
      new cepton_pcl::CeptonPointCloud());
  point_cloud_ptr->reserve(n_points);
  point_cloud_ptr->header.stamp = pcl_conversions::toPCL(ros::Time::now());
  point_cloud_ptr->header.frame_id = "cepton_frame_" + sensor_name;
  point_cloud_ptr->height = 1;

  for (std::size_t i_point = 0; i_point < n_points; ++i_point) {
    const CeptonSensorPoint &cepton_point = points[i_point];
    cepton_pcl::CeptonPoint pcl_point;
    pcl_point.timestamp = cepton_point.timestamp;
    pcl_point.x = cepton_point.x;
    pcl_point.y = cepton_point.y;
    pcl_point.z = cepton_point.z;
    pcl_point.intensity = cepton_point.intensity;
    point_cloud_ptr->points.push_back(pcl_point);
    ++(point_cloud_ptr->width);
  }

  get_sensor_publisher(sensor_name).publish(point_cloud_ptr);
}

void PointCloudPublisher::on_event(
    int error_code, CeptonSensorHandle sensor_handle,
    CeptonSensorInformation const *sensor_information_ptr, int sensor_event) {
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  if (!running_) {
    return;
  }

  if (error_code < 0) {
    ROS_WARN("on_event failed: %i", error_code);
    return;
  }

  std::string sensor_name = get_sensor_name(*sensor_information_ptr);

  switch (sensor_event) {
    case CEPTON_EVENT_ATTACH:
      ROS_INFO("sensor connected: %s", sensor_name.c_str());
      break;
    case CEPTON_EVENT_DETACH:
      ROS_INFO("sensor disconnected: %s", sensor_name.c_str());
      break;
    case CEPTON_EVENT_FRAME:
      break;
  }
}
}
