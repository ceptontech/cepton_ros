#include "driver_nodelet.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cepton_ros::DriverNodelet, nodelet::Nodelet);

namespace {

const std::size_t message_buffer_size_ = 10;

std::string get_sensor_name_(
    const CeptonSensorInformation &sensor_information) {
  return std::to_string(sensor_information.serial_number);
}
}

namespace cepton_ros {

DriverNodelet::~DriverNodelet() {
  auto &driver = cepton_ros::Driver::get_instance();
  driver.deinitialize();
}

void DriverNodelet::onInit() {
  this->node_handle_ = getNodeHandle();
  this->private_node_handle_ = getPrivateNodeHandle();

  // Get parameters
  private_node_handle_.param("namespace", ros_namespace_, ros_namespace_);
  private_node_handle_.param("combine_sensors", combine_sensors_,
                             combine_sensors_);

  if (combine_sensors_) {
    combined_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
        get_sensor_topic_id(""), message_buffer_size_);
  }

  // Initialize driver
  auto on_receive_callback =
      [this](int error_code, CeptonSensorHandle sensor_handle,
             std::size_t n_points, CeptonSensorPoint const *points) {
        return on_receive_(error_code, sensor_handle, n_points, points);
      };
  auto on_event_callback =
      [this](int error_code, CeptonSensorHandle sensor_handle,
             CeptonSensorInformation const *sensor_information_ptr,
             int sensor_event) {
        return on_event_(error_code, sensor_handle, sensor_information_ptr,
                         sensor_event);
      };
  auto &driver = cepton_ros::Driver::get_instance();
  if (!driver.initialize(on_receive_callback, on_event_callback)) {
    NODELET_FATAL("driver initialization failed");
  }
}

std::string DriverNodelet::get_sensor_topic_id(
    const std::string &sensor_name) const {
  if (combine_sensors_) {
    return (ros_namespace_ + "_points");
  } else {
    return (ros_namespace_ + "_points_" + sensor_name);
  }
}

std::string DriverNodelet::get_sensor_frame_id(
    const std::string &sensor_name) const {
  if (combine_sensors_) {
    return ros_namespace_;
  } else {
    return (ros_namespace_ + "_" + sensor_name);
  }
}

ros::Publisher &DriverNodelet::get_sensor_publisher(
    const std::string &sensor_name) {
  if (combine_sensors_) {
    return combined_publisher_;
  } else {
    if (!sensor_publishers_.count(sensor_name)) {
      const std::size_t message_buffer_size = 10;
      sensor_publishers_[sensor_name] =
          node_handle_.advertise<sensor_msgs::PointCloud2>(
              get_sensor_topic_id(sensor_name), message_buffer_size);
    }
    return sensor_publishers_.at(sensor_name);
  }
}

void DriverNodelet::on_receive_(int error_code,
                                CeptonSensorHandle sensor_handle,
                                std::size_t n_points,
                                CeptonSensorPoint const *points) {
  if (error_code < 0) {
    NODELET_WARN("on_receive failed: %i", error_code);
  }

  CeptonSensorInformation const *sensor_information_ptr =
      cepton_sdk_get_sensor_information(sensor_handle);
  std::string sensor_name = get_sensor_name_(*sensor_information_ptr);

  // Convert to point cloud
  cepton_pcl::CeptonPointCloud::Ptr point_cloud_ptr(
      new cepton_pcl::CeptonPointCloud());
  point_cloud_ptr->reserve(n_points);
  point_cloud_ptr->header.stamp = pcl_conversions::toPCL(ros::Time::now());
  point_cloud_ptr->header.frame_id = get_sensor_frame_id(sensor_name);
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

void DriverNodelet::on_event_(
    int error_code, CeptonSensorHandle sensor_handle,
    CeptonSensorInformation const *sensor_information_ptr, int sensor_event) {
  if (error_code < 0) {
    NODELET_WARN("on_event failed: %i", error_code);
    return;
  }

  std::string sensor_name = get_sensor_name_(*sensor_information_ptr);

  switch (sensor_event) {
    case CEPTON_EVENT_ATTACH:
      NODELET_INFO("sensor connected: %s", sensor_name.c_str());
      break;
    case CEPTON_EVENT_DETACH:
      NODELET_INFO("sensor disconnected: %s", sensor_name.c_str());
      break;
    case CEPTON_EVENT_FRAME:
      break;
  }
}
}
