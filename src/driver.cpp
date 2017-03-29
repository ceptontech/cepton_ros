#include "cepton/ros/driver.hpp"

#include <memory>

namespace {

const std::size_t message_buffer_size_ = 10;

std::string get_sensor_name_(
    const CeptonSensorInformation &sensor_information) {
  return std::to_string(sensor_information.serial_number);
}

void on_receive_global_(int error_code, CeptonSensorHandle sensor_handle,
                        std::size_t n_points, CeptonSensorPoint const *points) {
  auto &driver = cepton_ros::Driver::get_instance();
  driver.on_receive_(error_code, sensor_handle, n_points, points);
}

void on_event_global_(int error_code, CeptonSensorHandle sensor_handle,
                      CeptonSensorInformation const *sensor_information_ptr,
                      int sensor_event) {
  auto &driver = cepton_ros::Driver::get_instance();
  driver.on_event_(error_code, sensor_handle, sensor_information_ptr,
                   sensor_event);
}
}

namespace cepton_ros {

Driver &Driver::get_instance() {
  static Driver instance;

  return instance;
}

Driver &Driver::initialize(ros::NodeHandle &node_handle,
                           ros::NodeHandle &private_node_handle) {
  auto &driver = Driver::get_instance();
  driver.initialize_impl_(node_handle, private_node_handle);
  return driver;
}

void Driver::initialize_impl_(ros::NodeHandle &node_handle,
                              ros::NodeHandle &private_node_handle) {
  this->node_handle_ = node_handle;
  this->private_node_handle_ = private_node_handle;

  // Get parameters
  private_node_handle_.param("name_prefix", name_prefix_, name_prefix_);
  private_node_handle_.param("combine_sensors", combine_sensors_,
                             combine_sensors_);

  if (combine_sensors_) {
    combined_publisher_ =
        node_handle_.advertise<sensor_msgs::PointCloud2>(
            get_sensor_topic_id(""), message_buffer_size_);
  }

  // Initialize sdk
  int error_code;

  error_code = cepton_sdk_initialize(CEPTON_SDK_VERSION, 0, on_event_global_);
  if (error_code < 0) {
    ROS_WARN("cepton_sdk_initialize failed: %i", error_code);
  }

  error_code = cepton_sdk_listen_frames(on_receive_global_);
  if (error_code < 0) {
    ROS_WARN("cepton_sdk_listen_frames failed: %i", error_code);
  }

  bool previously_initialized = initialized_.exchange(true);
  assert(!previously_initialized);
}

void Driver::close() {
  if (running_) {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    running_.store(false);
    cepton_sdk_deinitialize();
  }
}

std::string Driver::get_sensor_topic_id(const std::string &sensor_name) const {
  if (combine_sensors_) {
    return (name_prefix_ + "_points");
  } else {
    return (name_prefix_ + "_points_" + sensor_name);
  }
}

std::string Driver::get_sensor_frame_id(const std::string &sensor_name) const {
  if (combine_sensors_) {
    return name_prefix_;
  } else {
    return (name_prefix_ + "_" + sensor_name);
  }
}

ros::Publisher &Driver::get_sensor_publisher(const std::string &sensor_name) {
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

void Driver::on_receive_(int error_code, CeptonSensorHandle sensor_handle,
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

void Driver::on_event_(int error_code, CeptonSensorHandle sensor_handle,
                      CeptonSensorInformation const *sensor_information_ptr,
                      int sensor_event) {
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  if (!running_) {
    return;
  }

  if (error_code < 0) {
    ROS_WARN("on_event failed: %i", error_code);
    return;
  }

  std::string sensor_name = get_sensor_name_(*sensor_information_ptr);

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
