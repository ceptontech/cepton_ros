#include "driver_nodelet.hpp"

#include <cmath>
#include <cstdint>

#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cepton_ros::DriverNodelet, nodelet::Nodelet);

namespace {

float square(float x) {
  return x * x;
}

std::string get_sensor_name(const CeptonSensorInformation &sensor_information) {
  return std::to_string(sensor_information.serial_number);
}
}  // namespace

namespace cepton_ros {

DriverNodelet::~DriverNodelet() {
  auto &driver = cepton_ros::Driver::get_instance();
  driver.deinitialize();
}

void DriverNodelet::onInit() {
  this->node_handle = getNodeHandle();
  this->private_node_handle = getPrivateNodeHandle();

  // Get parameters
  std::string capture_path = "";
  private_node_handle.param("capture_path", capture_path, capture_path);
  private_node_handle.param("combine_sensors", combine_sensors,
                            combine_sensors);
  int control_flags = 0;
  private_node_handle.param("control_flags", control_flags, control_flags);
  private_node_handle.param("output_namespace", output_namespace,
                            output_namespace);

  private_node_handle.param("use_sensor_time", use_sensor_time,
                            use_sensor_time);

  const std::string sensor_information_topic_id =
      output_namespace + "_sensor_information";
  sensor_information_publisher =
      node_handle.advertise<SensorInformation>(sensor_information_topic_id, 2);

  if (combine_sensors) {
    combined_image_points_publisher =
        node_handle.advertise<sensor_msgs::PointCloud2>(
            get_sensor_points_topic_id(""), 2);
    combined_points_publisher = node_handle.advertise<sensor_msgs::PointCloud2>(
        get_sensor_points_topic_id(""), 2);
  }

  // Initialize driver
  auto &driver = cepton_ros::Driver::get_instance();
  driver.set_event_callback(
      [this](int error_code, CeptonSensorHandle sensor_handle,
             CeptonSensorInformation const *sensor_information_ptr,
             int sensor_event) {
        event_callback(error_code, sensor_handle, sensor_information_ptr,
                       sensor_event);
      });
  driver.set_image_points_callback(
      [this](int error_code, CeptonSensorHandle sensor_handle,
             std::size_t n_points, CeptonSensorImagePoint const *image_points) {
        image_points_callback(error_code, sensor_handle, n_points,
                              image_points);
      });
  if (!driver.initialize()) {
    NODELET_FATAL("driver initialization failed");
  }

  // Start capture
  if (!capture_path.empty()) {
    int error_code;
    error_code = cepton_sdk_capture_replay_open(capture_path.c_str());
    error_code = cepton_sdk_capture_replay_resume(true);
  }
}

std::string DriverNodelet::get_sensor_image_points_topic_id(
    const std::string &sensor_name) const {
  if (combine_sensors) {
    return (output_namespace + "_image_points");
  } else {
    return (output_namespace + "_image_points_" + sensor_name);
  }
}

std::string DriverNodelet::get_sensor_points_topic_id(
    const std::string &sensor_name) const {
  if (combine_sensors) {
    return (output_namespace + "_points");
  } else {
    return (output_namespace + "_points_" + sensor_name);
  }
}

std::string DriverNodelet::get_sensor_frame_id(
    const std::string &sensor_name) const {
  if (combine_sensors) {
    return output_namespace;
  } else {
    return (output_namespace + "_" + sensor_name);
  }
}

ros::Publisher &DriverNodelet::get_sensor_image_points_publisher(
    const std::string &sensor_name) {
  if (combine_sensors) {
    return combined_image_points_publisher;
  } else {
    if (!sensor_image_points_publishers.count(sensor_name)) {
      std::string topic_id = get_sensor_image_points_topic_id(sensor_name);
      sensor_image_points_publishers[sensor_name] =
          node_handle.advertise<sensor_msgs::PointCloud2>(topic_id, 10);
    }
    return sensor_image_points_publishers.at(sensor_name);
  }
}

ros::Publisher &DriverNodelet::get_sensor_points_publisher(
    const std::string &sensor_name) {
  if (combine_sensors) {
    return combined_points_publisher;
  } else {
    if (!sensor_points_publishers.count(sensor_name)) {
      std::string topic_id = get_sensor_points_topic_id(sensor_name);
      sensor_points_publishers[sensor_name] =
          node_handle.advertise<sensor_msgs::PointCloud2>(topic_id, 10);
    }
    return sensor_points_publishers.at(sensor_name);
  }
}

void DriverNodelet::event_callback(
    int error_code, CeptonSensorHandle sensor_handle,
    CeptonSensorInformation const *sensor_information_ptr, int sensor_event) {
  if (error_code < 0) {
    NODELET_WARN("event callback failed: [error code %i]", error_code);
    return;
  }
  std::string sensor_name = get_sensor_name(*sensor_information_ptr);

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

void DriverNodelet::image_points_callback(
    int error_code, CeptonSensorHandle sensor_handle, std::size_t n_image_points,
    CeptonSensorImagePoint const *image_points) {
  if (error_code < 0) {
    NODELET_WARN("image points callback failed: error code %i", error_code);
    return;
  }

  CeptonSensorInformation const *sensor_information_ptr =
      cepton_sdk_get_sensor_information(sensor_handle);
  if (!sensor_information_ptr) {
    NODELET_WARN("failed to get sensor information: sensor handle %i",
                 sensor_handle);
    return;
  }
  std::string sensor_name = get_sensor_name(*sensor_information_ptr);

  // Convert to points
  points.clear();
  points.resize(n_image_points);
  std::size_t i_point = 0;
  for (std::size_t i_image_point = 0; i_image_point < n_image_points;
       ++i_image_point) {
    if (image_points[i_image_point].distance == 0) continue;

    convert_image_to_points(image_points[i_image_point], points[i_point]);
    ++i_point;
  }
  std::size_t n_points = i_point;
  points.resize(n_points);

  // Publish
  // uint64_t message_timestamp = image_points[n_image_points - 1].timestamp;
  uint64_t message_timestamp = pcl_conversions::toPCL(ros::Time::now());
  publish_sensor_information(*sensor_information_ptr);
  publish_image_points(sensor_name, message_timestamp, n_image_points,
                       image_points);
  publish_points(sensor_name, message_timestamp, n_points, points.data());
}

void DriverNodelet::convert_image_to_points(
    const CeptonSensorImagePoint &image_point, CeptonSensorPoint &point) {
  float hypotenuse_small =
      std::sqrt(square(image_point.image_x) + square(image_point.image_z) + 1);
  float ratio = image_point.distance / hypotenuse_small;
  point.x = -image_point.image_x * ratio;
  point.y = ratio;
  point.z = -image_point.image_z * ratio;

  point.timestamp = image_point.timestamp;
  point.intensity = image_point.intensity;
}

void DriverNodelet::publish_sensor_information(
    const CeptonSensorInformation &sensor_information) {
  cepton_ros::SensorInformation msg;
  msg.handle = sensor_information.handle;
  msg.serial_number = sensor_information.serial_number;
  msg.model_name = sensor_information.model_name;
  msg.firmware_version = sensor_information.firmware_version;
  sensor_information_publisher.publish(msg);
}

void DriverNodelet::publish_image_points(
    const std::string &sensor_name, uint64_t message_timestamp,
    std::size_t n_image_points, CeptonSensorImagePoint const *image_points) {
  CeptonImagePointCloud::Ptr image_point_cloud_ptr(new CeptonImagePointCloud());
  image_point_cloud_ptr->header.stamp = message_timestamp;
  image_point_cloud_ptr->header.frame_id = get_sensor_frame_id(sensor_name);
  image_point_cloud_ptr->height = 1;
  image_point_cloud_ptr->width = n_image_points;

  image_point_cloud_ptr->resize(n_image_points);
  for (std::size_t i_image_point = 0; i_image_point < n_image_points;
       ++i_image_point) {
    const auto &cepton_image_point = image_points[i_image_point];
    auto &pcl_image_point = image_point_cloud_ptr->points[i_image_point];
    pcl_image_point.timestamp = cepton_image_point.timestamp;
    pcl_image_point.image_x = cepton_image_point.image_x;
    pcl_image_point.distance = cepton_image_point.distance;
    pcl_image_point.image_z = cepton_image_point.image_z;
    pcl_image_point.intensity = cepton_image_point.intensity;
  }

  get_sensor_image_points_publisher(sensor_name).publish(image_point_cloud_ptr);
}

void DriverNodelet::publish_points(const std::string &sensor_name,
                                   uint64_t message_timestamp,
                                   std::size_t n_points,
                                   CeptonSensorPoint const *points) {
  CeptonPointCloud::Ptr point_cloud_ptr(new CeptonPointCloud());
  point_cloud_ptr->header.stamp = message_timestamp;
  point_cloud_ptr->header.frame_id = get_sensor_frame_id(sensor_name);
  point_cloud_ptr->height = 1;
  point_cloud_ptr->width = n_points;

  point_cloud_ptr->resize(n_points);
  for (std::size_t i_point = 0; i_point < n_points; ++i_point) {
    const auto &cepton_point = points[i_point];
    auto &pcl_point = point_cloud_ptr->points[i_point];
    pcl_point.timestamp = cepton_point.timestamp;
    pcl_point.x = cepton_point.x;
    pcl_point.y = cepton_point.y;
    pcl_point.z = cepton_point.z;
    pcl_point.intensity = cepton_point.intensity;
  }

  get_sensor_points_publisher(sensor_name).publish(point_cloud_ptr);
}

}  // namespace cepton_ros
