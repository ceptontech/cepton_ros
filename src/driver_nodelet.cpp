#include "driver_nodelet.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>

#include "cepton_ros/SensorInformation.h"
#include "cepton_ros/point.hpp"

PLUGINLIB_EXPORT_CLASS(cepton_ros::DriverNodelet, nodelet::Nodelet);

namespace cepton_ros {

#define FATAL_ERROR(error)       \
  if (error) {                   \
    NODELET_FATAL(error.what()); \
    return;                      \
  }

#define WARN_ERROR(error)       \
  if (error) {                  \
    NODELET_WARN(error.what()); \
    return;                     \
  }

DriverNodelet::~DriverNodelet() { cepton_sdk_deinitialize(); }

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

  const std::string sensor_information_topic_id =
      output_namespace + "/sensor_information";
  sensor_information_publisher =
      node_handle.advertise<SensorInformation>(sensor_information_topic_id, 2);

  if (combine_sensors) {
    combined_image_points_publisher =
        node_handle.advertise<sensor_msgs::PointCloud2>(get_points_topic_id(0),
                                                        2);
    combined_points_publisher = node_handle.advertise<sensor_msgs::PointCloud2>(
        get_points_topic_id(0), 2);
  }

  // Initialize sdk
  cepton_sdk::SensorError error;
  NODELET_INFO("cepton_sdk version: %s", cepton_sdk::get_version_string());

  error = error_callback.listen([this](cepton_sdk::SensorHandle handle,
                                       const cepton_sdk::SensorError &error) {
    NODELET_WARN("%s", error.what());
  });
  FATAL_ERROR(error)

  auto options = cepton_sdk::create_options();
  options.control_flags = control_flags;
  options.frame.mode = CEPTON_SDK_FRAME_CYCLE;
  error = cepton_sdk::initialize(
      CEPTON_SDK_VERSION, options,
      &cepton_sdk::api::SensorErrorCallback::global_on_callback,
      &error_callback);
  FATAL_ERROR(error)

  // Start capture
  if (!capture_path.empty()) {
    error = cepton_sdk::api::open_replay(capture_path);
    FATAL_ERROR(error)
    error = cepton_sdk::capture_replay::resume();
    FATAL_ERROR(error)
  }

  // Listen
  error = image_frame_callback.initialize();
  FATAL_ERROR(error);
  error = image_frame_callback.listen(this, &DriverNodelet::on_image_points);
  FATAL_ERROR(error)
}

std::string DriverNodelet::get_points_topic_id(
    uint64_t sensor_serial_number) const {
  if (combine_sensors) {
    return (output_namespace + "/points");
  } else {
    return (output_namespace + "/points/" +
            std::to_string(sensor_serial_number));
  }
}

std::string DriverNodelet::get_frame_id(uint64_t sensor_serial_number) const {
  if (combine_sensors) {
    return output_namespace;
  } else {
    return (output_namespace + "_" + std::to_string(sensor_serial_number));
  }
}

ros::Publisher &DriverNodelet::get_points_publisher(
    uint64_t sensor_serial_number) {
  if (combine_sensors) {
    return combined_points_publisher;
  } else {
    if (!points_publishers.count(sensor_serial_number)) {
      std::string topic_id = get_points_topic_id(sensor_serial_number);
      points_publishers[sensor_serial_number] =
          node_handle.advertise<CeptonPointCloud>(topic_id, 1);
    }
    return points_publishers.at(sensor_serial_number);
  }
}

void DriverNodelet::on_image_points(
    cepton_sdk::SensorHandle handle, std::size_t n_points,
    const cepton_sdk::SensorImagePoint *const c_image_points) {
  cepton_sdk::SensorError error;

  // Get sensor info
  cepton_sdk::SensorInformation sensor_info;
  error = cepton_sdk::get_sensor_information(handle, sensor_info);
  WARN_ERROR(error)

  // Cache image points
  image_points.reserve(n_points);
  for (std::size_t i = 0; i < n_points; ++i) {
    image_points.push_back(c_image_points[i]);
  }

  // Publish sensor info
  publish_sensor_information(sensor_info);

  // Publish points
  uint64_t message_timestamp = pcl_conversions::toPCL(ros::Time::now());
  publish_points(sensor_info.serial_number, message_timestamp);
  image_points.clear();
  points.clear();
}

void DriverNodelet::publish_sensor_information(
    const CeptonSensorInformation &sensor_information) {
  cepton_ros::SensorInformation msg;
  msg.handle = sensor_information.handle;
  msg.serial_number = sensor_information.serial_number;
  msg.model_name = sensor_information.model_name;
  msg.model = sensor_information.model;
  msg.firmware_version = sensor_information.firmware_version;
  sensor_information_publisher.publish(msg);
}

void DriverNodelet::publish_points(uint64_t sensor_serial_number,
                                   uint64_t message_timestamp) {
  // Convert image points to points
  points.clear();
  points.resize(image_points.size());
  std::size_t i_point = 0;
  for (const auto &image_point : image_points) {
    if (image_point.distance == 0.0f) continue;
    cepton_sdk::util::convert_sensor_image_point_to_point(image_point,
                                                          points[i_point]);
    ++i_point;
  }
  points.resize(i_point);

  CeptonPointCloud point_cloud;
  point_cloud.header.stamp = message_timestamp;
  point_cloud.header.frame_id = get_frame_id(sensor_serial_number);
  point_cloud.height = 1;
  point_cloud.width = points.size();
  point_cloud.resize(points.size());
  for (std::size_t i_point = 0; i_point < points.size(); ++i_point) {
    const auto &cepton_point = points[i_point];
    auto &pcl_point = point_cloud.points[i_point];
    pcl_point.timestamp = cepton_point.timestamp;
    pcl_point.image_x = cepton_point.image_x;
    pcl_point.image_z = cepton_point.image_z;
    pcl_point.distance = cepton_point.distance;
    pcl_point.x = cepton_point.x;
    pcl_point.y = cepton_point.y;
    pcl_point.z = cepton_point.z;
    pcl_point.intensity = cepton_point.intensity;
    pcl_point.return_type = cepton_point.return_type;
    pcl_point.valid = cepton_point.valid;
    pcl_point.saturated = cepton_point.saturated;
  }

  get_points_publisher(sensor_serial_number).publish(point_cloud);
}

}  // namespace cepton_ros
