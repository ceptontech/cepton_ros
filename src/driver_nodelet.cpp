#include "driver_nodelet.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>

#include "cepton_ros/SensorInformationStamped.h"

PLUGINLIB_EXPORT_CLASS(cepton_ros::DriverNodelet, nodelet::Nodelet);

namespace cepton_ros {

namespace rosutil {
ros::Time from_usec(int64_t usec) {
  ros::Time stamp;
  stamp.sec = double(usec) * 1e-6;
  usec -= int64_t(double(stamp.sec) * 1e6);
  stamp.nsec = double(usec) * 1e3f;
  return stamp;
}

int64_t to_usec(const ros::Time &stamp) {
  int64_t usec = 0;
  usec += double(stamp.sec) * 1e6;
  usec += double(stamp.nsec) * 1e-3;
  return usec;
}
}  // namespace rosutil

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

const std::map<std::string, cepton_sdk::FrameMode> frame_mode_lut = {
    {"COVER", CEPTON_SDK_FRAME_COVER},
    {"CYCLE", CEPTON_SDK_FRAME_CYCLE},
    {"STREAMING", CEPTON_SDK_FRAME_STREAMING},
};

void DriverNodelet::onInit() {
  this->node_handle = getNodeHandle();
  this->private_node_handle = getPrivateNodeHandle();

  // Get parameters
  private_node_handle.param("combine_sensors", combine_sensors,
                            combine_sensors);

  std::string capture_path = "";
  private_node_handle.param("capture_path", capture_path, capture_path);

  int control_flags = 0;
  private_node_handle.param("control_flags", control_flags, control_flags);

  std::string frame_mode_str = "CYCLE";
  private_node_handle.param("frame_mode", frame_mode_str, frame_mode_str);
  const cepton_sdk::FrameMode frame_mode = frame_mode_lut.at(frame_mode_str);

  sensor_information_publisher =
      node_handle.advertise<SensorInformationStamped>(
          "cepton/sensor_information", 2);
  points_publisher =
      node_handle.advertise<sensor_msgs::PointCloud2>("cepton/points", 2);

  // Initialize sdk
  cepton_sdk::SensorError error;
  NODELET_INFO("cepton_sdk %s", cepton_sdk::get_version_string());

  error = error_callback.listen([this](cepton_sdk::SensorHandle handle,
                                       const cepton_sdk::SensorError &error) {
    NODELET_WARN("%s", error.what());
  });
  FATAL_ERROR(error)

  auto options = cepton_sdk::create_options();
  options.control_flags = control_flags;
  options.frame.mode = frame_mode;
  error = cepton_sdk::initialize(
      CEPTON_SDK_VERSION, options,
      &cepton_sdk::api::SensorErrorCallback::global_on_callback,
      &error_callback);
  FATAL_ERROR(error)

  // Start capture
  if (!capture_path.empty()) {
    error = cepton_sdk::api::open_replay(capture_path);
    FATAL_ERROR(error)
    error = cepton_sdk::capture_replay::set_enable_loop(true);
    FATAL_ERROR(error);
    error = cepton_sdk::capture_replay::resume();
    FATAL_ERROR(error)
  }

  // Listen
  error = image_frame_callback.initialize();
  FATAL_ERROR(error);
  error = image_frame_callback.listen(this, &DriverNodelet::on_image_points);
  FATAL_ERROR(error)
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
  publish_points(sensor_info.serial_number);
  image_points.clear();
  points.clear();
}

void DriverNodelet::publish_sensor_information(
    const cepton_sdk::SensorInformation &sensor_information) {
  cepton_ros::SensorInformationStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.info.handle = sensor_information.handle;
  msg.info.serial_number = sensor_information.serial_number;
  msg.info.model_name = sensor_information.model_name;
  msg.info.model = sensor_information.model;
  msg.info.firmware_version = sensor_information.firmware_version;
  const uint8_t *const sensor_information_bytes =
      (const uint8_t *)&sensor_information;
  msg.info.data = std::vector<uint8_t>(
      sensor_information_bytes,
      sensor_information_bytes + sizeof(sensor_information));
  sensor_information_publisher.publish(msg);
}

void DriverNodelet::publish_points(uint64_t sensor_serial_number) {
  // Convert image points to points
  points.clear();
  points.resize(image_points.size());
  for (int i = 0; i < image_points.size(); ++i) {
    cepton_sdk::util::convert_sensor_image_point_to_point(image_points[i],
                                                          points[i]);
  }

  point_cloud.clear();
  point_cloud.header.stamp = rosutil::to_usec(ros::Time::now());
  point_cloud.header.frame_id =
      (combine_sensors) ? "cepton_0"
                        : ("cepton_" + std::to_string(sensor_serial_number));
  point_cloud.height = 1;
  point_cloud.width = points.size();
  point_cloud.resize(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    point_cloud.points[i] = points[i];
  }
  points_publisher.publish(point_cloud);
}

}  // namespace cepton_ros
