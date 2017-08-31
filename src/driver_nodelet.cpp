#include "driver_nodelet.hpp"

#include <cmath>
#include <cstdint>

#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cepton_ros::DriverNodelet, nodelet::Nodelet);

namespace {

float square(float x) { return x * x; }
} // namespace

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
  private_node_handle.param("n_frames_per_message", n_frames_per_message,
                            n_frames_per_message);
  private_node_handle.param("output_namespace", output_namespace,
                            output_namespace);

  const std::string sensor_information_topic_id =
      output_namespace + "_sensor_information";
  sensor_information_publisher =
      node_handle.advertise<SensorInformation>(sensor_information_topic_id, 2);

  if (combine_sensors) {
    combined_image_points_publisher =
        node_handle.advertise<sensor_msgs::PointCloud2>(get_points_topic_id(0),
                                                        2);
    combined_points_publisher = node_handle.advertise<sensor_msgs::PointCloud2>(
        get_points_topic_id(0), 2);
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
    if (error_code < 0) {
      NODELET_FATAL("capture replay failed: %s",
                    cepton_get_error_code_name(error_code));
      return;
    }

    error_code = cepton_sdk_capture_replay_resume(true);
    if (error_code < 0) {
      NODELET_FATAL("capture replay failed: %s",
                    cepton_get_error_code_name(error_code));
      return;
    }
  }
}

std::string
DriverNodelet::get_image_points_topic_id(uint64_t sensor_serial_number) const {
  if (combine_sensors) {
    return (output_namespace + "_image_points");
  } else {
    return (output_namespace + "_image_points_" +
            std::to_string(sensor_serial_number));
  }
}

std::string
DriverNodelet::get_points_topic_id(uint64_t sensor_serial_number) const {
  if (combine_sensors) {
    return (output_namespace + "_points");
  } else {
    return (output_namespace + "_points_" +
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

ros::Publisher &
DriverNodelet::get_image_points_publisher(uint64_t sensor_serial_number) {
  if (combine_sensors) {
    return combined_image_points_publisher;
  } else {
    if (!image_points_publishers.count(sensor_serial_number)) {
      std::string topic_id = get_image_points_topic_id(sensor_serial_number);
      image_points_publishers[sensor_serial_number] =
          node_handle.advertise<sensor_msgs::PointCloud2>(topic_id, 10);
    }
    return image_points_publishers.at(sensor_serial_number);
  }
}

ros::Publisher &
DriverNodelet::get_points_publisher(uint64_t sensor_serial_number) {
  if (combine_sensors) {
    return combined_points_publisher;
  } else {
    if (!points_publishers.count(sensor_serial_number)) {
      std::string topic_id = get_points_topic_id(sensor_serial_number);
      points_publishers[sensor_serial_number] =
          node_handle.advertise<sensor_msgs::PointCloud2>(topic_id, 10);
    }
    return points_publishers.at(sensor_serial_number);
  }
}

void DriverNodelet::event_callback(
    int error_code, CeptonSensorHandle sensor_handle,
    CeptonSensorInformation const *sensor_information_ptr, int sensor_event) {
  if (error_code < 0) {
    NODELET_WARN("event callback failed: %s",
                 cepton_get_error_code_name(error_code));
    return;
  }

  uint64_t sensor_serial_number = sensor_information_ptr->serial_number;

  switch (sensor_event) {
  case CEPTON_EVENT_ATTACH:
    NODELET_INFO("sensor connected: %i", sensor_serial_number);
    break;
  case CEPTON_EVENT_DETACH:
    NODELET_INFO("sensor disconnected: %i", sensor_serial_number);
    break;
  case CEPTON_EVENT_FRAME:
    break;
  }
}

void DriverNodelet::image_points_callback(
    int error_code, CeptonSensorHandle sensor_handle,
    std::size_t n_image_points, CeptonSensorImagePoint const *image_points) {
  if (error_code < 0) {
    NODELET_WARN("image points callback failed: %s",
                 cepton_get_error_code_name(error_code));
    return;
  }

  // Get sensor info
  CeptonSensorInformation const *sensor_information_ptr =
      cepton_sdk_get_sensor_information(sensor_handle);
  if (!sensor_information_ptr) {
    NODELET_WARN("failed to get sensor information: sensor handle %i",
                 sensor_handle);
    return;
  }
  uint64_t sensor_serial_number = sensor_information_ptr->serial_number;

  // Cache image points
  auto &image_points_cache_tmp = image_points_cache[sensor_serial_number];
  image_points_cache_tmp.reserve(image_points_cache_tmp.size() +
                                 n_image_points);
  for (std::size_t i_image_point = 0; i_image_point < n_image_points;
       ++i_image_point) {
    image_points_cache_tmp.push_back(image_points[i_image_point]);
  }

  if (!n_cached_frames.count(sensor_serial_number))
    n_cached_frames[sensor_serial_number] = 0;
  auto &n_cached_frames_tmp = n_cached_frames.at(sensor_serial_number);
  ++n_cached_frames_tmp;

  // Publish
  uint64_t message_timestamp = pcl_conversions::toPCL(ros::Time::now());
  publish_sensor_information(*sensor_information_ptr);
  if (n_cached_frames_tmp >= n_frames_per_message) {
    publish_image_points(sensor_serial_number, message_timestamp);
    publish_points(sensor_serial_number, message_timestamp);

    image_points_cache_tmp.clear();
    n_cached_frames_tmp = 0;
  }
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

void DriverNodelet::publish_image_points(uint64_t sensor_serial_number,
                                         uint64_t message_timestamp) {
  const auto &image_points = image_points_cache[sensor_serial_number];

  CeptonImagePointCloud::Ptr image_point_cloud_ptr(new CeptonImagePointCloud());
  image_point_cloud_ptr->header.stamp = message_timestamp;
  image_point_cloud_ptr->header.frame_id = get_frame_id(sensor_serial_number);
  image_point_cloud_ptr->height = 1;
  image_point_cloud_ptr->width = image_points.size();

  image_point_cloud_ptr->resize(image_points.size());
  for (std::size_t i_image_point = 0; i_image_point < image_points.size();
       ++i_image_point) {
    const auto &cepton_image_point = image_points[i_image_point];
    auto &pcl_image_point = image_point_cloud_ptr->points[i_image_point];
    pcl_image_point.timestamp = cepton_image_point.timestamp;
    pcl_image_point.image_x = cepton_image_point.image_x;
    pcl_image_point.distance = cepton_image_point.distance;
    pcl_image_point.image_z = cepton_image_point.image_z;
    pcl_image_point.intensity = cepton_image_point.intensity;
  }

  get_image_points_publisher(sensor_serial_number)
      .publish(image_point_cloud_ptr);
}

void DriverNodelet::publish_points(uint64_t sensor_serial_number,
                                   uint64_t message_timestamp) {
  const auto &image_points = image_points_cache[sensor_serial_number];
  auto &points = points_cache[sensor_serial_number];

  // Convert image points to points
  points.clear();
  points.resize(image_points.size());
  std::size_t i_point = 0;
  for (const auto &image_point : image_points) {
    if (image_point.distance == 0)
      continue;

    convert_image_to_points(image_point, points[i_point]);
    ++i_point;
  }
  points.resize(i_point);

  CeptonPointCloud::Ptr point_cloud_ptr(new CeptonPointCloud());
  point_cloud_ptr->header.stamp = message_timestamp;
  point_cloud_ptr->header.frame_id = get_frame_id(sensor_serial_number);
  point_cloud_ptr->height = 1;
  point_cloud_ptr->width = points.size();

  point_cloud_ptr->resize(points.size());
  for (std::size_t i_point = 0; i_point < points.size(); ++i_point) {
    const auto &cepton_point = points[i_point];
    auto &pcl_point = point_cloud_ptr->points[i_point];
    pcl_point.timestamp = cepton_point.timestamp;
    pcl_point.x = cepton_point.x;
    pcl_point.y = cepton_point.y;
    pcl_point.z = cepton_point.z;
    pcl_point.intensity = cepton_point.intensity;
  }

  get_points_publisher(sensor_serial_number).publish(point_cloud_ptr);
}

} // namespace cepton_ros
