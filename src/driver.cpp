#include "cepton_ros/driver.hpp"

#include <ros/ros.h>

namespace cepton_ros {

void driver_event_callback(
    int error_code, CeptonSensorHandle sensor_handle,
    CeptonSensorInformation const *sensor_information_ptr, int sensor_event) {
  auto &driver = cepton_ros::Driver::get_instance();
  std::lock_guard<std::mutex> lock(driver.internal_mutex);

  if (driver.event_callback) {
    driver.event_callback(error_code, sensor_handle, sensor_information_ptr,
                          sensor_event);
  }
}

void driver_image_points_callback(int error_code,
                                  CeptonSensorHandle sensor_handle,
                                  std::size_t n_points,
                                  CeptonSensorImagePoint const *image_points) {
  auto &driver = cepton_ros::Driver::get_instance();
  std::lock_guard<std::mutex> lock(driver.internal_mutex);

  if (driver.image_points_callback) {
    driver.image_points_callback(error_code, sensor_handle, n_points,
                                 image_points);
  }
}

void driver_points_callback(int error_code, CeptonSensorHandle sensor_handle,
                            std::size_t n_points,
                            CeptonSensorPoint const *points) {
  auto &driver = cepton_ros::Driver::get_instance();
  std::lock_guard<std::mutex> lock(driver.internal_mutex);

  if (driver.points_callback) {
    driver.points_callback(error_code, sensor_handle, n_points, points);
  }
}

Driver::~Driver() { deinitialize(); }

Driver &Driver::get_instance() {
  static Driver instance;

  return instance;
}

void Driver::set_event_callback(const EventCallback &callback) {
  std::lock_guard<std::mutex> lock(internal_mutex);

  this->event_callback = callback;
}

void Driver::set_image_points_callback(const ImagePointsCallback &callback) {
  std::lock_guard<std::mutex> lock(internal_mutex);

  this->image_points_callback = callback;
}

void Driver::set_points_callback(const PointsCallback &callback) {
  std::lock_guard<std::mutex> lock(internal_mutex);

  this->points_callback = callback;
}

bool Driver::initialize(int control_flags) {
  std::lock_guard<std::mutex> lock(internal_mutex);

  if (initialized) {
    return false;
  }

  int error_code;

  // Initialize sdk
  error_code = cepton_sdk_initialize(CEPTON_SDK_VERSION, control_flags,
                                     driver_event_callback);
  if (error_code < 0) {
    ROS_FATAL("cepton_sdk_initialize failed [error code %i]", error_code);
    return false;
  }

  // Listen
  // Image points
  error_code = cepton_sdk_listen_image_frames(driver_image_points_callback);
  if (error_code < 0) {
    ROS_WARN("cepton_sdk_listen_image_frames failed [error code %i]",
             error_code);
    return false;
  }

  // Points
  error_code = cepton_sdk_listen_frames(driver_points_callback);
  if (error_code < 0) {
    ROS_WARN("cepton_sdk_listen_frames failed [error code %i]", error_code);
    return false;
  }

  initialized.store(true);

  return true;
}

void Driver::deinitialize() {
  std::lock_guard<std::mutex> lock(internal_mutex);

  if (initialized) {
    cepton_sdk_deinitialize();
    initialized.store(false);
  }
}
}  // namespace cepton_ros
