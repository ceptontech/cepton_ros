#include "cepton_ros/driver.hpp"

#include <ros/ros.h>

namespace cepton_ros {

void driver_on_receive(int error_code, CeptonSensorHandle sensor_handle,
                       std::size_t n_points, CeptonSensorPoint const *points) {
  auto &driver = cepton_ros::Driver::get_instance();
  std::lock_guard<std::mutex> lock(driver.internal_mutex);

  if (driver.on_receive_callback) {
    driver.on_receive_callback(error_code, sensor_handle, n_points, points);
  }
}

void driver_on_event(int error_code, CeptonSensorHandle sensor_handle,
                     CeptonSensorInformation const *sensor_information_ptr,
                     int sensor_event) {
  auto &driver = cepton_ros::Driver::get_instance();
  std::lock_guard<std::mutex> lock(driver.internal_mutex);

  if (driver.on_event_callback) {
    driver.on_event_callback(error_code, sensor_handle, sensor_information_ptr,
                             sensor_event);
  }
}

Driver::~Driver() { deinitialize(); }

Driver &Driver::get_instance() {
  static Driver instance;

  return instance;
}

bool Driver::initialize(OnReceiveCallback on_receive_callback,
                        OnEventCallback on_event_callback) {
  std::lock_guard<std::mutex> lock(internal_mutex);

  if (initialized) {
    return false;
  }

  this->on_receive_callback = on_receive_callback;
  this->on_event_callback = on_event_callback;

  // Initialize sdk
  int error_code;

  error_code = cepton_sdk_initialize(CEPTON_SDK_VERSION, 0, driver_on_event);
  if (error_code < 0) {
    ROS_FATAL("cepton_sdk_initialize failed [error code %i]", error_code);
    return false;
  }

  error_code = cepton_sdk_listen_frames(driver_on_receive);
  if (error_code < 0) {
    ROS_FATAL("cepton_sdk_listen_frames failed [error code %i]", error_code);
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
