#include "cepton/ros/driver.hpp"

#include <ros/ros.h>

namespace cepton_ros {

void driver_on_receive_(int error_code, CeptonSensorHandle sensor_handle,
                        std::size_t n_points, CeptonSensorPoint const *points) {
  auto &driver = cepton_ros::Driver::get_instance();
  std::lock_guard<std::mutex> lock(driver.internal_mutex_);

  if (driver.on_receive_callback_) {
    driver.on_receive_callback_(error_code, sensor_handle, n_points, points);
  }
}

void driver_on_event_(int error_code, CeptonSensorHandle sensor_handle,
                      CeptonSensorInformation const *sensor_information_ptr,
                      int sensor_event) {
  auto &driver = cepton_ros::Driver::get_instance();
  std::lock_guard<std::mutex> lock(driver.internal_mutex_);

  if (driver.on_event_callback_) {
    driver.on_event_callback_(error_code, sensor_handle, sensor_information_ptr,
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
  std::lock_guard<std::mutex> lock(internal_mutex_);

  if (initialized_) {
    return false;
  }

  this->on_receive_callback_ = on_receive_callback;
  this->on_event_callback_ = on_event_callback;

  // Initialize sdk
  int error_code;

  error_code = cepton_sdk_initialize(CEPTON_SDK_VERSION, 0, driver_on_event_);
  if (error_code < 0) {
    ROS_WARN("cepton_sdk_initialize failed [error code %i]", error_code);
    return false;
  }

  error_code = cepton_sdk_listen_frames(driver_on_receive_);
  if (error_code < 0) {
    ROS_WARN("cepton_sdk_listen_frames failed [error code %i]", error_code);
    return false;
  }

  initialized_.store(true);

  return true;
}

void Driver::deinitialize() {
  std::lock_guard<std::mutex> lock(internal_mutex_);

  if (initialized_) {
    cepton_sdk_deinitialize();
    initialized_.store(false);
  }
}
}
