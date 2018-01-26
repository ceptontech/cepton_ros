#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>

#include <cepton_sdk.h>

namespace cepton_ros {

class Driver {
 public:
  using EventCallback = std::function<void(
      int, CeptonSensorHandle, CeptonSensorInformation const *, int)>;
  using ImagePointsCallback = std::function<void(
      int, CeptonSensorHandle, std::size_t, CeptonSensorImagePoint const *)>;

 public:
  Driver() = default;
  ~Driver();
  Driver(const Driver &) = delete;
  Driver &operator=(const Driver &) = delete;

  static Driver &get_instance();

  void set_event_callback(const EventCallback &callback);
  void set_image_points_callback(const ImagePointsCallback &callback);
  bool initialize(int control_flags = 0);
  void deinitialize();

  friend void driver_event_callback(
      int error_code, CeptonSensorHandle sensor_handle,
      CeptonSensorInformation const *sensor_information_ptr, int sensor_event);
  friend void driver_image_points_callback(
      int error_code, CeptonSensorHandle sensor_handle, std::size_t n_points,
      CeptonSensorImagePoint const *image_points);

 private:
  std::shared_ptr<Driver> instance_ptr;

  std::atomic<bool> initialized{false};
  std::mutex internal_mutex;

  EventCallback event_callback;
  ImagePointsCallback image_points_callback;
};
}  // namespace cepton_ros
