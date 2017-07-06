#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>

#include <cepton_sdk.h>

namespace cepton_ros {

class Driver {
 public:
  using OnReceiveCallback = std::function<void(
      int, CeptonSensorHandle, std::size_t, CeptonSensorPoint const *)>;
  using OnEventCallback = std::function<void(
      int, CeptonSensorHandle, CeptonSensorInformation const *, int)>;

 private:
  std::shared_ptr<Driver> instance_ptr;

  std::atomic<bool> initialized{false};
  std::mutex internal_mutex;

  OnReceiveCallback on_receive_callback;
  OnEventCallback on_event_callback;

 public:
  Driver() = default;
  ~Driver();
  Driver(const Driver &) = delete;
  Driver &operator=(const Driver &) = delete;

  static Driver &get_instance();

  bool initialize(OnReceiveCallback on_receive_callback,
                  OnEventCallback on_event_callback);
  void deinitialize();

  friend void driver_on_receive(int error_code,
                                CeptonSensorHandle sensor_handle,
                                std::size_t n_points,
                                CeptonSensorPoint const *points);
  friend void driver_on_event(
      int error_code, CeptonSensorHandle sensor_handle,
      CeptonSensorInformation const *sensor_information_ptr, int sensor_event);
};
}  // namespace cepton_ros
