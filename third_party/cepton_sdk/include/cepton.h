//
// Copyright Cepton Technologies Inc. 2017, All rights reserved.
//
// Cepton Sensor SDK v0.4 (Beta)
//
#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CEPTON_SDK_VERSION 4

typedef uint64_t CeptonSensorHandle;  // Handle of the sensor device

enum CeptonSensorErrorCode {
  CEPTON_SUCCESS = 0,
  CEPTON_ERROR_GENERIC = -1,
  CEPTON_ERROR_OUT_OF_MEMORY = -2,
  CEPTON_ERROR_TOO_MANY_SENSORS = -3,
  CEPTON_ERROR_SENSOR_NOT_FOUND = -4,
  CEPTON_ERROR_SDK_VERSION_MISMATCH = -5,
  CEPTON_ERROR_COMMUNICATION = -6, // Error communicating with the sensor
  CEPTON_ERROR_TOO_MANY_CALLBACKS = -7,
  CEPTON_ERROR_INVALID_ARGUMENTS = -8,
  CEPTON_ERROR_ALREADY_INITIALIZED = -9,
  CEPTON_ERROR_NOT_INITIALIZED = -10,
};

enum CeptonSensorEvent {
  CEPTON_EVENT_ATTACH = 1,
  CEPTON_EVENT_DETACH = 2, // For now never fired
  CEPTON_EVENT_FRAME = 3,
};

#define CEPTON_SDK_CALIBRATION_SIGNATURE 0xB8435343
#define cepton_sdk_max_lasers_per_lidar 8

struct CeptonLaserCalibration {
  int16_t encoder_offset_x;  // [encoder units]
  int16_t encoder_offset_z;  // [encoder units]
  int16_t image_offset_x;    // [encoder units]
  int16_t image_offset_z;    // [encoder units]
  int16_t distance_offset;   // [fpga units]

  int16_t image_min_x;  // [encoder units]
  int16_t image_min_z;  // [encoder units]
  int16_t image_max_x;  // [encoder units]
  int16_t image_max_z;  // [encoder units]

  float image_scale_x;
  float image_scale_z;
  float distance_scale;
  float focal_length;  // [m]
};

struct CeptonSensorCalibration {
  uint32_t signature;
  // Intrinsic calibrations
  float image_clip_min_x;     // [m]
  float image_clip_min_z;     // [m]
  float image_clip_max_x;     // [m]
  float image_clip_max_z;     // [m]
  float image_clip_radius_x;  // [m]
  float image_clip_radius_z;  // [m]

  struct CeptonLaserCalibration
      laser_calibrations[cepton_sdk_max_lasers_per_lidar];

  // Extrinsic calibrations
  float sensor_x;  // [m]
  float sensor_y;  // [m]
  float sensor_z;  // [m]

  // Rotation matrix
  float sensor_m00, sensor_m01, sensor_m02, sensor_m10, sensor_m11, sensor_m12,
      sensor_m20, sensor_m21, sensor_m22;

  // Internal device information
  uint16_t min_depth_cutoff;  // [fpga units]
};

struct CeptonSensorInformation {
  CeptonSensorHandle handle;
  uint64_t serial_number;
  char model_name[32];
  char firmware_version[32];

  float last_reported_temperature; // Celsius
  float last_reported_humidity; // %
  float last_reported_age; // hours

  // Internal data, these will change over time, please don't depend on them
  struct CeptonSensorCalibration calibration;
  uint64_t timestamp_offset;

  // Internal flags
  uint32_t is_mocked : 1; // Set if this device is created through cepton_sdk_mock_network_receive
  uint32_t is_pps_connected : 1; // Set if GPS/PPS is available
  uint32_t is_nmea_connected : 1; // Set if GPS/NMEA is available
  uint32_t unused_flags : 31;
};
//--------------------------------------------
// Global state/service management

typedef void (*FpCeptonSensorEventCallback)(int error_code, CeptonSensorHandle sensor,
  struct CeptonSensorInformation const *p_info, int sensor_event);

enum {
  CEPTON_SDK_CONTROL_RETURN_UNMEASURABLE = 1,
};

// initialize will allocate buffers, make connections, launch threads etc.
// Flag is a bit field defined by the enum above
int cepton_sdk_initialize(int ver, unsigned flags, FpCeptonSensorEventCallback cb);

// deallocate and disconnect
int cepton_sdk_deinitialize();

//--------------------------------------------
// Receiving data from sensor
struct CeptonSensorPoint {
  uint64_t timestamp;  // Microseconds since last successful cepton_sdk_initialize()
  float x, y, z;       // These measurements in meters
  float intensity;     // 0-1 range
};
typedef void (*FpCeptonSensorDataCallback)(int error_code, CeptonSensorHandle sensor,
  size_t n_points, struct CeptonSensorPoint const *p_points);

// Register callbacks that will be called once per "frame"
int cepton_sdk_listen_frames(FpCeptonSensorDataCallback cb);
int cepton_sdk_unlisten_frames(FpCeptonSensorDataCallback cb);

// Register calbacks that will be called once per "scan-line"
int cepton_sdk_listen_scanlines(FpCeptonSensorDataCallback cb);
int cepton_sdk_unlisten_scanlines(FpCeptonSensorDataCallback cb);

//--------------------------------------------
// Discover connected sensors
int cepton_sdk_get_number_of_sensors();

struct CeptonSensorInformation const *cepton_sdk_get_sensor_information(CeptonSensorHandle h);
struct CeptonSensorInformation const *cepton_sdk_get_sensor_information_by_index(int sensor_index);

//--------------------------------------------
// Sensor calibrations
int cepton_sdk_set_calibration(CeptonSensorHandle h,
  struct CeptonSensorCalibration const *cal);

//--------------------------------------------
// Mock Sensor replay and capture
void cepton_sdk_mock_network_receive(uint64_t ipv4_address, uint8_t const *mac,
  uint8_t const *buffer, size_t size);

typedef void(*FpCeptonNetworkReceiveCb)(int error_code, uint64_t ipv4_address, uint8_t const *mac,
  uint8_t const *buffer, size_t size);
int cepton_sdk_listen_network_packet(FpCeptonNetworkReceiveCb cb);

#ifndef CEPTON_PUBLIC_SDK_ONLY // &&&&&&&&&&&&&&&&&& CUT LINE &&&&&&&&&&&&&&&&

//--------------------------------------------
// Raw Sensor output data (pre-calibration)
#pragma pack(push, 1)
struct CeptonSensorDevicePoint {
  int16_t x, z;
  uint16_t distance;
  uint8_t intensity;
};

struct CeptonSensorRawPointWithTime {
  struct CeptonSensorDevicePoint r;
  uint8_t empty; // pack(1) alignment
  uint8_t laser_id;
  uint8_t filled;
  uint32_t timestamp;
};

#pragma pack(pop)


typedef void(*FpCeptonRawDataCallback)(int error_code, CeptonSensorHandle sensor,
  size_t n_points, struct CeptonSensorRawPointWithTime const *p_points);

int cepton_sdk_listen_raw_packets(FpCeptonRawDataCallback cb);
int cepton_sdk_unlisten_raw_packets(FpCeptonRawDataCallback cb);

int cepton_sdk_listen_raw_scanlines(FpCeptonRawDataCallback cb);
int cepton_sdk_unlisten_raw_scanlines(FpCeptonRawDataCallback cb);

int cepton_sdk_listen_raw_frames(FpCeptonRawDataCallback cb);
int cepton_sdk_unlisten_raw_frames(FpCeptonRawDataCallback cb);

int cepton_sdk_calculate_point(struct CeptonSensorCalibration const *cal,
  struct CeptonSensorRawPointWithTime const *p, float *px, float *py, float *pz);

//int cepton_sdk_write_calibrations_serial(int devno, struct CeptonSensorCalibration const *cal);
//--------------------------------------------
// SDK control flags
// NOTE THIS IS CONTINUATION FROM OTHER ENUM ABOVE THE CUTLINE
enum {
  CEPTON_SDK_CONTROL_SNAP_TO_SCANLINE = 0x100,
  CEPTON_SDK_CONTROL_SKIP_CALIBRATION_BLOB = 0x200,
  CEPTON_SDK_CONTROL_UNCORRECTED = 0x400,

  CEPTON_SDK_CONTROL_DEFAULT = 0, // CEPTON_SDK_CONTROL_SNAP_TO_SCANLINE,
};

uint32_t cepton_sdk_control(uint32_t mask, uint32_t flag);

// SDK value keys (use enum instead of functions intentionally)
enum {
  CEPTON_SDK_VALUE_SWEEP_DIR = 1,
  CEPTON_SDK_VALUE_SENSOR_F = 2,
  CEPTON_SDK_VALUE_GAIN_SLOW = 3,
  CEPTON_SDK_VALUE_GAIN_FAST = 4,
  CEPTON_SDK_VALUE_ENCODER_RANGE_SLOW = 5,
  CEPTON_SDK_VALUE_ENCODER_RANGE_FAST = 6,
};

int cepton_sdk_get(CeptonSensorHandle h, int sdk_value, void *pvalue, int value_size);

#endif // CEPTON_PUBLIC_SDK_ONLY &&&&&&&&&&&&&&&&&& CUT LINE &&&&&&&&&&&&&&&&
#ifdef __cplusplus
} // extern "C"
#endif
