#pragma once

#include <cstdint>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace cepton_ros {

// http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php

struct CeptonImagePoint {
  int64_t timestamp;
  float image_x;
  float distance;
  float image_z;
  float intensity;
  CeptonSensorReturnType return_type;
#ifdef SIMPLE
  uint8_t flags;
#else
  union {
    uint8_t flags;
    struct {
      uint8_t valid : 1;
      uint8_t saturated : 1;
    };
  };
#endif

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CeptonPoint {
  int64_t timestamp;
  float x;
  float y;
  float z;
  float intensity;
  CeptonSensorReturnType return_type;
#ifdef SIMPLE
  uint8_t flags;
#else
  union {
    uint8_t flags;
    struct {
      uint8_t valid : 1;
      uint8_t saturated : 1;
    };
  };
#endif

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using CeptonImagePointCloud = pcl::PointCloud<CeptonImagePoint>;
using CeptonPointCloud = pcl::PointCloud<CeptonPoint>;
}  // namespace cepton_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(cepton_ros::CeptonImagePoint,
    (double, timestamp, timestamp)
    (float, image_x, image_x)
    (float, distance, distance)
    (float, image_z, image_z)
    (float, intensity, intensity)
    (CeptonSensorReturnType, return_type, return_type)
    (uint8_t, flags, flags)
  )
POINT_CLOUD_REGISTER_POINT_STRUCT(cepton_ros::CeptonPoint,
    (double, timestamp, timestamp)
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (CeptonSensorReturnType, return_type, return_type)
    (uint8_t, flags, flags)
  )
// clang-format on
