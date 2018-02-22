#pragma once

#include <cstdint>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace cepton_ros {

// http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php

struct CeptonImagePoint {
  double timestamp;
  float image_x;
  float image_z;
  float distance;
  float intensity;
  uint8_t return_number;
  uint8_t valid;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct CeptonPoint {
  double timestamp;
  float x;
  float y;
  float z;
  float intensity;
  uint8_t return_number;
  uint8_t valid;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using CeptonImagePointCloud = pcl::PointCloud<CeptonImagePoint>;
using CeptonPointCloud = pcl::PointCloud<CeptonPoint>;
}  // namespace cepton_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(cepton_ros::CeptonImagePoint,
    (double, timestamp, timestamp)
    (float, image_x, image_x)
    (float, image_z, image_z)
    (float, distance, distance)
    (float, intensity, intensity)
    (uint8_t, return_number, return_number)
    (uint8_t, valid, valid)
  )
POINT_CLOUD_REGISTER_POINT_STRUCT(cepton_ros::CeptonPoint,
    (double, timestamp, timestamp)
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint8_t, return_number, return_number)
    (uint8_t, valid, valid)
  )
// clang-format on
