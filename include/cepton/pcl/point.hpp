#pragma once

#include <cstdint>

// #define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace cepton_pcl {

// http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php

struct CeptonPoint {
  uint64_t timestamp;
  PCL_ADD_POINT4D;
  float intensity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

using CeptonPointCloud = pcl::PointCloud<CeptonPoint>;
}

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(cepton_pcl::CeptonPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
  )
// clang-format on
