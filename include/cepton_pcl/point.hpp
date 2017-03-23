#pragma once

// #define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace cepton_pcl {
//
// // http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php
//
// struct CeptonPoint {
//   PCL_ADD_POINT4D;
//   float intensity;
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// }
// EIGEN_ALIGN16;
//
// // clang-format off
// POINT_CLOUD_REGISTER_POINT_STRUCT (CeptonPoint,
//     (float, x, x)
//     (float, y, y)
//     (float, z, z)
//     (float, intensity, intensity)
//   )
// // clang-format on

using CeptonPoint = pcl::PointXYZ ;
using CeptonPointCloud = pcl::PointCloud<CeptonPoint>;
}
