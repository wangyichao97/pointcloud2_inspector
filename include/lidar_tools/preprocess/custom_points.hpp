#pragma once

#include <cstdint>

#include <pcl/point_types.h>

struct EIGEN_ALIGN16 PointXYZIRTTag
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  double time;
  std::uint8_t tag;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRTTag,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring)
                                  (double, time, time)
                                  (std::uint8_t, tag, tag))

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring)
                                  (double, time, time))
