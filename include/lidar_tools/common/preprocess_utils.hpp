#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_tools/common/pointcloud_types.hpp"

namespace lidar_tools
{

enum class LidarType : int
{
  VELO16 = 1,
  LS16 = 2,
  RS16 = 3,
  VELO16SIM = 4,
  AVIA = 5,
  MID360 = 6,
  HESAIXT32 = 7,
  JT128 = 8
};

struct PreprocessStats
{
  std::size_t raw_count = 0;
  std::size_t after_nan_count = 0;
  std::size_t after_ring_count = 0;
  std::size_t after_blind_count = 0;
  std::size_t after_sampling_count = 0;
  std::size_t pl_full_size = 0;
  std::size_t pl_surf_size = 0;
  std::uint16_t min_ring = 0;
  std::uint16_t max_ring = 0;
  double min_time = 0.0;
  double max_time = 0.0;
  bool has_non_finite = false;
  bool monotonic_time = true;
};

std::string lidarTypeToString(LidarType type);
LidarType lidarTypeFromInt(int value);
std::string formatPreprocessStats(const PreprocessStats& stats, const std::string& lidar_type_name);

double pointDistance(const PreprocessPoint& p);
bool isFinite(const PreprocessPoint& p);

}  // namespace lidar_tools
