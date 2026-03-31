#include "lidar_tools/common/preprocess_utils.hpp"

#include <cmath>
#include <sstream>

namespace lidar_tools
{

std::string lidarTypeToString(const LidarType type)
{
  switch (type)
  {
    case LidarType::VELO16:
      return "VELO16";
    case LidarType::LS16:
      return "LS16";
    case LidarType::RS16:
      return "RS16";
    case LidarType::VELO16SIM:
      return "VELO16SIM";
    case LidarType::AVIA:
      return "AVIA";
    case LidarType::MID360:
      return "MID360";
    case LidarType::HESAIXT32:
      return "HESAIXT32";
    case LidarType::JT128:
      return "JT128";
    default:
      return "UNKNOWN";
  }
}

LidarType lidarTypeFromInt(const int value)
{
  switch (value)
  {
    case 1:
      return LidarType::VELO16;
    case 2:
      return LidarType::LS16;
    case 3:
      return LidarType::RS16;
    case 4:
      return LidarType::VELO16SIM;
    case 5:
      return LidarType::AVIA;
    case 6:
      return LidarType::MID360;
    case 7:
      return LidarType::HESAIXT32;
    case 8:
      return LidarType::JT128;
    default:
      return LidarType::MID360;
  }
}

std::string formatPreprocessStats(const PreprocessStats& stats, const std::string& lidar_type_name)
{
  std::ostringstream oss;
  oss << "========== LidarPreprocessTester ==========" << '\n';
  oss << "lidar_type      : " << lidar_type_name << '\n';
  oss << "raw_count       : " << stats.raw_count << '\n';
  oss << "after_nan_count : " << stats.after_nan_count << '\n';
  oss << "after_ring_count: " << stats.after_ring_count << '\n';
  oss << "after_blind_count: " << stats.after_blind_count << '\n';
  oss << "after_sample_count: " << stats.after_sampling_count << '\n';
  oss << "ring[min,max]   : [" << stats.min_ring << ", " << stats.max_ring << "]" << '\n';
  oss << "time[min,max]   : [" << stats.min_time << ", " << stats.max_time << "]" << '\n';
  oss << "time_span       : " << (stats.max_time - stats.min_time) << '\n';
  oss << "monotonic_time  : " << (stats.monotonic_time ? "true" : "false") << '\n';
  oss << "has_non_finite  : " << (stats.has_non_finite ? "true" : "false") << '\n';
  oss << "pl_full_size    : " << stats.pl_full_size << '\n';
  oss << "pl_surf_size    : " << stats.pl_surf_size << '\n';
  oss << "===========================================";
  return oss.str();
}

double pointDistance(const PreprocessPoint& p)
{
  return std::sqrt(static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y + static_cast<double>(p.z) * p.z);
}

bool isFinite(const PreprocessPoint& p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(p.intensity) && std::isfinite(p.time);
}

}  // namespace lidar_tools
