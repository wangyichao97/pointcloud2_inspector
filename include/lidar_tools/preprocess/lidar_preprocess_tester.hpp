#pragma once

#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_tools/common/preprocess_utils.hpp"

namespace lidar_tools
{

struct PreprocessConfig
{
  LidarType lidar_type = LidarType::MID360;
  double blind = 1.0;
  int point_filter_num = 1;
  int n_scans = 6;
  bool sort_by_time = false;
};

struct PreprocessOutput
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pl_full{new pcl::PointCloud<pcl::PointXYZI>()};
  pcl::PointCloud<pcl::PointXYZI>::Ptr pl_surf{new pcl::PointCloud<pcl::PointXYZI>()};
  PreprocessStats stats;
};

class LidarPreprocessTester
{
public:
  explicit LidarPreprocessTester(const PreprocessConfig& config);

  void setConfig(const PreprocessConfig& config);
  const PreprocessConfig& config() const;

  PreprocessOutput process(const std::vector<PreprocessPoint>& points) const;

private:
  PreprocessOutput genericHandler(const std::vector<PreprocessPoint>& points) const;
  PreprocessOutput mid360Handler(const std::vector<PreprocessPoint>& points) const;
  PreprocessOutput jt128Handler(const std::vector<PreprocessPoint>& points) const;

  PreprocessConfig config_;
};

}  // namespace lidar_tools
