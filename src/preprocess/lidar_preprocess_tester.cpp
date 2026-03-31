#include "lidar_tools/preprocess/lidar_preprocess_tester.hpp"

#include <algorithm>

namespace lidar_tools
{

namespace
{

void fillTimeRingStats(const std::vector<PreprocessPoint>& points, PreprocessStats* stats)
{
  if (points.empty())
  {
    return;
  }

  stats->min_ring = points.front().ring;
  stats->max_ring = points.front().ring;
  stats->min_time = points.front().time;
  stats->max_time = points.front().time;
  stats->monotonic_time = true;

  double prev = points.front().time;
  for (const auto& p : points)
  {
    stats->min_ring = std::min(stats->min_ring, p.ring);
    stats->max_ring = std::max(stats->max_ring, p.ring);
    stats->min_time = std::min(stats->min_time, p.time);
    stats->max_time = std::max(stats->max_time, p.time);
    if (p.time < prev)
    {
      stats->monotonic_time = false;
    }
    prev = p.time;
  }
}

pcl::PointXYZI toXYZI(const PreprocessPoint& p, const double t0)
{
  pcl::PointXYZI out;
  out.x = p.x;
  out.y = p.y;
  out.z = p.z;
  out.intensity = static_cast<float>(p.time - t0);
  return out;
}

}  // namespace

LidarPreprocessTester::LidarPreprocessTester(const PreprocessConfig& config) : config_(config)
{
  if (config_.point_filter_num <= 0)
  {
    config_.point_filter_num = 1;
  }
  if (config_.n_scans <= 0)
  {
    config_.n_scans = 1;
  }
}

void LidarPreprocessTester::setConfig(const PreprocessConfig& config)
{
  config_ = config;
  if (config_.point_filter_num <= 0)
  {
    config_.point_filter_num = 1;
  }
  if (config_.n_scans <= 0)
  {
    config_.n_scans = 1;
  }
}

const PreprocessConfig& LidarPreprocessTester::config() const
{
  return config_;
}

PreprocessOutput LidarPreprocessTester::process(const std::vector<PreprocessPoint>& points) const
{
  switch (config_.lidar_type)
  {
    case LidarType::MID360:
      return mid360Handler(points);
    case LidarType::JT128:
      return jt128Handler(points);
    case LidarType::AVIA:
    case LidarType::VELO16:
    case LidarType::LS16:
    case LidarType::RS16:
    case LidarType::VELO16SIM:
    case LidarType::HESAIXT32:
    default:
      return genericHandler(points);
  }
}

PreprocessOutput LidarPreprocessTester::genericHandler(const std::vector<PreprocessPoint>& points) const
{
  PreprocessOutput output;
  output.stats.raw_count = points.size();

  std::vector<PreprocessPoint> valid;
  valid.reserve(points.size());
  for (const auto& p : points)
  {
    if (!isFinite(p))
    {
      output.stats.has_non_finite = true;
      continue;
    }
    valid.push_back(p);
  }
  output.stats.after_nan_count = valid.size();

  std::vector<PreprocessPoint> ring_filtered;
  ring_filtered.reserve(valid.size());
  for (const auto& p : valid)
  {
    if (static_cast<int>(p.ring) >= 0 && static_cast<int>(p.ring) < config_.n_scans)
    {
      ring_filtered.push_back(p);
    }
  }
  output.stats.after_ring_count = ring_filtered.size();

  std::vector<PreprocessPoint> blind_filtered;
  blind_filtered.reserve(ring_filtered.size());
  for (const auto& p : ring_filtered)
  {
    if (pointDistance(p) > config_.blind)
    {
      blind_filtered.push_back(p);
    }
  }
  output.stats.after_blind_count = blind_filtered.size();

  fillTimeRingStats(blind_filtered, &output.stats);

  const double t0 = blind_filtered.empty() ? 0.0 : blind_filtered.front().time;
  for (std::size_t i = 0; i < blind_filtered.size(); ++i)
  {
    if ((i % static_cast<std::size_t>(config_.point_filter_num)) != 0)
    {
      continue;
    }
    const auto pt = toXYZI(blind_filtered[i], t0);
    output.pl_full->push_back(pt);
    if ((i % 2U) == 0U)
    {
      output.pl_surf->push_back(pt);
    }
  }

  output.stats.after_sampling_count = output.pl_full->size();
  output.stats.pl_full_size = output.pl_full->size();
  output.stats.pl_surf_size = output.pl_surf->size();
  return output;
}

PreprocessOutput LidarPreprocessTester::mid360Handler(const std::vector<PreprocessPoint>& points) const
{
  PreprocessOutput output = genericHandler(points);

  // MID360 扩展: tag 过滤(保留 tag==0 或 tag==16)
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_full(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_surf(new pcl::PointCloud<pcl::PointXYZI>());

  for (std::size_t i = 0; i < points.size(); ++i)
  {
    const auto& p = points[i];
    if (p.tag != 0U && p.tag != 16U)
    {
      continue;
    }
    if (!isFinite(p) || pointDistance(p) <= config_.blind)
    {
      continue;
    }
    if ((i % static_cast<std::size_t>(config_.point_filter_num)) != 0U)
    {
      continue;
    }

    pcl::PointXYZI out = toXYZI(p, points.empty() ? 0.0 : points.front().time);
    filtered_full->push_back(out);
    if ((i % 3U) == 0U)
    {
      filtered_surf->push_back(out);
    }
  }

  output.pl_full = filtered_full;
  output.pl_surf = filtered_surf;
  output.stats.after_sampling_count = output.pl_full->size();
  output.stats.pl_full_size = output.pl_full->size();
  output.stats.pl_surf_size = output.pl_surf->size();
  return output;
}

PreprocessOutput LidarPreprocessTester::jt128Handler(const std::vector<PreprocessPoint>& points) const
{
  std::vector<PreprocessPoint> local = points;

  if (config_.sort_by_time)
  {
    std::sort(local.begin(), local.end(), [](const PreprocessPoint& a, const PreprocessPoint& b) {
      return a.time < b.time;
    });
  }

  PreprocessOutput output = genericHandler(local);

  // JT128 更强调 ring 与时间顺序检查
  output.stats.monotonic_time = true;
  double prev_time = -1.0;
  for (const auto& p : local)
  {
    if (!isFinite(p))
    {
      continue;
    }
    if (prev_time >= 0.0 && p.time < prev_time)
    {
      output.stats.monotonic_time = false;
      break;
    }
    prev_time = p.time;
  }

  return output;
}

}  // namespace lidar_tools
