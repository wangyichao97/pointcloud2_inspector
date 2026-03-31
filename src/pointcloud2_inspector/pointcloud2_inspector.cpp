#include "lidar_tools/pointcloud2_inspector/pointcloud2_inspector.hpp"

#include "lidar_tools/common/pointcloud_utils.hpp"

namespace lidar_tools
{

PointCloud2Inspector::PointCloud2Inspector(std::string topic_name, int print_every_n_msg)
  : topic_name_(std::move(topic_name)),
    print_every_n_msg_(print_every_n_msg),
    message_count_(0)
{
  if (print_every_n_msg_ <= 0)
  {
    print_every_n_msg_ = 1;
  }
}

bool PointCloud2Inspector::shouldPrint()
{
  ++message_count_;
  return (message_count_ % static_cast<std::uint64_t>(print_every_n_msg_)) == 0U;
}

std::string PointCloud2Inspector::inspect(const PointCloud2View& view) const
{
  return formatPointCloud2Summary(view, topic_name_);
}

}  // namespace lidar_tools
