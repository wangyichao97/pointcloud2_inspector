#pragma once

#include <cstdint>
#include <string>

#include "lidar_tools/common/pointcloud_types.hpp"

namespace lidar_tools
{

class PointCloud2Inspector
{
public:
  PointCloud2Inspector(std::string topic_name, int print_every_n_msg);

  bool shouldPrint();
  std::string inspect(const PointCloud2View& view) const;

  const std::string& topicName() const { return topic_name_; }
  int printEveryNMsg() const { return print_every_n_msg_; }

private:
  std::string topic_name_;
  int print_every_n_msg_;
  std::uint64_t message_count_;
};

}  // namespace lidar_tools
