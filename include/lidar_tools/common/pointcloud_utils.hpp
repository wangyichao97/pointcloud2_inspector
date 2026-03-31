#pragma once

#include <cstdint>
#include <string>

#include "lidar_tools/common/pointcloud_types.hpp"

namespace lidar_tools
{

std::string pointFieldDatatypeToString(uint8_t datatype);
std::string formatPointCloud2Summary(const PointCloud2View& view, const std::string& topic_name);

}  // namespace lidar_tools
