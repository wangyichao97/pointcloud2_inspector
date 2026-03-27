#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace pointcloud2_inspector
{

struct PointFieldView
{
  std::string name;
  std::uint32_t offset;
  std::uint8_t datatype;
  std::uint32_t count;
};

struct PointCloud2View
{
  std::string frame_id;
  std::uint32_t height;
  std::uint32_t width;
  bool is_bigendian;
  std::uint32_t point_step;
  std::uint32_t row_step;
  bool is_dense;
  std::vector<PointFieldView> fields;
};

std::string pointFieldDatatypeToString(uint8_t datatype);

std::string formatPointCloud2Report(
  const PointCloud2View& msg,
  const std::string& topic_name,
  const std::string& timestamp_str);

}  // namespace pointcloud2_inspector
