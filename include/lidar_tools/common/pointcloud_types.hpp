#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace lidar_tools
{

struct PointFieldInfo
{
  std::string name;
  std::uint32_t offset;
  std::uint8_t datatype;
  std::uint32_t count;
};

struct PointCloud2View
{
  std::string frame_id;
  std::string stamp_str;
  std::uint32_t height;
  std::uint32_t width;
  bool is_bigendian;
  std::uint32_t point_step;
  std::uint32_t row_step;
  bool is_dense;
  std::vector<PointFieldInfo> fields;
};

struct PreprocessPoint
{
  float x;
  float y;
  float z;
  float intensity;
  std::uint16_t ring;
  double time;
  std::uint8_t tag;
  bool finite;
};

}  // namespace lidar_tools
