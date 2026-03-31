#include "lidar_tools/common/pointcloud_utils.hpp"

#include <sstream>

namespace lidar_tools
{

std::string pointFieldDatatypeToString(const uint8_t datatype)
{
  switch (datatype)
  {
    case 1:
      return "INT8";
    case 2:
      return "UINT8";
    case 3:
      return "INT16";
    case 4:
      return "UINT16";
    case 5:
      return "INT32";
    case 6:
      return "UINT32";
    case 7:
      return "FLOAT32";
    case 8:
      return "FLOAT64";
    default:
      return "UNKNOWN";
  }
}

std::string formatPointCloud2Summary(const PointCloud2View& view, const std::string& topic_name)
{
  std::ostringstream oss;
  oss << "========== PointCloud2 Inspector ==========" << '\n';
  oss << "topic       : " << topic_name << '\n';
  oss << "frame_id    : " << view.frame_id << '\n';
  oss << "stamp       : " << view.stamp_str << '\n';
  oss << "width/height: " << view.width << " / " << view.height << '\n';
  oss << "point_step  : " << view.point_step << '\n';
  oss << "row_step    : " << view.row_step << '\n';
  oss << "is_dense    : " << (view.is_dense ? "true" : "false") << '\n';
  oss << "is_bigendian: " << (view.is_bigendian ? "true" : "false") << '\n';
  oss << "fields:" << '\n';

  if (view.fields.empty())
  {
    oss << "  [warning] message contains no fields." << '\n';
  }
  else
  {
    for (std::size_t i = 0; i < view.fields.size(); ++i)
    {
      const auto& field = view.fields[i];
      oss << "  [" << i << "] name=" << field.name
          << ", offset=" << field.offset
          << ", datatype=" << static_cast<int>(field.datatype)
          << "(" << pointFieldDatatypeToString(field.datatype) << ")"
          << ", count=" << field.count << '\n';
    }
  }

  if (view.width == 0)
  {
    oss << "  [warning] width is zero." << '\n';
  }
  if (view.point_step == 0)
  {
    oss << "  [warning] point_step is zero." << '\n';
  }

  oss << "summary     : total fields=" << view.fields.size() << '\n';
  oss << "===========================================";
  return oss.str();
}

}  // namespace lidar_tools
