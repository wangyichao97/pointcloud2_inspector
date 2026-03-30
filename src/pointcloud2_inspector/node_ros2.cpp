#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lidar_tools/common/pointcloud_types.hpp"
#include "lidar_tools/pointcloud2_inspector/pointcloud2_inspector.hpp"

namespace lidar_tools
{

class PointCloud2InspectorRos2 : public rclcpp::Node
{
public:
  PointCloud2InspectorRos2()
    : Node("pointcloud2_inspector_node")
  {
    this->declare_parameter<std::string>("topic_name", "/points_raw");
    this->declare_parameter<int>("print_every_n_msg", 1);

    const auto topic_name = this->get_parameter("topic_name").as_string();
    const auto print_every_n_msg = this->get_parameter("print_every_n_msg").as_int();

    inspector_ = std::make_shared<PointCloud2Inspector>(topic_name, static_cast<int>(print_every_n_msg));

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_name,
      rclcpp::SensorDataQoS(),
      std::bind(&PointCloud2InspectorRos2::callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "[lidar_tools/pointcloud2_inspector] Startup: middleware=ROS2, topic_name=%s, print_every_n_msg=%d",
                inspector_->topicName().c_str(), inspector_->printEveryNMsg());
  }

private:
  static PointCloud2View toView(const sensor_msgs::msg::PointCloud2& msg)
  {
    PointCloud2View view;
    view.frame_id = msg.header.frame_id;
    std::ostringstream stamp;
    const double sec = static_cast<double>(msg.header.stamp.sec) + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    stamp << "sec=" << msg.header.stamp.sec << ", nanosec=" << msg.header.stamp.nanosec << ", to_sec="
          << std::fixed << std::setprecision(9) << sec;
    view.stamp_str = stamp.str();
    view.height = msg.height;
    view.width = msg.width;
    view.is_bigendian = msg.is_bigendian;
    view.point_step = msg.point_step;
    view.row_step = msg.row_step;
    view.is_dense = msg.is_dense;
    view.fields.reserve(msg.fields.size());
    for (const auto& f : msg.fields)
    {
      view.fields.push_back(PointFieldInfo{f.name, f.offset, f.datatype, f.count});
    }
    return view;
  }

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!inspector_->shouldPrint())
    {
      return;
    }
    RCLCPP_INFO(this->get_logger(), "%s", inspector_->inspect(toView(*msg)).c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  std::shared_ptr<PointCloud2Inspector> inspector_;
};

}  // namespace lidar_tools

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_tools::PointCloud2InspectorRos2>());
  rclcpp::shutdown();
  return 0;
}
