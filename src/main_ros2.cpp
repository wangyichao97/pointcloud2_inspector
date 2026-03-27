#include <cinttypes>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pointcloud2_inspector/pointcloud2_inspector.hpp"

namespace pointcloud2_inspector
{

class PointCloud2InspectorRos2 : public rclcpp::Node
{
public:
  PointCloud2InspectorRos2()
  : Node("pointcloud2_inspector"),
    topic_name_("/points_raw"),
    print_every_n_msg_(1),
    message_count_(0)
  {
    this->declare_parameter<std::string>("topic_name", topic_name_);
    this->declare_parameter<int>("print_every_n_msg", print_every_n_msg_);

    this->get_parameter("topic_name", topic_name_);
    this->get_parameter("print_every_n_msg", print_every_n_msg_);

    if (print_every_n_msg_ <= 0)
    {
      RCLCPP_WARN(this->get_logger(),
                  "[pointcloud2_inspector] Invalid print_every_n_msg=%d, clamping to 1.",
                  print_every_n_msg_);
      print_every_n_msg_ = 1;
    }

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_name_,
      rclcpp::SensorDataQoS(),
      std::bind(&PointCloud2InspectorRos2::pointCloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "[pointcloud2_inspector] Startup: middleware=ROS2 (Foxy/Humble), topic_name=%s, print_every_n_msg=%d",
                topic_name_.c_str(), print_every_n_msg_);
  }

private:
  static PointCloud2View toView(const sensor_msgs::msg::PointCloud2& msg)
  {
    PointCloud2View view;
    view.frame_id = msg.header.frame_id;
    view.height = msg.height;
    view.width = msg.width;
    view.is_bigendian = msg.is_bigendian;
    view.point_step = msg.point_step;
    view.row_step = msg.row_step;
    view.is_dense = msg.is_dense;
    view.fields.reserve(msg.fields.size());

    for (const auto& field : msg.fields)
    {
      PointFieldView field_view;
      field_view.name = field.name;
      field_view.offset = field.offset;
      field_view.datatype = field.datatype;
      field_view.count = field.count;
      view.fields.push_back(field_view);
    }

    return view;
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    ++message_count_;
    if ((message_count_ % static_cast<std::uint64_t>(print_every_n_msg_)) != 0U)
    {
      return;
    }

    const double stamp_sec = static_cast<double>(msg->header.stamp.sec) +
                             static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

    std::ostringstream stamp_ss;
    stamp_ss << "sec=" << msg->header.stamp.sec
             << ", nanosec=" << msg->header.stamp.nanosec
             << ", to_sec=" << std::fixed << std::setprecision(9) << stamp_sec;

    const PointCloud2View view = toView(*msg);
    RCLCPP_INFO(this->get_logger(), "%s", formatPointCloud2Report(view, topic_name_, stamp_ss.str()).c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  std::string topic_name_;
  int print_every_n_msg_;
  std::uint64_t message_count_;
};

}  // namespace pointcloud2_inspector

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pointcloud2_inspector::PointCloud2InspectorRos2>());
  rclcpp::shutdown();
  return 0;
}
