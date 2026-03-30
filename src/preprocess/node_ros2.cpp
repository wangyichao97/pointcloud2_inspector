#include <memory>
#include <string>
#include <vector>

#include <pcl/common/point_tests.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lidar_tools/common/preprocess_utils.hpp"
#include "lidar_tools/preprocess/custom_points.hpp"
#include "lidar_tools/preprocess/lidar_preprocess_tester.hpp"

namespace lidar_tools
{

class LidarPreprocessTesterRos2 : public rclcpp::Node
{
public:
  LidarPreprocessTesterRos2() : Node("lidar_preprocess_tester_node")
  {
    this->declare_parameter<std::string>("topic_name", "/points_raw");
    this->declare_parameter<int>("lidar_type", 6);
    this->declare_parameter<double>("blind", 1.0);
    this->declare_parameter<int>("point_filter_num", 1);
    this->declare_parameter<int>("n_scans", 6);
    this->declare_parameter<bool>("sort_by_time", false);
    this->declare_parameter<bool>("publish_full_cloud", true);
    this->declare_parameter<bool>("publish_surf_cloud", true);

    const auto topic_name = this->get_parameter("topic_name").as_string();
    config_.lidar_type = lidarTypeFromInt(this->get_parameter("lidar_type").as_int());
    config_.blind = this->get_parameter("blind").as_double();
    config_.point_filter_num = static_cast<int>(this->get_parameter("point_filter_num").as_int());
    config_.n_scans = static_cast<int>(this->get_parameter("n_scans").as_int());
    config_.sort_by_time = this->get_parameter("sort_by_time").as_bool();
    publish_full_cloud_ = this->get_parameter("publish_full_cloud").as_bool();
    publish_surf_cloud_ = this->get_parameter("publish_surf_cloud").as_bool();

    tester_ = std::make_shared<LidarPreprocessTester>(config_);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_name, rclcpp::SensorDataQoS(),
      std::bind(&LidarPreprocessTesterRos2::callback, this, std::placeholders::_1));

    if (publish_full_cloud_)
    {
      pub_full_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_preprocess_tester/pl_full", 10);
    }
    if (publish_surf_cloud_)
    {
      pub_surf_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_preprocess_tester/pl_surf", 10);
    }

    RCLCPP_INFO(this->get_logger(),
                "[lidar_tools/lidar_preprocess_tester] Startup: middleware=ROS2, topic_name=%s, lidar_type=%s, blind=%.3f, point_filter_num=%d, n_scans=%d, sort_by_time=%s",
                topic_name.c_str(), lidarTypeToString(config_.lidar_type).c_str(), config_.blind, config_.point_filter_num,
                config_.n_scans, config_.sort_by_time ? "true" : "false");
  }

private:
  std::vector<PreprocessPoint> convertPoints(const sensor_msgs::msg::PointCloud2& msg)
  {
    std::vector<PreprocessPoint> out;

    if (config_.lidar_type == LidarType::MID360)
    {
      pcl::PointCloud<PointXYZIRTTag> cloud;
      pcl::fromROSMsg(msg, cloud);
      out.reserve(cloud.size());
      for (const auto& p : cloud)
      {
        out.push_back(PreprocessPoint{p.x, p.y, p.z, p.intensity, p.ring, p.time, p.tag, pcl::isFinite(p)});
      }
      return out;
    }

    if (config_.lidar_type == LidarType::JT128)
    {
      pcl::PointCloud<PointXYZIRT> cloud;
      pcl::fromROSMsg(msg, cloud);
      out.reserve(cloud.size());
      for (const auto& p : cloud)
      {
        out.push_back(PreprocessPoint{p.x, p.y, p.z, p.intensity, p.ring, p.time, 0U, pcl::isFinite(p)});
      }
      return out;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(msg, cloud);
    out.reserve(cloud.size());
    for (std::size_t i = 0; i < cloud.size(); ++i)
    {
      const auto& p = cloud[i];
      out.push_back(PreprocessPoint{p.x, p.y, p.z, p.intensity,
                                    static_cast<std::uint16_t>(i % static_cast<std::size_t>(config_.n_scans)),
                                    static_cast<double>(i) * 1e-4, 0U, pcl::isFinite(p)});
    }
    return out;
  }

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto points = convertPoints(*msg);
    const auto result = tester_->process(points);

    RCLCPP_INFO(this->get_logger(), "%s", formatPreprocessStats(result.stats, lidarTypeToString(config_.lidar_type)).c_str());

    if (publish_full_cloud_ && pub_full_)
    {
      sensor_msgs::msg::PointCloud2 full_msg;
      pcl::toROSMsg(*result.pl_full, full_msg);
      full_msg.header = msg->header;
      pub_full_->publish(full_msg);
    }

    if (publish_surf_cloud_ && pub_surf_)
    {
      sensor_msgs::msg::PointCloud2 surf_msg;
      pcl::toROSMsg(*result.pl_surf, surf_msg);
      surf_msg.header = msg->header;
      pub_surf_->publish(surf_msg);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_full_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_surf_;

  PreprocessConfig config_;
  bool publish_full_cloud_ = true;
  bool publish_surf_cloud_ = true;
  std::shared_ptr<LidarPreprocessTester> tester_;
};

}  // namespace lidar_tools

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_tools::LidarPreprocessTesterRos2>());
  rclcpp::shutdown();
  return 0;
}
