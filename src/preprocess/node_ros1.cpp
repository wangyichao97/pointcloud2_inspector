#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <pcl/common/point_tests.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "lidar_tools/common/preprocess_utils.hpp"
#include "lidar_tools/preprocess/custom_points.hpp"
#include "lidar_tools/preprocess/lidar_preprocess_tester.hpp"

namespace lidar_tools
{

class LidarPreprocessTesterRos1
{
public:
  LidarPreprocessTesterRos1() : pnh_("~")
  {
    std::string topic_name = "/points_raw";
    int lidar_type = 6;
    double blind = 1.0;
    int point_filter_num = 1;
    int n_scans = 6;
    bool sort_by_time = false;
    bool publish_full_cloud = true;
    bool publish_surf_cloud = true;

    pnh_.param<std::string>("topic_name", topic_name, topic_name);
    pnh_.param<int>("lidar_type", lidar_type, lidar_type);
    pnh_.param<double>("blind", blind, blind);
    pnh_.param<int>("point_filter_num", point_filter_num, point_filter_num);
    pnh_.param<int>("n_scans", n_scans, n_scans);
    pnh_.param<bool>("sort_by_time", sort_by_time, sort_by_time);
    pnh_.param<bool>("publish_full_cloud", publish_full_cloud, publish_full_cloud);
    pnh_.param<bool>("publish_surf_cloud", publish_surf_cloud, publish_surf_cloud);

    config_.lidar_type = lidarTypeFromInt(lidar_type);
    config_.blind = blind;
    config_.point_filter_num = point_filter_num;
    config_.n_scans = n_scans;
    config_.sort_by_time = sort_by_time;

    publish_full_cloud_ = publish_full_cloud;
    publish_surf_cloud_ = publish_surf_cloud;

    tester_ = std::make_shared<LidarPreprocessTester>(config_);

    sub_ = nh_.subscribe(topic_name, 10, &LidarPreprocessTesterRos1::callback, this);
    if (publish_full_cloud_)
    {
      pub_full_ = nh_.advertise<sensor_msgs::PointCloud2>("/lidar_preprocess_tester/pl_full", 10);
    }
    if (publish_surf_cloud_)
    {
      pub_surf_ = nh_.advertise<sensor_msgs::PointCloud2>("/lidar_preprocess_tester/pl_surf", 10);
    }

    ROS_INFO_STREAM("[lidar_tools/lidar_preprocess_tester] Startup: middleware=ROS1, topic_name=" << topic_name
                    << ", lidar_type=" << lidarTypeToString(config_.lidar_type)
                    << ", blind=" << config_.blind
                    << ", point_filter_num=" << config_.point_filter_num
                    << ", n_scans=" << config_.n_scans
                    << ", sort_by_time=" << (config_.sort_by_time ? "true" : "false"));
  }

private:
  std::vector<PreprocessPoint> convertPoints(const sensor_msgs::PointCloud2& msg)
  {
    std::vector<PreprocessPoint> out;

    if (config_.lidar_type == LidarType::MID360)
    {
      pcl::PointCloud<PointXYZIRTTag> cloud;
      pcl::fromROSMsg(msg, cloud);
      out.reserve(cloud.size());
      for (const auto& p : cloud)
      {
        out.push_back(PreprocessPoint{p.x, p.y, p.z, p.intensity, p.ring, p.time, p.tag,
                                      pcl::isFinite(p)});
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
                                    static_cast<double>(i) * 1e-4,
                                    0U,
                                    pcl::isFinite(p)});
    }
    return out;
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    const auto points = convertPoints(*msg);
    const auto result = tester_->process(points);

    ROS_INFO_STREAM(formatPreprocessStats(result.stats, lidarTypeToString(config_.lidar_type)));

    if (publish_full_cloud_ && pub_full_)
    {
      sensor_msgs::PointCloud2 full_msg;
      pcl::toROSMsg(*result.pl_full, full_msg);
      full_msg.header = msg->header;
      pub_full_.publish(full_msg);
    }

    if (publish_surf_cloud_ && pub_surf_)
    {
      sensor_msgs::PointCloud2 surf_msg;
      pcl::toROSMsg(*result.pl_surf, surf_msg);
      surf_msg.header = msg->header;
      pub_surf_.publish(surf_msg);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_full_;
  ros::Publisher pub_surf_;

  PreprocessConfig config_;
  bool publish_full_cloud_ = true;
  bool publish_surf_cloud_ = true;
  std::shared_ptr<LidarPreprocessTester> tester_;
};

}  // namespace lidar_tools

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_preprocess_tester_node");
  lidar_tools::LidarPreprocessTesterRos1 node;
  ros::spin();
  return 0;
}
