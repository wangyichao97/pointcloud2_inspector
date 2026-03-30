#include <iomanip>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "lidar_tools/common/pointcloud_types.hpp"
#include "lidar_tools/pointcloud2_inspector/pointcloud2_inspector.hpp"

namespace lidar_tools
{

class PointCloud2InspectorRos1
{
public:
  PointCloud2InspectorRos1()
    : pnh_("~")
  {
    std::string topic_name = "/points_raw";
    int print_every_n_msg = 1;
    pnh_.param<std::string>("topic_name", topic_name, topic_name);
    pnh_.param<int>("print_every_n_msg", print_every_n_msg, print_every_n_msg);

    inspector_ = std::make_shared<PointCloud2Inspector>(topic_name, print_every_n_msg);

    sub_ = nh_.subscribe(topic_name, 10, &PointCloud2InspectorRos1::callback, this);
    ROS_INFO_STREAM("[lidar_tools/pointcloud2_inspector] Startup: middleware=ROS1, topic_name="
                    << inspector_->topicName() << ", print_every_n_msg=" << inspector_->printEveryNMsg());
  }

private:
  static PointCloud2View toView(const sensor_msgs::PointCloud2& msg)
  {
    PointCloud2View view;
    view.frame_id = msg.header.frame_id;
    std::ostringstream stamp;
    stamp << std::fixed << std::setprecision(9) << msg.header.stamp.toSec() << " sec";
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

  void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (!inspector_->shouldPrint())
    {
      return;
    }
    ROS_INFO_STREAM(inspector_->inspect(toView(*msg)));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  std::shared_ptr<PointCloud2Inspector> inspector_;
};

}  // namespace lidar_tools

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud2_inspector_node");
  lidar_tools::PointCloud2InspectorRos1 node;
  ros::spin();
  return 0;
}
