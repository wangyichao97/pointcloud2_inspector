#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "pointcloud2_inspector/pointcloud2_inspector.hpp"

namespace pointcloud2_inspector
{

class PointCloud2InspectorRos1
{
public:
  PointCloud2InspectorRos1()
    : pnh_("~"),
      topic_name_("/points_raw"),
      print_every_n_msg_(1),
      message_count_(0)
  {
    pnh_.param<std::string>("topic_name", topic_name_, topic_name_);
    pnh_.param<int>("print_every_n_msg", print_every_n_msg_, print_every_n_msg_);

    if (print_every_n_msg_ <= 0)
    {
      ROS_WARN("[pointcloud2_inspector] Invalid print_every_n_msg=%d, clamping to 1.", print_every_n_msg_);
      print_every_n_msg_ = 1;
    }

    sub_ = nh_.subscribe(topic_name_, 10, &PointCloud2InspectorRos1::pointCloudCallback, this);

    ROS_INFO_STREAM("[pointcloud2_inspector] Startup: middleware=ROS1 (Noetic), topic_name="
                    << topic_name_ << ", print_every_n_msg=" << print_every_n_msg_);
  }

private:
  static PointCloud2View toView(const sensor_msgs::PointCloud2& msg)
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

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    ++message_count_;
    if ((message_count_ % static_cast<std::uint64_t>(print_every_n_msg_)) != 0U)
    {
      return;
    }

    std::ostringstream stamp_ss;
    stamp_ss << std::fixed << std::setprecision(9) << msg->header.stamp.toSec() << " sec";

    const PointCloud2View view = toView(*msg);
    ROS_INFO_STREAM(formatPointCloud2Report(view, topic_name_, stamp_ss.str()));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;

  std::string topic_name_;
  int print_every_n_msg_;
  std::uint64_t message_count_;
};

}  // namespace pointcloud2_inspector

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud2_inspector");
  pointcloud2_inspector::PointCloud2InspectorRos1 node;
  ros::spin();
  return 0;
}
