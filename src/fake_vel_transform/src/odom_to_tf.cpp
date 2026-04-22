// Converts nav_msgs/Odometry to odom→base_link TF.
// Used in Gazebo simulation where the bridge provides odometry but no TF.

#include "fake_vel_transform/odom_to_tf.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace fake_vel_transform
{

OdomToTf::OdomToTf(const rclcpp::NodeOptions & options)
: Node("odom_to_tf", options)
{
  this->declare_parameter<std::string>("odom_topic", "chassis_odometry_gt");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_link");

  std::string odom_topic;
  this->get_parameter("odom_topic", odom_topic);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10,
    std::bind(&OdomToTf::odomCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "Publishing TF [%s] -> [%s] from topic [%s]",
    odom_frame_.c_str(), base_frame_.c_str(), odom_topic.c_str());
}

void OdomToTf::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = msg->header.stamp;
  t.header.frame_id = odom_frame_;
  t.child_frame_id = base_frame_;
  t.transform.translation.x = msg->pose.pose.position.x;
  t.transform.translation.y = msg->pose.pose.position.y;
  t.transform.translation.z = msg->pose.pose.position.z;
  t.transform.rotation = msg->pose.pose.orientation;
  tf_broadcaster_->sendTransform(t);
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::OdomToTf)
