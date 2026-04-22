#ifndef FAKE_VEL_TRANSFORM__ODOM_TO_TF_HPP_
#define FAKE_VEL_TRANSFORM__ODOM_TO_TF_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace fake_vel_transform
{
class OdomToTf : public rclcpp::Node
{
public:
  explicit OdomToTf(const rclcpp::NodeOptions & options);

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string odom_frame_;
  std::string base_frame_;
};

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__ODOM_TO_TF_HPP_
