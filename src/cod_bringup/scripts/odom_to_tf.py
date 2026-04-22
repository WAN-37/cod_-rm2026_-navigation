#!/usr/bin/env python3
"""
Converts nav_msgs/Odometry messages into odom→base_link TF broadcasts.
Used in Gazebo simulation where the bridge provides odometry but no TF.

Uses raw=True subscription to work around the ROS2 Humble + ros_gz_bridge
deserialization bug (RuntimeError: Unable to convert call argument to Python object).
"""

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        self.declare_parameter('odom_topic', 'chassis_odometry_gt')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_broadcaster = TransformBroadcaster(self)
        # raw=True: receive CDR bytes to avoid C++ deserialization bug with
        # ros_gz_bridge Odometry messages on ROS2 Humble
        self.sub = self.create_subscription(
            Odometry, odom_topic, self.odom_cb, 10, raw=True)

        self.get_logger().info(
            f'Publishing TF [{self.odom_frame}] -> [{self.base_frame}] '
            f'from topic [{odom_topic}]')

    def odom_cb(self, raw_msg):
        msg = deserialize_message(raw_msg, Odometry)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
