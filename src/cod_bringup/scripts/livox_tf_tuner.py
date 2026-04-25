#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class LivoxTfTuner(Node):
    def __init__(self):
        super().__init__('livox_tf_tuner')

        self.declare_parameter('x', 0.22679)
        self.declare_parameter('y', 0.06741)
        self.declare_parameter('z', 0.41959)
        self.declare_parameter('roll', -1.071025)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 1.789491)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('child_frame_id', 'livox_frame')

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.add_on_set_parameters_callback(self.on_set_parameters)

        self.needs_publish = True
        self.timer = self.create_timer(0.1, self.publish_transform_if_needed)
        self.publish_transform()
        self.needs_publish = False

    def on_set_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name in {'x', 'y', 'z', 'roll', 'pitch', 'yaw'}:
                try:
                    float(parameter.value)
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False, reason=f'{parameter.name} must be numeric')
            elif parameter.name in {'frame_id', 'child_frame_id'}:
                if not isinstance(parameter.value, str) or not parameter.value:
                    return SetParametersResult(successful=False, reason=f'{parameter.name} must be a non-empty string')
        self.needs_publish = True
        return SetParametersResult(successful=True)

    def publish_transform_if_needed(self):
        if self.needs_publish:
            self.publish_transform()
            self.needs_publish = False

    def publish_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.get_parameter('frame_id').value
        transform.child_frame_id = self.get_parameter('child_frame_id').value
        transform.transform.translation.x = float(self.get_parameter('x').value)
        transform.transform.translation.y = float(self.get_parameter('y').value)
        transform.transform.translation.z = float(self.get_parameter('z').value)

        qx, qy, qz, qw = quaternion_from_euler(
            float(self.get_parameter('roll').value),
            float(self.get_parameter('pitch').value),
            float(self.get_parameter('yaw').value),
        )
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)


def quaternion_from_euler(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = LivoxTfTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
