"""Publish the simulated spawn pose to AMCL until localization responds."""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__('week7_initial_pose_publisher')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('max_publish_sec', 60.0)
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.started_at = self.get_clock().now()
        self.received_amcl = False
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, 10)
        self.timer = self.create_timer(
            max(float(self.get_parameter('publish_period_sec').value), 0.2),
            self.publish_pose)
        self.publish_pose()

    def on_amcl_pose(self, _message: PoseWithCovarianceStamped) -> None:
        if not self.received_amcl:
            self.received_amcl = True
            self.get_logger().info('AMCL da phan hoi; dung publish initial pose.')

    def publish_pose(self) -> None:
        if self.received_amcl:
            return
        elapsed = (self.get_clock().now() - self.started_at).nanoseconds / 1e9
        if elapsed > float(self.get_parameter('max_publish_sec').value):
            self.get_logger().warning('Het thoi gian cho amcl_pose; dung initial pose.')
            self.timer.cancel()
            return
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        # Zero means use the latest available TF. A current timestamp can be
        # slightly newer than /odom during startup and trigger a false
        # future-extrapolation warning in AMCL.
        message.header.stamp.sec = 0
        message.header.stamp.nanosec = 0
        message.pose.pose.position.x = float(self.get_parameter('x').value)
        message.pose.pose.position.y = float(self.get_parameter('y').value)
        yaw = float(self.get_parameter('yaw').value)
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Small uncertainty around the known Gazebo spawn pose.
        message.pose.covariance[0] = 0.15
        message.pose.covariance[7] = 0.15
        message.pose.covariance[35] = 0.10
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
