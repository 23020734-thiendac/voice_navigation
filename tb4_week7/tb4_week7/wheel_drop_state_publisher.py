"""Publish the fixed wheel-drop joint states used by the simulated Create 3."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WheelDropStatePublisher(Node):
    """Keep wheel-drop joints available to robot_state_publisher.

    The Ignition controller publishes the two drive-wheel joints, but the
    simulated mechanical wheel-drop joints are not controller joints. Without
    their zero position, RViz cannot build the TF chain to left_wheel and
    right_wheel.
    """

    def __init__(self) -> None:
        super().__init__('wheel_drop_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_states)

    def publish_states(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ['wheel_drop_left_joint', 'wheel_drop_right_joint']
        message.position = [0.0, 0.0]
        message.velocity = [0.0, 0.0]
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelDropStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
