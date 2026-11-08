"""Print recognized voice commands in a dedicated terminal."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('voice_monitor_node')
        self.subscription = self.create_subscription(
            String, '/voice_command', self.on_voice_command, 10)
        self.get_logger().info(
            'Voice monitor san sang; dang cho du lieu tu /voice_command.')

    def on_voice_command(self, message: String) -> None:
        text = message.data.strip()
        if text:
            print(f'[VOICE] {text}', flush=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceMonitorNode()
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
