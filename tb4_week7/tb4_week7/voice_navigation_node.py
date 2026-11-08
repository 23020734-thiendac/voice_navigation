"""Parse Vietnamese/English area commands and send NavigateToPose goals."""

from __future__ import annotations

import math
from pathlib import Path
import re
import threading
import unicodedata
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def normalize(text: str) -> str:
    text = text.lower().replace('đ', 'd')
    text = unicodedata.normalize('NFD', text)
    return ''.join(char for char in text if unicodedata.category(char) != 'Mn')


class VoiceNavigationNode(Node):
    """Route one or more semantic destinations through Nav2."""

    def __init__(self) -> None:
        super().__init__('voice_navigation_node')
        self.declare_parameter('zones_yaml', '')
        self.declare_parameter('action_name', '/navigate_to_pose')
        self.declare_parameter('goal_timeout_sec', 180.0)
        self.declare_parameter('stop_on_failure', True)
        zones_path = Path(str(self.get_parameter('zones_yaml').value))
        self.zones: Dict[str, dict] = yaml.safe_load(zones_path.read_text())['zones']
        self.aliases: List[Tuple[str, str]] = []
        self.targets: Dict[str, Tuple[float, float, float]] = {}
        for name, zone in self.zones.items():
            self.targets[name] = tuple(float(v) for v in zone['target'])
            for alias in zone.get('aliases', []):
                self.aliases.append((normalize(alias), name))
        self.aliases.sort(key=lambda item: len(item[0]), reverse=True)

        self.action_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter('action_name').value))
        self.voice_sub = self.create_subscription(
            String, '/voice_command', self.on_command, 10)
        self.text_sub = self.create_subscription(
            String, '/voice_command_text', self.on_command, 10)
        self.status_pub = self.create_publisher(
            String, '/voice_navigation_status', 10)
        self.route_pub = self.create_publisher(
            MarkerArray, '/semantic_zones/route', 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.busy = False
        self.active_goal_handle = None
        self.stop_requested = False
        self.lock = threading.Lock()
        self.get_logger().info(
            'Voice navigation san sang. Nhan /voice_command hoac '
            '/voice_command_text.')

    def publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self.status_pub.publish(message)
        self.get_logger().info(text)

    def find_area_mentions(self, text: str) -> List[str]:
        normalized = normalize(text)
        found: List[Tuple[int, str]] = []
        for alias, name in self.aliases:
            start = 0
            while True:
                index = normalized.find(alias, start)
                if index < 0:
                    break
                found.append((index, name))
                start = index + len(alias)
        found.sort(key=lambda item: (item[0], -len(item[1])))
        result: List[str] = []
        for _, name in found:
            if not result or result[-1] != name:
                result.append(name)
        return result

    def is_stop_command(self, text: str) -> bool:
        value = normalize(text).strip()
        stop_phrases = {
            'stop', 'dung', 'dung lai', 'dung robot', 'dung di',
            'dung dieu huong', 'dung navigation',
        }
        return value in stop_phrases or value.startswith('stop ')

    def stop_robot(self) -> None:
        # Cancel Nav2 first, then publish a zero twist as an immediate safety
        # command. The active goal can be absent for a short send-goal race;
        # stop_requested covers that case as well.
        with self.lock:
            self.stop_requested = True
            goal_handle = self.active_goal_handle
        self.cmd_vel_pub.publish(Twist())
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        self.publish_status('Da dung robot theo lenh Stop.')

    def on_command(self, message: String) -> None:
        text = message.data.strip()
        if not text:
            return
        if self.is_stop_command(text):
            self.stop_robot()
            return
        route = self.find_area_mentions(text)
        if not route:
            self.publish_status(
                'Khong tim thay khu vuc. Hay dung: bedroom, living room, '
                'kitchen hoac fitness area.')
            return
        with self.lock:
            if self.busy:
                self.publish_status('Robot dang xu ly lenh truoc; bo qua lenh moi.')
                return
            self.busy = True
            self.stop_requested = False
            self.active_goal_handle = None
        self.publish_route(route)
        thread = threading.Thread(
            target=self.execute_route, args=(route,), daemon=True)
        thread.start()

    def publish_route(self, route: List[str]) -> None:
        message = MarkerArray()
        for index, name in enumerate(route):
            x, y, _ = self.targets[name]
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns = 'voice_route'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.12
            marker.scale.x = marker.scale.y = marker.scale.z = 0.28
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 1.0
            message.markers.append(marker)
        self.route_pub.publish(message)

    def make_goal(self, target: Tuple[float, float, float]) -> PoseStamped:
        x, y, yaw = target
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def execute_route(self, route: List[str]) -> None:
        try:
            self.publish_status('Bat dau lo trinh: ' + ' -> '.join(
                self.zones[name]['label'] for name in route))
            if not self.action_client.wait_for_server(timeout_sec=30.0):
                self.publish_status('Khong tim thay action /navigate_to_pose cua Nav2.')
                return
            timeout = float(self.get_parameter('goal_timeout_sec').value)
            for index, name in enumerate(route, start=1):
                target = self.targets[name]
                self.publish_status(
                    f'Dang di toi {self.zones[name]["label"]} '
                    f'({index}/{len(route)}): x={target[0]:.2f}, y={target[1]:.2f}')
                goal = NavigateToPose.Goal()
                goal.pose = self.make_goal(target)
                done = threading.Event()
                future = self.action_client.send_goal_async(goal)
                future.add_done_callback(lambda _: done.set())
                if not done.wait(timeout=30.0):
                    self.publish_status('Timeout khi gui goal Nav2.')
                    return
                goal_handle = future.result()
                if goal_handle is None or not goal_handle.accepted:
                    self.publish_status('Nav2 tu choi goal.')
                    return
                with self.lock:
                    self.active_goal_handle = goal_handle
                    stop_requested = self.stop_requested
                if stop_requested:
                    goal_handle.cancel_goal_async()
                result_done = threading.Event()
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(lambda _: result_done.set())
                if not result_done.wait(timeout=timeout):
                    self.publish_status('Het thoi gian cho goal; huy lo trinh.')
                    goal_handle.cancel_goal_async()
                    return
                result = result_future.result()
                status = int(result.status)
                with self.lock:
                    stopped = self.stop_requested
                if stopped:
                    self.publish_status('Da huy lo trinh theo lenh Stop.')
                    return
                if status != 4:  # GoalStatus.STATUS_SUCCEEDED
                    self.publish_status(
                        f'Goal {self.zones[name]["label"]} that bai, status={status}.')
                    if bool(self.get_parameter('stop_on_failure').value):
                        return
                else:
                    self.publish_status(
                        f'Da den {self.zones[name]["label"]} ({index}/{len(route)}).')
            self.publish_status('Hoan thanh lo trinh giong noi.')
        finally:
            with self.lock:
                self.busy = False
                self.active_goal_handle = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceNavigationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
