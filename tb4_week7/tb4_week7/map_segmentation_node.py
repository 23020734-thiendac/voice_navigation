"""Publish the Small House occupancy segmentation and semantic zone markers."""

from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from geometry_msgs.msg import Point as PointMsg
from visualization_msgs.msg import Marker, MarkerArray
import yaml


Point = Tuple[float, float]


def _read_pgm(path: Path) -> Tuple[int, int, List[int]]:
    """Read binary or ASCII PGM without requiring PIL/OpenCV."""
    raw = path.read_bytes()
    index = 0

    def token() -> bytes:
        nonlocal index
        while index < len(raw) and raw[index:index + 1].isspace():
            index += 1
        while raw[index:index + 1] == b'#':
            while index < len(raw) and raw[index:index + 1] not in (b'\n', b'\r'):
                index += 1
            while index < len(raw) and raw[index:index + 1].isspace():
                index += 1
        start = index
        while index < len(raw) and not raw[index:index + 1].isspace():
            index += 1
        return raw[start:index]

    magic = token()
    if magic not in (b'P2', b'P5'):
        raise ValueError(f'Khong ho tro dinh dang PGM: {magic!r}')
    width, height, maximum = int(token()), int(token()), int(token())
    if maximum > 255:
        raise ValueError('PGM 16-bit khong duoc ho tro')
    if magic == b'P2':
        pixels = [int(token()) for _ in range(width * height)]
    else:
        while index < len(raw) and raw[index:index + 1].isspace():
            index += 1
        pixels = list(raw[index:index + width * height])
    if len(pixels) != width * height:
        raise ValueError('PGM thieu du lieu pixel')
    return width, height, pixels


def _point_in_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    x, y = point
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        crosses = (y1 > y) != (y2 > y)
        if crosses and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
        previous = current
    return inside


class SemanticMapNode(Node):
    """Convert free cells and calibrated ROIs into a semantic map."""

    def __init__(self) -> None:
        super().__init__('semantic_map_node')
        package_share = Path(get_package_share_directory('tb4_week7'))
        self.declare_parameter(
            'map_yaml', str(package_share / 'maps' / 'small_house_slam.yaml'))
        self.declare_parameter(
            'zones_yaml', str(package_share / 'config' / 'zones.yaml'))
        self.declare_parameter('publish_hz', 1.0)

        self.map_yaml_path = Path(self.get_parameter('map_yaml').value).resolve()
        self.zones_yaml_path = Path(self.get_parameter('zones_yaml').value).resolve()
        self.map_data = yaml.safe_load(self.map_yaml_path.read_text())
        image_path = Path(self.map_data['image'])
        if not image_path.is_absolute():
            image_path = self.map_yaml_path.parent / image_path
        self.width, self.height, self.pixels = _read_pgm(image_path.resolve())
        self.resolution = float(self.map_data['resolution'])
        self.origin_x = float(self.map_data['origin'][0])
        self.origin_y = float(self.map_data['origin'][1])

        zone_data = yaml.safe_load(self.zones_yaml_path.read_text())
        self.zones: Dict[str, dict] = zone_data['zones']
        for zone in self.zones.values():
            zone['polygon'] = [tuple(point) for point in zone['polygon']]

        self.segmentation_pub = self.create_publisher(
            OccupancyGrid, '/semantic_map/segmentation', 1)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/semantic_zones/markers', 1)
        self.target_pub = self.create_publisher(
            MarkerArray, '/semantic_zones/targets', 1)
        self.timer = self.create_timer(
            1.0 / max(float(self.get_parameter('publish_hz').value), 0.1),
            self.publish_all)
        self.publish_all()
        self.get_logger().info(
            f'Da nap map {self.width}x{self.height} va gan nhan '
            f'{len(self.zones)} khu vuc Small House.')

    def world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        px = int((x - self.origin_x) / self.resolution)
        py = self.height - 1 - int((y - self.origin_y) / self.resolution)
        return px, py

    def pixel_is_free(self, px: int, py: int) -> bool:
        if px < 0 or py < 0 or px >= self.width or py >= self.height:
            return False
        value = self.pixels[py * self.width + px]
        return value >= 250

    def nearest_free_target(self, target: Sequence[float]) -> Tuple[float, float, float]:
        x, y, yaw = [float(value) for value in target]
        px, py = self.world_to_pixel(x, y)
        if self.pixel_is_free(px, py):
            return x, y, yaw
        for radius in range(1, 31):
            candidates = []
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    if self.pixel_is_free(px + dx, py + dy):
                        candidates.append((dx * dx + dy * dy, dx, dy))
            if candidates:
                _, dx, dy = min(candidates)
                return (
                    self.origin_x + (px + dx + 0.5) * self.resolution,
                    self.origin_y + (self.height - py - dy - 0.5) * self.resolution,
                    yaw,
                )
        self.get_logger().warning(f'Khong tim thay o free gan target {target}')
        return x, y, yaw

    def publish_all(self) -> None:
        now = self.get_clock().now().to_msg()
        grid = OccupancyGrid()
        grid.header.stamp = now
        grid.header.frame_id = 'map'
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.orientation.w = 1.0
        # PGM rows are stored from top to bottom, while a ROS
        # OccupancyGrid stores row 0 at the map origin (bottom to top).
        # Keep the vertical conversion here; otherwise semantic colors are
        # mirrored vertically relative to /map in RViz.
        values = [-1] * (self.width * self.height)
        for pgm_y in range(self.height):
            grid_y = self.height - 1 - pgm_y
            for px in range(self.width):
                pixel = self.pixels[pgm_y * self.width + px]
                grid_index = grid_y * self.width + px
                if pixel < 80:
                    values[grid_index] = 100
                elif pixel >= 250:
                    values[grid_index] = 0
        for zone in self.zones.values():
            for py in range(self.height):
                for px in range(self.width):
                    if not self.pixel_is_free(px, py):
                        continue
                    x = self.origin_x + (px + 0.5) * self.resolution
                    y = self.origin_y + (self.height - py - 0.5) * self.resolution
                    if _point_in_polygon((x, y), zone['polygon']):
                        grid_y = self.height - 1 - py
                        values[grid_y * self.width + px] = int(zone['id']) * 20
        grid.data = values
        self.segmentation_pub.publish(grid)

        markers = MarkerArray()
        targets = MarkerArray()
        for index, (name, zone) in enumerate(self.zones.items()):
            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp = now
            line.ns = 'semantic_zone_boundary'
            line.id = index
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.04
            line.color.r, line.color.g, line.color.b, line.color.a = 0.1, 0.8, 1.0, 0.9
            for x, y in zone['polygon'] + [zone['polygon'][0]]:
                line.points.append(PointMsg(x=float(x), y=float(y), z=0.04))
            markers.markers.append(line)

            tx, ty, _ = self.nearest_free_target(zone['target'])
            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = now
            label.ns = 'semantic_zone_label'
            label.id = index + 100
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = tx
            label.pose.position.y = ty
            label.pose.position.z = 0.45
            label.scale.z = 0.32
            label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 1.0, 0.2, 1.0
            label.text = zone['label']
            markers.markers.append(label)

            target = Marker()
            target.header.frame_id = 'map'
            target.header.stamp = now
            target.ns = 'semantic_zone_target'
            target.id = index + 200
            target.type = Marker.SPHERE
            target.action = Marker.ADD
            target.pose.position.x = tx
            target.pose.position.y = ty
            target.pose.position.z = 0.08
            target.scale.x = target.scale.y = target.scale.z = 0.22
            target.color.r, target.color.g, target.color.b, target.color.a = 1.0, 0.2, 0.1, 0.9
            targets.markers.append(target)
        self.marker_pub.publish(markers)
        self.target_pub.publish(targets)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticMapNode()
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
