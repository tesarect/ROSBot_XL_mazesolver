import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

import math


class ScanRangeMarker(Node):

    def __init__(self):
        super().__init__('scan_range_marker')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.marker_pub = self.create_publisher(
            Marker,
            '/scan_range_marker',
            10)

        # target direction in degrees (0 = front)
        self.target_angle_deg = 0

    def scan_callback(self, msg):

        target_angle = math.radians(self.target_angle_deg)

        index = int((target_angle - msg.angle_min) / msg.angle_increment)

        if index < 0 or index >= len(msg.ranges):
            return

        distance = msg.ranges[index]

        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "range_text"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = 0.5
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5

        marker.scale.z = 0.2

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.text = f"Front: {distance:.2f}m"

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRangeMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()