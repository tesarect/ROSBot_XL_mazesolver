import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

import math

import tf2_ros
from tf_transformations import euler_from_quaternion


class ScanRangesTF(Node):

    def __init__(self):

        super().__init__('scan_ranges_marker_tf')

        self.base_frame = "base_link"

        self.laser_init = False
        self.laser_yaw_offset = 0.0

        self.angle_min = 0
        self.angle_inc = 0
        self.num_rays = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.marker_pub = self.create_publisher(
            Marker,
            '/scan_range_marker',
            10)

    def init_laser(self, msg):

        try:

            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                rclpy.time.Time())

        except Exception as ex:

            self.get_logger().warn(f"TF not ready: {ex}")
            return False

        q = tf.transform.rotation

        roll, pitch, yaw = euler_from_quaternion([
            q.x, q.y, q.z, q.w
        ])

        self.laser_yaw_offset = yaw

        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment
        self.num_rays = len(msg.ranges)

        self.laser_init = True

        self.get_logger().info(
            f"Laser init done | yaw_offset={yaw:.3f}")

        return True

    def to_index(self, base_deg):

        laser_rad = math.radians(base_deg) - self.laser_yaw_offset

        while laser_rad > math.pi:
            laser_rad -= 2 * math.pi

        while laser_rad < -math.pi:
            laser_rad += 2 * math.pi

        idx = round((laser_rad - self.angle_min) / self.angle_inc)

        idx = max(0, min(self.num_rays - 1, idx))

        return idx

    def scan_callback(self, msg):

        if not self.laser_init:

            if not self.init_laser(msg):
                return

        front = msg.ranges[self.to_index(0)]
        left = msg.ranges[self.to_index(90)]
        right = msg.ranges[self.to_index(-90)]
        back = msg.ranges[self.to_index(180)]

        text = (
            f"Front: {front:.2f} m\n"
            f"Left: {left:.2f} m\n"
            f"Right: {right:.2f} m\n"
            f"Back: {back:.2f} m"
        )

        marker = Marker()

        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "scan_text"
        marker.id = 0

        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = 0.5
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.6

        marker.scale.z = 0.3

        marker.color.a = 1.0
        marker.color.g = 1.0

        marker.text = text

        self.marker_pub.publish(marker)


def main(args=None):

    rclpy.init(args=args)

    node = ScanRangesTF()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()