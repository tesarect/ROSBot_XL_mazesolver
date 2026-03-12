import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import math

import tf2_ros
from tf_transformations import euler_from_quaternion

class ScanRangesTF(Node):

    def __init__(self):

        super().__init__('scan_ranges_marker_tf')

        self.base_frame = "base_link"

        self.laser_init = False
        self.laser_yaw_offset = 0.0

        self.angle_min = 0.0
        self.angle_inc = 0.0
        self.num_rays = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/scan_range_markers',
            10)

        # Directions in base_link frame (degrees)
        self.directions = {
            "F": 0,
            "L": 90,
            "R": -90,
            "B": 180,

            "FL": 67.5,
            "FR": -67.5,
            "BL": 112.5,
            "BR": -112.5,

            "CFL": 35.0,
            "CFR": -35.0,
            "CBL": 145.0,
            "CBR": -145.0
        }

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

        markers = MarkerArray()

        marker_id = 0

        for name, angle in self.directions.items():

            idx = self.to_index(angle)
            dist = msg.ranges[idx]

            if math.isinf(dist) or math.isnan(dist):
                continue

            angle_rad = math.radians(angle)

            x = dist * math.cos(angle_rad)
            y = dist * math.sin(angle_rad)

            # ---------- LINE RAY ----------
            line = Marker()

            line.header.frame_id = self.base_frame
            line.header.stamp = self.get_clock().now().to_msg()

            line.ns = "scan_lines"
            line.id = marker_id
            marker_id += 1

            # line.type = Marker.LINE_STRIP
            # line.action = Marker.ADD

            # line.scale.x = 0.02

            line.type = Marker.ARROW
            line.action = Marker.ADD

            line.scale.x = 0.01   # shaft diameter
            line.scale.y = 0.02   # head diameter
            line.scale.z = 0.010   # head length

            line.color.a = 1.0
            line.color.g = 1.0

            p0 = Point()
            p0.x = 0.0
            p0.y = 0.0
            p0.z = 0.05

            p1 = Point()
            p1.x = x
            p1.y = y
            p1.z = 0.05

            line.points.append(p0)
            line.points.append(p1)

            markers.markers.append(line)

            # ---------- SPHERE HIT ----------
            sphere = Marker()

            sphere.header.frame_id = self.base_frame
            sphere.header.stamp = line.header.stamp

            sphere.ns = "scan_hits"
            sphere.id = marker_id
            marker_id += 1

            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD

            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.05

            sphere.scale.x = 0.05
            sphere.scale.y = 0.05
            sphere.scale.z = 0.05

            # sphere.color.r = 1.0
            if name.startswith("C"):
                line.color.r = 1.0
                line.color.g = 0.5
                line.color.b = 0.0
            else:
                line.color.g = 1.0
            sphere.color.a = 0.6

            markers.markers.append(sphere)

            # ---------- TEXT ----------
            text = Marker()

            text.header.frame_id = self.base_frame
            text.header.stamp = line.header.stamp

            text.ns = "scan_text"
            text.id = marker_id
            marker_id += 1

            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD

            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.15

            text.scale.z = 0.08

            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 1.0

            text.text = f"{name}:{dist:.2f}"

            markers.markers.append(text)

        self.marker_pub.publish(markers)


def main(args=None):

    rclpy.init(args=args)

    node = ScanRangesTF()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()