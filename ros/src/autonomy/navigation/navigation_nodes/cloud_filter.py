"""Removes floor and robot returns from lidar observations used by Nav2."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class CloudFilter(Node):
    def __init__(self):
        super().__init__('navigation_cloud_filter')
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.half_length = self.declare_parameter('half_length', 0.30).value
        self.half_width = self.declare_parameter('half_width', 0.26).value
        self.min_height = self.declare_parameter('min_height', 0.15).value
        self.max_height = self.declare_parameter('max_height', 2.0).value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PointCloud2, '/navigation/cloud', qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/unilidar/cloud', self.filter_cloud, qos_profile_sensor_data)

    def filter_cloud(self, message):
        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, message.header.frame_id, Time.from_msg(message.header.stamp))
        except TransformException as error:
            self.get_logger().warning(f'Waiting for lidar transform: {error}', throttle_duration_sec=5.0)
            return

        cloud = do_transform_cloud(message, transform)
        points = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)
        outside_robot = (np.abs(points[:, 0]) > self.half_length) | (np.abs(points[:, 1]) > self.half_width)
        within_height = (points[:, 2] >= self.min_height) & (points[:, 2] <= self.max_height)
        self.publisher.publish(point_cloud2.create_cloud_xyz32(cloud.header, points[outside_robot & within_height]))


def main():
    rclpy.init()
    node = CloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
