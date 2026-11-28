#!/usr/bin/env python3
"""Subscribe to standard ROS 2 images published by usb_cam."""

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class WebcamSubscriber(Node):
    def __init__(self):
        super().__init__('usb_webcam_subscriber')
        self.declare_parameter('image_topic', '/image_raw')
        self.bridge = CvBridge()
        self.latest_frame = None
        topic = str(self.get_parameter('image_topic').value)
        self.image_subscription = self.create_subscription(
            Image, topic, self.image_callback, qos_profile_sensor_data
        )
        self.get_logger().info(f'Subscribed to webcam images: {topic}')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except (CvBridgeError, Exception) as exc:
            self.get_logger().error(f'Failed to convert webcam image: {exc}')
            return

        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        record = {
            'image': frame,
            'ros_timestamp_ns': stamp_ns,
            'frame_id': msg.header.frame_id,
            'encoding': 'bgr8',
            'width': msg.width,
            'height': msg.height,
            'camera_timestamp_ms': None,
        }
        self.latest_frame = record
        self.handle_frame(record)

    def handle_frame(self, record):
        """Override this method to process each new image."""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = WebcamSubscriber()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
