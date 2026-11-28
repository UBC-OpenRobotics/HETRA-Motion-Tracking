#!/usr/bin/env python3
"""ROS 2 RealSense image and metadata subscriber."""

import json
from collections import OrderedDict
from functools import partial

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from realsense2_camera_msgs.msg import Metadata
from sensor_msgs.msg import Image


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        self.declare_parameter('camera_prefix', '/camera/camera')
        self.declare_parameter('log_every', 30)
        self.declare_parameter('subscribe_depth', True)

        prefix = str(self.get_parameter('camera_prefix').value).rstrip('/')
        self.log_every = max(1, int(self.get_parameter('log_every').value))
        subscribe_depth = bool(self.get_parameter('subscribe_depth').value)

        self.bridge = CvBridge()
        self.max_pending = 100
        self.pending_images = {'color': OrderedDict(), 'depth': OrderedDict()}
        self.pending_metadata = {'color': OrderedDict(), 'depth': OrderedDict()}
        self.latest_matched = {'color': None, 'depth': None}
        self.frame_counts = {'color': 0, 'depth': 0}
        self.subs = [] # previously self.subscriptions[] -> ros2 node defines this as a read only attribute so we can't reassign

        streams = ['color'] + (['depth'] if subscribe_depth else [])
        for stream in streams:
            image_topic = (
                f'{prefix}/color/image_raw' if stream == 'color'
                else f'{prefix}/depth/image_rect_raw'
            )
            metadata_topic = f'{prefix}/{stream}/metadata'

            self.subs.append(self.create_subscription(
                Image, image_topic, partial(self.image_callback, stream=stream),
                qos_profile_sensor_data
            ))
            self.subs.append(self.create_subscription(
                Metadata, metadata_topic,
                partial(self.metadata_callback, stream=stream),
                qos_profile_sensor_data
            ))
            self.get_logger().info(f'{stream}: {image_topic} + {metadata_topic}')

    @staticmethod
    def stamp_to_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def image_callback(self, msg, stream):
        stamp_ns = self.stamp_to_ns(msg.header.stamp)
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except (CvBridgeError, Exception) as exc:
            self.get_logger().error(f'{stream}: failed to convert image: {exc}')
            return

        record = {
            'image': frame,                  # OpenCV / NumPy array
            'ros_timestamp_ns': stamp_ns,    # timestamp from Image.header
            'frame_id': msg.header.frame_id,
            'encoding': msg.encoding,
            'width': msg.width,
            'height': msg.height,
        }

        metadata = self.pending_metadata[stream].pop(stamp_ns, None)
        if metadata is not None:
            self.handle_matched_frame(stream, record, metadata)
        else:
            self.pending_images[stream][stamp_ns] = record
            self.trim(self.pending_images[stream])

    def metadata_callback(self, msg, stream):
        stamp_ns = self.stamp_to_ns(msg.header.stamp)
        try:
            metadata = json.loads(msg.json_data)
            if not isinstance(metadata, dict):
                metadata = {'raw_metadata': metadata}
        except json.JSONDecodeError:
            self.get_logger().warning(f'{stream}: invalid metadata JSON')
            return

        image = self.pending_images[stream].pop(stamp_ns, None)
        if image is not None:
            self.handle_matched_frame(stream, image, metadata)
        else:
            self.pending_metadata[stream][stamp_ns] = metadata
            self.trim(self.pending_metadata[stream])

    def handle_matched_frame(self, stream, image_record, metadata):
        """Called only when an image and its metadata share a ROS timestamp."""
        record = dict(image_record)
        record['metadata'] = metadata
        record['frame_number'] = metadata.get('frame_number', metadata.get('frame_counter'))
        record['camera_timestamp_ms'] = metadata.get('frame_timestamp')
        record['clock_domain'] = metadata.get('clock_domain')
        self.latest_matched[stream] = record
        self.frame_counts[stream] += 1

        if self.frame_counts[stream] % self.log_every == 0:
            self.get_logger().info(
                f'{stream}: count={self.frame_counts[stream]}, '
                f'frame_number={record["frame_number"]}, '
                f'size={record["width"]}x{record["height"]}, '
                f'encoding={record["encoding"]}, '
                f'ROS timestamp={record["ros_timestamp_ns"]} ns, '
                f'camera timestamp={record["camera_timestamp_ms"]} ms, '
                f'clock_domain={record["clock_domain"]}'
            )

        # Other processing (marker detection, solvePnP, etc.) can go here.
        # Current color frame: self.latest_matched['color']['image']

    def trim(self, cache):
        while len(cache) > self.max_pending:
            cache.popitem(last=False)


def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
