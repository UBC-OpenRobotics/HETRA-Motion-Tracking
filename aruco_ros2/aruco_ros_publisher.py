
#!/usr/bin/env python3
"""ROS 2 ArUco detection publisher."""

import json

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from realsense_frame_subscriber import CameraSubscriber


class ArucoPublisher(CameraSubscriber):
    def __init__(self):
        # Parent subscribes to RealSense images and metadata.
        super().__init__()

        self.declare_parameter('show_preview', False)
        self.declare_parameter('detections_topic', '/aruco/detections')

        self.show_preview = bool(
            self.get_parameter('show_preview').value
        )

        topic = str(
            self.get_parameter('detections_topic').value
        )

        self.publisher = self.create_publisher(
            String, topic, 10
        )

        # ArUco detector (compatible with old and new OpenCV)
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_6X6_250
        )

        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.parameters = cv2.aruco.DetectorParameters()
        else:
            self.parameters = cv2.aruco.DetectorParameters_create()

        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary,
                self.parameters
            )
        else:
            self.detector = None

        self.total_interval_ms = 0.0
        self.interval_count = 0
        self.prev_timestamp_ns = None

        self.get_logger().info(
            f'Publishing ArUco detections on {topic}'
        )

    def handle_matched_frame(self, stream, image_record, metadata):
        # Maintain parent subscriber bookkeeping.
        super().handle_matched_frame(
            stream, image_record, metadata
        )

        if stream != 'color':
            return

        record = self.latest_matched['color']
        frame = record['image']
        encoding = record['encoding'].lower()

        if encoding == 'rgb8':
            gray = cv2.cvtColor(
                frame, cv2.COLOR_RGB2GRAY
            )
            display_frame = (
                cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                if self.show_preview else None
            )

        elif encoding == 'bgr8':
            gray = cv2.cvtColor(
                frame, cv2.COLOR_BGR2GRAY
            )
            display_frame = (
                frame.copy() if self.show_preview else None
            )

        elif encoding == 'mono8':
            gray = frame
            display_frame = (
                cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                if self.show_preview else None
            )

        else:
            self.get_logger().warning(
                f'Unsupported color encoding: {encoding}'
            )
            return

        # Detect markers using the installed OpenCV API.
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(
                gray
            )
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self.dictionary,
                parameters=self.parameters
            )

        markers = []

        if ids is not None:
            for marker_id, marker_corners in zip(
                ids.flatten(), corners
            ):
                markers.append({
                    'id': int(marker_id),
                    'corners': marker_corners.reshape(
                        4, 2
                    ).astype(float).tolist()
                })

        # Timestamp belongs to the captured image.
        stamp_ns = int(record['ros_timestamp_ns'])

        output = {
            'ros_timestamp_ns': stamp_ns,
            'ros_timestamp_sec': stamp_ns / 1e9,
            'camera_timestamp_ms': record.get(
                'camera_timestamp_ms'
            ),
            'frame_id': record['frame_id'],
            'markers': markers
        }

        msg = String()
        msg.data = json.dumps(output)
        self.publisher.publish(msg)

        # Calculate average frame interval.
        if self.prev_timestamp_ns is not None:
            self.total_interval_ms += (
                stamp_ns - self.prev_timestamp_ns
            ) / 1e6

            self.interval_count += 1

        self.prev_timestamp_ns = stamp_ns

        if self.interval_count >= 100:
            self.get_logger().info(
                'Average color frame timestamp interval: '
                f'{self.total_interval_ms / self.interval_count:.3f} ms'
            )

            self.total_interval_ms = 0.0
            self.interval_count = 0

        # Optional camera preview.
        if self.show_preview:
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(
                    display_frame, corners, ids
                )

            cv2.imshow('ArUco Detection', display_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info(
                    'Preview quit requested; stopping ROS.'
                )
                if rclpy.ok():
                    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = ArucoPublisher()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if node is not None:
            node.destroy_node()

        cv2.destroyAllWindows()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
