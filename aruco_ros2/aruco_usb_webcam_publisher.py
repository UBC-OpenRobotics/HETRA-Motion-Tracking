
#!/usr/bin/env python3
"""Detect ArUco markers from usb_cam and publish corners, IDs and timestamps."""

import json

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from usb_webcam_subscriber import WebcamSubscriber


class UsbArucoPublisher(WebcamSubscriber):
    def __init__(self):
        super().__init__()

        self.declare_parameter('detections_topic', '/aruco/detections')
        self.declare_parameter('show_preview', False)

        topic = str(self.get_parameter('detections_topic').value)
        self.show_preview = bool(self.get_parameter('show_preview').value)

        self.detection_publisher = self.create_publisher(
            String, topic, 10
        )

        # Support both old and new OpenCV ArUco APIs.
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

        self.previous_timestamp_ns = None
        self.interval_total_ms = 0.0
        self.interval_count = 0

        self.get_logger().info(f'Publishing detections on {topic}')

    def handle_frame(self, record):
        frame = record['image']

        # Handle image encodings supported by the subscriber.
        encoding = record.get('encoding', 'bgr8').lower()

        if encoding == 'rgb8':
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            display = (
                cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                if self.show_preview else None
            )
        elif encoding == 'bgr8':
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            display = frame.copy() if self.show_preview else None
        elif encoding == 'mono8':
            gray = frame
            display = (
                cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                if self.show_preview else None
            )
        else:
            self.get_logger().warning(
                f'Unsupported image encoding: {encoding}'
            )
            return

        # Detect using the API available in this OpenCV version.
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
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

        # Timestamp from the original ROS image header.
        timestamp_ns = int(record['ros_timestamp_ns'])

        message = {
            'ros_timestamp_ns': timestamp_ns,
            'ros_timestamp_sec': timestamp_ns / 1e9,
            'camera_timestamp_ms': None,
            'frame_id': record['frame_id'],
            'markers': markers
        }

        ros_msg = String()
        ros_msg.data = json.dumps(message)
        self.detection_publisher.publish(ros_msg)

        # Compute average image timestamp interval.
        if self.previous_timestamp_ns is not None:
            interval_ms = (
                timestamp_ns - self.previous_timestamp_ns
            ) / 1e6

            self.interval_total_ms += interval_ms
            self.interval_count += 1

        self.previous_timestamp_ns = timestamp_ns

        if self.interval_count >= 100:
            self.get_logger().info(
                'Average image timestamp interval: '
                f'{self.interval_total_ms / self.interval_count:.3f} ms'
            )

            self.interval_total_ms = 0.0
            self.interval_count = 0

        # Optional annotated preview.
        if self.show_preview:
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(
                    display, corners, ids
                )

            cv2.imshow('USB Webcam ArUco Detection', display)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Preview quit requested.')
                if rclpy.ok():
                    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = UsbArucoPublisher()
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
