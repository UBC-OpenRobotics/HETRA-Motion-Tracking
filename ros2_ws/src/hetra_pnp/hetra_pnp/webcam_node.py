"""Publish frames from an OpenCV source as sensor_msgs/Image + CameraInfo.

`source` is either a device index ("0" -> /dev/video0, Linux only) or a URL such as
http://host.docker.internal:8081/video served by scripts/webcam_server.py on Windows
(Docker Desktop cannot pass a webcam into the container).

The header stamp is taken right after the frame is received. With the HTTP stream this
is later than the true exposure time (by the JPEG encode + network + decode delay).
"""
import array
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from hetra_pnp.pnp_core import load_camera_calibration


class WebcamNode(Node):
    def __init__(self):
        super().__init__("webcam_node")
        self.declare_parameter("source", "http://host.docker.internal:8081/video")
        self.declare_parameter("width", 640)        # only applied to local devices
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("mono", False)       # publish mono8 instead of bgr8

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.source = str(g("source"))
        self.frame_id = g("frame_id")
        self.mono = g("mono")
        self.req = (int(g("width")), int(g("height")), float(g("fps")))

        self.K = self.D = None
        calib = g("calibration_file")
        if calib:
            self.K, self.D, cw, ch = load_camera_calibration(calib)
            self.calib_size = (cw, ch)
            self.get_logger().info(f"loaded calibration {calib}")
        else:
            self.get_logger().warn("no calibration_file: camera_info will have K = 0 (uncalibrated)")

        self.pub_img = self.create_publisher(Image, "image_raw", qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, "camera_info", qos_profile_sensor_data)
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def open(self):
        src = int(self.source) if self.source.isdigit() else self.source
        # URLs: force FFMPEG, otherwise OpenCV tries GStreamer/image-sequence backends too
        cap = cv2.VideoCapture(src) if isinstance(src, int) else cv2.VideoCapture(src, cv2.CAP_FFMPEG)
        if isinstance(src, int):
            w, h, fps = self.req
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # keep latency low; ignored by some backends
        return cap

    def loop(self):
        cap = None
        n, t_last = 0, time.monotonic()
        while self.running and rclpy.ok():
            if cap is None or not cap.isOpened():
                cap = self.open()
                if not cap.isOpened():
                    self.get_logger().warn(f"cannot open {self.source}, retrying in 2 s",
                                           throttle_duration_sec=10.0)
                    time.sleep(2.0)
                    continue
                self.get_logger().info(f"opened {self.source}")
            ok, frame = cap.read()
            stamp = self.get_clock().now().to_msg()
            if not ok:
                self.get_logger().warn("frame grab failed, reconnecting", throttle_duration_sec=5.0)
                cap.release()
                cap = None
                time.sleep(0.5)
                continue
            self.publish(frame, stamp)
            n += 1
            if time.monotonic() - t_last > 10.0:
                self.get_logger().info(f"{n / (time.monotonic() - t_last):.1f} fps, "
                                       f"{frame.shape[1]}x{frame.shape[0]}")
                n, t_last = 0, time.monotonic()
        if cap is not None:
            cap.release()

    def publish(self, frame: np.ndarray, stamp):
        h, w = frame.shape[:2]
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self.frame_id
        img.height, img.width = h, w
        if self.mono:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img.encoding, img.step = "mono8", w
        else:
            img.encoding, img.step = "bgr8", w * 3
        # array('B') hits rclpy's fast path; assigning bytes validates every element (~2x slower)
        img.data = array.array("B", np.ascontiguousarray(frame).tobytes())

        info = CameraInfo()
        info.header = img.header
        info.width, info.height = w, h
        if self.K is not None:
            cw, ch = self.calib_size
            if (cw, ch) not in ((0, 0), (w, h)):
                self.get_logger().error(
                    f"calibration is for {cw}x{ch} but frames are {w}x{h}; not publishing K",
                    throttle_duration_sec=10.0)
            else:
                info.k = self.K.ravel().tolist()
                info.d = self.D.tolist()
                info.distortion_model = "plumb_bob" if len(self.D) <= 5 else "rational_polynomial"
                info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                P = np.zeros((3, 4))
                P[:3, :3] = self.K
                info.p = P.ravel().tolist()
        self.pub_img.publish(img)
        self.pub_info.publish(info)

    def destroy_node(self):
        self.running = False
        self.thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
