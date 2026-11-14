"""ROS 2 node: image -> markers -> camera / forearm pose in the world frame.

Subscribes
  image_raw     sensor_msgs/Image
  camera_info   sensor_msgs/CameraInfo  (K all zeros = uncalibrated -> guessed intrinsics)
  <imu_topic>   sensor_msgs/Imu         (optional, use_imu:=true; only gravity is used)

Publishes
  ~/forearm_pose  geometry_msgs/PoseWithCovarianceStamped   (world frame) -> EKF input
  ~/camera_pose   geometry_msgs/PoseWithCovarianceStamped   (world frame)
  ~/debug_image   sensor_msgs/Image (bgr8)
  TF world -> camera_frame (dynamic), camera_frame -> forearm_frame (static)

All stamps are the image capture stamp, not processing time.
All the maths lives in pnp_core; this file only wires ROS to it.
"""
import array
import math
import os
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from hetra_pnp import pnp_core as pc

# --------------------------------------------------------------------------------------
# Image conversion (no cv_bridge, avoids numpy/cv_bridge ABI issues)
# --------------------------------------------------------------------------------------


def image_to_gray(msg: Image) -> np.ndarray:
    enc = msg.encoding.lower()
    h, w, step = msg.height, msg.width, msg.step
    buf = np.frombuffer(msg.data, np.uint8).reshape(h, step)
    if enc in ("mono8", "8uc1"):
        return buf[:, :w]
    if enc in ("rgb8", "bgr8", "8uc3"):
        img = buf[:, :w * 3].reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if enc == "rgb8" else cv2.COLOR_BGR2GRAY)
    if enc in ("rgba8", "bgra8"):
        img = buf[:, :w * 4].reshape(h, w, 4)
        return cv2.cvtColor(img, cv2.COLOR_RGBA2GRAY if enc == "rgba8" else cv2.COLOR_BGRA2GRAY)
    if enc in ("yuv422_yuy2", "yuyv"):
        return cv2.cvtColor(buf[:, :w * 2].reshape(h, w, 2), cv2.COLOR_YUV2GRAY_YUY2)
    if enc in ("yuv422", "uyvy"):
        return cv2.cvtColor(buf[:, :w * 2].reshape(h, w, 2), cv2.COLOR_YUV2GRAY_UYVY)
    raise ValueError(f"unsupported image encoding {msg.encoding!r}")


def bgr_to_image(img: np.ndarray, header) -> Image:
    m = Image()
    m.header = header
    m.height, m.width = img.shape[:2]
    m.encoding = "bgr8"
    m.step = m.width * 3
    m.data = array.array("B", np.ascontiguousarray(img).tobytes())  # fast path in rclpy
    return m


def fill_pose(msg_pose, T: np.ndarray) -> None:
    msg_pose.position.x, msg_pose.position.y, msg_pose.position.z = (float(v) for v in T[:3, 3])
    q = pc.R_to_quat(T[:3, :3])
    msg_pose.orientation.x, msg_pose.orientation.y = q[0], q[1]
    msg_pose.orientation.z, msg_pose.orientation.w = q[2], q[3]


def make_tf(T: np.ndarray, parent: str, child: str, stamp) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (
        float(v) for v in T[:3, 3])
    q = pc.R_to_quat(T[:3, :3])
    (t.transform.rotation.x, t.transform.rotation.y,
     t.transform.rotation.z, t.transform.rotation.w) = q
    return t


def T_from_xyz_rpy_deg(xyz, rpy_deg) -> np.ndarray:
    return pc.make_T(pc.rpy_to_R(*(math.radians(v) for v in rpy_deg)), xyz)


class PnPNode(Node):
    def __init__(self):
        super().__init__("pnp_node")
        share = get_package_share_directory("hetra_pnp")
        p = self.declare_parameter
        p("marker_map_file", os.path.join(share, "config", "marker_map.yaml"))
        p("dictionary", "")                     # "" = take it from the marker map
        p("detector", "aruco")
        p("corner_refinement", "subpix")        # none | subpix | contour | apriltag
        p("max_reproj_error", 3.0)              # px, per marker
        p("world_frame", "world")
        p("camera_frame", "")                   # "" = image header frame_id
        p("forearm_frame", "forearm")
        p("cam_to_forearm_xyz", [0.0, 0.0, 0.0])      # pose of forearm frame in camera frame
        p("cam_to_forearm_rpy_deg", [0.0, 0.0, 0.0])
        p("use_imu", False)
        p("imu_topic", "/imu/data")
        p("imu_max_age", 0.2)                   # s, ignore older IMU orientation
        p("cam_to_imu_rpy_deg", [0.0, 0.0, 0.0])      # orientation of IMU frame in camera frame
        p("world_up", [0.0, 0.0, 1.0])
        p("fallback_hfov_deg", 60.0)
        p("publish_tf", True)
        p("publish_debug_image", True)
        p("stats_every_n_frames", 150)
        p("cov_pos_sigma", 0.01)
        p("cov_rot_sigma_deg", 1.0)
        p("cov_single_factor", 4.0)
        p("cov_single_no_imu_factor", 2.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.map = pc.MarkerMap.from_yaml(g("marker_map_file"))
        dictionary = g("dictionary") or self.map.dictionary
        self.detector = pc.make_detector(g("detector"), dictionary=dictionary,
                                         corner_refinement=g("corner_refinement"))
        self.estimator = pc.PoseEstimator(self.map, np.eye(3), np.zeros(5),
                                          g("max_reproj_error"), g("world_up"))
        self.cov = pc.CovarianceModel(
            pos_sigma=g("cov_pos_sigma"), rot_sigma=math.radians(g("cov_rot_sigma_deg")),
            single_factor=g("cov_single_factor"),
            single_no_imu_factor=g("cov_single_no_imu_factor"))
        self.world_frame = g("world_frame")
        self.camera_frame = g("camera_frame")
        self.forearm_frame = g("forearm_frame")
        self.T_cam_forearm = T_from_xyz_rpy_deg(g("cam_to_forearm_xyz"),
                                                g("cam_to_forearm_rpy_deg"))
        self.R_cam_imu = T_from_xyz_rpy_deg([0, 0, 0], g("cam_to_imu_rpy_deg"))[:3, :3]
        self.imu_max_age = g("imu_max_age")
        self.hfov = g("fallback_hfov_deg")
        self.publish_tf = g("publish_tf")
        self.publish_debug = g("publish_debug_image")
        self.stats_n = int(g("stats_every_n_frames"))

        self.have_calib = False
        self.calib_K = None
        self.calib_D = None
        self.warned_uncalibrated = False
        self.last_up_imu = None          # world-up in IMU frame
        self.last_imu_time = None

        self.pub_forearm = self.create_publisher(PoseWithCovarianceStamped, "~/forearm_pose", 10)
        self.pub_camera = self.create_publisher(PoseWithCovarianceStamped, "~/camera_pose", 10)
        self.pub_debug = self.create_publisher(Image, "~/debug_image", 1)
        self.pub_status = self.create_publisher(String, "~/status", 10)
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_sent = False

        self.create_subscription(CameraInfo, "camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "image_raw", self.on_image, qos_profile_sensor_data)
        if g("use_imu"):
            self.create_subscription(Imu, g("imu_topic"), self.on_imu, qos_profile_sensor_data)

        self.t_detect = deque(maxlen=self.stats_n)
        self.t_solve = deque(maxlen=self.stats_n)
        self.t_age = deque(maxlen=self.stats_n)
        self.frames = 0
        self.frames_with_pose = 0
        self.method_counts = {"single": 0, "multi": 0}

        self.get_logger().info(
            f"marker map: {len(self.map.markers)} markers, dictionary {dictionary}, "
            f"imu flip check {'ON' if g('use_imu') else 'OFF'}")

    # ---------------------------------------------------------------- callbacks
    def on_info(self, msg: CameraInfo):
        K = np.array(msg.k, np.float64).reshape(3, 3)
        if pc.intrinsics_are_valid(K):
            self.calib_K = K
            self.calib_D = np.array(msg.d, np.float64) if len(msg.d) else np.zeros(5)
            if not self.have_calib:
                self.get_logger().info(f"using camera_info intrinsics fx={K[0, 0]:.1f}")
            self.have_calib = True

    def on_imu(self, msg: Imu):
        q = msg.orientation
        if q.x == 0 and q.y == 0 and q.z == 0 and q.w == 0:
            return
        R_ref_imu = pc.quat_to_R(q.x, q.y, q.z, q.w)
        self.last_up_imu = R_ref_imu.T @ np.array([0.0, 0.0, 1.0])  # Madgwick ref frame is z-up
        self.last_imu_time = self.get_clock().now()

    def on_image(self, msg: Image):
        t0 = time.perf_counter()
        try:
            gray = image_to_gray(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return

        if self.have_calib:
            self.estimator.set_intrinsics(self.calib_K, self.calib_D)
        else:
            self.estimator.set_intrinsics(*pc.fallback_intrinsics(msg.width, msg.height, self.hfov))
            if not self.warned_uncalibrated:
                self.get_logger().warn(
                    f"no valid camera_info yet: guessing intrinsics ({self.hfov} deg HFOV). "
                    "Poses will be off in scale until the camera is calibrated.")
                self.warned_uncalibrated = True

        dets = self.detector.detect(gray)
        t1 = time.perf_counter()
        result = self.estimator.estimate(dets, self.up_in_camera())
        t2 = time.perf_counter()

        self.frames += 1
        self.t_detect.append((t1 - t0) * 1e3)
        self.t_solve.append((t2 - t1) * 1e3)
        self.t_age.append((self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-6)

        cam_frame = self.camera_frame or msg.header.frame_id or "camera"
        if result is not None:
            self.frames_with_pose += 1
            self.method_counts[result.method] += 1
            self.publish_pose(result, msg.header.stamp, cam_frame)
        self.pub_status.publish(String(data=self.status_text(dets, result)))
        if self.publish_debug and self.pub_debug.get_subscription_count() > 0:
            self.pub_debug.publish(bgr_to_image(self.draw(gray, dets, result), msg.header))
        if self.stats_n > 0 and self.frames % self.stats_n == 0:
            self.log_stats()

    # ---------------------------------------------------------------- helpers
    def up_in_camera(self):
        if self.last_up_imu is None:
            return None
        age = (self.get_clock().now() - self.last_imu_time).nanoseconds * 1e-9
        if age > self.imu_max_age:
            return None
        return self.R_cam_imu @ self.last_up_imu

    def publish_pose(self, r: pc.PoseResult, stamp, cam_frame: str):
        cov = self.cov.covariance(r).ravel().tolist()
        T_world_forearm = r.T_world_cam @ self.T_cam_forearm
        for pub, T in ((self.pub_camera, r.T_world_cam), (self.pub_forearm, T_world_forearm)):
            m = PoseWithCovarianceStamped()
            m.header.stamp = stamp
            m.header.frame_id = self.world_frame
            fill_pose(m.pose.pose, T)
            m.pose.covariance = cov
            pub.publish(m)
        if self.publish_tf:
            self.tf_pub.sendTransform(make_tf(r.T_world_cam, self.world_frame, cam_frame, stamp))
            if not self.static_sent:
                self.static_tf.sendTransform(
                    make_tf(self.T_cam_forearm, cam_frame, self.forearm_frame, stamp))
                self.static_sent = True

    def status_text(self, dets, r) -> str:
        """One line for ~/status, e.g. 'multi used=[0, 1] rms=0.42px rejected=[] unknown=[7]'."""
        seen = [d.id for d in dets]
        unknown = sorted({i for i in seen if i not in self.map})
        dup = sorted({i for i in seen if seen.count(i) > 1 and i in self.map})
        if r is None:
            known = sorted({i for i in seen if i in self.map})
            why = "no markers seen" if not seen else (
                "only duplicate ids" if known and set(known) <= set(dup) else
                "no marker from the map" if not known else "reprojection error too high")
            return f"none ({why}) seen={sorted(seen)} unknown={unknown} duplicate={dup}"
        return (f"{r.method} used={sorted(r.marker_ids)} rms={r.rms:.2f}px "
                f"rejected={sorted(r.rejected_ids)} unknown={unknown}")

    def draw(self, gray, dets, r):
        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        used = set(r.marker_ids) if r else set()
        rejected = set(r.rejected_ids) if r else set()
        for d in dets:
            if d.id in used:
                col = (0, 200, 0)
            elif d.id in rejected:
                col = (0, 0, 255)
            elif d.id in self.map:
                col = (0, 165, 255)
            else:
                col = (160, 160, 160)
            pts = d.corners.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(img, [pts], True, col, 2)
            cv2.circle(img, tuple(pts[0, 0]), 4, (255, 0, 0), -1)  # TL corner
            cv2.putText(img, str(d.id), tuple(pts[0, 0] + np.array([5, -5])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        if r is not None:
            K, D = self.estimator.K, self.estimator.D
            T_cw = pc.inv_T(r.T_world_cam)
            for mid in r.marker_ids:
                rv, tv = pc.T_to_rvec_tvec(T_cw @ self.map[mid].T_world_marker)
                cv2.drawFrameAxes(img, K, D, rv, tv, self.map[mid].size * 0.6)
            p = r.T_world_cam[:3, 3]
            txt = (f"{r.method} n={len(r.marker_ids)} rms={r.rms:.2f}px "
                   f"cam=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})m")
            cv2.putText(img, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        else:
            cv2.putText(img, "no pose", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        return img

    def log_stats(self):
        def s(v):
            a = np.asarray(v)
            return f"med {np.median(a):.2f} / p95 {np.percentile(a, 95):.2f} / max {a.max():.2f}"
        self.get_logger().info(
            f"[{self.frames} frames, pose in {100.0 * self.frames_with_pose / self.frames:.0f}%, "
            f"single {self.method_counts['single']} multi {self.method_counts['multi']}] "
            f"detect ms {s(self.t_detect)} | solve ms {s(self.t_solve)} | "
            f"image age ms {s(self.t_age)}")


def main(args=None):
    rclpy.init(args=args)
    node = PnPNode()
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
