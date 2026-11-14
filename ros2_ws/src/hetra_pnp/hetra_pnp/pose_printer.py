"""Live 6-DOF readout in the terminal (for demos / presentations).

  ros2 run hetra_pnp pose_printer                       # forearm pose, refreshed 10x/s
  ros2 run hetra_pnp pose_printer --ros-args -p topic:=/pnp_node/camera_pose
  ros2 run hetra_pnp pose_printer --ros-args -p scroll:=true   # one line per update (for logs)

Shows x, y, z (cm) and roll, pitch, yaw (deg) in the world frame, the update rate, the
1-sigma position uncertainty from the covariance, and pnp_node's status line
(single/multi, which marker ids were used or rejected).
"""
import math
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String

from hetra_pnp.pnp_core import R_to_rpy, quat_to_R

CLEAR = "\033[H\033[2J"
BOLD, DIM, GREEN, RED, RESET = "\033[1m", "\033[2m", "\033[32m", "\033[31m", "\033[0m"


class PosePrinter(Node):
    def __init__(self):
        super().__init__("pose_printer")
        self.declare_parameter("topic", "/pnp_node/forearm_pose")
        self.declare_parameter("status_topic", "/pnp_node/status")
        self.declare_parameter("rate", 10.0)          # screen refreshes per second
        self.declare_parameter("stale_after", 0.5)    # s without a pose -> "LOST"
        self.declare_parameter("scroll", False)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.topic, self.stale_after, self.scroll = g("topic"), g("stale_after"), g("scroll")

        self.pose = None
        self.t_pose = None
        self.stamps = deque(maxlen=60)
        self.status = "waiting for pnp_node..."
        self.create_subscription(PoseWithCovarianceStamped, self.topic, self.on_pose, 10)
        self.create_subscription(String, g("status_topic"), self.on_status, 10)
        self.create_timer(1.0 / g("rate"), self.render)

    def on_pose(self, msg):
        self.pose = msg
        self.t_pose = time.monotonic()
        self.stamps.append(self.t_pose)

    def on_status(self, msg):
        self.status = msg.data

    def rate_hz(self):
        now = time.monotonic()
        recent = [t for t in self.stamps if now - t < 2.0]
        return (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 and recent[-1] > recent[0] else 0.0

    def render(self):
        lost = self.t_pose is None or time.monotonic() - self.t_pose > self.stale_after
        if self.pose is None:
            values = None
        else:
            p, q = self.pose.pose.pose.position, self.pose.pose.pose.orientation
            r, pi, y = (math.degrees(a) for a in R_to_rpy(quat_to_R(q.x, q.y, q.z, q.w)))
            sigma_cm = 100.0 * math.sqrt(max(self.pose.pose.covariance[0], 0.0))
            values = (100 * p.x, 100 * p.y, 100 * p.z, r, pi, y, sigma_cm)

        if self.scroll:
            if values and not lost:
                print("x {:+7.1f}  y {:+7.1f}  z {:+7.1f} cm | roll {:+7.1f}  pitch {:+7.1f}  "
                      "yaw {:+7.1f} deg | {}".format(*values[:6], self.status), flush=True)
            return

        state = f"{RED}{BOLD}LOST{RESET}" if lost else f"{GREEN}{BOLD}TRACKING{RESET}"
        lines = [f"{BOLD}HETRA forearm pose{RESET}  {DIM}({self.topic}, frame "
                 f"{self.pose.header.frame_id if self.pose else '?'}){RESET}",
                 f"{state}   {self.rate_hz():5.1f} Hz", ""]
        if values:
            dim = DIM if lost else ""
            x, y, z, r, pi, ya, s = values
            lines += [f"{dim}  position     x {x:+8.1f} cm    y {y:+8.1f} cm    z {z:+8.1f} cm{RESET}",
                      f"{dim}  orientation  roll {r:+7.1f} deg  pitch {pi:+7.1f} deg  yaw {ya:+7.1f} deg{RESET}",
                      f"{dim}  uncertainty  ±{s:.1f} cm (1σ, from covariance){RESET}"]
        else:
            lines += ["  no pose received yet"]
        lines += ["", f"  {DIM}{self.status}{RESET}", "", f"{DIM}Ctrl+C to quit{RESET}"]
        print(CLEAR + "\n".join(lines), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = PosePrinter()
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
