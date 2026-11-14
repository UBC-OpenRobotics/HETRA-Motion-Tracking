"""Camera -> marker detection -> SQPnP / IPPE -> forearm pose.

  # Windows + Docker Desktop (webcam_server.py running on the host):
  ros2 launch hetra_pnp pnp.launch.py

  # other stream / device:
  ros2 launch hetra_pnp pnp.launch.py source:=http://host.docker.internal:8081/video
  ros2 launch hetra_pnp pnp.launch.py driver:=usb_cam device:=/dev/video0      # native Linux

  # images come from somewhere else (rosbag, another driver):
  ros2 launch hetra_pnp pnp.launch.py driver:=none

  # extras
  ros2 launch hetra_pnp pnp.launch.py calibration_file:=/home/ros/ros2_ws/src/hetra_pnp/config/camera.yaml
  ros2 launch hetra_pnp pnp.launch.py marker_map_file:=/path/to/marker_map.yaml use_imu:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    get = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    share = get_package_share_directory("hetra_pnp")
    driver, calib = get("driver"), get("calibration_file")

    pnp_params = [os.path.join(share, "config", "pnp.yaml")]
    overrides = {"use_imu": get("use_imu").lower() in ("1", "true", "yes")}
    if get("marker_map_file"):
        overrides["marker_map_file"] = get("marker_map_file")
    pnp_params.append(overrides)

    out = []
    if driver == "webcam":
        out.append(Node(
            package="hetra_pnp", executable="webcam_node", name="webcam_node", output="screen",
            parameters=[{"source": get("source"), "calibration_file": calib, "frame_id": "camera"}]))
    elif driver == "usb_cam":
        p = {"video_device": get("device"), "image_width": 640, "image_height": 480,
             "framerate": 30.0, "pixel_format": "mjpeg2rgb", "frame_id": "camera"}
        if calib:
            p["camera_info_url"] = "file://" + calib
        out.append(Node(package="usb_cam", executable="usb_cam_node_exe", name="usb_cam",
                        output="screen", parameters=[p]))
    elif driver != "none":
        raise RuntimeError(f"driver must be webcam | usb_cam | none, got {driver!r}")

    out.append(Node(package="hetra_pnp", executable="pnp_node", name="pnp_node",
                    output="screen", parameters=pnp_params))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("driver", default_value="webcam", description="webcam | usb_cam | none"),
        DeclareLaunchArgument("source", default_value="http://host.docker.internal:8081/video",
                              description="webcam driver: device index or stream URL"),
        DeclareLaunchArgument("device", default_value="/dev/video0", description="usb_cam device"),
        DeclareLaunchArgument("calibration_file", default_value="",
                              description="ROS camera calibration YAML (absolute path)"),
        DeclareLaunchArgument("marker_map_file", default_value="",
                              description="override config/marker_map.yaml"),
        DeclareLaunchArgument("use_imu", default_value="false",
                              description="use IMU gravity to resolve single-marker flips"),
        OpaqueFunction(function=nodes),
    ])
