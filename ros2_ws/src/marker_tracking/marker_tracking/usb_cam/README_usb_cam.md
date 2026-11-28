# ROS 2 USB webcam ArUco detection

This is the webcam alternative to the RealSense implementation. It uses the ROS 2 `usb_cam` driver instead of PyAV/DirectShow, detects ArUco markers, and publishes the **image timestamp, marker ID, and four pixel corners**.

## Files

- `usb_webcam_subscriber.py` — subscribes to `sensor_msgs/msg/Image` and converts each frame with `cv_bridge`.
- `aruco_usb_webcam_publisher.py` — imports the subscriber class, detects ArUco markers, and publishes JSON detections.

Keep both Python files together. Run the **publisher** file only; it creates the subscriber itself.

## Requirements

ROS 2 Humble; `usb_cam`; Python ROS packages `rclpy`, `sensor_msgs`, `std_msgs`, `cv_bridge`; and OpenCV 4.7+ with `cv2.aruco.ArucoDetector` (plus NumPy).

For Ubuntu/ROS 2 Humble, the driver can normally be installed with:

```bash
sudo apt update
sudo apt install ros-humble-usb-cam ros-humble-cv-bridge
```

Verify your webcam is visible to the Linux environment/container (typically `/dev/video0`). If running Docker on Windows, camera access from the container must be configured first; the program cannot resolve USB passthrough issues.

## Run

Terminal 1 — start the webcam driver:

```bash
source /opt/ros/humble/setup.bash
ros2 run usb_cam usb_cam_node_exe
```

The default `usb_cam` topic is commonly `/image_raw`; confirm with:

```bash
ros2 topic list
ros2 topic hz /image_raw
```

Terminal 2 — start the detector, in the folder containing both Python files:

```bash
source /opt/ros/humble/setup.bash
python3 aruco_usb_webcam_publisher.py
```

Terminal 3 — inspect the detections:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /aruco/detections
```

If your driver publishes a different image topic, explicitly set it, for example:

```bash
python3 aruco_usb_webcam_publisher.py --ros-args -p image_topic:=/usb_cam/image_raw
```

Preview is disabled by default, so it works headlessly. To show images (requires desktop/X display):

```bash
python3 aruco_usb_webcam_publisher.py --ros-args -p show_preview:=true
```

Press `q` in the preview to stop.

## Published format

Both camera implementations publish `std_msgs/msg/String` JSON to `/aruco/detections` using the same fields:

```json
{
  "ros_timestamp_ns": 1794600000123456789,
  "ros_timestamp_sec": 1794600000.1234567,
  "camera_timestamp_ms": null,
  "frame_id": "camera",
  "markers": [
    {"id": 17, "corners": [[120.5, 80.2], [170.1, 81.3], [169.8, 131.4], [119.9, 130.7]]}
  ]
}
```

The sample values above are illustrative. A message is published for **every image frame**, even if no marker is found (`"markers": []`). The marker dictionary is `DICT_6X6_250`.

- `ros_timestamp_ns`: **source image header timestamp**, not detection completion time.
- `ros_timestamp_sec`: same timestamp expressed as floating-point seconds.
- `camera_timestamp_ms`: always **null** for the webcam, because there is no separate RealSense metadata message. The header timestamp is supplied by `usb_cam`; its exact clock meaning depends on the driver settings and does not necessarily equal the camera sensor exposure time.
- `markers`: list of IDs and `[x, y]` pixel coordinates ordered top-left, top-right, bottom-right, bottom-left.

The detector also logs mean inter-frame timestamp intervals every 100 intervals. For `solvePnP`, you will still need physical marker size and camera calibration.

## Troubleshooting

- No messages: check `ros2 topic list`, `ros2 topic hz /image_raw`, and the `image_topic` argument.
- No camera in Linux/Docker: check `/dev/video*` and device permissions/passthrough.
- Missing `cv2.aruco.ArucoDetector`: use an OpenCV build with the modern ArUco API.
- `cv_bridge` import error: install/source `ros-humble-cv-bridge` and use the compatible Python environment.
- OpenCV GUI error: leave preview disabled.

The scripts have been Python syntax-checked, but not tested on physical webcam hardware here.
