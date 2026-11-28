# ROS 2 RealSense ArUco detector

This project receives color images and frame metadata from a RealSense D435 in ROS 2 Humble, detects ArUco markers, and publishes the image timestamp, marker IDs, and four pixel-space corners for every detection.

## Files

- `realsense_frame_subscriber.py` — ROS 2 node class `CameraSubscriber`, subscribing to color/depth images and corresponding camera metadata, matching messages by their header timestamps.
- `aruco_ros_publisher.py` — subclass `ArucoPublisher`, using the subscriber's matched color frames to run OpenCV ArUco detection and publish results.

Both Python files must be in the **same directory**. Normally run **only** `aruco_ros_publisher.py`, which imports the subscriber class.

## Requirements (all added in package.xml)

- ROS 2 Humble with `rclpy`, `sensor_msgs`, `std_msgs`, and `cv_bridge`
- RealSense ROS driver and `realsense2_camera_msgs` package
- Python 3, NumPy, and OpenCV with `cv2.aruco.ArucoDetector` (OpenCV 4.7+)
- A working RealSense D435 connection, with the RealSense ROS driver running and publishing image **and metadata** topics

A Docker GUI/X display is only needed when `show_preview` is enabled. Detection and publishing work headlessly.

## Run

1. In (ROS) terminal 1, launch the RealSense driver:

    ```bash
    cd ~/ros2_ws
    ros2 launch marker_tracking realsense_camera.launch.py
    ```

    Check for the relevant topics in another terminal if needed:

    ```bash
    ros2 topic list
    ```

    By default, the code expects:

    - `/camera/camera/color/image_raw`
    - `/camera/camera/color/metadata`
    - `/camera/camera/depth/image_rect_raw` and `/camera/camera/depth/metadata` (optional; enabled by default)

2. In terminal 2, run the aruco publisher:

    ```bash
    cd ~/ros2_ws
    ros2 run marker_tracking aruco_publisher
    ```

3. In terminal 3, echo to see the detections:

    ```bash
    cd ~/ros2_ws
    ros2 topic echo /aruco/detections
    ```

4. (Optional) To enable the OpenCV preview window (press `q` to exit):

    ```bash
    python3 aruco_ros_publisher.py --ros-args -p show_preview:=true
    ```

    To disable depth subscriptions when only using the color camera:

    ```bash
    python3 aruco_ros_publisher.py --ros-args -p subscribe_depth:=false
    ```

    If your camera is launched under a different prefix:

    ```bash
    python3 aruco_ros_publisher.py --ros-args -p camera_prefix:=/camera/camera
    ```

## Published topic and message format

The detector publishes `std_msgs/msg/String` messages containing JSON on `/aruco/detections`. Each **matched color frame** produces one message, even when there are no markers (`"markers": []`). These values are illustrative:

```json
{
  "ros_timestamp_ns": 1794600000123456789,
  "ros_timestamp_sec": 1794600000.1234567,
  "camera_timestamp_ms": 123456.78,
  "frame_id": "camera_color_optical_frame",
  "markers": [
    {
      "id": 17,
      "corners": [[120.5, 80.2], [170.1, 81.3], [169.8, 131.4], [119.9, 130.7]]
    }
  ]
}
```

- `ros_timestamp_ns` is the source image header timestamp in **nanoseconds**, not the time detection finished.
- `ros_timestamp_sec` is the same header timestamp in seconds.
- `camera_timestamp_ms` is read from RealSense metadata (`frame_timestamp`), in **milliseconds**, and may be `null` if absent.
- `frame_id` is the image header's coordinate-frame identifier.
- `markers` is a list of detected marker objects, each containing an integer ID and four `[x, y]` pixel coordinates in **top-left, top-right, bottom-right, bottom-left** order.

The marker dictionary is `DICT_6X6_250`. The detector also logs the mean interval between image timestamps after every 100 intervals.

## Common issues

- **No detections published:** Verify that the camera driver is running and **both color image and color metadata** topics exist and publish messages. The subscriber waits for matching header timestamps before processing.
- **Wrong topic names:** Check `ros2 topic list` and override `camera_prefix` if appropriate.
- **`ModuleNotFoundError: realsense2_camera_msgs`:** Install/source the RealSense ROS message package in your ROS environment.
- **`cv2.aruco` missing or lacks `ArucoDetector`:** Use a compatible OpenCV build/version.
- **Camera not found in Docker:** USB passthrough/device access must work before the RealSense driver can publish images.
- **OpenCV preview won't open:** Leave `show_preview` off if the container has no graphical display.

## Notes

This publishes 2D marker corners, **not** the 3D marker pose. For `solvePnP`, additionally supply physical marker corner coordinates and matching camera intrinsics/distortion coefficients. The files are verified on a running RealSense D435.
