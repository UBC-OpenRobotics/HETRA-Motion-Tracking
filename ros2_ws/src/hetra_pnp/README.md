# hetra_pnp

HETRA camera tracking: forearm camera → fiducial markers → **SQPnP / IPPE** → camera and
forearm pose in the `world` frame, published for the EKF.

```
image_raw ─► detect markers ─► look up marker map (world corners)
                 1 known marker  → IPPE_SQUARE → 2 candidates → pick by IMU gravity (else lowest reproj error)
                 2+ known markers → SQPnP on all corners together (no LM refinement)
                 drop worst marker while its reprojection RMS > max_reproj_error, re-solve
             ─► T_world_cam = inv(T_cam_world),  T_world_forearm = T_world_cam · T_cam_forearm
             ─► PoseWithCovarianceStamped (stamp = image capture time)
```

Why this design: in simulation, SQPnP on all corners from several markers gave ~10 mm / 0 %
flips vs ~36 mm / 27 % flips for the best single marker; LM after SQPnP changed nothing;
EPnP fails on coplanar points. Every solver runs in < 0.2 ms, so latency is dominated by the
camera and detection, not PnP.

## Files

| Path | What |
|---|---|
| `hetra_pnp/pnp_core.py` | All the maths, **no ROS** (detector, marker map, solvers, covariance). Unit-tested. |
| `hetra_pnp/pnp_node.py` | ROS 2 wiring only |
| `hetra_pnp/webcam_node.py` | Publishes `image_raw` + `camera_info` from a device index or an HTTP stream |
| `hetra_pnp/pose_printer.py` | Live 6-DOF readout in the terminal (demos) |
| `scripts/webcam_server.py` | Runs **on Windows**, streams your webcam to the container |
| `scripts/make_marker.py` | Printable markers and 2×2 boards |
| `config/marker_map.yaml` | Where every marker/board is in the world frame (default: marker 0 only) |
| `config/marker_map_board.yaml` | Multi-marker demo: one 2×2 board with ids 0–3 |
| `config/pnp.yaml` | Node parameters (extrinsics, IMU, thresholds, covariance) |
| `launch/pnp.launch.py` | Camera driver + pnp_node |
| `test/test_pnp_core.py` | Synthetic render → detect → recover pose |

## Topics

| Topic | Type | Notes |
|---|---|---|
| `image_raw`, `camera_info` (in) | Image, CameraInfo | `camera_info` with K = 0 → guessed intrinsics + warning |
| `/imu/data` (in, optional) | Imu | only with `use_imu:=true`; only gravity direction is used |
| `/pnp_node/forearm_pose` | PoseWithCovarianceStamped | **EKF input**, `world` frame |
| `/pnp_node/camera_pose` | PoseWithCovarianceStamped | camera optical frame in `world` |
| `/pnp_node/status` | String | e.g. `multi used=[0, 1, 2, 3] rms=0.4px rejected=[] unknown=[]`, or why there is no pose |
| `/pnp_node/debug_image` | Image (bgr8) | green = used, red = rejected outlier, orange = in map but unused, grey = unknown id |
| TF | `world → camera → forearm` | `camera → forearm` is static |

Topic names / message format are placeholders until the sensor-fusion track publishes the
interface contract.

## Run it (Windows + Docker Desktop)

Docker Desktop can't pass a webcam into the container, so a small server on Windows streams
it and the container reads `http://host.docker.internal:8081/video` (8080 is taken by noVNC).

**1. Windows terminal: start the webcam stream** (needs `pip install opencv-python`)

```bash
python ros2_ws/src/hetra_pnp/scripts/webcam_server.py
```

Check http://localhost:8081/ in a browser. Options: `--device 1`, `--width 1280 --height 800
--fps 60 --mjpg`, `--settings` (driver dialog for exposure). Leave it running.

**2. Print a marker**

```bash
python ros2_ws/src/hetra_pnp/scripts/make_marker.py --id 0
```

Print it, **measure the black square** and set `size` in `config/marker_map.yaml` (default
assumes 5 cm, taped upright on a wall, centre = world origin, world z up).

**3. Container: build and launch**

```bash
docker compose up -d
docker compose exec ros bash
```

Inside the container:

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select hetra_pnp && source install/setup.bash
ros2 launch hetra_pnp pnp.launch.py
```

**4. Look at it** (another `docker compose exec ros bash`)

```bash
ros2 topic echo /pnp_node/forearm_pose
ros2 run rqt_image_view rqt_image_view /pnp_node/debug_image     # view at http://localhost:8080/vnc.html
rviz2                                                            # Fixed Frame: world, add TF
```

**Live 6-DOF readout** (good for presentations; run one command per terminal, not pasted together):

```bash
ros2 run hetra_pnp pose_printer                                    # refreshing dashboard
ros2 run hetra_pnp pose_printer --ros-args -p scroll:=true         # one line per update
```

It shows x, y, z (cm), roll, pitch, yaw (deg), the rate, the 1σ position uncertainty from the
covariance (heuristic for now), and `/pnp_node/status`.

**Resize the rqt window in noVNC:** drag its bottom-right corner, or double-click the title bar
to maximise. The image scales with the window.

The node logs latency every 150 frames: `detect ms`, `solve ms` and `image age ms`
(now − image stamp), as median / p95 / max.

Native Linux later: `ros2 launch hetra_pnp pnp.launch.py driver:=usb_cam device:=/dev/video0`
(needs `ros-humble-usb-cam`), or `driver:=webcam source:=0`. `driver:=none` if images come
from a rosbag or another node.

## Calibrate before trusting numbers

Until a calibration is loaded the node guesses intrinsics (60° HFOV) and the scale of the pose
is wrong. Any calibration that produces the standard ROS YAML works (team plan: ChArUco on a
monitor). Quick checkerboard route inside the container:

```bash
sudo apt update && sudo apt install -y ros-humble-camera-calibration
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 --ros-args -r image:=/image_raw -r camera:=/
```

Save, extract `ost.yaml` from `/tmp/calibrationdata.tar.gz` into `hetra_pnp/config/`, then:

```bash
ros2 launch hetra_pnp pnp.launch.py calibration_file:=/home/ros/ros2_ws/src/hetra_pnp/config/ost.yaml
```

Calibrate at the **same resolution** the stream uses; the webcam node refuses a mismatched file.

## Marker map

Only markers **listed in the map** are used: the solver needs to know where each one is in the
world. A marker with another id shows as `unknown` in `/pnp_node/status` (grey in the debug
image) and is ignored; two copies of the same id show as `duplicate` and are both dropped.

**Multi-marker demo (2×2 board):**

```bash
python ros2_ws/src/hetra_pnp/scripts/make_marker.py --board 0 1 2 3          # Windows, prints on letter
```

Measure a black square and the gap, update `config/marker_map_board.yaml`, then in the container:

```bash
ros2 launch hetra_pnp pnp.launch.py marker_map_file:=$(ros2 pkg prefix hetra_pnp)/share/hetra_pnp/config/marker_map_board.yaml
```

The status switches from `single` to `multi` once two or more board markers are in view.

`config/marker_map.yaml` lists single markers and 2×2 boards with their pose in `world`
(`position` + `rpy_deg` or `quaternion`). Board ids are `[TL, TR, BL, BR]` seen from the front;
`gap` is the white space between black squares. Generate a board with
`make_marker.py --board 10 11 12 13`. Duplicate ids are an error.

For AprilTag, print with `--dict DICT_APRILTAG_36h11` and set `dictionary` in the map. OpenCV's
detector handles it with the same corner order. Another detector library can be added by
implementing `MarkerDetector.detect()` in `pnp_core.py` (corner order TL, TR, BR, BL).

## Extrinsics and IMU

* `cam_to_forearm_xyz` / `cam_to_forearm_rpy_deg`: pose of the forearm frame in the camera
  optical frame (x right, y down, z forward). Identity until the mount is designed.
* `use_imu: true` + `cam_to_imu_rpy_deg`: resolves the single-marker flip by comparing the
  gravity direction from the Madgwick orientation with each IPPE candidate. Only gravity is
  used, so a 6-axis IMU's yaw drift doesn't matter.

## Tests

No ROS needed:

```bash
cd ~/ros2_ws/src/hetra_pnp && python3 -m pytest -q -p no:cacheprovider test
```

They cover a single marker, markers on several planes (SQPnP), 2×2 boards, outlier
rejection, duplicate/unknown ids, IMU flip selection, covariance and world-frame
correctness. They pass on OpenCV 4.5.4 (container, legacy ArUco API) and 5.0 (new API).

## Not done yet

* Marker-map calibration script (estimate marker poses from photos with 2+ markers visible).
* Real-camera accuracy check, and a covariance tuned against ground truth.
* Interface contract with the EKF (topic names, message type).
