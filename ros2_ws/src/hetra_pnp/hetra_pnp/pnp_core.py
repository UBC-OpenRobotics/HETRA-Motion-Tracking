"""ROS-free marker pose estimation core for HETRA camera tracking.

Pipeline (see README):

    detect markers -> look up ids in the marker map -> world-frame corners
      1 known marker  : IPPE_SQUARE -> 2 candidates -> pick by IMU gravity, else lowest reproj error
      2+ known markers: SQPnP on all corners together (no LM refinement)
      drop the worst marker while its reprojection RMS > threshold, re-solve
    T_world_cam = inv(T_cam_world);  T_world_forearm = T_world_cam @ T_cam_forearm

Conventions
-----------
* ``T_a_b`` is a 4x4 homogeneous transform mapping points in frame b into frame a
  (equivalently: pose of frame b expressed in frame a).
* Marker frame: origin at the centre of the black square, x right, y up, z out of the
  printed face. Corners are ordered TL, TR, BR, BL = (-s/2,+s/2), (+s/2,+s/2),
  (+s/2,-s/2), (-s/2,-s/2) at z = 0. This is the order OpenCV's ArUco detector
  returns and the order SOLVEPNP_IPPE_SQUARE requires.
* Camera frame: OpenCV optical frame (x right, y down, z forward).

Nothing here imports ROS so it can be unit-tested on any machine with OpenCV.
"""
from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence

import cv2
import numpy as np

# --------------------------------------------------------------------------------------
# Transform helpers
# --------------------------------------------------------------------------------------


def make_T(R: np.ndarray, t: Iterable[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.ravel(t)
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    return make_T(R.T, -R.T @ t)


def rvec_tvec_to_T(rvec, tvec) -> np.ndarray:
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return make_T(R, tvec)


def T_to_rvec_tvec(T: np.ndarray):
    rvec, _ = cv2.Rodrigues(np.ascontiguousarray(T[:3, :3]))
    return rvec.ravel(), T[:3, 3].copy()


def rpy_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Fixed-axis roll/pitch/yaw in radians (ROS convention): R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def quat_to_R(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError("zero-length quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R: np.ndarray):
    """Rotation matrix -> (x, y, z, w), w >= 0."""
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return tuple(float(v) for v in q)


def R_to_rpy(R: np.ndarray):
    """Inverse of rpy_to_R: (roll, pitch, yaw) in radians, R = Rz(yaw) Ry(pitch) Rx(roll)."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(R[2, 0]) < 1.0 - 1e-9:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:  # gimbal lock: pitch = +-90 deg, roll and yaw share one axis
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    return roll, pitch, yaw


def rotation_angle(Ra: np.ndarray, Rb: np.ndarray) -> float:
    """Angle (rad) of the relative rotation between two rotation matrices."""
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(math.acos(max(-1.0, min(1.0, c))))


def angle_between(a, b) -> float:
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    c = float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.acos(max(-1.0, min(1.0, c)))


def pose_from_config(d: dict) -> np.ndarray:
    """Parse ``{position: [x,y,z], rpy_deg: [r,p,y]}`` or ``{position, quaternion: [x,y,z,w]}``."""
    t = d.get("position", [0.0, 0.0, 0.0])
    if "quaternion" in d:
        R = quat_to_R(*d["quaternion"])
    else:
        r, p, y = (math.radians(v) for v in d.get("rpy_deg", [0.0, 0.0, 0.0]))
        R = rpy_to_R(r, p, y)
    return make_T(R, t)


# --------------------------------------------------------------------------------------
# Camera intrinsics
# --------------------------------------------------------------------------------------


def fallback_intrinsics(width: int, height: int, hfov_deg: float = 60.0):
    """Pinhole guess for an uncalibrated camera. Good enough to see something, not to measure."""
    f = 0.5 * width / math.tan(math.radians(hfov_deg) / 2.0)
    K = np.array([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1]])
    return K, np.zeros(5)


def intrinsics_are_valid(K) -> bool:
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    return K[0, 0] > 0 and K[1, 1] > 0


def load_camera_calibration(path: str):
    """Read a ROS ``camera_calibration`` YAML. Returns (K, D, width, height)."""
    import yaml

    with open(path, "r") as f:
        c = yaml.safe_load(f)
    K = np.array(c["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    D = np.array(c["distortion_coefficients"]["data"], dtype=np.float64).ravel()
    return K, D, int(c.get("image_width", 0)), int(c.get("image_height", 0))


# --------------------------------------------------------------------------------------
# Marker detection (swappable)
# --------------------------------------------------------------------------------------


@dataclass
class Detection:
    id: int
    corners: np.ndarray  # (4, 2) float64 image points, order TL, TR, BR, BL


class MarkerDetector(ABC):
    """Interface: any fiducial detector that returns corners in TL, TR, BR, BL order."""

    @abstractmethod
    def detect(self, image: np.ndarray) -> List[Detection]:
        ...


class ArucoDetector(MarkerDetector):
    """OpenCV ArUco detector. Works with the legacy (<4.7) and new (>=4.7) APIs.

    Also covers AprilTag families through OpenCV's built-in dictionaries
    (``DICT_APRILTAG_36h11`` etc.), with the same corner order.
    """

    _REFINE = {"none": "CORNER_REFINE_NONE", "subpix": "CORNER_REFINE_SUBPIX",
               "contour": "CORNER_REFINE_CONTOUR", "apriltag": "CORNER_REFINE_APRILTAG"}

    def __init__(self, dictionary: str = "DICT_6X6_250", corner_refinement: str = "subpix"):
        aruco = cv2.aruco
        if not hasattr(aruco, dictionary):
            raise ValueError(f"unknown ArUco dictionary {dictionary!r}")
        self.dictionary_name = dictionary
        self._dict = aruco.getPredefinedDictionary(getattr(aruco, dictionary))
        if hasattr(aruco, "DetectorParameters_create"):  # legacy API
            params = aruco.DetectorParameters_create()
        else:
            params = aruco.DetectorParameters()
        params.cornerRefinementMethod = getattr(aruco, self._REFINE[corner_refinement])
        self._params = params
        if hasattr(aruco, "ArucoDetector"):
            self._detector = aruco.ArucoDetector(self._dict, params)
        else:
            self._detector = None

    def detect(self, image: np.ndarray) -> List[Detection]:
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._dict, parameters=self._params)
        if ids is None:
            return []
        return [Detection(int(i), np.asarray(c, dtype=np.float64).reshape(4, 2))
                for c, i in zip(corners, np.ravel(ids))]


def make_detector(kind: str = "aruco", **kwargs) -> MarkerDetector:
    if kind == "aruco":
        return ArucoDetector(**kwargs)
    raise ValueError(f"unknown detector {kind!r} (available: aruco)")


def generate_marker_image(dictionary: str, marker_id: int, side_px: int) -> np.ndarray:
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(d, marker_id, side_px)
    return cv2.aruco.drawMarker(d, marker_id, side_px)


# --------------------------------------------------------------------------------------
# Marker map
# --------------------------------------------------------------------------------------


def marker_object_points(size: float) -> np.ndarray:
    """Corners of a marker in its own frame, TL, TR, BR, BL (IPPE_SQUARE order)."""
    h = size / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)


@dataclass
class MarkerEntry:
    id: int
    size: float
    T_world_marker: np.ndarray
    group: Optional[str] = None  # board name if the marker belongs to a board

    @property
    def world_corners(self) -> np.ndarray:
        P = marker_object_points(self.size)
        return (self.T_world_marker[:3, :3] @ P.T).T + self.T_world_marker[:3, 3]


@dataclass
class MarkerMap:
    markers: Dict[int, MarkerEntry] = field(default_factory=dict)
    dictionary: str = "DICT_6X6_250"

    def __contains__(self, marker_id: int) -> bool:
        return marker_id in self.markers

    def __getitem__(self, marker_id: int) -> MarkerEntry:
        return self.markers[marker_id]

    def add(self, entry: MarkerEntry) -> None:
        if entry.id in self.markers:
            raise ValueError(f"marker id {entry.id} defined twice in marker map")
        self.markers[entry.id] = entry

    @classmethod
    def from_dict(cls, cfg: dict) -> "MarkerMap":
        """Build from the structure in ``config/marker_map.yaml``.

        Boards are 2x2 grids; ``ids`` are listed row-major as seen from the front:
        [top-left, top-right, bottom-left, bottom-right]. ``gap`` is the white space
        between neighbouring black squares. Board frame uses the marker convention.
        """
        m = cls(dictionary=cfg.get("dictionary", "DICT_6X6_250"))
        default_size = cfg.get("default_marker_size")
        for d in cfg.get("markers", []) or []:
            size = d.get("size", default_size)
            if size is None:
                raise ValueError(f"marker {d['id']} has no size")
            m.add(MarkerEntry(int(d["id"]), float(size), pose_from_config(d)))
        for b in cfg.get("boards", []) or []:
            ids = [int(i) for i in b["ids"]]
            if len(ids) != 4:
                raise ValueError(f"board {b.get('name')} must list exactly 4 ids")
            s = float(b.get("marker_size", default_size))
            c = (s + float(b.get("gap", 0.0))) / 2.0
            T_wb = pose_from_config(b)
            for mid, (cx, cy) in zip(ids, [(-c, c), (c, c), (-c, -c), (c, -c)]):
                m.add(MarkerEntry(mid, s, T_wb @ make_T(np.eye(3), [cx, cy, 0.0]),
                                  group=b.get("name")))
        return m

    @classmethod
    def from_yaml(cls, path: str) -> "MarkerMap":
        import yaml

        with open(path, "r") as f:
            return cls.from_dict(yaml.safe_load(f))


# --------------------------------------------------------------------------------------
# Solvers
# --------------------------------------------------------------------------------------


def reprojection_errors(obj_pts, img_pts, T_cam_world, K, D) -> np.ndarray:
    """Per-point reprojection error (px)."""
    rvec, tvec = T_to_rvec_tvec(T_cam_world)
    proj, _ = cv2.projectPoints(np.asarray(obj_pts, np.float64).reshape(-1, 1, 3), rvec, tvec, K, D)
    return np.linalg.norm(proj.reshape(-1, 2) - np.asarray(img_pts).reshape(-1, 2), axis=1)


def rms(errs: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(errs)))) if len(errs) else float("inf")


@dataclass
class PoseCandidate:
    T_world_cam: np.ndarray
    rms: float


def solve_single(corners_img: np.ndarray, marker: MarkerEntry, K, D) -> List[PoseCandidate]:
    """IPPE_SQUARE on one marker. Returns both candidates, sorted by reprojection RMS.

    IPPE_SQUARE needs object points in the marker's own frame (centred, z = 0, exact
    corner order); the result is then chained with the marker's world pose.
    """
    obj = marker_object_points(marker.size)
    img = np.asarray(corners_img, np.float64).reshape(4, 1, 2)
    n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj.reshape(4, 1, 3), img, K, D,
                                             flags=cv2.SOLVEPNP_IPPE_SQUARE)
    out = []
    for i in range(int(n)):
        T_cam_marker = rvec_tvec_to_T(rvecs[i], tvecs[i])
        T_cam_world = T_cam_marker @ inv_T(marker.T_world_marker)
        e = rms(reprojection_errors(marker.world_corners, corners_img, T_cam_world, K, D))
        out.append(PoseCandidate(inv_T(T_cam_world), e))
    out.sort(key=lambda c: c.rms)
    return out


def solve_multi(obj_world: np.ndarray, img_pts: np.ndarray, K, D) -> PoseCandidate:
    """SQPnP on all correspondences (any geometry, >= 4 points; >= 8 recommended)."""
    obj = np.asarray(obj_world, np.float64).reshape(-1, 1, 3)
    img = np.asarray(img_pts, np.float64).reshape(-1, 1, 2)
    n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img, K, D, flags=cv2.SOLVEPNP_SQPNP)
    if int(n) < 1:
        raise RuntimeError("SQPnP returned no solution")
    T_cam_world = rvec_tvec_to_T(rvecs[0], tvecs[0])
    e = rms(reprojection_errors(obj, img, T_cam_world, K, D))
    return PoseCandidate(inv_T(T_cam_world), e)


def select_candidate(cands: Sequence[PoseCandidate], up_in_camera=None,
                     world_up=(0.0, 0.0, 1.0)) -> PoseCandidate:
    """Resolve the single-marker flip ambiguity.

    With ``up_in_camera`` (world-up direction measured by the IMU, expressed in the
    camera frame) pick the candidate whose predicted up direction agrees best. This
    only uses gravity, so it works with a 6-axis IMU whose yaw drifts. Without it,
    fall back to the lowest reprojection error.
    """
    if up_in_camera is None or len(cands) < 2:
        return cands[0]
    wu = np.asarray(world_up, np.float64)

    def score(c: PoseCandidate) -> float:
        return angle_between(c.T_world_cam[:3, :3].T @ wu, up_in_camera)

    return min(cands, key=score)


@dataclass
class PoseResult:
    T_world_cam: np.ndarray
    rms: float                      # px, over the corners used
    method: str                     # "single" or "multi"
    marker_ids: List[int]
    rejected_ids: List[int]
    n_points: int
    candidates: List[PoseCandidate] = field(default_factory=list)  # single only
    used_imu: bool = False


class PoseEstimator:
    """Camera pose in the world frame from detected markers + a marker map."""

    def __init__(self, marker_map: MarkerMap, K, D, max_reproj_error: float = 3.0,
                 world_up=(0.0, 0.0, 1.0)):
        self.map = marker_map
        self.K = np.asarray(K, np.float64).reshape(3, 3)
        self.D = np.asarray(D, np.float64).ravel()
        self.max_reproj_error = float(max_reproj_error)
        self.world_up = np.asarray(world_up, np.float64)

    def set_intrinsics(self, K, D) -> None:
        self.K = np.asarray(K, np.float64).reshape(3, 3)
        self.D = np.asarray(D, np.float64).ravel()

    def estimate(self, detections: Sequence[Detection], up_in_camera=None) -> Optional[PoseResult]:
        # Keep one detection per known id (duplicates = misdetections or reflections).
        seen: Dict[int, Detection] = {}
        dup = set()
        for d in detections:
            if d.id in self.map:
                if d.id in seen:
                    dup.add(d.id)
                seen[d.id] = d
        known = [d for i, d in seen.items() if i not in dup]
        rejected: List[int] = sorted(dup)

        while known:
            if len(known) == 1:
                d = known[0]
                cands = solve_single(d.corners, self.map[d.id], self.K, self.D)
                if not cands:
                    return None
                best = select_candidate(cands, up_in_camera, self.world_up)
                if best.rms > self.max_reproj_error:
                    return None
                return PoseResult(best.T_world_cam, best.rms, "single", [d.id], rejected, 4,
                                  cands, used_imu=up_in_camera is not None)

            obj = np.vstack([self.map[d.id].world_corners for d in known])
            img = np.vstack([d.corners for d in known])
            cand = solve_multi(obj, img, self.K, self.D)
            errs = reprojection_errors(obj, img, inv_T(cand.T_world_cam), self.K, self.D)
            per_marker = [rms(errs[4 * k:4 * k + 4]) for k in range(len(known))]
            worst = int(np.argmax(per_marker))
            if per_marker[worst] > self.max_reproj_error:
                rejected.append(known[worst].id)
                known = known[:worst] + known[worst + 1:]
                continue
            return PoseResult(cand.T_world_cam, cand.rms, "multi", [d.id for d in known],
                              rejected, len(obj))
        return None


# --------------------------------------------------------------------------------------
# Covariance for the EKF (heuristic; tune against ground truth once available)
# --------------------------------------------------------------------------------------


@dataclass
class CovarianceModel:
    pos_sigma: float = 0.01                    # m, multi-marker at ref_rms with ref_points
    rot_sigma: float = math.radians(1.0)       # rad
    single_factor: float = 4.0                 # single marker is much weaker (planar ambiguity)
    single_no_imu_factor: float = 2.0          # extra if the flip was not checked against the IMU
    ref_rms: float = 0.5                       # px
    ref_points: int = 8

    def covariance(self, r: PoseResult) -> np.ndarray:
        """6x6 row-major (x, y, z, roll, pitch, yaw) diagonal covariance."""
        k = max(1.0, r.rms / self.ref_rms) * math.sqrt(self.ref_points / max(r.n_points, 4))
        if r.method == "single":
            k *= self.single_factor * (1.0 if r.used_imu else self.single_no_imu_factor)
        ps, rs = (self.pos_sigma * k) ** 2, (self.rot_sigma * k) ** 2
        return np.diag([ps, ps, ps, rs, rs, rs])
