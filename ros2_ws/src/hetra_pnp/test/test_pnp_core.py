"""Synthetic tests for hetra_pnp.pnp_core.

Markers are rendered at known world poses with projectPoints + warpPerspective, run
through the real detector, and the recovered camera pose is compared to ground truth.
World frame: z up. Markers face the camera (marker +z points toward it), otherwise the
render is mirrored and does not decode.
"""
import math

import cv2
import numpy as np
import pytest

from hetra_pnp.pnp_core import (
    ArucoDetector, CovarianceModel, Detection, MarkerMap, PoseEstimator, R_to_quat,
    angle_between, generate_marker_image, inv_T, make_T, marker_object_points, quat_to_R,
    rotation_angle, rpy_to_R, select_candidate, solve_single,
)

W, H = 640, 480
K = np.array([[500.0, 0, 320], [0, 500.0, 240], [0, 0, 1]])
D = np.zeros(5)
DICT = "DICT_6X6_250"



def look_at(cam_pos, target, up=(0, 0, 1)):
    """T_world_cam for an OpenCV camera (x right, y down, z forward)."""
    z = np.asarray(target, float) - np.asarray(cam_pos, float)
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return make_T(np.column_stack([x, y, z]), cam_pos)


def render(mmap: MarkerMap, T_world_cam, noise_px=0.0, seed=0):
    T_cam_world = inv_T(T_world_cam)
    rvec, _ = cv2.Rodrigues(T_cam_world[:3, :3])
    tvec = T_cam_world[:3, 3]
    canvas = np.full((H, W), 255, np.uint8)
    visible = []
    for e in mmap.markers.values():
        pc = (T_cam_world[:3, :3] @ e.world_corners.T).T + tvec
        normal_cam = T_cam_world[:3, :3] @ e.T_world_marker[:3, 2]
        if np.any(pc[:, 2] < 0.05) or normal_cam @ pc.mean(0) >= 0:
            continue
        dst, _ = cv2.projectPoints(e.world_corners, rvec, tvec, K, D)
        dst = dst.reshape(4, 2)
        if np.any(dst < 2) or np.any(dst[:, 0] > W - 3) or np.any(dst[:, 1] > H - 3):
            continue
        mk = generate_marker_image(DICT, e.id, 400)
        # outer edges of the 400 px image are at -0.5 / 399.5 in pixel-centre coordinates
        src = np.float32([[-0.5, -0.5], [399.5, -0.5], [399.5, 399.5], [-0.5, 399.5]])
        Hm = cv2.getPerspectiveTransform(src, dst.astype(np.float32))
        warped = cv2.warpPerspective(mk, Hm, (W, H), flags=cv2.INTER_LINEAR, borderValue=255)
        mask = cv2.warpPerspective(np.full_like(mk, 255), Hm, (W, H))
        canvas[mask > 127] = warped[mask > 127]
        visible.append(e.id)
    if noise_px:
        rng = np.random.default_rng(seed)
        canvas = np.clip(canvas + rng.normal(0, noise_px, canvas.shape), 0, 255).astype(np.uint8)
    return canvas, visible


def pose_errors(T_est, T_true):
    return (np.linalg.norm(T_est[:3, 3] - T_true[:3, 3]),
            math.degrees(rotation_angle(T_est[:3, :3], T_true[:3, :3])))


def multi_plane_map():
    """Front wall, a left marker angled 45 deg toward the operator, a floor marker tilted up."""
    return MarkerMap.from_dict({"dictionary": DICT, "default_marker_size": 0.12, "markers": [
        {"id": 0, "position": [0.0, 1.0, 0.15], "rpy_deg": [90, 0, 0]},
        {"id": 1, "position": [0.3, 1.0, -0.05], "rpy_deg": [90, 0, 0]},
        {"id": 2, "position": [-0.4, 0.9, 0.05], "rpy_deg": [90, 0, 45]},
        {"id": 3, "position": [0.05, 0.8, -0.3], "rpy_deg": [45, 0, 0]},
    ]})


MULTI_CAM = ([0.0, 0.0, 0.1], [0.0, 1.0, -0.05])


# ------------------------------------------------------------------------- transforms


def test_quat_roundtrip():
    for rpy in [(0, 0, 0), (math.pi, 0, 0), (0.5, -1.2, 2.0), (0.1, 1.5, -3.0)]:
        R = rpy_to_R(*rpy)
        assert np.allclose(quat_to_R(*R_to_quat(R)), R, atol=1e-9)


def test_rpy_roundtrip():
    from hetra_pnp.pnp_core import R_to_rpy
    for rpy in [(0, 0, 0), (0.3, -0.4, 2.5), (-2.9, 1.2, -0.7), (0.0, math.pi / 2, 0.4)]:
        R = rpy_to_R(*rpy)
        assert np.allclose(rpy_to_R(*R_to_rpy(R)), R, atol=1e-9)


def test_inverse():
    T = make_T(rpy_to_R(0.3, -0.2, 1.0), [0.1, 0.2, 0.5])
    assert np.allclose(T @ inv_T(T), np.eye(4), atol=1e-12)


# ------------------------------------------------------------------------- marker map


def test_board_geometry():
    m = MarkerMap.from_dict({
        "dictionary": DICT,
        "boards": [{"name": "b", "ids": [4, 5, 6, 7], "marker_size": 0.08, "gap": 0.02,
                    "position": [1, 2, 3], "rpy_deg": [90, 0, 0]}],
    })
    tl, tr, bl, br = (m[i].T_world_marker[:3, 3] for i in (4, 5, 6, 7))
    assert np.isclose(np.linalg.norm(tr - tl), 0.10)
    assert np.isclose(np.linalg.norm(bl - tl), 0.10)
    assert np.allclose((tl + br) / 2, [1, 2, 3])
    # rpy (90,0,0): board y axis -> world +z, so the top row is above the bottom row
    assert tl[2] > bl[2]
    # all 16 corners coplanar
    pts = np.vstack([m[i].world_corners for i in (4, 5, 6, 7)])
    assert np.ptp(pts @ m[4].T_world_marker[:3, 2]) < 1e-12


def test_duplicate_id_rejected():
    with pytest.raises(ValueError):
        MarkerMap.from_dict({"markers": [{"id": 1, "size": 0.1}, {"id": 1, "size": 0.1}]})


def test_marker_map_yaml_loads():
    import os
    for name, ids in (("marker_map.yaml", [0]), ("marker_map_board.yaml", [0, 1, 2, 3])):
        m = MarkerMap.from_yaml(os.path.join(os.path.dirname(__file__), "..", "config", name))
        assert sorted(m.markers) == ids


# ------------------------------------------------------------------------- end to end


def test_single_marker_world_pose():
    m = MarkerMap.from_dict({"dictionary": DICT, "markers": [
        {"id": 7, "size": 0.10, "position": [0.2, 1.0, 0.3], "rpy_deg": [90, 0, 0]}]})
    T_true = look_at([0.0, 0.7, 0.5], [0.2, 1.0, 0.3])  # ~43 deg oblique, 0.41 m
    img, vis = render(m, T_true)
    assert vis == [7]
    dets = ArucoDetector(DICT).detect(img)
    r = PoseEstimator(m, K, D).estimate(dets)
    assert r is not None and r.method == "single" and len(r.candidates) == 2
    dp, dr = pose_errors(r.T_world_cam, T_true)
    assert dp < 5e-3 and dr < 1.0, (dp, dr)


def test_multi_plane_sqpnp():
    m = multi_plane_map()
    T_true = look_at(*MULTI_CAM)
    img, vis = render(m, T_true, noise_px=2.0)
    assert len(vis) >= 3, vis
    dets = ArucoDetector(DICT).detect(img)
    r = PoseEstimator(m, K, D).estimate(dets)
    assert r.method == "multi" and sorted(r.marker_ids) == sorted(vis)
    assert r.n_points == 4 * len(vis)
    dp, dr = pose_errors(r.T_world_cam, T_true)
    # ~1 m away with image noise; research sim gave ~10 mm median for this case
    assert dp < 15e-3 and dr < 1.0, (dp, dr)


def test_board_2x2():
    m = MarkerMap.from_dict({"dictionary": DICT, "boards": [
        {"name": "front", "ids": [10, 11, 12, 13], "marker_size": 0.06, "gap": 0.015,
         "position": [0, 0.8, 0], "rpy_deg": [90, 0, 0]}]})
    T_true = look_at([0.15, 0.2, 0.1], [0, 0.8, 0])
    img, vis = render(m, T_true)
    assert sorted(vis) == [10, 11, 12, 13]
    r = PoseEstimator(m, K, D).estimate(ArucoDetector(DICT).detect(img))
    assert r.method == "multi" and r.n_points == 16
    dp, dr = pose_errors(r.T_world_cam, T_true)
    assert dp < 5e-3 and dr < 0.5, (dp, dr)


# ------------------------------------------------------------------------- robustness


def _project(m, T_world_cam, ids):
    T_cw = inv_T(T_world_cam)
    rvec, _ = cv2.Rodrigues(T_cw[:3, :3])
    out = []
    for i in ids:
        p, _ = cv2.projectPoints(m[i].world_corners, rvec, T_cw[:3, 3], K, D)
        out.append(Detection(i, p.reshape(4, 2)))
    return out


def test_outlier_marker_rejected():
    m = multi_plane_map()
    T_true = look_at(*MULTI_CAM)
    dets = _project(m, T_true, [0, 1, 2, 3])
    dets[1].corners = dets[1].corners + np.array([25.0, -15.0])  # e.g. marker was bumped
    r = PoseEstimator(m, K, D, max_reproj_error=3.0).estimate(dets)
    assert r.rejected_ids == [1] and sorted(r.marker_ids) == [0, 2, 3]
    dp, dr = pose_errors(r.T_world_cam, T_true)
    assert dp < 1e-4 and dr < 0.01


def test_unknown_and_duplicate_ids_ignored():
    m = multi_plane_map()
    T_true = look_at(*MULTI_CAM)
    dets = _project(m, T_true, [0, 2, 3])
    dets.append(Detection(42, dets[0].corners + 50))  # not in map
    dets.append(Detection(3, dets[2].corners + 40))   # same id seen twice
    r = PoseEstimator(m, K, D).estimate(dets)
    assert sorted(r.marker_ids) == [0, 2] and r.rejected_ids == [3]


def test_imu_resolves_flip():
    """Far, near-frontal single marker: both IPPE candidates fit the pixels well.
    The IMU's gravity direction picks the right one regardless of which has lower error."""
    m = MarkerMap.from_dict({"dictionary": DICT, "markers": [
        {"id": 0, "size": 0.08, "position": [0, 1.0, 0], "rpy_deg": [90, 0, 0]}]})
    T_true = look_at([0.03, 0.0, 0.05], [0, 1.0, 0])
    det = _project(m, T_true, [0])[0]
    cands = solve_single(det.corners, m[0], K, D)
    assert len(cands) == 2
    # the two candidates genuinely differ
    assert math.degrees(rotation_angle(cands[0].T_world_cam[:3, :3],
                                       cands[1].T_world_cam[:3, :3])) > 2.0
    up_cam_true = T_true[:3, :3].T @ np.array([0, 0, 1.0])
    for c in cands:  # whichever candidate the IMU agrees with is chosen
        up_c = c.T_world_cam[:3, :3].T @ np.array([0, 0, 1.0])
        assert select_candidate(cands, up_c) is c
    best = select_candidate(cands, up_cam_true)
    assert math.degrees(rotation_angle(best.T_world_cam[:3, :3], T_true[:3, :3])) < 0.5
    assert math.degrees(angle_between(best.T_world_cam[:3, :3].T @ [0, 0, 1], up_cam_true)) < 0.5


def test_covariance_single_is_larger_than_multi():
    m = multi_plane_map()
    T_true = look_at(*MULTI_CAM)
    est = PoseEstimator(m, K, D)
    multi = est.estimate(_project(m, T_true, [0, 1, 2, 3]))
    single = est.estimate(_project(m, T_true, [0]))
    cm = CovarianceModel()
    assert cm.covariance(single)[0, 0] > cm.covariance(multi)[0, 0]


def test_marker_object_points_order():
    P = marker_object_points(0.1)
    assert np.allclose(P[0], [-0.05, 0.05, 0])   # TL
    assert np.allclose(P[2], [0.05, -0.05, 0])   # BR
