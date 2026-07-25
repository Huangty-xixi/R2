# -*- coding: utf-8 -*-
"""
Geometry utilities: target center depth sampling, camera back-projection,
hand-eye calibration coordinate transform.

Pinhole camera model back-projection (pixel + depth -> camera frame 3D):
    Z = depth(u, v)
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy

Hand-eye calibration (eye-in-hand): given 4x4 homogeneous matrix T_gripper_camera,
transform camera-frame point p_cam to gripper frame:
    [p_grip; 1] = T_gripper_camera * [p_cam; 1]
"""

from typing import Optional, Tuple

import numpy as np


def sample_center_depth(depth_image: np.ndarray,
                        u: int, v: int,
                        depth_scale: float,
                        window: int = 5,
                        min_m: float = 0.05,
                        max_m: float = 5.0) -> Optional[float]:
    """Read depth (unit: m) at target center (u, v).

    To suppress depth holes/noise, takes the median of valid depths
    within a window x window neighborhood around the center point --
    the median still represents a robust depth estimate of the "center point"
    rather than a box average.

    depth_image : raw depth image (uint16, unit determined by depth_scale; typically mm)
    depth_scale : raw value -> meter scale (D435i default 0.001)
    Returns: depth (m); None if no valid depth.
    """
    h, w = depth_image.shape[:2]
    if not (0 <= u < w and 0 <= v < h):
        return None
    r = max(window // 2, 0)
    u0, u1 = max(u - r, 0), min(u + r + 1, w)
    v0, v1 = max(v - r, 0), min(v + r + 1, h)
    patch = depth_image[v0:v1, u0:u1].astype(np.float32) * depth_scale
    valid = patch[(patch > min_m) & (patch < max_m)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def deproject_pixel(u: float, v: float, z: float,
                    fx: float, fy: float, cx: float, cy: float
                    ) -> Tuple[float, float, float]:
    """Pixel + depth -> 3D point in camera frame (m)."""
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return x, y, z


def transform_point(T: np.ndarray,
                    p: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Transform 3D point p using 4x4 homogeneous matrix T."""
    vec = np.array([p[0], p[1], p[2], 1.0], dtype=np.float64)
    out = T @ vec
    return float(out[0]), float(out[1]), float(out[2])


def mask_centroid(mask: np.ndarray) -> Optional[Tuple[int, int]]:
    """Compute binary mask centroid pixel (u, v). Returns None if mask is empty."""
    ys, xs = np.nonzero(mask)
    if xs.size == 0:
        return None
    return int(round(xs.mean())), int(round(ys.mean()))


def build_tf_matrix(translation, quaternion=None, rotation_matrix=None) -> np.ndarray:
    """Assemble 4x4 homogeneous matrix from translation + (quaternion or 3x3 rotation).

    translation     : [x, y, z]
    quaternion      : [x, y, z, w] (choose one: quaternion or rotation_matrix)
    rotation_matrix : 3x3 list/array
    """
    T = np.eye(4, dtype=np.float64)
    if rotation_matrix is not None:
        R = np.asarray(rotation_matrix, dtype=np.float64).reshape(3, 3)
    elif quaternion is not None:
        R = _quat_to_rot(quaternion)
    else:
        R = np.eye(3, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return T


def _quat_to_rot(q) -> np.ndarray:
    x, y, z, w = q
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n == 0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)
