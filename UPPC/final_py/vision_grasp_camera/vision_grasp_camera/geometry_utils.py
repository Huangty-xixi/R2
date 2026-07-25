# -*- coding: utf-8 -*-
"""
几何工具函数集 (geometry_utils.py)

针孔相机反投影（像素 + 深度 → 相机系 3D 坐标）：
    Z = depth(u, v)
    X = (u - cx) * Z / fx       # u = 水平像素列坐标
    Y = (v - cy) * Z / fy       # v = 垂直像素行坐标

手眼标定变换（eye-in-hand）：
    给 4×4 齐次矩阵 T_gripper_camera → 把相机系的 3D 点 p_cam 转到夹爪系 p_grip：
    [p_grip; 1] = T_gripper_camera * [p_cam; 1]
"""

from typing import Optional, Tuple   # 类型注解

import numpy as np                   # Python 数值计算库 (= C 里手写的矩阵运算)
                                     # np.ndarray = 多维数组，np.mean/median = 均值/中值


def sample_center_depth(depth_image: np.ndarray,       # 原始深度图（uint16，单位由 depth_scale 定义）
                        u: int, v: int,                # 目标中心像素坐标
                        depth_scale: float,            # 深度的物理比例（D435i 默认 0.001 即毫米→米）
                        window: int = 5,               # 邻域窗口半径（像素），越大越抗噪
                        min_m: float = 0.05,           # 有效深度下限（米）：小于此值视为无效
                        max_m: float = 5.0             # 有效深度上限（米）：大于此值视为无效
                        ) -> Optional[float]:           # 返回值：深度（米）或 None
    """
    在目标中心 (u,v) 周围取一个 window×window 的邻域，
    对邻域内所有有效深度值取中值（median），作为鲁棒的中心深度估计。
    中值滤波（不像均值）能有效抑制空洞和椒盐噪声。
    """
    h, w = depth_image.shape[:2]               # shape = (高度, 宽度)，[:2]只取前两个维度
    # 检查 u,v 是否在图像范围内
    if not (0 <= u < w and 0 <= v < h):        # Python 链式比较：等价于 0<=u and u<w and 0<=v and v<h
        return None
    r = max(window // 2, 0)                    # // = 整除（= C 的整数除法）
    # 邻域边界（不超出图像边）
    u0, u1 = max(u - r, 0), min(u + r + 1, w)
    v0, v1 = max(v - r, 0), min(v + r + 1, h)
    # 取邻域 → 物理单位（×depth_scale）→ 过滤无效深度
    patch = depth_image[v0:v1, u0:u1].astype(np.float32) * depth_scale  # astype 类型转换
    valid = patch[(patch > min_m) & (patch < max_m)]                    # NumPy 布尔索引
    if valid.size == 0:                            # 邻域内没有有效深度
        return None
    return float(np.median(valid))                 # np.median = C 里冒泡排序后取中间值


def deproject_pixel(u: float, v: float, z: float,   # 像素坐标 + 深度
                    fx: float, fy: float,            # 相机焦距（像素）
                    cx: float, cy: float             # 光心坐标（像素）
                    ) -> Tuple[float, float, float]: # 返回 (X, Y, Z) 相机系 3D 坐标
    """
    针孔相机反投影：已知像素坐标 (u,v) 和该点的深度 Z，
    通过相机内参计算出该点在相机坐标系下的 3D 坐标 (X, Y, Z)。

    公式（初中相似三角形——物距/像距 = 实际尺寸/像素尺寸）：
    X = (u - cx) * Z / fx    水平方向：像素偏差 × 深度 ÷ 焦距
    Y = (v - cy) * Z / fy    竖直方向：像素偏差 × 深度 ÷ 焦距
    """
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return x, y, z               # Python 可以返回多个值


def transform_point(T: np.ndarray,                    # 4×4 齐次变换矩阵
                    p: Tuple[float, float, float]     # 3D 输入点 (X, Y, Z)
                    ) -> Tuple[float, float, float]:  # 变换后的 (X', Y', Z')
    """
    用 4×4 齐次变换矩阵 T 对 3D 点 p 做坐标变换。
    T @ vec 是 Python 的矩阵乘法（numpy 的 @ 运算符 = 矩阵相乘）。
    """
    vec = np.array([p[0], p[1], p[2], 1.0], dtype=np.float64)  # 齐次坐标：补 1
    out = T @ vec                                              # @ = 矩阵乘法（Python 3.5+）
    return float(out[0]), float(out[1]), float(out[2])         # 转回普通 3D 坐标


def mask_centroid(mask: np.ndarray             # 二值掩膜（True/False 的 numpy 数组）
                  ) -> Optional[Tuple[int, int]]:  # 返回 (u, v) 质心像素坐标
    """
    计算二值掩膜中所有 True 像素的质心。
    np.nonzero = C 里遍历数组找所有不为 0 的索引。
    """
    ys, xs = np.nonzero(mask)                    # 返回两个数组：[y坐标列表], [x坐标列表]
    if xs.size == 0:                             # NumPy 的 .size 属性 = 元素个数
        return None                              # 掩膜是空的
    return int(round(xs.mean())), int(round(ys.mean()))  # 均值 → 四舍五入取整


def build_tf_matrix(translation,                  # 平移 [x, y, z]（列表或数组）
                    quaternion=None,              # 四元数 [x, y, z, w]（和 rotation_matrix 二选一）
                    rotation_matrix=None          # 3×3 旋转矩阵（9 元素）
                    ) -> np.ndarray:              # 返回 4×4 齐次变换矩阵
    """
    组装手眼标定的 4×4 齐次变换矩阵 T_gripper_camera
    格式：
        [ R(3×3)  t(3×1) ]
        [ 0 0 0      1   ]

    np.eye(N) = N×N 单位矩阵（对角线=1，其余=0）
    """
    T = np.eye(4, dtype=np.float64)              # 先建一个 4×4 单位矩阵
    if rotation_matrix is not None:
        R = np.asarray(rotation_matrix, dtype=np.float64).reshape(3, 3)  # 确保是 3×3
    elif quaternion is not None:
        R = _quat_to_rot(quaternion)             # 四元数 → 旋转矩阵
    else:
        R = np.eye(3, dtype=np.float64)          # 都没有 → 单位矩阵（不旋转）
    T[:3, :3] = R                                # Python 切片：T[0:3, 0:3] = R（前 3 行×前 3 列赋为 R）
    T[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)  # 第 4 列（索引从0开始=第3列）赋平移
    return T


def _quat_to_rot(q) -> np.ndarray:
    """
    四元数 [x, y, z, w] → 3×3 旋转矩阵（标准公式，和 ROS tf2 库一致）

    四元数转旋转矩阵的数学公式（见《机器人学导论》）：
    前提：q 必须是单位四元数（x²+y²+z²+w²=1）
    """
    x, y, z, w = q
    # 归一化：防浮点精度导致非单位长度
    n = (x * x + y * y + z * z + w * w) ** 0.5  # **0.5 = 开平方（Python没有 sqrt 函数，用幂运算代替）
    if n == 0: return np.eye(3)                  # 零长度 → 返回单位矩阵
    x, y, z, w = x / n, y / n, z / n, w / n     # 归一化
    return np.array([                            # 3×3 旋转矩阵
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ], dtype=np.float64)
