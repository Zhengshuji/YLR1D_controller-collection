"""由标准 DH 表重建正运动学（可用于未来手动实现 IK/雅可比时复用）。"""
from __future__ import annotations

import math
from typing import Dict, List, Tuple

import numpy as np

from .dh import DHChain


def dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """标准 DH 关节变换 A = Rz(theta)·Tz(d)·Tx(a)·Rx(alpha)。"""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    T = np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,      ca,     d],
        [0.0,     0.0,     0.0,   1.0],
    ])
    return T


def dh_forward_kinematics(chain: DHChain,
                          q: Dict[str, float] | List[float]) -> np.ndarray:
    """DH 正运动学：T = T_base_to_0 · A_1(q) · ... · A_n(q) · T_n_to_end。

    q 可以是 {关节名: 值} 字典，也可以是按 chain.joints 顺序的长度 n 列表。
    """
    if isinstance(q, dict):
        qv = [q.get(j, 0.0) for j in chain.joints]
    else:
        qv = list(q)
    T = chain.base_transform.copy()
    for row, qi in zip(chain.rows, qv):
        theta, d = row.variable_parameter(qi)
        T = T @ dh_transform(theta, d, row.a, row.alpha)
    return T @ chain.tool_transform


def urdf_forward_kinematics(robot, base_link: str,
                            end_link: str,
                            q: Dict[str, float]) -> np.ndarray:
    """用 URDF 自身语义计算的 T_{base->end}（验证基准）。"""
    from .urdf_model import compute_transforms
    T = compute_transforms(robot, base_link, q)
    return T[end_link]


def pose_error(T1: np.ndarray, T2: np.ndarray) -> Tuple[float, float]:
    """返回 (平移误差, 旋转角度误差)。"""
    pos_err = float(np.linalg.norm(T1[:3, 3] - T2[:3, 3]))
    R = T1[:3, :3] @ T2[:3, :3].T
    # 旋转角度 = arccos((trace(R)-1)/2)
    cosang = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    ang_err = float(math.acos(cosang))
    return pos_err, ang_err
