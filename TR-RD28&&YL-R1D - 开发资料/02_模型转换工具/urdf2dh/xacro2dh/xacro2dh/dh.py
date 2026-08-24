"""从 URDF 链提取标准 DH（Craig / 1955 约定）参数。

约定（标准 DH，变换 T_i = Rz(theta_i)·Tz(d_i)·Tx(a_i)·Rx(alpha_i)）：
  - z_{i-1}：关节 i 的旋转/平移轴（在基座系中的直线）；
  - x_i：z_{i-1} 到 z_i 的公垂线方向；
  - a_i：沿 x_i 从 z_{i-1} 到 z_i 的距离；
  - alpha_i：绕 x_i 从 z_{i-1} 到 z_i 的角度；
  - d_i：沿 z_{i-1} 从 O_{i-1} 到公垂线足点的距离；
  - theta_i：绕 z_{i-1} 从 x_{i-1} 到 x_i 的角度（theta_offset 为 q=0 时的值）。

输出除 DH 表外，还包含基座变换 T_{base->{0}} 与末端变换 T_{{n}->end_link}，
使 DH 正运动学可与 URDF 正运动学逐点严格一致。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from .urdf_model import Robot, compute_transforms, find_chain

_TOL = 1e-9


@dataclass
class DHRow:
    joint: str
    type: str                 # revolute / continuous / prismatic / fixed
    a: float
    alpha: float
    d: float
    theta_offset: float       # q=0 时的 theta
    lower: float
    upper: float
    effort: float
    velocity: float

    def variable_parameter(self, q: float) -> Tuple[float, float]:
        """返回 (theta, d)，可变参数随 q 变化。"""
        if self.type in ("revolute", "continuous"):
            return self.theta_offset + q, self.d
        if self.type == "prismatic":
            return self.theta_offset, self.d + q
        return self.theta_offset, self.d


@dataclass
class DHChain:
    name: str
    base_link: str
    end_link: str
    joints: List[str] = field(default_factory=list)
    rows: List[DHRow] = field(default_factory=list)
    base_transform: np.ndarray = field(default_factory=lambda: np.eye(4))
    tool_transform: np.ndarray = field(default_factory=lambda: np.eye(4))
    # 调试/参考：DH 各帧的轴与原点（基座系中）
    z_axes: List[Tuple[np.ndarray, np.ndarray]] = field(default_factory=list)
    x_axes: List[np.ndarray] = field(default_factory=list)
    origins: List[np.ndarray] = field(default_factory=list)

    @property
    def n(self) -> int:
        return len(self.rows)


class Line:
    """空间直线：point + unit direction。"""

    __slots__ = ("p", "u")

    def __init__(self, point, direction):
        u = np.asarray(direction, dtype=float)
        n = np.linalg.norm(u)
        if n < 1e-14:
            raise ValueError("直线方向不能为零")
        self.p = np.asarray(point, dtype=float)
        self.u = u / n


def closest_points(line_a: Line, line_b: Line) -> Tuple[np.ndarray, np.ndarray]:
    """两空间直线的最接近点 (A on a, B on b)。"""
    p1, u1 = line_a.p, line_a.u
    p2, u2 = line_b.p, line_b.u
    w = p2 - p1
    a = np.dot(u1, u1)
    b = np.dot(u1, u2)
    d = np.dot(u1, w)
    e = np.dot(u2, w)
    denom = a - b * b
    if abs(denom) > 1e-12:
        # 最小化 |(P1+t1 U1) - (P2+t2 U2)|^2 的解
        t1 = (d - b * e) / denom
        t2 = (b * d - e) / denom
    else:  # 平行：取 P2 在 L1 上的投影为 A，B = P2
        t1 = d / a
        t2 = 0.0
    return p1 + t1 * u1, p2 + t2 * u2


def _pick_perpendicular(dirn: np.ndarray) -> np.ndarray:
    """选一个垂直于 dirn 的单位向量（确定性）。"""
    u = np.asarray(dirn, dtype=float)
    u = u / np.linalg.norm(u)
    for cand in ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]):
        v = np.asarray(cand, dtype=float)
        v = v - np.dot(v, u) * u
        n = np.linalg.norm(v)
        if n > 1e-9:
            return v / n
    raise ValueError("无法构造垂直向量")


def extract_standard_dh(robot: Robot,
                        base_link: str,
                        end_link: str,
                        chain_name: str = "") -> DHChain:
    """提取 base_link -> end_link 链的标准 DH 参数。"""
    joints = find_chain(robot, base_link, end_link)
    n = len(joints)
    T = compute_transforms(robot, base_link)          # 基座 = base_link 系
    base_T = np.eye(4)                                 # base_link 自身

    # ---- 各关节轴在基座系中的直线 z_0..z_{n-1}，以及末端 z_n ----
    zlines: List[Line] = []
    for jnt in joints:
        T_child = T[jnt.child]
        # 轴方向：在子系(零位)中给出，转到基座系；轴过子系原点
        dirn = T_child[:3, :3] @ np.asarray(jnt.axis, dtype=float)
        zlines.append(Line(T_child[:3, 3], dirn))
    T_end = T[end_link]
    zlines.append(Line(T_end[:3, 3], T_end[:3, :3] @ np.array([0.0, 0.0, 1.0])))

    # ---- 构造 DH 帧 ----
    xvecs: List[np.ndarray] = [None] * (n + 1)          # x_0..x_n
    origins: List[np.ndarray] = [None] * (n + 1)        # O_0..O_n
    foots: List[np.ndarray] = [None] * (n + 1)          # 公垂线在 z_{i-1} 上的足点

    for i in range(1, n + 1):
        A, B = closest_points(zlines[i - 1], zlines[i])
        foots[i] = A
        delta = B - A
        nd = np.linalg.norm(delta)
        cross = np.cross(zlines[i - 1].u, zlines[i].u)
        if nd > _TOL:
            x = delta / nd
        elif np.linalg.norm(cross) > _TOL:
            x = cross / np.linalg.norm(cross)           # 两轴相交，a=0
        else:                                            # 两轴平行共线
            x = xvecs[i - 1] if xvecs[i - 1] is not None else _pick_perpendicular(zlines[i - 1].u)
        xvecs[i] = x
        origins[i] = B

    # 第 0 帧：取第一条公垂线（x_0 = x_1，O_0 = 公垂线在 z_0 上的足点）
    if n >= 1:
        origins[0] = foots[1]
        xvecs[0] = xvecs[1]
    else:
        origins[0] = T_end[:3, 3]
        xvecs[0] = _pick_perpendicular(zlines[0].u)

    # ---- 计算 DH 参数 ----
    rows: List[DHRow] = []
    for i in range(1, n + 1):
        jnt = joints[i - 1]
        z_prev, z_cur = zlines[i - 1].u, zlines[i].u
        x_prev, x_cur = xvecs[i - 1], xvecs[i]
        O_prev, O_cur = origins[i - 1], origins[i]
        foot = foots[i]

        a = float(np.dot(O_cur - foot, x_cur))
        alpha = math.atan2(np.dot(np.cross(z_prev, z_cur), x_cur), np.dot(z_prev, z_cur))
        d = float(np.dot(foot - O_prev, z_prev))
        theta = math.atan2(np.dot(np.cross(x_prev, x_cur), z_prev), np.dot(x_prev, x_cur))

        lo = jnt.limit.lower if jnt.limit and jnt.limit.lower > -1e30 else float("-inf")
        hi = jnt.limit.upper if jnt.limit and jnt.limit.upper < 1e30 else float("inf")
        eff = jnt.limit.effort if jnt.limit else 0.0
        vel = jnt.limit.velocity if jnt.limit else 0.0

        rows.append(DHRow(
            joint=jnt.name, type=jnt.type,
            a=a, alpha=alpha, d=d, theta_offset=theta,
            lower=lo, upper=hi, effort=eff, velocity=vel,
        ))

    # ---- 基座变换 T_{base->{0}} 与末端变换 T_{{n}->end_link} ----
    def frame_matrix(origin, xvec, zvec) -> np.ndarray:
        xv = np.asarray(xvec, dtype=float)
        xv = xv / np.linalg.norm(xv)
        zv = np.asarray(zvec, dtype=float)
        zv = zv / np.linalg.norm(zv)
        yv = np.cross(zv, xv)
        M = np.eye(4)
        M[:3, 0] = xv
        M[:3, 1] = yv
        M[:3, 2] = zv
        M[:3, 3] = origin
        return M

    T_base_to_0 = frame_matrix(origins[0], xvecs[0], zlines[0].u)
    T_base_to_n = frame_matrix(origins[n], xvecs[n], zlines[n].u)
    T_0_to_base = np.linalg.inv(T_base_to_0)
    T_n_to_end = np.linalg.inv(T_base_to_n) @ T_end

    return DHChain(
        name=chain_name,
        base_link=base_link,
        end_link=end_link,
        joints=[j.name for j in joints],
        rows=rows,
        base_transform=T_base_to_0,
        tool_transform=T_n_to_end,
        z_axes=[(l.p, l.u) for l in zlines],
        x_axes=xvecs,
        origins=origins,
    )
