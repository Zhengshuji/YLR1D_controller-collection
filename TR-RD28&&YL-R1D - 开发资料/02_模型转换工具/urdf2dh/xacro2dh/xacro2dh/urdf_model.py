"""URDF 数据模型与正运动学。

仅依赖标准库 + numpy。支持 fixed / revolute / continuous / prismatic 关节。
关节运动语义遵循 URDF 规范：

    T_parent_to_child(q) = Pose(origin) * Motion(axis, q)

其中 Pose(origin) = Trans(xyz) * Rz(yaw) * Ry(pitch) * Rx(roll)，
revolute/continuous 时 Motion = Rot(axis, q)，prismatic 时 Motion = Trans(axis * q)。
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

DEFAULT_AXIS = (0.0, 0.0, 1.0)


def rpy_to_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF fixed-axis RPY 旋转矩阵 R = Rz(yaw) @ Ry(pitch) @ Rx(roll)。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, cr, -sr],
                   [0.0, sr, cr]])
    Ry = np.array([[cp, 0.0, sp],
                   [0.0, 1.0, 0.0],
                   [-sp, 0.0, cp]])
    Rz = np.array([[cy, -sy, 0.0],
                   [sy, cy, 0.0],
                   [0.0, 0.0, 1.0]])
    return Rz @ Ry @ Rx


def rotation_about_axis(axis: Tuple[float, float, float], angle: float) -> np.ndarray:
    """Rodrigues 公式：绕单位轴 axis 旋转 angle 弧度的 3x3 矩阵。"""
    u = np.asarray(axis, dtype=float)
    n = np.linalg.norm(u)
    if n < 1e-14:
        return np.eye(3)
    u = u / n
    c, s = math.cos(angle), math.sin(angle)
    K = np.array([[0.0, -u[2], u[1]],
                  [u[2], 0.0, -u[0]],
                  [-u[1], u[0], 0.0]])
    return np.eye(3) + s * K + (1.0 - c) * (K @ K)


@dataclass
class Pose:
    """URDF origin：平移 xyz + 固定轴 RPY 旋转。"""
    xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def to_matrix(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = rpy_to_rotation(*self.rpy)
        T[:3, 3] = self.xyz
        return T


@dataclass
class Limit:
    lower: float = -math.inf
    upper: float = math.inf
    effort: float = 0.0
    velocity: float = 0.0


@dataclass
class Mimic:
    joint: str = ""
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass
class Inertial:
    mass: float = 0.0
    origin: Pose = field(default_factory=Pose)  # 质心在 link 系中的位置
    inertia: Dict[str, float] = field(default_factory=dict)  # ixx..izz

    @property
    def com(self) -> Tuple[float, float, float]:
        return self.origin.xyz


@dataclass
class Geometry:
    kind: str = "mesh"      # mesh / box / cylinder / sphere
    filename: str = ""
    kwargs: Dict = field(default_factory=dict)


@dataclass
class Visual:
    origin: Pose = field(default_factory=Pose)
    geometry: Geometry = field(default_factory=Geometry)
    material: str = ""


@dataclass
class Link:
    name: str
    inertial: Optional[Inertial] = None
    visual: Optional[Visual] = None
    collision: Optional[Visual] = None


@dataclass
class Joint:
    name: str
    type: str                    # fixed / revolute / continuous / prismatic
    parent: str
    child: str
    origin: Pose = field(default_factory=Pose)
    axis: Tuple[float, float, float] = DEFAULT_AXIS
    limit: Optional[Limit] = None
    mimic: Optional[Mimic] = None
    calibration: Dict[str, float] = field(default_factory=dict)
    dynamics: Dict[str, float] = field(default_factory=dict)

    @property
    def is_movable(self) -> bool:
        return self.type in ("revolute", "continuous", "prismatic")

    def motion_matrix(self, q: float) -> np.ndarray:
        """关节运动子矩阵：Pose 之后的相对运动。"""
        T = np.eye(4)
        if self.type in ("revolute", "continuous"):
            T[:3, :3] = rotation_about_axis(self.axis, q)
        elif self.type == "prismatic":
            a = np.asarray(self.axis, dtype=float)
            n = np.linalg.norm(a)
            if n < 1e-14:
                a = np.asarray(DEFAULT_AXIS, dtype=float)
            else:
                a = a / n
            T[:3, 3] = a * q
        return T

    def transform(self, q: float = 0.0) -> np.ndarray:
        """关节 q 下的 4x4 变换（父系 -> 子系）。"""
        return self.origin.to_matrix() @ self.motion_matrix(q)


@dataclass
class Robot:
    name: str
    links: Dict[str, Link] = field(default_factory=dict)
    joints: Dict[str, Joint] = field(default_factory=dict)

    def link(self, name: str) -> Link:
        return self.links[name]

    def joint(self, name: str) -> Joint:
        return self.joints[name]


def _parse_pose(el) -> Pose:
    if el is None:
        return Pose()
    xyz = _parse_vec(el.get("xyz"), 3)
    rpy = _parse_vec(el.get("rpy"), 3)
    return Pose(xyz=tuple(xyz), rpy=tuple(rpy))


def _parse_vec(s, n):
    if not s:
        return [0.0] * n
    parts = s.replace(",", " ").split()
    vals = [float(p) for p in parts]
    if len(vals) < n:
        vals += [0.0] * (n - len(vals))
    return vals[:n]


def _parse_inertial(el) -> Inertial:
    mass = 0.0
    origin = Pose()
    inertia = {}
    for sub in el:
        tag = sub.tag.split("}")[-1]
        if tag == "mass":
            mass = float(sub.get("value", "0"))
        elif tag == "origin":
            origin = _parse_pose(sub)
        elif tag == "inertia":
            for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                inertia[k] = float(sub.get(k, "0"))
    return Inertial(mass=mass, origin=origin, inertia=inertia)


def _parse_geometry(el) -> Geometry:
    g = Geometry()
    for sub in el:
        tag = sub.tag.split("}")[-1]
        g.kind = tag
        if tag == "mesh":
            g.filename = sub.get("filename", "")
        else:
            for k, v in sub.attrib.items():
                g.kwargs[k] = v
    return g


def _parse_visual(el) -> Visual:
    origin = Pose()
    geom = Geometry()
    material = ""
    for sub in el:
        tag = sub.tag.split("}")[-1]
        if tag == "origin":
            origin = _parse_pose(sub)
        elif tag == "geometry":
            geom = _parse_geometry(sub)
        elif tag == "material":
            material = sub.get("name", "")
    return Visual(origin=origin, geometry=geom, material=material)


def parse_urdf(xml_text: str) -> Robot:
    root = ET.fromstring(xml_text)
    robot = Robot(name=root.get("name", "robot"))

    for child in root:
        tag = child.tag.split("}")[-1]
        if tag == "link":
            name = child.get("name", "")
            link = Link(name=name)
            for sub in child:
                stag = sub.tag.split("}")[-1]
                if stag == "inertial":
                    link.inertial = _parse_inertial(sub)
                elif stag == "visual":
                    link.visual = _parse_visual(sub)
                elif stag == "collision":
                    link.collision = _parse_visual(sub)
            robot.links[name] = link

        elif tag == "joint":
            name = child.get("name", "")
            jtype = child.get("type", "fixed")
            jnt = Joint(name=name, type=jtype, parent="", child="")
            for sub in child:
                stag = sub.tag.split("}")[-1]
                if stag == "parent":
                    jnt.parent = sub.get("link", "")
                elif stag == "child":
                    jnt.child = sub.get("link", "")
                elif stag == "origin":
                    jnt.origin = _parse_pose(sub)
                elif stag == "axis":
                    jnt.axis = tuple(_parse_vec(sub.get("xyz"), 3))
                elif stag == "limit":
                    jnt.limit = Limit(
                        lower=float(sub.get("lower", "-inf")),
                        upper=float(sub.get("upper", "inf")),
                        effort=float(sub.get("effort", "0")),
                        velocity=float(sub.get("velocity", "0")),
                    )
                elif stag == "mimic":
                    jnt.mimic = Mimic(
                        joint=sub.get("joint", ""),
                        multiplier=float(sub.get("multiplier", "1")),
                        offset=float(sub.get("offset", "0")),
                    )
                elif stag == "calibration":
                    jnt.calibration = {
                        k: float(sub.get(k, "0"))
                        for k in ("rising", "falling")
                    }
                elif stag == "dynamics":
                    jnt.dynamics = {
                        k: float(sub.get(k, "0"))
                        for k in ("damping", "friction")
                    }
            robot.joints[name] = jnt

    return robot


def compute_transforms(robot: Robot,
                       root_link: str,
                       joint_values: Optional[Dict[str, float]] = None) -> Dict[str, np.ndarray]:
    """以 root_link 为基座计算所有 link 的世界位姿（4x4）。

    joint_values 为空时即 URDF 零位形（关节 origin 描述的位形）。
    树形遍历：按父 link 已计算的前提逐关节累积。
    """
    jv = joint_values or {}
    # 建立 父link -> [(joint, child)] 邻接表
    children: Dict[str, List[Tuple[Joint, str]]] = {name: [] for name in robot.links}
    for jnt in robot.joints.values():
        children.setdefault(jnt.parent, []).append((jnt, jnt.child))

    transforms = {root_link: np.eye(4)}
    # 迭代遍历（树结构，顺序无关但需要父先于子）
    order = [root_link]
    idx = 0
    while idx < len(order):
        cur = order[idx]
        idx += 1
        T_cur = transforms[cur]
        for jnt, child in children.get(cur, []):
            q = jv.get(jnt.name, 0.0)
            transforms[child] = T_cur @ jnt.transform(q)
            order.append(child)
    return transforms


def find_chain(robot: Robot, base_link: str, end_link: str) -> List[Joint]:
    """返回从 base_link 到 end_link 的关节序列（沿 parent->child 边）。

    若 end_link 不是 base_link 的后代，抛出 ValueError。
    """
    child_of: Dict[str, str] = {}   # child -> parent
    parent_of: Dict[str, str] = {}  # parent -> child (单亲树，仅用于向上回溯)
    for jnt in robot.joints.values():
        parent_of.setdefault(jnt.parent, []).append((jnt, jnt.child))
        child_of[jnt.child] = (jnt, jnt.parent)

    # 向上回溯 end_link -> base_link
    path_rev: List[Joint] = []
    cur = end_link
    visited = set()
    while cur != base_link:
        if cur in visited or cur not in child_of:
            raise ValueError(
                f"无法从 {base_link} 到达 {end_link}（{cur} 没有父关节或成环）")
        visited.add(cur)
        jnt, parent = child_of[cur]
        path_rev.append(jnt)
        cur = parent
    path_rev.reverse()
    return path_rev


def joint_limits(robot: Robot) -> Dict[str, Tuple[float, float]]:
    """返回各可动关节的 (lower, upper) 限位。"""
    out = {}
    for name, jnt in robot.joints.items():
        if not jnt.is_movable:
            continue
        lo = jnt.limit.lower if jnt.limit and jnt.limit.lower > -math.inf else -3.14
        hi = jnt.limit.upper if jnt.limit and jnt.limit.upper < math.inf else 3.14
        out[name] = (lo, hi)
    return out
