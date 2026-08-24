"""把 DH 链与动力学参数写成 YAML 文件。"""
from __future__ import annotations

from pathlib import Path
from typing import Dict, List

import numpy as np
import yaml

from .dh import DHChain


def mat_to_dict(M: np.ndarray) -> dict:
    """4x4 矩阵 -> {translation, rotation_quat:[x,y,z,w]}。"""
    t = M[:3, 3].tolist()
    q = matrix_to_quat(M[:3, :3])
    return {"translation": t, "rotation_quat": q}


def matrix_to_quat(R: np.ndarray) -> List[float]:
    """3x3 旋转矩阵 -> 单位四元数 [x,y,z,w]（w>=0 分支）。"""
    m = np.asarray(R, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    q = q / np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return [float(v) for v in q]


def _limit_dict(row) -> dict:
    lo, hi = row.lower, row.upper
    return {
        "lower": None if lo == float("-inf") else lo,
        "upper": None if hi == float("inf") else hi,
        "effort": row.effort,
        "velocity": row.velocity,
    }


def chain_to_dict(chain: DHChain) -> dict:
    table = []
    for r in chain.rows:
        table.append({
            "joint": r.joint,
            "type": r.type,
            "a": r.a,
            "alpha": r.alpha,
            "d": r.d,
            "theta_offset": r.theta_offset,
            "limit": _limit_dict(r),
        })
    return {
        "base_link": chain.base_link,
        "end_link": chain.end_link,
        "joints": chain.joints,
        "base_transform": mat_to_dict(chain.base_transform),
        "tool_transform": mat_to_dict(chain.tool_transform),
        "dh_table": table,
    }


def _header(comment: str) -> str:
    lines = []
    for line in comment.splitlines():
        lines.append(f"# {line}".rstrip())
    return "\n".join(lines) + "\n"


def write_kinematics_yaml(chains: Dict[str, DHChain],
                          robot_name: str,
                          source: dict,
                          path: Path) -> None:
    doc = {
        "generated_by": "xacro2dh",
        "robot": robot_name,
        "source": source,
        "convention": "standard_dh",
        "convention_note": ("标准DH(Craig)：A_i = Rz(theta)·Tz(d)·Tx(a)·Rx(alpha)，"
                            "x_i 为 z_{i-1}->z_i 的公垂线。theta_offset 为 q=0 时取值。"
                            "FK = base_transform · Π A_i(q) · tool_transform，"
                            "与 URDF 正运动学逐点一致（见测试）。"),
        "chains": {name: chain_to_dict(c) for name, c in chains.items()},
    }
    comment = (
        "运动学参数（标准 DH）。\n"
        f"机器人: {robot_name}\n"
        "base_transform: URDF base_link 系 -> DH 第0帧 的常量变换\n"
        "tool_transform: DH 第 n 帧 -> end_link 系 的常量变换\n"
        "revolute/continuous 关节 q 作用于 theta；prismatic 关节 q 作用于 d。"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    text = _header(comment) + yaml.safe_dump(doc, sort_keys=False, allow_unicode=True,
                                             width=120, default_flow_style=False)
    path.write_text(text, encoding="utf-8")


def write_sensors_yaml(sensors: Dict[str, dict], robot_name: str,
                       source: dict, base_frame: str, path: Path) -> None:
    doc = {
        "generated_by": "xacro2dh",
        "robot": robot_name,
        "source": source,
        "base_frame": base_frame,
        "convention_note": (
            "mount.transform 为常量 T_{parent_link -> sensor}（固定关节 origin）；"
            "pose_base 为零位形(q=0)下 T_{base_frame -> sensor}。"
            "挂在 DH 链末端(chain)的传感器，任意 q 下："
            "T_Base^sensor(q) = T_Base^{chain_base}(q) · DH_FK(chain, q) · mount.transform，"
            "详见 kinematics.yaml 的链 FK。"),
        "sensors": sensors,
    }
    comment = (
        "传感器位姿参数，逐传感器提取自展开后的 URDF。\n"
        f"机器人: {robot_name}\n"
        f"base_frame: {base_frame}\n"
        "mount.transform: T_{parent_link -> sensor} 常量变换\n"
        "pose_base: 零位形下 T_{base_frame -> sensor}\n"
        "chain: 若挂在 DH 链末端则为链名（left_arm/right_arm/body），否则空"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    text = _header(comment) + yaml.safe_dump(doc, sort_keys=False, allow_unicode=True,
                                             width=120, default_flow_style=False)
    path.write_text(text, encoding="utf-8")


def write_dynamics_yaml(links: Dict[str, dict], robot_name: str,
                        source: dict, path: Path) -> None:
    doc = {
        "generated_by": "xacro2dh",
        "robot": robot_name,
        "source": source,
        "convention_note": ("惯量定义在过质心、与 link 系平行的坐标系中（URDF 规范）。"
                            "com 为质心在 link 系中的位置。"),
        "links": links,
    }
    comment = (
        "动力学参数（质量 / 质心 / 惯量张量），逐 link 提取自展开后的 URDF。"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    text = _header(comment) + yaml.safe_dump(doc, sort_keys=False, allow_unicode=True,
                                             width=120, default_flow_style=False)
    path.write_text(text, encoding="utf-8")
