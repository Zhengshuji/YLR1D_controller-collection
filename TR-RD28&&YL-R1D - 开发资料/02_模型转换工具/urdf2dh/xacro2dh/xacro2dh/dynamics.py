"""提取各 link 的动力学参数（质量 / 质心 / 惯量张量）。"""
from __future__ import annotations

from typing import Dict

from .urdf_model import Robot


def extract_dynamics(robot: Robot) -> Dict[str, dict]:
    """返回 {link_name: {mass, com:[x,y,z], inertia:{ixx..izz}}}。

    值与展开后的 URDF <inertial> 完全一致；惯量定义在过质心、与 link 系
    平行的坐标系中（URDF 规范）。
    """
    out = {}
    for name, link in robot.links.items():
        if link.inertial is None:
            continue
        out[name] = {
            "mass": link.inertial.mass,
            "com": list(link.inertial.com),
            "inertia": {k: link.inertial.inertia.get(k, 0.0)
                        for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")},
        }
    return out
