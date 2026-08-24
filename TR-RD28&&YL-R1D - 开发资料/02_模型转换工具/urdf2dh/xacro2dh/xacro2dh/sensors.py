"""提取传感器位姿参数（挂载关系 + 零位形基座系位姿）。

ylr1d 的每个传感器 link 通过一个 fixed 关节挂到父 link 上。对算法设计最
有用的两类信息：
  - mount.transform：常量 T_{parent_link -> sensor}（即该 fixed 关节的 origin）；
  - pose_base：零位形下 T_{Link_Base -> sensor}。
挂在 DH 链末端的传感器（如左右手相机）可进一步用
  T_Base^sensor(q) = T_Base^{chain_base}(q) · DH_FK(chain, q) · mount.transform
在任意关节角下计算，其中 mount.transform 为常量。
"""
from __future__ import annotations

from typing import Dict, Optional

import numpy as np

from .output import mat_to_dict
from .urdf_model import Robot, compute_transforms


def extract_sensors(robot: Robot,
                    sensors_config: Dict[str, dict],
                    chain_end_links: Dict[str, str],
                    base_link: str = "Link_Base") -> Dict[str, dict]:
    """返回 {sensor_id: {type, link, mount, chain, pose_base}}。

    sensors_config: config/sensors.yaml 解析结果（id -> {type, link, ...}）。
    chain_end_links: {链名: 末端 link}，用于把挂在链末端的传感器标记上链名。
    mount.transform / pose_base 均为 {translation, rotation_quat}。
    """
    # fixed 关节：child(sensor link) -> (joint名, parent, T_{parent->child})
    mount_of: Dict[str, tuple] = {}
    for jnt in robot.joints.values():
        if jnt.type == "fixed":
            mount_of[jnt.child] = (jnt.name, jnt.parent, jnt.origin.to_matrix())

    chain_of: Dict[str, Optional[str]] = {}
    for cname, end in chain_end_links.items():
        chain_of[end] = cname

    T = compute_transforms(robot, base_link)          # 零位形各 link 位姿
    out: Dict[str, dict] = {}
    for sid, cfg in sensors_config.items():
        link = cfg.get("link", "")
        if link not in robot.links:
            continue
        entry = {
            "type": cfg.get("type", ""),
            "link": link,
        }
        if link in mount_of:
            jname, parent, T_mount = mount_of[link]
            entry["mount"] = {
                "joint": jname,
                "parent_link": parent,
                "transform": mat_to_dict(T_mount),
            }
            # 挂在链末端 link 上的传感器归属该 DH 链（链名按父 link 判定）
            entry["chain"] = chain_of.get(parent)
        else:
            entry["mount"] = None
            entry["chain"] = None
        entry["pose_base"] = mat_to_dict(T[link])
        out[sid] = entry
    return out


def sensor_forward_kinematics(robot: Robot,
                              sensor_link: str,
                              q: Optional[Dict[str, float]] = None,
                              base_link: str = "Link_Base") -> np.ndarray:
    """T_{base_link -> sensor_link}（URDF 语义，q 缺省即零位形）。"""
    T = compute_transforms(robot, base_link, q or {})
    return T[sensor_link]
