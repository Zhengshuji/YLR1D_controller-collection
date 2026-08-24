"""简易入口：展开 xacro -> 提取 DH 运动学 + 动力学 -> 写 YAML 并打印摘要。

用法：python run.py
默认输入 ../urdf/ylr1d.xacro + ../config（urdf2dh 项目根下的权威模型），
输出 output/kinematics.yaml、output/dynamics.yaml。若需要其它输入，直接改常量即可。
"""
from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import yaml

from xacro2dh import (
    expand_xacro,
    parse_urdf,
    extract_standard_dh,
    extract_dynamics,
    extract_sensors,
    sensor_forward_kinematics,
    dh_forward_kinematics,
    urdf_forward_kinematics,
    write_kinematics_yaml,
    write_dynamics_yaml,
    write_sensors_yaml,
)
from xacro2dh.kinematics import pose_error
from xacro2dh.urdf_model import compute_transforms

ROOT = Path(__file__).resolve().parent
XACRO_PATH = ROOT.parent / "urdf" / "ylr1d.xacro"
CONFIG_DIR = ROOT.parent / "config"
OUT_DIR = ROOT / "output"
ROBOT_NAME = "ylr1d"

# 需要提取 DH 的链：(name, base_link, end_link)
CHAINS = [
    ("left_arm", "Link_Body2", "Link_LeftArm7"),
    ("right_arm", "Link_Body2", "Link_RightArm7"),
    ("body", "Link_Base", "Link_Body4"),
]


def main() -> None:
    xml = expand_xacro(str(XACRO_PATH), str(CONFIG_DIR))
    robot = parse_urdf(xml)

    source = {
        "xacro": os.path.relpath(XACRO_PATH, ROOT),
        "config": os.path.relpath(CONFIG_DIR, ROOT),
    }

    # 运动学
    chains = {}
    for name, base, end in CHAINS:
        chains[name] = extract_standard_dh(robot, base, end, name)
    write_kinematics_yaml(chains, ROBOT_NAME, source, OUT_DIR / "kinematics.yaml")

    # 动力学
    links = extract_dynamics(robot)
    write_dynamics_yaml(links, ROBOT_NAME, source, OUT_DIR / "dynamics.yaml")

    # 传感器位姿（id/type/link 来自 config/sensors.yaml）
    sensors_config = yaml.safe_load((CONFIG_DIR / "sensors.yaml").read_text(encoding="utf-8"))
    chain_end_links = {name: end for name, _, end in CHAINS}
    sensors = extract_sensors(robot, sensors_config, chain_end_links)
    write_sensors_yaml(sensors, ROBOT_NAME, source, "Link_Base", OUT_DIR / "sensors.yaml")

    # 摘要 + FK 验证
    print(f"机器人: {ROBOT_NAME}")
    print(f"DH 链: {', '.join(chains)}")
    print("\n各链 DH 表:")
    for name, chain in chains.items():
        print(f"\n[{name}] {chain.base_link} -> {chain.end_link}  (n={chain.n})")
        print(f"  {'joint':<18}{'type':<12}{'a':>8}{'alpha':>9}{'d':>9}{'theta_off':>10}")
        for r in chain.rows:
            print(f"  {r.joint:<18}{r.type:<12}{r.a:>8.4f}{r.alpha:>9.4f}"
                  f"{r.d:>9.4f}{r.theta_offset:>10.4f}")

    print("\n动力学 link 数:", len(links))

    print("\n传感器位姿（共", len(sensors), "个，零位形基座系平移）:")
    print(f"  {'id':<24}{'type':<8}{'chain':<10}{'mount_link':<24}base_xyz")
    for sid, s in sensors.items():
        parent = s["mount"]["parent_link"] if s["mount"] else "?"
        t = s["pose_base"]["translation"]
        print(f"  {sid:<24}{s['type']:<8}{str(s['chain']):<10}{parent:<24}"
              f"({t[0]:8.4f}, {t[1]:8.4f}, {t[2]:8.4f})")

    # FK 验证（随机关节角，DH 结果 vs URDF 结果）
    rng = np.random.default_rng(0)
    print("\nFK 验证（随机关节角 q）：")
    for name, chain in chains.items():
        worst_pos = 0.0
        worst_ang = 0.0
        for _ in range(50):
            q = {j: float(rng.uniform(-1.5, 1.5)) for j in chain.joints}
            T_dh = dh_forward_kinematics(chain, q)
            T_urdf = urdf_forward_kinematics(robot, chain.base_link, chain.end_link, q)
            pe, ae = pose_error(T_dh, T_urdf)
            worst_pos = max(worst_pos, pe)
            worst_ang = max(worst_ang, ae)
        print(f"  {name:<12} 最大位置误差={worst_pos:.2e}  最大角度误差={worst_ang:.2e}")

    # 传感器 FK 验证
    print("\n传感器 FK 验证：")
    worst_base = 0.0
    for sid, s in sensors.items():
        T = sensor_forward_kinematics(robot, s["link"])
        d = np.linalg.norm(T[:3, 3] - np.array(s["pose_base"]["translation"]))
        worst_base = max(worst_base, d)
    print(f"  pose_base(q=0) 重算最大平移误差 = {worst_base:.2e}")

    def _dict_to_mat(d: dict) -> np.ndarray:
        t = np.array(d["translation"], dtype=float)
        x, y, z, w = d["rotation_quat"]
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ])
        M = np.eye(4)
        M[:3, :3] = R
        M[:3, 3] = t
        return M

    n_chain = 0
    worst_chain = 0.0
    for sid, s in sensors.items():
        if not s["chain"]:
            continue
        n_chain += 1
        chain = chains[s["chain"]]
        T_mount = _dict_to_mat(s["mount"]["transform"])
        for _ in range(50):
            q = {j: float(rng.uniform(-1.5, 1.5)) for j in chain.joints}
            T_base = compute_transforms(robot, "Link_Base", q)
            T_dh = T_base[chain.base_link] @ dh_forward_kinematics(chain, q) @ T_mount
            pe, _ = pose_error(T_dh, T_base[s["link"]])
            worst_chain = max(worst_chain, pe)
    print(f"  链端传感器(n={n_chain}) DH·mount 组合 vs URDF 最大位置误差 = {worst_chain:.2e}")

    print(f"\n已写出:")
    print(f"  {OUT_DIR / 'kinematics.yaml'}")
    print(f"  {OUT_DIR / 'dynamics.yaml'}")
    print(f"  {OUT_DIR / 'sensors.yaml'}")


if __name__ == "__main__":
    main()
