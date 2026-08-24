"""验证传感器位姿提取：挂载关系、零位形 pose_base、链端传感器 DH·mount 组合。"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

from xacro2dh import (
    expand_xacro,
    parse_urdf,
    extract_sensors,
    extract_standard_dh,
    sensor_forward_kinematics,
)
from xacro2dh.kinematics import dh_forward_kinematics, pose_error
from xacro2dh.urdf_model import compute_transforms

ROOT = Path(__file__).resolve().parent.parent
XACRO_PATH = ROOT.parent / "urdf" / "ylr1d.xacro"
CONFIG_DIR = ROOT.parent / "config"

CHAINS = [
    ("left_arm", "Link_Body2", "Link_LeftArm7"),
    ("right_arm", "Link_Body2", "Link_RightArm7"),
    ("body", "Link_Base", "Link_Body4"),
]


@pytest.fixture(scope="module")
def robot():
    xml = expand_xacro(str(XACRO_PATH), str(CONFIG_DIR))
    return parse_urdf(xml)


@pytest.fixture(scope="module")
def sensors(robot):
    cfg = yaml.safe_load((CONFIG_DIR / "sensors.yaml").read_text(encoding="utf-8"))
    chain_end_links = {name: end for name, _, end in CHAINS}
    return extract_sensors(robot, cfg, chain_end_links)


@pytest.fixture(scope="module")
def chains(robot):
    return {name: extract_standard_dh(robot, base, end, name)
            for name, base, end in CHAINS}


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


def test_all_sensors_extracted_with_fields(sensors):
    assert len(sensors) == 15
    for sid, s in sensors.items():
        assert s["type"] in ("camera", "depth", "ray", "imu")
        assert s["link"].startswith("Link_")
        assert s["mount"] is not None
        assert s["mount"]["joint"].endswith(s["link"][len("Link_"):])
        assert s["pose_base"]["translation"][:3]
        assert s["pose_base"]["rotation_quat"][:4]


def test_chain_assignment(sensors):
    assert sensors["global_rgb_camera"]["chain"] == "body"
    assert sensors["left_rgb_camera"]["chain"] == "left_arm"
    assert sensors["right_depth_camera"]["chain"] == "right_arm"
    for sid in ("imu_sensor", "radar_sensor", "lf_ultrasonic_sensor",
                "rf_ultrasonic_sensor", "lb_ultrasonic_sensor",
                "rb_ultrasonic_sensor"):
        assert sensors[sid]["chain"] is None
        assert sensors[sid]["mount"]["parent_link"] == "Link_Base"


def test_pose_base_matches_fk_at_zero(robot, sensors):
    """零位形下 pose_base 与 URDF 正运动学严格一致。"""
    for sid, s in sensors.items():
        T = sensor_forward_kinematics(robot, s["link"])
        pe, ae = pose_error(T, _dict_to_mat(s["pose_base"]))
        assert pe < 1e-9, f"{sid}: 平移误差 {pe}"
        assert ae < 1e-9, f"{sid}: 角度误差 {ae}"


def test_chain_sensor_dh_mount_composition(robot, sensors, chains):
    """链端传感器：T_Base^{chain_base}(q)·DH_FK(q)·mount.transform 与 URDF 一致。"""
    rng = np.random.default_rng(0)
    for sid, s in sensors.items():
        if not s["chain"]:
            continue
        chain = chains[s["chain"]]
        T_mount = _dict_to_mat(s["mount"]["transform"])
        for _ in range(50):
            q = {j: float(rng.uniform(-1.5, 1.5)) for j in chain.joints}
            T_base = compute_transforms(robot, "Link_Base", q)
            T_dh = T_base[chain.base_link] @ dh_forward_kinematics(chain, q) @ T_mount
            pe, ae = pose_error(T_dh, T_base[s["link"]])
            # 容差与 test_dh_fk.py 一致：DH 提取有 ~1e-8 数值精度
            assert pe < 1e-6, f"{sid}: 平移误差 {pe}"
            assert ae < 1e-6, f"{sid}: 角度误差 {ae}"
