"""验证每条链的 DH 正运动学与 URDF 正运动学在随机关节角下一致。"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from xacro2dh import expand_xacro, parse_urdf, extract_standard_dh
from xacro2dh.kinematics import dh_forward_kinematics, urdf_forward_kinematics, pose_error

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


@pytest.mark.parametrize("name,base,end", CHAINS)
def test_dh_fk_matches_urdf(robot, name, base, end):
    chain = extract_standard_dh(robot, base, end, name)
    rng = np.random.default_rng(hash(name) % 2**32)
    for _ in range(100):
        q = {j: float(rng.uniform(-3.0, 3.0)) for j in chain.joints}
        T_dh = dh_forward_kinematics(chain, q)
        T_urdf = urdf_forward_kinematics(robot, base, end, q)
        pos_err, ang_err = pose_error(T_dh, T_urdf)
        # 角度误差用矩阵差的 Frobenius 范数，避免 acos 在接近 1 时的病态
        rot_err = np.linalg.norm(T_dh[:3, :3] - T_urdf[:3, :3])
        assert pos_err < 1e-6, f"{name} 位置误差过大: {pos_err}"
        assert rot_err < 1e-6, f"{name} 旋转误差过大: {rot_err}"


@pytest.mark.parametrize("name,base,end", CHAINS)
def test_dh_at_zero_matches_urdf_zero(robot, name, base, end):
    """q=0 处 DH 与 URDF 也应严格一致（验证 base/tool transform 正确）。"""
    chain = extract_standard_dh(robot, base, end, name)
    q = {j: 0.0 for j in chain.joints}
    T_dh = dh_forward_kinematics(chain, q)
    T_urdf = urdf_forward_kinematics(robot, base, end, q)
    assert np.linalg.norm(T_dh - T_urdf) < 1e-6


def test_prismatic_body_chain_joints(robot):
    """躯干链第一个关节应为 prismatic，且作用于 d。"""
    chain = extract_standard_dh(robot, "Link_Base", "Link_Body4", "body")
    row = chain.rows[0]
    assert row.joint == "Joint_Base_to_Body1"
    assert row.type == "prismatic"
    # prismatic: q 改变 d；验证可变参数
    theta, d = row.variable_parameter(0.0)
    assert d == row.d
    theta, d = row.variable_parameter(0.3)
    assert abs(d - (row.d + 0.3)) < 1e-12
