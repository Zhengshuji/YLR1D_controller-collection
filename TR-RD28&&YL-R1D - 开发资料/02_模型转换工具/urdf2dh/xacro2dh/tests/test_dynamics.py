"""验证动力学参数（质量 / 质心 / 惯量）与展开后 URDF 的 <inertial> 逐 link 一致。"""
from __future__ import annotations

from pathlib import Path

import pytest

from xacro2dh import expand_xacro, parse_urdf, extract_dynamics

ROOT = Path(__file__).resolve().parent.parent
XACRO_PATH = ROOT.parent / "urdf" / "ylr1d.xacro"
CONFIG_DIR = ROOT.parent / "config"
REF_URDF = ROOT.parent / "urdf" / "ylr1d.urdf"


@pytest.fixture(scope="module")
def robot():
    xml = expand_xacro(str(XACRO_PATH), str(CONFIG_DIR))
    return parse_urdf(xml)


@pytest.fixture(scope="module")
def ref_robot():
    return parse_urdf(REF_URDF.read_text(encoding="utf-8"))


def test_dynamics_matches_expanded_urdf(robot):
    dyn = extract_dynamics(robot)
    keys = ("mass", "com", "inertia")
    count = 0
    for name, link in robot.links.items():
        if link.inertial is None:
            continue
        count += 1
        d = dyn[name]
        assert d["mass"] == pytest.approx(link.inertial.mass)
        assert d["com"] == pytest.approx(list(link.inertial.com))
        for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            assert d["inertia"][k] == pytest.approx(link.inertial.inertia.get(k, 0.0))
    # 至少提取到主要连杆（参考 config 中 dynamics.yaml 的条目数应被覆盖）
    assert count >= 20
    assert len(dyn) == count


def test_dynamics_matches_reference_urdf(robot, ref_robot):
    """当前 config 展开后的动力学与参考 ylr1d.urdf 完全一致。"""
    dyn = extract_dynamics(robot)
    for name, link in ref_robot.links.items():
        if link.inertial is None:
            continue
        assert name in dyn, f"参考 URDF 的 link {name} 缺失动力学参数"
        d = dyn[name]
        assert d["mass"] == pytest.approx(link.inertial.mass, abs=1e-9)
        assert d["com"] == pytest.approx(list(link.inertial.com), abs=1e-9)
        for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            assert d["inertia"][k] == pytest.approx(
                link.inertial.inertia.get(k, 0.0), abs=1e-9)
