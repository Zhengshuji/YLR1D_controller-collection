"""用可手算的简单 URDF 验证标准 DH 提取算法本身。"""
from __future__ import annotations

import math

import numpy as np
import pytest

from xacro2dh import parse_urdf, extract_standard_dh
from xacro2dh.kinematics import dh_forward_kinematics


def _urdf(origin1, axis1, origin2, axis2):
    """构造 base_link -[j1]-> link1 -[j2]-> link2 的 URDF 文本。"""
    def _vec(v):
        return " ".join(str(x) for x in v)

    return f"""<?xml version="1.0"?>
<robot name="hand2">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="{_vec(origin1)}"/>
    <axis xyz="{_vec(axis1)}"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="{_vec(origin2)}"/>
    <axis xyz="{_vec(axis2)}"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>"""


def _extract(origin1, axis1, origin2, axis2):
    robot = parse_urdf(_urdf(origin1, axis1, origin2, axis2))
    return extract_standard_dh(robot, "base_link", "link2", "hand")


def test_planar_2r():
    """平面 2R：两轴都沿 z，第二关节 origin 平移 (0.25, 0, 0.1)。

    标准 DH 中第 1 行捕获 轴1 与 轴2 之间的几何：
    a=0.25（公垂线长度）、d=0（两轴等高时无 z 偏移）。
    """
    chain = _extract((0, 0, 0), (0, 0, 1), (0.25, 0, 0.1), (0, 0, 1))

    r1, r2 = chain.rows
    assert r1.a == pytest.approx(0.25)
    assert r1.alpha == pytest.approx(0.0)
    assert r1.d == pytest.approx(0.0)
    assert r1.theta_offset == pytest.approx(0.0)
    assert r2.a == pytest.approx(0.0)
    assert r2.alpha == pytest.approx(0.0)
    assert r2.d == pytest.approx(0.0)
    assert r2.theta_offset == pytest.approx(0.0)


def test_rotated_axis_alpha():
    """第二关节轴绕 Y：alpha = -pi/2 出现在第 1 行（轴1->轴2 夹角），
    轴2 与末端 z 轴夹角 pi/2 出现在第 2 行（手算）。"""
    chain = _extract((0, 0, 0), (0, 0, 1), (0.2, 0, 0.15), (0, 1, 0))

    r1, r2 = chain.rows
    assert r1.a == pytest.approx(0.2)
    assert r1.alpha == pytest.approx(-math.pi / 2)
    assert r1.d == pytest.approx(0.0)
    assert r1.theta_offset == pytest.approx(0.0)
    assert r2.a == pytest.approx(0.0)
    assert r2.alpha == pytest.approx(math.pi / 2)
    assert r2.d == pytest.approx(0.0)
    assert r2.theta_offset == pytest.approx(0.0)


def test_prismatic_d_effect():
    """prismatic 关节：q 作用于 d 而非 theta。"""
    # 把 j2 换成 prismatic 重新提取
    text = _urdf((0, 0, 0), (0, 0, 1), (0.25, 0, 0.1), (0, 0, 1)).replace(
        '<joint name="j2" type="revolute">', '<joint name="j2" type="prismatic">')
    robot2 = parse_urdf(text)
    chain2 = extract_standard_dh(robot2, "base_link", "link2", "hand")

    theta0, d0 = chain2.rows[1].variable_parameter(0.0)
    theta1, d1 = chain2.rows[1].variable_parameter(0.7)
    assert theta1 == theta0                       # theta 不变
    assert d1 - d0 == pytest.approx(0.7)          # d 增加 0.7

    # FK 应等于 URDF 平移效果：j2 prismatic q=0.7 -> link2 沿 z 移 0.7
    q = {"j1": 0.0, "j2": 0.7}
    T = dh_forward_kinematics(chain2, q)
    assert T[2, 3] == pytest.approx(0.1 + 0.7)    # z = 0.1(origin) + 0.7(q)
