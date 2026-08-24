"""验证 xacro 展开结果与参考 ylr1d.urdf 一致（结构 + 数值）。"""
from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from xacro2dh import expand_xacro, parse_urdf
from xacro2dh.urdf_model import compute_transforms

ROOT = Path(__file__).resolve().parent.parent
XACRO_PATH = ROOT.parent / "urdf" / "ylr1d.xacro"
CONFIG_DIR = ROOT.parent / "config"
REF_URDF = ROOT.parent / "urdf" / "ylr1d.urdf"

_NUM_RE = re.compile(r"[+-]?\d*\.?\d+(?:[eE][+-]?\d+)?\Z")


def _is_num(s: str) -> bool:
    return bool(_NUM_RE.match(s.strip()))


def _norm_elem(e1: ET.Element, e2: ET.Element, path: str, tol: float = 1e-9):
    """递归比较两棵 XML 树：tag、属性（数值容差）、子元素顺序。"""
    assert e1.tag == e2.tag, f"{path}: tag 不同 {e1.tag} != {e2.tag}"

    a1, a2 = dict(e1.attrib), dict(e2.attrib)
    assert a1.keys() == a2.keys(), f"{path}({e1.tag}): 属性不同 {list(a1)} != {list(a2)}"
    for k in a1:
        v1, v2 = a1[k], a2[k]
        if _is_num(v1) and _is_num(v2):
            assert float(v1) == pytest.approx(float(v2), abs=tol, rel=tol), \
                f"{path}.{k}: {v1} != {v2}"
        else:
            assert v1 == v2, f"{path}.{k}: {v1!r} != {v2!r}"

    c1 = [c for c in e1 if c.tag is not ET.Comment]
    c2 = [c for c in e2 if c.tag is not ET.Comment]
    assert len(c1) == len(c2), f"{path}({e1.tag}): 子元素数量不同 {len(c1)} != {len(c2)}"
    for i, (x, y) in enumerate(zip(c1, c2)):
        _norm_elem(x, y, f"{path}/{e1.tag}[{i}]", tol)


def _strip_decl(text: str) -> str:
    """去掉 <?xml ...?> 声明（两文件的声明格式不同）。"""
    if text.lstrip().startswith("<?xml"):
        _, _, rest = text.partition("?>")
        return rest
    return text


def test_expand_matches_reference_urdf():
    expanded = _strip_decl(expand_xacro(str(XACRO_PATH), str(CONFIG_DIR)))
    reference = _strip_decl(REF_URDF.read_text(encoding="utf-8"))
    _norm_elem(ET.fromstring(expanded), ET.fromstring(reference), "robot")


def test_expanded_parses_and_same_geometry():
    """展开后的 URDF 与参考 URDF 在零位处的逐 link 变换一致。"""
    expanded = expand_xacro(str(XACRO_PATH), str(CONFIG_DIR))
    reference = REF_URDF.read_text(encoding="utf-8")

    r1 = parse_urdf(expanded)
    r2 = parse_urdf(reference)
    T1 = compute_transforms(r1, "Link_Base")
    T2 = compute_transforms(r2, "Link_Base")

    assert set(T1.keys()) == set(T2.keys())
    for name in T1:
        diff = abs(T1[name] - T2[name]).max()
        assert diff < 1e-12, f"link {name} 变换差异过大: {diff}"
