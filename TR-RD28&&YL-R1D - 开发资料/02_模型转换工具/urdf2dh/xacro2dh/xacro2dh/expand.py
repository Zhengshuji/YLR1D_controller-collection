"""轻量 xacro 展开器。

针对本项目的 ylr1d.xacro：把 ${links.X.Y} / ${colors.X} / ${limits.X.Y} /
${calibration.X.Y} / ${dynamics.X.Y} 用 config 目录下的 YAML 替换，
再把 <xacro:ylr1d prefix=""/> 宏调用展开（${prefix} 替换为实际值），
并移除由 launch 运行时注入的 ${controllers_yaml_path} 占位元素。

不需要安装 ROS/xacro 包，纯 Python 即可完成。
"""
from __future__ import annotations

import os
import re
import yaml

# 参与展开的 config 文件（与 launch 脚本一致）
_CONFIG_NAMES = ["links", "colors", "limits", "scale", "calibration", "dynamics"]

_EXPR_RE = re.compile(r"\$\{([^}]+)\}")
_SIMPLE_ID_RE = re.compile(r"^[a-zA-Z_]\w*$")

_MACRO_DEF_RE = re.compile(
    r"<xacro:macro\s+name=\"([^\"]+)\"\s+params=\"([^\"]*)\"[^>]*>(.*?)</xacro:macro>",
    re.S,
)
_CALL_RE = re.compile(
    r"<xacro:([A-Za-z_]\w*)\s+([^>]*?)\s*/?>|"
    r"<xacro:([A-Za-z_]\w*)\s+([^>]*?)>(.*?)</xacro:\2>",
    re.S,
)
_ATTR_RE = re.compile(r"(\w+)\s*=\s*\"([^\"]*)\"")
_NS_RE = re.compile(r'\sxmlns:xacro="[^"]*"')
_XACRO_TAG_RE = re.compile(r"</?xacro:[\w]+")


def load_configs(config_dir: str) -> dict:
    configs = {}
    for name in _CONFIG_NAMES:
        path = os.path.join(config_dir, f"{name}.yaml")
        if os.path.isfile(path):
            with open(path, encoding="utf-8") as f:
                data = yaml.safe_load(f)
            if isinstance(data, dict):
                configs[name] = data
    return configs


def _resolve_expr(expr: str, configs: dict) -> str | None:
    """把 ${a.b.c} 从 config 数据里解析出来；解析不了返回 None。"""
    parts = expr.split(".")
    if parts[0] not in configs:
        return None
    val = configs[parts[0]]
    try:
        for p in parts[1:]:
            val = val[p]
    except (KeyError, TypeError):
        return None
    if isinstance(val, float):
        return repr(val)
    if isinstance(val, int):
        return str(val)
    return str(val)


def resolve_yaml_refs(content: str, configs: dict) -> str:
    def _repl(match: re.Match) -> str:
        expr = match.group(1).strip()
        if _SIMPLE_ID_RE.match(expr):          # 简单变量(prefix等)留给宏展开
            return match.group(0)
        val = _resolve_expr(expr, configs)
        return val if val is not None else match.group(0)

    return _EXPR_RE.sub(_repl, content)


def _expand_macros(content: str) -> str:
    """展开 <xacro:macro> 定义，并把对应调用替换为宏体。"""
    # 收集宏定义
    macros = {}
    for m in _MACRO_DEF_RE.finditer(content):
        name, params, body = m.group(1), m.group(2), m.group(3)
        macros[name] = (params.split(), body)
    content = _MACRO_DEF_RE.sub("", content)

    def _expand_call(match: re.Match) -> str:
        name = match.group(1) or match.group(3)
        attr_text = match.group(2) or match.group(4) or ""
        if name not in macros:
            return match.group(0)
        params, body = macros[name]
        attrs = dict(_ATTR_RE.findall(attr_text))
        out = body
        for p in params:
            out = out.replace("${" + p + "}", attrs.get(p, ""))
        return out

    # 反复替换直到稳定（宏体里可能再有调用，本项目只有一层）
    for _ in range(8):
        new = _CALL_RE.sub(_expand_call, content)
        if new == content:
            break
        content = new

    content = _NS_RE.sub("", content)
    content = _XACRO_TAG_RE.sub("", content)
    return content


# launch 运行时注入的外部参数占位符：整行元素应被移除（与 xacro2urdf 一致）
_EXTERNAL_PLACEHOLDER = "controllers_yaml_path"
_PLACEHOLDER_ELEM_RE = re.compile(
    r"^\s*<[\w:-]+>\s*\$\{" + re.escape(_EXTERNAL_PLACEHOLDER) + r"\}\s*</[\w:-]+>\s*$",
    re.MULTILINE,
)


def remove_external_placeholder(content: str) -> str:
    """移除内容恰好为 ${controllers_yaml_path} 的元素（由 launch 注入）。"""
    return _PLACEHOLDER_ELEM_RE.sub("", content)


def remove_fixed_joint_axis(content: str) -> str:
    """URDF 规范：fixed 关节不能带 <axis>，移除之（与 xacro2urdf 一致）。"""

    def _clean(m: re.Match) -> str:
        return m.group(1) + re.sub(r"<axis\b[^>]*/>", "", m.group(2)) + m.group(3)

    return re.sub(
        r'(<joint\b[^>]*\btype="fixed"[^>]*>)(.*?)(</joint>)', _clean, content,
        flags=re.DOTALL,
    )


def expand_xacro(xacro_path: str, config_dir: str) -> str:
    with open(xacro_path, encoding="utf-8") as f:
        content = f.read()
    configs = load_configs(config_dir)
    content = resolve_yaml_refs(content, configs)
    content = _expand_macros(content)
    content = remove_external_placeholder(content)
    content = remove_fixed_joint_axis(content)
    return content


def load_urdf(urdf_path: str) -> str:
    with open(urdf_path, encoding="utf-8") as f:
        return f.read()
