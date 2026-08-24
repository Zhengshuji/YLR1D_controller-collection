#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert the ylr1d xacro + YAML config files into a self-contained URDF.

The generated URDF has every ``${...}`` placeholder resolved from the config
directory, so it needs no external parameters at load time. Only a small,
focused subset of the xacro language is needed for this model:

  * a single ``<xacro:macro name="ylr1d" params="prefix">`` definition,
    expanded once via ``<xacro:ylr1d prefix="" />``;
  * ``${...}`` substitutions of the form
      - ``${prefix}``              -> macro argument (may be concatenated),
      - ``${<ns>.<key>.<subkey>..}`` -> lookups into the YAML config dicts
        (links / colors / limits / calibration / dynamics);
  * the placeholder ``${controllers_yaml_path}`` which is an *external*
    runtime parameter injected by the ROS launch file. It is dropped from the
    output so the URDF stays self-contained.

The output preserves the source structure, comments and ordering of the xacro,
with all values inlined.

Usage::

    python -m xacro2urdf
    python -m xacro2urdf --output out.urdf
"""

from __future__ import annotations

import argparse
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.exit("PyYAML is required. Install it with: pip install pyyaml")

PROJECT_ROOT = Path(__file__).resolve().parent.parent

DEFAULT_XACRO = PROJECT_ROOT / "urdf" / "ylr1d.xacro"
DEFAULT_CONFIG_DIR = PROJECT_ROOT / "config"
DEFAULT_OUTPUT = PROJECT_ROOT / "urdf" / "ylr1d.urdf"

# YAML config files that back the namespaces used by ${...} expressions.
CONFIG_FILES = {
    "links": "links.yaml",
    "colors": "colors.yaml",
    "limits": "limits.yaml",
    "calibration": "calibration.yaml",
    "dynamics": "dynamics.yaml",
    "sensors": "sensors.yaml",
}

# External runtime parameter injected by the launch file; must not survive
# into the static URDF (its wrapping element is removed).
EXTERNAL_PLACEHOLDER = "controllers_yaml_path"

_VAR_RE = re.compile(r"\$\{([^}]*)\}")
_COMMENT_RE = re.compile(r"<!--.*?-->", re.DOTALL)
_MACRO_DEF_RE = re.compile(r"<xacro:macro\b[^>]*>.*?</xacro:macro>", re.DOTALL)
_MACRO_OPEN_RE = re.compile(r"<xacro:macro\b([^>]*)>")
_INVOCATION_RE = re.compile(r"<xacro:(?!macro\b)[\w:-]+\b([^>]*)/>")
_ATTR_RE = re.compile(r'([\w:.-]+)\s*=\s*"([^"]*)"')
_SORT_TAG_RE = re.compile(
    r"^(\s*)(<[\w:.-]+)((?:\s+[\w:.-]+=\"[^\"]*\")*)\s*(/>|>)\s*$"
)


# --------------------------------------------------------------------------- #
# config loading and expression resolution
# --------------------------------------------------------------------------- #
def load_configs(config_dir: Path) -> dict:
    """Load every YAML config file referenced by the xacro expressions."""
    configs = {}
    for name, fname in CONFIG_FILES.items():
        path = Path(config_dir) / fname
        with open(path, "r", encoding="utf-8") as fh:
            configs[name] = yaml.safe_load(fh)
    return configs


def _format_value(value):
    """Render a resolved YAML value back to a URDF-attribute string."""
    return str(value)


def resolve_expression(expr: str, namespace: dict) -> str:
    """Resolve a single ``${...}`` body to its string value."""
    expr = expr.strip()
    if expr == EXTERNAL_PLACEHOLDER:
        # keep the full placeholder text; the wrapping element is removed in
        # post-processing so the URDF stays self-contained
        return f"${{{EXTERNAL_PLACEHOLDER}}}"

    parts = expr.split(".")
    root = parts[0]
    if root not in namespace:
        raise ValueError(f"undefined xacro expression: ${{{expr}}}")

    value = namespace[root]
    for key in parts[1:]:
        if not isinstance(value, dict) or key not in value:
            raise ValueError(f"undefined xacro expression: ${{{expr}}}")
        value = value[key]
    return _format_value(value)


def substitute_text(text: str, namespace: dict) -> str:
    """Replace ``${...}`` everywhere except inside XML comments."""
    out = []
    pos = 0
    while True:
        start = text.find("<!--", pos)
        if start == -1:
            out.append(_VAR_RE.sub(lambda m: resolve_expression(m.group(1), namespace),
                                   text[pos:]))
            break
        out.append(_VAR_RE.sub(lambda m: resolve_expression(m.group(1), namespace),
                               text[pos:start]))
        end = text.find("-->", start)
        if end == -1:
            out.append(text[start:])
            break
        out.append(text[start:end + 3])  # comment kept verbatim (not substituted)
        pos = end + 3
    return "".join(out)


# --------------------------------------------------------------------------- #
# macro expansion
# --------------------------------------------------------------------------- #
def _parse_attrs(tag_attrs: str) -> dict:
    return dict(_ATTR_RE.findall(tag_attrs))


def expand_macros(text: str, configs: dict) -> dict:
    """Expand the single xacro macro, returning (document, namespace)."""
    match = _MACRO_DEF_RE.search(text)
    if not match:
        return text, {**configs, "prefix": ""}

    full = match.group(0)
    open_tag = _MACRO_OPEN_RE.search(full)
    body = full[len(open_tag.group(0)):-len("</xacro:macro>")]

    inv = _INVOCATION_RE.search(text)
    args = _parse_attrs(inv.group(1)) if inv else {}
    prefix = args.get("prefix", "")

    namespace = {**configs, "prefix": prefix}

    doc = text.replace(full, body)                  # macro definition -> its body
    if inv:
        doc = doc.replace(inv.group(0), "", 1)      # remove the invocation line
    return doc, namespace


# --------------------------------------------------------------------------- #
# post-processing
# --------------------------------------------------------------------------- #
def remove_external_placeholder(doc: str) -> str:
    """Drop the element whose whole content is the external placeholder."""
    pattern = re.compile(
        r"^\s*<\w+>\s*\$\{" + re.escape(EXTERNAL_PLACEHOLDER) + r"\}\s*</\w+>\s*$",
        re.MULTILINE,
    )
    return pattern.sub("", doc)


def remove_fixed_joint_axis(doc: str) -> str:
    """URDF spec: fixed joints cannot carry an <axis> element; drop them."""
    def _clean(m):
        return m.group(1) + re.sub(r"<axis\b[^>]*/>", "", m.group(2)) + m.group(3)

    return re.sub(
        r'(<joint\b[^>]*\btype="fixed"[^>]*>)(.*?)(</joint>)', _clean, doc, flags=re.DOTALL
    )


def _sort_attrs(line: str) -> str:
    """Sort element attributes alphabetically on a single-tag line."""
    if "<!--" in line or "-->" in line:
        return line
    m = _SORT_TAG_RE.match(line)
    if not m or not m.group(3):
        return line
    attrs = _ATTR_RE.findall(m.group(3))
    if len(attrs) <= 1:
        return line
    sorted_attrs = " ".join(f'{k}="{v}"' for k, v in sorted(attrs))
    return f"{m.group(1)}{m.group(2)} {sorted_attrs}{m.group(4)}"


def sort_all_attrs(doc: str) -> str:
    """Sort attributes of real elements; leave comment blocks untouched."""
    out = []
    in_comment = False
    for line in doc.split("\n"):
        if "<!--" in line:
            in_comment = True
        if not in_comment:
            line = _sort_attrs(line)
        if "-->" in line:
            in_comment = False
        out.append(line)
    return "\n".join(out)


def drop_unused_xacro_namespace(doc: str) -> str:
    return re.sub(r'\s+xmlns:xacro="[^"]*"', "", doc, count=1)


# --------------------------------------------------------------------------- #
# main entry point
# --------------------------------------------------------------------------- #
def convert(xacro_path=None, config_dir=None, output_path=None) -> str:
    """Run the full conversion and return the generated URDF text."""
    xacro_path = Path(xacro_path or DEFAULT_XACRO)
    config_dir = Path(config_dir or DEFAULT_CONFIG_DIR)
    output_path = Path(output_path or DEFAULT_OUTPUT)

    configs = load_configs(config_dir)
    source = xacro_path.read_text(encoding="utf-8")

    doc, namespace = expand_macros(source, configs)
    doc = substitute_text(doc, namespace)
    doc = remove_external_placeholder(doc)
    doc = remove_fixed_joint_axis(doc)
    doc = sort_all_attrs(doc)
    doc = drop_unused_xacro_namespace(doc)

    # sanity check: must be well-formed XML with no leftover substitution
    # (only scan outside comments; the xacro keeps literal ${...} in comments)
    remaining = _VAR_RE.findall(_COMMENT_RE.sub("", doc))
    remaining = [r for r in remaining if r.strip() != EXTERNAL_PLACEHOLDER]
    if remaining:
        raise ValueError(f"unresolved xacro expression(s): {sorted(set(remaining))}")
    ET.fromstring(doc)

    output_path.write_text(doc, encoding="utf-8")
    return str(output_path)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Convert the ylr1d xacro + YAML config into a self-contained URDF."
    )
    parser.add_argument("--xacro", default=str(DEFAULT_XACRO),
                        help="path to the .xacro file")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_DIR),
                        help="directory with the config YAML files")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT),
                        help="path of the generated .urdf file")
    args = parser.parse_args(argv)

    out = convert(args.xacro, args.config, args.output)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
