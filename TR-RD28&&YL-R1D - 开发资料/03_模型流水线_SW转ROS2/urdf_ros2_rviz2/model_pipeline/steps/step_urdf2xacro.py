"""Step 6 (optional): Convert URDF to Xacro format.

Wraps the robot model in a xacro:macro with a prefix parameter,
allowing multiple instances with different namespace prefixes.

Replaces hardcoded values (mass, inertia, colors, joint limits)
with references to YAML configuration files, enabling parameter
tuning without editing the model file.

Uses string-based XML processing to avoid ElementTree namespace issues
for the prefix injection, but uses XML parsing for value replacement.
"""

import logging
import os
import re
import xml.etree.ElementTree as ET
from xml.dom import minidom

logger = logging.getLogger(__name__)

PREFIX_VAR = "prefix"

# XML elements that should get the ${prefix} treatment
PREFIX_ATTRS = ("name", "parent", "child")


def _serialize_xml(root: ET.Element) -> str:
    """Serialize an ElementTree root to a pretty-printed string.

    Strips the XML declaration; returns only the <robot>...</robot> block.
    """
    rough = ET.tostring(root, encoding="unicode")
    dom = minidom.parseString(rough.encode())
    pretty = dom.toprettyxml(indent="  ")
    # Remove blank lines but keep XML declaration if present
    lines = [l for l in pretty.splitlines() if l.strip()]
    return "\n".join(lines)


def _inject_yaml_refs(root: ET.Element):
    """Replace hardcoded inertial / color / limit values with yaml references.

    Modifies the tree in-place:
      - <inertial><mass value="X"/>  →  mass value="${links.<name>.mass}"
      - <inertial><inertia ixx="X"/> →  ixx="${links.<name>.inertia.ixx}"
      - <material><color rgba="X"/>  →  rgba="${colors.<name>}"
      - <limit effort="X"/>          →  effort="${limits.<joint>.effort}"
      - <calibration rising="X"/>    →  rising="${calibration.<joint>.rising}"
      - <dynamics damping="X"/>      →  damping="${dynamics.<joint>.damping}"
    """
    for link in root.findall("link"):
        name = link.get("name", "")
        if not name:
            continue

        inertial = link.find("inertial")
        if inertial is not None:
            mass_elem = inertial.find("mass")
            if mass_elem is not None and mass_elem.get("value"):
                mass_elem.set("value", f"${{links.{name}.mass}}")
            inertia_elem = inertial.find("inertia")
            if inertia_elem is not None:
                for attr in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                    if inertia_elem.get(attr):
                        inertia_elem.set(attr, f"${{links.{name}.inertia.{attr}}}")

        visual = link.find("visual")
        if visual is not None:
            material = visual.find("material")
            if material is not None:
                color = material.find("color")
                if color is not None and color.get("rgba"):
                    color.set("rgba", f"${{colors.{name}}}")

    for joint in root.findall("joint"):
        name = joint.get("name", "")
        if not name:
            continue
        limit = joint.find("limit")
        if limit is not None:
            for attr in ("effort", "velocity", "lower", "upper"):
                if limit.get(attr):
                    limit.set(attr, f"${{limits.{name}.{attr}}}")

        calibration = joint.find("calibration")
        if calibration is not None:
            for attr in ("rising", "falling"):
                if calibration.get(attr):
                    calibration.set(attr, f"${{calibration.{name}.{attr}}}")

        dynamics = joint.find("dynamics")
        if dynamics is not None:
            for attr in ("damping", "friction"):
                if dynamics.get(attr):
                    dynamics.set(attr, f"${{dynamics.{name}.{attr}}}")


def _add_prefix_to_xml(xml_string: str) -> str:
    """Add ${prefix} to name/parent/child attributes in link and joint tags."""
    def _add_prefix_in_tag(match):
        full_tag = match.group(0)
        if re.match(r"<\s*(link|joint)\s", full_tag, re.IGNORECASE):
            def _replace_attr(m):
                return f'{m.group(1)}="${{{PREFIX_VAR}}}{m.group(2)}"'
            return re.sub(
                r'\b(name|parent|child)\s*=\s*"([^"]*)"',
                _replace_attr,
                full_tag,
            )
        return full_tag

    processed = re.sub(r"<[^>]+>", _add_prefix_in_tag, xml_string)

    # Handle mimic joints
    processed = re.sub(
        r'(<mimic\s[^>]*?\b)joint\s*=\s*"([^"]*)"',
        r'\1joint="${prefix}\2"',
        processed,
    )

    return processed


def urdf_to_xacro(urdf_path: str, xacro_path: str, robot_name: str = None):
    """Convert a URDF file to Xacro format with YAML configuration support.

    The generated Xacro loads parameter values from YAML config files
    (links.yaml, colors.yaml, limits.yaml) at runtime, so users can
    tune physical properties by editing the YAML files.

    Args:
        urdf_path: Input URDF file path
        xacro_path: Output Xacro file path
        robot_name: Robot name for the macro. If None, read from URDF.
    """
    # ---------------------------------------------------------------
    # Phase 1: Parse URDF, inject yaml references
    # ---------------------------------------------------------------
    try:
        tree = ET.parse(urdf_path)
    except ET.ParseError as e:
        logger.error(f"Failed to parse URDF: {e}")
        return

    root = tree.getroot()

    if robot_name is None:
        robot_name = root.get("name", "robot")

    _inject_yaml_refs(root)

    # ---------------------------------------------------------------
    # Phase 2: Serialize and extract inner content
    # ---------------------------------------------------------------
    xml_text = _serialize_xml(root)

    robot_match = re.search(
        r"<robot[^>]*>\s*(.*?)\s*</robot>", xml_text, re.DOTALL
    )
    if not robot_match:
        logger.error("Could not find <robot> tag in serialized XML")
        return

    inner_content = robot_match.group(1)

    # ---------------------------------------------------------------
    # Phase 3: Inject ${prefix} into name/parent/child attributes
    # ---------------------------------------------------------------
    inner_processed = _add_prefix_to_xml(inner_content)

    # ---------------------------------------------------------------
    # Phase 4: Build final Xacro document
    # ---------------------------------------------------------------
    lines = ['<?xml version="1.0" encoding="utf-8"?>']
    lines.append('<robot xmlns:xacro="http://wiki.ros.org/xacro"')
    lines.append(f'       name="{robot_name}">')
    lines.append("")
    lines.append("  <!-- ============================================ -->")
    lines.append("  <!-- YAML config values (links.yaml, colors.yaml,  -->")
    lines.append("  <!-- limits.yaml, scale.yaml) are passed via xacro -->")
    lines.append("  <!-- mappings from the launch file at runtime.     -->")
    lines.append("  <!-- Edit those YAML files under config/ to tune   -->")
    lines.append("  <!-- physical properties without touching the      -->")
    lines.append("  <!-- model file itself.                            -->")
    lines.append("  <!-- ============================================ -->")
    lines.append("")
    lines.append(f'  <xacro:macro name="{robot_name}" params="{PREFIX_VAR}">')
    lines.append(inner_processed)
    lines.append("  </xacro:macro>")
    lines.append("")
    lines.append(f'  <xacro:{robot_name} prefix="" />')
    lines.append("")
    lines.append("</robot>")

    output = "\n".join(lines)

    os.makedirs(os.path.dirname(xacro_path) or ".", exist_ok=True)
    with open(xacro_path, "w", encoding="utf-8") as f:
        f.write(output)

    logger.info(f"Xacro generated (with YAML config support): {xacro_path}")
