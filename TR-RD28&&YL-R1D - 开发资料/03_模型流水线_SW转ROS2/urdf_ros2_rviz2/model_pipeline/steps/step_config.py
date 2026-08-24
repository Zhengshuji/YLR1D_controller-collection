"""Step 4 (extended): Extract URDF configuration data to YAML files.

Generates the following config files from the cleaned URDF:
  - links.yaml:   Inertial data (mass, inertia tensor) per link
  - colors.yaml:  Visual material colors per link
  - scale.yaml:   Density scale (default only)
  - limits.yaml:  Joint limits (effort, velocity, lower, upper)

These configs are used by Xacro launch files via xacro.load_yaml().
"""

import logging
import os
import xml.etree.ElementTree as ET
from typing import Optional

logger = logging.getLogger(__name__)

# Key order for consistent YAML output
INERTIA_KEYS = ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]


def extract_configs(urdf_path: str, config_dir: str):
    """Extract links.yaml, colors.yaml, scale.yaml, limits.yaml from URDF.

    Args:
        urdf_path: Path to the (cleaned) URDF file
        config_dir: Output directory for config YAML files
    """
    os.makedirs(config_dir, exist_ok=True)

    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
    except ET.ParseError as e:
        logger.error(f"Failed to parse URDF: {e}")
        return

    _extract_links_yaml(root, os.path.join(config_dir, "links.yaml"))
    _extract_colors_yaml(root, os.path.join(config_dir, "colors.yaml"))
    _extract_scale_yaml(root, os.path.join(config_dir, "scale.yaml"))
    _extract_limits_yaml(root, os.path.join(config_dir, "limits.yaml"))
    _extract_calibration_yaml(root, os.path.join(config_dir, "calibration.yaml"))
    _extract_dynamics_yaml(root, os.path.join(config_dir, "dynamics.yaml"))

    logger.info(f"Config files generated in: {config_dir}")


def _extract_links_yaml(root: ET.Element, output_path: str):
    """Extract link inertial data to links.yaml."""
    lines = []
    for link in root.findall("link"):
        name = link.get("name", "")
        inertial = link.find("inertial")
        if inertial is None:
            continue

        mass_elem = inertial.find("mass")
        mass = float(mass_elem.get("value", 0)) if mass_elem is not None else 0

        inertia_elem = inertial.find("inertia")
        inertia = {}
        if inertia_elem is not None:
            for key in INERTIA_KEYS:
                val = inertia_elem.get(key, "0")
                # Format with appropriate precision: use scientific for small values
                fval = float(val)
                inertia[key] = fval

        lines.append(f"{name}:")
        lines.append("  inertia:")
        for key in INERTIA_KEYS:
            v = inertia.get(key, 0.0)
            lines.append(f"    {key}: {_fmt_float(v)}")
        lines.append(f"  mass: {_fmt_float(mass)}")

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    logger.info(f"Generated: {output_path} ({len(lines)} lines)")


def _extract_colors_yaml(root: ET.Element, output_path: str):
    """Extract link visual colors to colors.yaml.

    Format: 'link_name: r g b a'  (space-separated, 0-1 range)
    """
    lines = []
    for link in root.findall("link"):
        name = link.get("name", "")
        visual = link.find("visual")
        if visual is None:
            continue
        material = visual.find("material")
        if material is None:
            continue
        color = material.find("color")
        if color is None:
            continue
        rgba = color.get("rgba", "")
        if rgba:
            # Normalize: ensure 4 space-separated floats
            parts = rgba.strip().split()
            if len(parts) == 4:
                lines.append(f"{name}: {rgba.strip()}")

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    logger.info(f"Generated: {output_path} ({len(lines)} links)")


def _extract_scale_yaml(root: ET.Element, output_path: str):
    """Generate scale.yaml with per-material density configuration.

    Extracts all unique material names from the URDF and lists each
    with a default density of 1.0 — users edit these values to match
    their physical materials. Links without a material use 'default'.
    """
    material_names = set()
    for link in root.findall("link"):
        for child in link:
            if child.tag in ("visual", "collision"):
                material = child.find("material")
                if material is not None:
                    mat_name = material.get("name")
                    if mat_name:
                        material_names.add(mat_name)

    lines = [
        "# Density configuration",
        "# Each material found in the URDF is listed below.",
        "# Edit the values to match your physical materials.",
        "# Links without a material use 'default'.",
        "density:",
        "  default: 1.0",
    ]
    for name in sorted(material_names):
        lines.append(f"  {name}: 1.0")

    content = "\n".join(lines) + "\n"
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(content)
    logger.info(f"Generated: {output_path} ({len(material_names)} material(s) + default)")


def _extract_limits_yaml(root: ET.Element, output_path: str):
    """Extract joint limits to limits.yaml."""
    lines = []
    for joint in root.findall("joint"):
        name = joint.get("name", "")
        limit = joint.find("limit")
        if limit is None:
            continue

        effort = float(limit.get("effort", 0))
        velocity = float(limit.get("velocity", 0))
        lower = float(limit.get("lower", 0))
        upper = float(limit.get("upper", 0))

        lines.append(f"{name}:")
        lines.append(f"  effort: {_fmt_float(effort)}")
        lines.append(f"  lower: {_fmt_float(lower)}")
        lines.append(f"  upper: {_fmt_float(upper)}")
        lines.append(f"  velocity: {_fmt_float(velocity)}")

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    logger.info(f"Generated: {output_path} ({len(lines)} lines)")


def _extract_calibration_yaml(root: ET.Element, output_path: str):
    """Extract joint calibration data to calibration.yaml."""
    lines = []
    for joint in root.findall("joint"):
        name = joint.get("name", "")
        calib = joint.find("calibration")
        if calib is None:
            continue
        rising = calib.get("rising", "0")
        falling = calib.get("falling", "0")
        lines.append(f"{name}:")
        lines.append(f"  rising: {rising}")
        lines.append(f"  falling: {falling}")

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    logger.info(f"Generated: {output_path} ({len(lines)} lines)")


def _extract_dynamics_yaml(root: ET.Element, output_path: str):
    """Extract joint dynamics data to dynamics.yaml."""
    lines = []
    for joint in root.findall("joint"):
        name = joint.get("name", "")
        dyn = joint.find("dynamics")
        if dyn is None:
            continue
        damping = dyn.get("damping", "0")
        friction = dyn.get("friction", "0")
        lines.append(f"{name}:")
        lines.append(f"  damping: {damping}")
        lines.append(f"  friction: {friction}")

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    logger.info(f"Generated: {output_path} ({len(lines)} lines)")


def _fmt_float(v: float) -> str:
    """Format a float value for YAML output."""
    if v == 0.0:
        return "0.0"
    # Use repr for very small or very large numbers
    if abs(v) < 1e-6 or abs(v) >= 1e6:
        return repr(v)
    # Otherwise use minimal precision
    s = f"{v:.15g}"
    if "." not in s and "e" not in s.lower():
        s += ".0"
    return s
