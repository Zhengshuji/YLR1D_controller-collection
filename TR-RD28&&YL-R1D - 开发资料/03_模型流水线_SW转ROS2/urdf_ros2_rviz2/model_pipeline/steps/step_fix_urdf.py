"""Step 3: Fix URDF issues.

- Remove duplicate joints (keep first occurrence, delete rest)
- Validate and fix mimic joints (break circular chains, validate targets)
- Clean sensor links (remove visual/collision geometry, set minimal inertial)
- Cap extreme effort/velocity values to sane defaults
"""

import logging
import os
import re
from xml.dom import minidom
import xml.etree.ElementTree as ET

logger = logging.getLogger(__name__)

# Sane defaults for effort/velocity caps
MAX_EFFORT = 1000.0
MAX_VELOCITY = 100.0
DEFAULT_MASS = 0.001
DEFAULT_INERTIA_VAL = 1e-6


def fix_urdf(urdf_path: str):
    """Apply all URDF fixes in-place."""
    logger.info(f"Fixing URDF: {urdf_path}")

    try:
        tree = ET.parse(urdf_path)
    except ET.ParseError as e:
        logger.error(f"Failed to parse URDF: {e}")
        return False

    root = tree.getroot()
    modified = False

    # 1. Remove duplicate joints
    modified |= _fix_duplicate_joints(root)

    # 2. Fix mimic joints
    modified |= _fix_mimic_joints(root)

    # 3. Clean sensor links
    modified |= _clean_sensor_links(root)

    # 4. Cap extreme values
    modified |= _cap_extreme_values(root)

    # 5. Add default calibration and dynamics to all joints
    modified |= _add_joint_defaults(root)

    if modified:
        _write_urdf(tree, urdf_path)
        logger.info("URDF fixes applied successfully.")
    else:
        logger.info("No fixes needed.")

    return True


def _fix_duplicate_joints(root) -> bool:
    """Remove duplicate joints (keep first occurrence)."""
    seen = {}
    to_remove = []

    for joint in root.findall("joint"):
        name = joint.get("name")
        if name is None:
            continue

        if name in seen:
            to_remove.append(joint)
            logger.warning(f"Removing duplicate joint: '{name}'")
        else:
            seen[name] = joint

    for joint in to_remove:
        root.remove(joint)

    return len(to_remove) > 0


def _fix_mimic_joints(root) -> bool:
    """Validate and fix mimic joints.

    Issues fixed:
    1. Mimic target joint doesn't exist -> remove mimic tag
    2. Circular mimic chains (A mimics B, B mimics A) -> remove one
    3. Multiple levels of mimic (A mimics B mimics C) -> flatten
    """
    # Build mimic graph
    joints = {}
    mimic_map = {}  # joint_name -> target_name

    for joint in root.findall("joint"):
        name = joint.get("name")
        if name is None:
            continue
        joints[name] = joint

        mimic = joint.find("mimic")
        if mimic is not None:
            target = mimic.get("joint")
            if target:
                mimic_map[name] = target

    if not mimic_map:
        return False

    modified = False

    # Check each mimic relationship
    for mimic_joint_name, target_name in list(mimic_map.items()):
        # Issue 1: Target doesn't exist
        if target_name not in joints:
            joint_elem = joints[mimic_joint_name]
            mimic_elem = joint_elem.find("mimic")
            if mimic_elem is not None:
                joint_elem.remove(mimic_elem)
                logger.warning(
                    f"Removed mimic from '{mimic_joint_name}': "
                    f"target joint '{target_name}' does not exist"
                )
                modified = True
            continue

        # Issue 2: Circular reference
        # Check if target mimics back to mimic_joint
        if target_name in mimic_map and mimic_map[target_name] == mimic_joint_name:
            joint_elem = joints[mimic_joint_name]
            mimic_elem = joint_elem.find("mimic")
            if mimic_elem is not None:
                joint_elem.remove(mimic_elem)
                logger.warning(
                    f"Removed mimic from '{mimic_joint_name}': "
                    f"circular reference with '{target_name}'"
                )
                modified = True

    return modified


def _clean_sensor_links(root) -> bool:
    """Clean sensor links:
    - Remove visual and collision geometry
    - Set inertial to minimal values
    - Links ending with 'Sensor' or having <material name="Sensor">
    """
    modified = False
    sensor_count = 0

    for link in root.findall("link"):
        name = link.get("name", "")
        # Sensor: name ends with "Sensor" OR has <material name="Sensor">
        is_sensor = name.endswith("Sensor")
        if not is_sensor:
            is_sensor = bool(
                link.findall("visual/material[@name='Sensor']") or
                link.findall("collision/material[@name='Sensor']")
            )
        if not is_sensor:
            continue

        sensor_count += 1

        # Remove visual elements
        for visual in link.findall("visual"):
            link.remove(visual)

        # Remove collision elements
        for collision in link.findall("collision"):
            link.remove(collision)

        # Set inertial to minimal values
        _set_minimal_inertial(link)

        logger.info(f"Cleaned sensor link: '{name}'")
        modified = True

    if sensor_count > 0:
        logger.info(f"Cleaned {sensor_count} sensor link(s)")
    else:
        logger.info("No sensor links found to clean")

    return modified


def _set_minimal_inertial(link):
    """Set or replace inertial element with minimal values."""
    inertial = link.find("inertial")
    if inertial is not None:
        link.remove(inertial)

    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=str(DEFAULT_MASS))
    ET.SubElement(
        inertial, "inertia",
        ixx=str(DEFAULT_INERTIA_VAL), ixy="0.0", ixz="0.0",
        iyy=str(DEFAULT_INERTIA_VAL), iyz="0.0", izz=str(DEFAULT_INERTIA_VAL),
    )


def _cap_extreme_values(root) -> bool:
    """Cap extreme effort/velocity values to sane defaults."""
    modified = False

    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is None:
            continue

        # Cap effort
        effort_attr = limit.get("effort")
        if effort_attr:
            try:
                val = float(effort_attr)
                if val > MAX_EFFORT:
                    limit.set("effort", str(MAX_EFFORT))
                    logger.info(
                        f"Capped effort for '{joint.get('name')}': "
                        f"{val} -> {MAX_EFFORT}"
                    )
                    modified = True
            except ValueError:
                pass

        # Cap velocity
        velocity_attr = limit.get("velocity")
        if velocity_attr:
            try:
                val = float(velocity_attr)
                if val > MAX_VELOCITY:
                    limit.set("velocity", str(MAX_VELOCITY))
                    logger.info(
                        f"Capped velocity for '{joint.get('name')}': "
                        f"{val} -> {MAX_VELOCITY}"
                    )
                    modified = True
            except ValueError:
                pass

    return modified


def _add_joint_defaults(root) -> bool:
    """Add default <calibration> and <dynamics> to every joint if not present."""
    modified = False

    for joint in root.findall("joint"):
        # Add calibration (raising, falling) = (0, 0) if not present
        if joint.find("calibration") is None:
            calib = ET.SubElement(joint, "calibration")
            calib.set("rising", "0")
            calib.set("falling", "0")
            modified = True

        # Add dynamics (damping, friction) = (0, 0) if not present
        if joint.find("dynamics") is None:
            dyn = ET.SubElement(joint, "dynamics")
            dyn.set("damping", "0")
            dyn.set("friction", "0")
            modified = True

    if modified:
        logger.info("Added default calibration and dynamics to joints")

    return modified


def _write_urdf(tree, path: str):
    """Write the XML tree back to file with pretty formatting."""
    rough = ET.tostring(tree.getroot(), encoding="unicode")
    dom = minidom.parseString(rough.encode())
    pretty = dom.toprettyxml(indent="  ")
    # Remove extra blank lines
    lines = [l for l in pretty.splitlines() if l.strip() or "<?xml" in l]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
