"""Step 5: Add Gazebo support with ROS2 plugins.

For each sensor defined in sensors.yaml, adds:
1. <gazebo reference="link_name"> tag with sensor configuration
2. Proper ROS2 plugin for the sensor type (libgazebo_ros_*.so)

Also adds basic robot-level plugins (e.g., diff drive for wheeled robots).
"""

import logging
import os
import re
import xml.etree.ElementTree as ET
from xml.dom import minidom
from typing import Optional

from model_pipeline.steps.step_sensors import _extract_core_name

logger = logging.getLogger(__name__)

# Xacro namespace
XACRO_NS = "http://wiki.ros.org/xacro"
ET.register_namespace("xacro", XACRO_NS)

# ROS2 Gazebo plugin mappings
#
# Camera (libgazebo_ros_camera.so):
#   Handles both camera and depth SDF types. Uses explicit topic from <camera_name>.
#
# Depth: Uses SDF <sensor type="depth"> with same plugin libgazebo_ros_camera.so.
#   ROS2 Humble has NO separate libgazebo_ros_depth_camera.so — the camera plugin
#   detects SDF type "depth" and adds point cloud output automatically.
#
# IMU (libgazebo_ros_imu_sensor.so):
#   Publishes on hardcoded ~/out topic. Use <ros><remapping> to redirect.
#
# Ray/Lidar (libgazebo_ros_ray_sensor.so):
#   Publishes on hardcoded ~/out. Use <ros><remapping> to redirect.
#   Sonar/ultrasonic sensors use this plugin with single-beam ray config.
#
SENSOR_PLUGINS = {
    "camera": {
        "filename": "libgazebo_ros_camera.so",
        "params": [
            ("camera_name", "{camera_name}"),
            ("cameraName", "{camera_name}"),
            ("image_topic_name", "image_raw"),
            ("imageTopicName", "image_raw"),
            ("camera_info_topic_name", "camera_info"),
            ("cameraInfoTopicName", "camera_info"),
            ("frame_name", "{link_name}"),
            ("frameName", "{link_name}"),
        ],
    },
    "depth": {
        # Same plugin as camera, just different SDF type
        "filename": "libgazebo_ros_camera.so",
        "params": [
            ("camera_name", "{camera_name}"),
            ("cameraName", "{camera_name}"),
            ("image_topic_name", "image_raw"),
            ("imageTopicName", "image_raw"),
            ("camera_info_topic_name", "camera_info"),
            ("cameraInfoTopicName", "camera_info"),
            ("point_cloud_topic_name", "points"),
            ("pointCloudTopicName", "points"),
            ("frame_name", "{link_name}"),
            ("frameName", "{link_name}"),
        ],
    },
    "imu": {
        "filename": "libgazebo_ros_imu_sensor.so",
        "remap": "~/out:={topic}",
        "params": [],
    },
    "ray": {
        "filename": "libgazebo_ros_ray_sensor.so",
        "remap": "~/out:={topic}",
        "params": [
            ("output_type", "sensor_msgs/LaserScan"),
        ],
    },
}


def add_gazebo_support(
    urdf_path: str,
    sensors_yaml: str,
    output_urdf: str,
    xacro_path: Optional[str] = None,
    output_xacro: Optional[str] = None,
):
    """Add Gazebo support to URDF and optionally to Xacro.

    Args:
        urdf_path: Path to cleaned URDF file
        sensors_yaml: Path to sensors.yaml configuration
        output_urdf: Output path for URDF with Gazebo tags
        xacro_path: Path to the generated Xacro file (if exists)
        output_xacro: Output path for Xacro with Gazebo tags (if xacro_path given)
    """
    # Read sensors config
    sensors = _read_sensors_yaml(sensors_yaml)

    # Add to URDF
    logger.info("Adding Gazebo support to URDF...")
    _add_gazebo_to_file(urdf_path, output_urdf, sensors)

    # Add to Xacro if requested
    if xacro_path and output_xacro:
        logger.info("Adding Gazebo support to Xacro...")
        _add_gazebo_to_xacro(xacro_path, output_xacro, sensors)

    logger.info(
        f"Gazebo support added: {output_urdf}"
        + (f", {output_xacro}" if output_xacro else "")
    )


def _read_sensors_yaml(yaml_path: str) -> list:
    """Read sensors from YAML file."""
    if not os.path.exists(yaml_path):
        logger.warning(f"Sensors YAML not found: {yaml_path}")
        return []

    try:
        import yaml

        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return data.get("sensors", [])
    except ImportError:
        logger.warning("PyYAML not available, returning empty sensor list")
        return []
    except Exception as e:
        logger.error(f"Error reading sensors YAML: {e}")
        return []


def _add_gazebo_to_file(
    input_path: str,
    output_path: str,
    sensors: list,
):
    """Add Gazebo sensor tags with plugins to a URDF file (standard XML).

    Groups multiple sensors on the same link into a single <gazebo> element.
    """
    try:
        tree = ET.parse(input_path)
    except ET.ParseError as e:
        logger.error(f"Failed to parse {input_path}: {e}")
        return

    root = tree.getroot()

    # Collect existing link names and gazebo references
    link_names = [link.get("name") for link in root.findall(".//link")]
    existing_refs = set()
    for gz in root.findall(".//gazebo"):
        ref = gz.get("reference")
        if ref is not None:
            existing_refs.add(ref)

    # Group sensors by link_name
    from collections import OrderedDict
    groups = OrderedDict()
    for sensor in sensors:
        link_name = sensor.get("link_name", "")
        if not link_name:
            continue
        if link_name not in link_names:
            logger.debug(f"Skipping sensor: link '{link_name}' not found in model")
            continue
        if link_name not in groups:
            groups[link_name] = []
        groups[link_name].append(sensor)

    added_count = 0
    for link_name, link_sensors in groups.items():
        # Skip if this link already has a <gazebo reference> (user manually added)
        if link_name in existing_refs:
            logger.info(f"  Gazebo tag exists for '{link_name}', skipping")
            continue

        gz_elem = _create_gazebo_sensors_group(link_sensors)
        root.append(gz_elem)
        added_count += len(link_sensors)
        types = [s.get("type", "?") for s in link_sensors]
        logger.info(f"  Added Gazebo sensors for '{link_name}': {', '.join(types)}")

    if added_count == 0:
        logger.info("  No new sensors added")

    # Also add a generic robot-level gazebo element
    _add_robot_gazebo(root, link_names)

    # Write output
    _write_xml(root, output_path)


def _add_gazebo_to_xacro(
    input_path: str,
    output_path: str,
    sensors: list,
):
    """Add Gazebo sensor tags with plugins to a Xacro file.

    Groups multiple sensors on the same link into a single <gazebo> element.
    Gazebo elements are inserted INSIDE the xacro:macro (before </xacro:macro>).
    """
    with open(input_path, "r", encoding="utf-8") as f:
        content = f.read()

    # Find links inside the xacro to validate
    link_pattern = re.compile(r'<link\s+name="(\$\{prefix\}[^"]+)"')
    xacro_link_names = link_pattern.findall(content)

    # Group sensors by link_name
    from collections import OrderedDict
    groups = OrderedDict()
    for sensor in sensors:
        link_name = sensor.get("link_name", "")
        if not link_name:
            continue
        prefixed = f"${{prefix}}{link_name}"
        if prefixed not in xacro_link_names:
            logger.debug(f"Skipping sensor: link '{link_name}' not found in xacro")
            continue
        if link_name not in groups:
            groups[link_name] = []
        groups[link_name].append(sensor)

    added = []
    for link_name, link_sensors in groups.items():
        gz_xml = _create_gazebo_sensors_group_xml_text(link_sensors, use_prefix=True)
        added.append(gz_xml)
        types = [s.get("type", "?") for s in link_sensors]
        logger.info(f"  Added Gazebo sensors for '{link_name}': {', '.join(types)}")

    if added:
        gazebo_block = "\n".join(added)
        content = content.replace("</xacro:macro>", f"{gazebo_block}\n  </xacro:macro>")
        logger.info(f"  Inserted {len(added)} gazebo block(s) inside xacro:macro")
    else:
        logger.info("  No new sensors added")

    # Add robot-level gazebo if not present
    macro_section = content.split("</xacro:macro>")[0] if "</xacro:macro>" in content else content
    if '<gazebo>' not in macro_section:
        robot_gz = (
            '  <gazebo>\n'
            '    <visualise_children>true</visualise_children>\n'
            '  </gazebo>'
        )
        content = content.replace("</xacro:macro>", f"{robot_gz}\n  </xacro:macro>")
        logger.info("  Added robot-level <gazebo> element inside xacro:macro")

    with open(output_path, "w", encoding="utf-8") as f:
        f.write(content)


def _create_gazebo_sensor_xml_text(sensor: dict, use_prefix: bool = False) -> str:
    """Generate gazebo sensor XML as text (for xacro files).

    Uses _extract_core_name() to derive a clean topic-friendly name
    from the link name (e.g. 'Link_GlobalCameraSensor' -> 'global_camera').
    """
    link_name = sensor.get("link_name", "")
    sensor_type = sensor.get("type", "generic")
    # Remap: libgazebo_ros_sonar.so doesn't exist in ROS2 — use ray sensor SDF
    if sensor_type == "sonar":
        sensor_type = "ray"
    ref_name = f"${{prefix}}{link_name}" if use_prefix else link_name
    core_name = _extract_core_name(link_name)
    sensor_name = f"{core_name}_sensor"
    update_rate = sensor.get("update_rate", 10)
    topic = sensor.get("topic", f"/{sensor_type}/data")

    lines = [f'  <gazebo reference="{ref_name}">']
    lines.append(f'    <sensor type="{sensor_type}" name="{sensor_name}">')
    # Sensor-level params
    lines.append("      <always_on>true</always_on>")
    lines.append(f"      <update_rate>{update_rate}</update_rate>")
    lines.append(f"      <topic>{topic}</topic>")
    lines.append(f"      <frame_name>{ref_name}</frame_name>")

    if sensor_type == "camera":
        cam = sensor.get("camera", {})
        lines.append("      <camera>")
        lines.append(f"        <horizontal_fov>{cam.get('horizontal_fov', 1.0472)}</horizontal_fov>")
        lines.append("        <image>")
        lines.append(f"          <width>{cam.get('image', {}).get('width', 640)}</width>")
        lines.append(f"          <height>{cam.get('image', {}).get('height', 480)}</height>")
        lines.append("        </image>")
        lines.append("        <clip>")
        lines.append(f"          <near>{cam.get('clip', {}).get('near', 0.1)}</near>")
        lines.append(f"          <far>{cam.get('clip', {}).get('far', 100.0)}</far>")
        lines.append("        </clip>")
        lines.append("      </camera>")
    elif sensor_type == "ray":
        ray = sensor.get("ray", {})
        lines.append("      <ray>")
        lines.append("        <scan>")
        lines.append("          <horizontal>")
        hz = ray.get("scan", {}).get("horizontal", {})
        lines.append(f"            <samples>{hz.get('samples', 360)}</samples>")
        lines.append(f"            <resolution>{hz.get('resolution', 1.0)}</resolution>")
        lines.append(f"            <min_angle>{hz.get('min_angle', -3.14159)}</min_angle>")
        lines.append(f"            <max_angle>{hz.get('max_angle', 3.14159)}</max_angle>")
        lines.append("          </horizontal>")
        lines.append("        </scan>")
        lines.append("        <range>")
        rng = ray.get("range", {})
        lines.append(f"          <min>{rng.get('min', 0.1)}</min>")
        lines.append(f"          <max>{rng.get('max', 30.0)}</max>")
        lines.append(f"          <resolution>{rng.get('resolution', 0.01)}</resolution>")
        lines.append("        </range>")
        lines.append("      </ray>")

    # Add noise
    noise = sensor.get("noise", {})
    if noise:
        lines.append(f'      <noise type="{noise.get("type", "gaussian")}">')
        lines.append(f"        <mean>{noise.get('mean', 0.0)}</mean>")
        lines.append(f"        <stddev>{noise.get('stddev', 0.01)}</stddev>")
        lines.append("      </noise>")

    # Add plugin
    plugin_info = SENSOR_PLUGINS.get(sensor_type)
    if plugin_info:
        lines.append(
            f'      <plugin name="{core_name}_{sensor_type}_plugin" '
            f'filename="{plugin_info["filename"]}">'
        )

        # Topic remapping (for plugins with hardcoded ~/out topic)
        if "remap" in plugin_info:
            remap_text = plugin_info["remap"].format(topic=topic)
            lines.append("        <ros>")
            lines.append(f"          <remapping>{remap_text}</remapping>")
            lines.append("        </ros>")

        # Plugin-specific params
        cam_ns = _camera_ns_from_topic(topic) if sensor_type in ("camera", "depth") else core_name
        for key, val_template in plugin_info.get("params", []):
            val = val_template.format(
                link_name=ref_name if use_prefix else link_name,
                topic=topic,
                update_rate=update_rate,
                camera_name=cam_ns,
            )
            lines.append(f"        <{key}>{val}</{key}>")

        lines.append("      </plugin>")

    lines.append("    </sensor>")
    lines.append("  </gazebo>")
    return "\n".join(lines)


def _create_gazebo_sensor(sensor: dict) -> ET.Element:
    """Create a <gazebo> element with sensor config and ROS2 plugin.

    Uses _extract_core_name() to derive a clean topic-friendly name
    from the link name (e.g. 'Link_GlobalCameraSensor' -> 'global_camera').

    Structure follows official gazebo_ros_pkgs test worlds:
      <sensor> level: always_on, update_rate, topic, frame_name
      <plugin> level: type-specific params + <ros><remapping> for topic redirect
    """
    link_name = sensor.get("link_name", "")
    sensor_type = sensor.get("type", "generic")
    # Remap: libgazebo_ros_sonar.so doesn't exist in ROS2 — use ray sensor SDF
    if sensor_type == "sonar":
        sensor_type = "ray"
    core_name = _extract_core_name(link_name)
    sensor_name = f"{core_name}_sensor"
    update_rate = sensor.get("update_rate", 10)
    topic = sensor.get("topic", f"/{sensor_type}/data")

    gz = ET.Element("gazebo", reference=link_name)
    sensor_elem = ET.SubElement(gz, "sensor", type=sensor_type, name=sensor_name)

    # Sensor-level params (read by sensor system and SensorFrameID)
    ET.SubElement(sensor_elem, "always_on").text = "true"
    ET.SubElement(sensor_elem, "update_rate").text = str(update_rate)
    ET.SubElement(sensor_elem, "topic").text = topic
    ET.SubElement(sensor_elem, "frame_name").text = link_name

    # Add type-specific configuration
    if sensor_type == "camera":
        cam_config = sensor.get("camera", {})
        cam_elem = ET.SubElement(sensor_elem, "camera")
        ET.SubElement(cam_elem, "horizontal_fov").text = str(
            cam_config.get("horizontal_fov", 1.0472)
        )
        img = ET.SubElement(cam_elem, "image")
        ET.SubElement(img, "width").text = str(cam_config.get("image", {}).get("width", 640))
        ET.SubElement(img, "height").text = str(cam_config.get("image", {}).get("height", 480))
        clip = ET.SubElement(cam_elem, "clip")
        ET.SubElement(clip, "near").text = str(cam_config.get("clip", {}).get("near", 0.1))
        ET.SubElement(clip, "far").text = str(cam_config.get("clip", {}).get("far", 100.0))

    elif sensor_type == "ray":
        ray_config = sensor.get("ray", {})
        ray_elem = ET.SubElement(sensor_elem, "ray")
        scan = ET.SubElement(ray_elem, "scan")
        hz = ET.SubElement(scan, "horizontal")
        hz_config = ray_config.get("scan", {}).get("horizontal", {})
        ET.SubElement(hz, "samples").text = str(hz_config.get("samples", 360))
        ET.SubElement(hz, "resolution").text = str(hz_config.get("resolution", 1.0))
        ET.SubElement(hz, "min_angle").text = str(hz_config.get("min_angle", -3.14159))
        ET.SubElement(hz, "max_angle").text = str(hz_config.get("max_angle", 3.14159))

        rng = ET.SubElement(ray_elem, "range")
        rng_config = ray_config.get("range", {})
        ET.SubElement(rng, "min").text = str(rng_config.get("min", 0.1))
        ET.SubElement(rng, "max").text = str(rng_config.get("max", 30.0))
        ET.SubElement(rng, "resolution").text = str(rng_config.get("resolution", 0.01))

    # Add noise if configured
    noise_config = sensor.get("noise", {})
    if noise_config:
        noise_elem = ET.SubElement(sensor_elem, "noise", type=noise_config.get("type", "gaussian"))
        ET.SubElement(noise_elem, "mean").text = str(noise_config.get("mean", 0.0))
        ET.SubElement(noise_elem, "stddev").text = str(noise_config.get("stddev", 0.01))

    # Add ROS2 plugin (sensor_type already remapped sonar→ray in this function)
    plugin_info = SENSOR_PLUGINS.get(sensor_type)
    if plugin_info:
        # Include sensor_type in plugin name for uniqueness across sensors on same link
        plugin = ET.SubElement(sensor_elem, "plugin",
            name=f"{core_name}_{sensor_type}_plugin",
            filename=plugin_info["filename"],
        )

        # Topic remapping (for plugins with hardcoded ~/out topic)
        if "remap" in plugin_info:
            ros = ET.SubElement(plugin, "ros")
            remap_text = plugin_info["remap"].format(topic=topic)
            ET.SubElement(ros, "remapping").text = remap_text

        # Plugin-specific params
        cam_ns = _camera_ns_from_topic(topic) if sensor_type in ("camera", "depth") else core_name
        for key, val_template in plugin_info.get("params", []):
            val = val_template.format(
                link_name=link_name,
                topic=topic,
                update_rate=update_rate,
                camera_name=cam_ns,
            )
            ET.SubElement(plugin, key).text = val

    return gz


def _camera_ns_from_topic(topic: str) -> str:
    """Extract camera namespace from a full topic path.

    The gazebo_ros_camera plugin publishes to
    ``/{camera_ns}/{image_topic_name}``, so we derive the camera
    namespace by stripping the last path segment from the configured topic.

    Examples:
        /global_camera/rgb/image_raw  ->  global_camera/rgb
        /camera/image_raw              ->  camera
        /rgb/image_raw                 ->  rgb
    """
    parts = topic.strip("/").rsplit("/", 1)
    return parts[0] if len(parts) > 1 else parts[0]


def _sonar_to_ray_sdf(sonar_config: dict) -> dict:
    """Convert sonar config to ray SDF params.

    libgazebo_ros_sonar.so doesn't exist in ROS2, so simulate sonar
    as single-beam ray sensors using libgazebo_ros_ray_sensor.so.
    """
    radius = float(sonar_config.get("radius", 0.5236))
    return {
        "scan": {
            "horizontal": {
                "samples": 1,
                "resolution": 1,
                "min_angle": -radius,
                "max_angle": radius,
            },
        },
        "range": {
            "min": float(sonar_config.get("min", 0.25)),
            "max": float(sonar_config.get("max", 4.5)),
            "resolution": 0.01,
        },
    }


def _create_gazebo_sensors_group(sensors: list) -> ET.Element:
    """Create a <gazebo> element with multiple <sensor> children for the same link.

    Groups multiple sensors (e.g. RGB + Depth + Infrared) sharing one link
    into a single <gazebo reference="link_name"> block.
    """
    if not sensors:
        return None

    link_name = sensors[0].get("link_name", "")
    gz = ET.Element("gazebo", reference=link_name)

    for sensor in sensors:
        sensor_type = sensor.get("type", "generic")
        sensor_name = sensor.get("sensor_name", f"{sensor_type}_sensor")

        # Determine the actual SDF type
        sdf_type = sensor_type
        if sdf_type == "sonar":
            sdf_type = "ray"

        sensor_elem = ET.SubElement(gz, "sensor", type=sdf_type, name=sensor_name)

        _populate_sensor_elem(sensor_elem, sensor, sensor_type, link_name)

    return gz


def _create_gazebo_sensors_group_xml_text(sensors: list, use_prefix: bool = False) -> str:
    """Generate <gazebo> XML text with multiple sensors for the same link (for Xacro)."""
    if not sensors:
        return ""

    link_name = sensors[0].get("link_name", "")
    ref_name = f"${{prefix}}{link_name}" if use_prefix else link_name

    lines = [f'  <gazebo reference="{ref_name}">']

    for sensor in sensors:
        sensor_type = sensor.get("type", "generic")
        sensor_name = sensor.get("sensor_name", f"{sensor_type}_sensor")
        sdf_type = "ray" if sensor_type == "sonar" else sensor_type

        lines.append(f'    <sensor type="{sdf_type}" name="{sensor_name}">')
        lines.append("      <always_on>true</always_on>")
        lines.append(f"      <update_rate>{sensor.get('update_rate', 10)}</update_rate>")
        lines.append(f"      <topic>{sensor.get('topic', '/unknown')}</topic>")
        lines.append(f"      <frame_name>{ref_name}</frame_name>")

        _populate_sensor_xml_lines(lines, sensor, sensor_type, ref_name, use_prefix)

        lines.append("    </sensor>")

    lines.append("  </gazebo>")
    return "\n".join(lines)


def _get_camera_or_sonar_config(sensor: dict, sensor_type: str) -> dict:
    """Get the type-specific config dict (camera/ray/sonar)."""
    if sensor_type == "sonar":
        return _sonar_to_ray_sdf(sensor.get("sonar", {}))
    # Both "camera" and "depth" types store camera config under the "camera" key
    return sensor.get("camera", sensor.get(sensor_type, {}))


def _populate_sensor_elem(sensor_elem: ET.Element, sensor: dict, sensor_type: str, link_name: str):
    """Populate an ET <sensor> element with type-specific config and plugin."""
    update_rate = sensor.get("update_rate", 10)
    topic = sensor.get("topic", "/unknown")
    core_name = sensor.get("sensor_name", _extract_core_name(link_name))
    link_core = _extract_core_name(link_name)

    ET.SubElement(sensor_elem, "always_on").text = "true"
    ET.SubElement(sensor_elem, "update_rate").text = str(update_rate)
    ET.SubElement(sensor_elem, "topic").text = topic
    ET.SubElement(sensor_elem, "frame_name").text = link_name

    # Add type-specific configuration
    sdf_type = "ray" if sensor_type == "sonar" else sensor_type

    if sdf_type in ("camera", "depth"):
        cam_config = _get_camera_or_sonar_config(sensor, sensor_type)
        cam_elem = ET.SubElement(sensor_elem, "camera")
        ET.SubElement(cam_elem, "horizontal_fov").text = str(cam_config.get("horizontal_fov", 1.0472))
        img = ET.SubElement(cam_elem, "image")
        img_config = cam_config.get("image", {})
        ET.SubElement(img, "width").text = str(img_config.get("width", 640))
        ET.SubElement(img, "height").text = str(img_config.get("height", 480))
        if "format" in img_config:
            ET.SubElement(img, "format").text = img_config["format"]
        clip = ET.SubElement(cam_elem, "clip")
        ET.SubElement(clip, "near").text = str(cam_config.get("clip", {}).get("near", 0.1))
        ET.SubElement(clip, "far").text = str(cam_config.get("clip", {}).get("far", 100.0))

    elif sdf_type == "ray":
        ray_config = _get_camera_or_sonar_config(sensor, sensor_type)
        ray_elem = ET.SubElement(sensor_elem, "ray")
        scan = ET.SubElement(ray_elem, "scan")
        hz = ET.SubElement(scan, "horizontal")
        hz_config = ray_config.get("scan", {}).get("horizontal", {})
        ET.SubElement(hz, "samples").text = str(hz_config.get("samples", 360))
        ET.SubElement(hz, "resolution").text = str(hz_config.get("resolution", 1.0))
        ET.SubElement(hz, "min_angle").text = str(hz_config.get("min_angle", -3.14159))
        ET.SubElement(hz, "max_angle").text = str(hz_config.get("max_angle", 3.14159))
        rng = ET.SubElement(ray_elem, "range")
        rng_config = ray_config.get("range", {})
        ET.SubElement(rng, "min").text = str(rng_config.get("min", 0.1))
        ET.SubElement(rng, "max").text = str(rng_config.get("max", 30.0))
        ET.SubElement(rng, "resolution").text = str(rng_config.get("resolution", 0.01))

    # Add noise if configured
    noise_config = sensor.get("noise", {})
    if noise_config:
        noise_elem = ET.SubElement(sensor_elem, "noise", type=noise_config.get("type", "gaussian"))
        ET.SubElement(noise_elem, "mean").text = str(noise_config.get("mean", 0.0))
        ET.SubElement(noise_elem, "stddev").text = str(noise_config.get("stddev", 0.01))

    # Add ROS2 plugin (use sdf_type for lookup, since sonar→ray)
    plugin_info = SENSOR_PLUGINS.get(sdf_type)
    if plugin_info:
        plugin = ET.SubElement(sensor_elem, "plugin",
            name=f"{link_core}_{core_name}_plugin",
            filename=plugin_info["filename"],
        )
        if "remap" in plugin_info:
            ros = ET.SubElement(plugin, "ros")
            remap_text = plugin_info["remap"].format(topic=topic)
            ET.SubElement(ros, "remapping").text = remap_text
        # camera_name must match the topic namespace (not sensor_name)
        # so the plugin publishes to the correct ROS topic hierarchy
        cam_ns = _camera_ns_from_topic(topic) if sensor_type in ("camera", "depth") else core_name
        for key, val_template in plugin_info.get("params", []):
            val = val_template.format(
                link_name=link_name,
                topic=topic,
                update_rate=update_rate,
                camera_name=cam_ns,
            )
            ET.SubElement(plugin, key).text = val


def _populate_sensor_xml_lines(lines: list, sensor: dict, sensor_type: str,
                                ref_name: str, use_prefix: bool):
    """Append type-specific XML lines for a sensor (for Xacro text generation)."""
    topic = sensor.get("topic", "/unknown")
    link_name = sensor.get("link_name", "")
    core_name = sensor.get("sensor_name", _extract_core_name(link_name))
    link_core = _extract_core_name(link_name)
    sdf_type = "ray" if sensor_type == "sonar" else sensor_type

    if sdf_type in ("camera", "depth"):
        cam_config = _get_camera_or_sonar_config(sensor, sensor_type)
        lines.append(f"      <camera>")
        lines.append(f"        <horizontal_fov>{cam_config.get('horizontal_fov', 1.0472)}</horizontal_fov>")
        lines.append("        <image>")
        img = cam_config.get("image", {})
        lines.append(f"          <width>{img.get('width', 640)}</width>")
        lines.append(f"          <height>{img.get('height', 480)}</height>")
        if "format" in img:
            lines.append(f"          <format>{img['format']}</format>")
        lines.append("        </image>")
        lines.append("        <clip>")
        lines.append(f"          <near>{cam_config.get('clip', {}).get('near', 0.1)}</near>")
        lines.append(f"          <far>{cam_config.get('clip', {}).get('far', 100.0)}</far>")
        lines.append("        </clip>")
        lines.append("      </camera>")

    elif sdf_type == "ray":
        ray_config = _get_camera_or_sonar_config(sensor, sensor_type)
        lines.append("      <ray>")
        lines.append("        <scan>")
        lines.append("          <horizontal>")
        hz = ray_config.get("scan", {}).get("horizontal", {})
        lines.append(f"            <samples>{hz.get('samples', 360)}</samples>")
        lines.append(f"            <resolution>{hz.get('resolution', 1.0)}</resolution>")
        lines.append(f"            <min_angle>{hz.get('min_angle', -3.14159)}</min_angle>")
        lines.append(f"            <max_angle>{hz.get('max_angle', 3.14159)}</max_angle>")
        lines.append("          </horizontal>")
        lines.append("        </scan>")
        lines.append("        <range>")
        rng = ray_config.get("range", {})
        lines.append(f"          <min>{rng.get('min', 0.1)}</min>")
        lines.append(f"          <max>{rng.get('max', 30.0)}</max>")
        lines.append(f"          <resolution>{rng.get('resolution', 0.01)}</resolution>")
        lines.append("        </range>")
        lines.append("      </ray>")

    # Noise
    noise = sensor.get("noise", {})
    if noise:
        lines.append(f'      <noise type="{noise.get("type", "gaussian")}">')
        lines.append(f"        <mean>{noise.get('mean', 0.0)}</mean>")
        lines.append(f"        <stddev>{noise.get('stddev', 0.01)}</stddev>")
        lines.append("      </noise>")

    # Plugin (use sdf_type for lookup, since sonar→ray)
    plugin_info = SENSOR_PLUGINS.get(sdf_type)
    if plugin_info:
        lines.append(f'      <plugin name="{link_core}_{core_name}_plugin" filename="{plugin_info["filename"]}">')
        if "remap" in plugin_info:
            lines.append("        <ros>")
            lines.append(f'          <remapping>{plugin_info["remap"].format(topic=topic)}</remapping>')
            lines.append("        </ros>")
        cam_ns = _camera_ns_from_topic(topic) if sensor_type in ("camera", "depth") else core_name
        for key, val_template in plugin_info.get("params", []):
            val = val_template.format(
                link_name=ref_name if use_prefix else link_name,
                topic=topic,
                update_rate=sensor.get("update_rate", 10),
                camera_name=cam_ns,
            )
            lines.append(f"        <{key}>{val}</{key}>")
        lines.append("      </plugin>")


def _add_robot_gazebo(root: ET.Element, link_names: list):
    """Add robot-level Gazebo elements if missing."""
    # Check if any generic <gazebo> (without reference) exists
    has_robot_gazebo = any(
        gz.get("reference") is None for gz in root.findall(".//gazebo")
    )
    if has_robot_gazebo:
        return

    # Add a minimal robot-level gazebo
    gz = ET.Element("gazebo")
    ET.SubElement(gz, "visualise_children").text = "true"
    root.append(gz)
    logger.info("  Added robot-level <gazebo> element")


def _write_xml(root: ET.Element, path: str):
    """Write XML tree to file with pretty formatting."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    rough = ET.tostring(root, encoding="unicode")
    dom = minidom.parseString(rough.encode())
    pretty = dom.toprettyxml(indent="  ")
    lines = [l for l in pretty.splitlines() if l.strip() or "<?xml" in l]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
