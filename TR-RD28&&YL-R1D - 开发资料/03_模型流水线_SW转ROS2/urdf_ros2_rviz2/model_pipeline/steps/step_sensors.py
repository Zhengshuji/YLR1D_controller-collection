"""Step 4: Extract sensor information from URDF to YAML configuration.

Finds all links ending with 'Sensor', detects their type
(IMU, Camera, Ray/Radar), and generates a sensors.yaml config file.
"""

import copy
import logging
import os
import re
import xml.etree.ElementTree as ET
from typing import Optional

logger = logging.getLogger(__name__)

# Default sensor configurations (fallback when sensor_descriptions.yaml not provided)
SENSOR_DEFAULTS = {
    "imu": {
        "type": "imu",
        "update_rate": 50,
        "topic": "/imu_data",
        "noise": {"type": "gaussian", "mean": 0.0, "stddev": 0.01},
    },
    "camera": {
        "type": "camera",
        "update_rate": 30,
        "topic": "/camera/image_raw",
        "camera": {
            "horizontal_fov": 1.0472,
            "image": {"width": 640, "height": 480, "format": "R8G8B8"},
            "clip": {"near": 0.1, "far": 100.0},
        },
    },
    "depth": {
        "type": "depth",
        "update_rate": 30,
        "topic": "/camera/depth/image_raw",
        "camera": {
            "horizontal_fov": 1.0472,
            "image": {"width": 640, "height": 480, "format": "R8G8B8"},
            "clip": {"near": 0.2, "far": 4.0},
        },
    },
    "ray": {
        "type": "ray",
        "update_rate": 10,
        "topic": "/scan",
        "ray": {
            "scan": {
                "horizontal": {
                    "samples": 360,
                    "resolution": 1,
                    "min_angle": -3.14159,
                    "max_angle": 3.14159,
                }
            },
            "range": {"min": 0.1, "max": 30.0, "resolution": 0.01},
        },
        "noise": {"type": "gaussian", "mean": 0.0, "stddev": 0.01},
    },
    "sonar": {
        # Note: libgazebo_ros_sonar.so doesn't exist in ROS2.
        # Sonar sensors are converted to single-beam ray sensors at detection time.
        "type": "sonar",
        "update_rate": 20,
        "topic": "/sonar/range",
        "sonar": {"min": 0.25, "max": 4.5, "radius": 0.5236},
    },
}


def _extract_core_name(link_name: str) -> str:
    """Extract core sensor name from a link name.

    Examples:
        Link_LeftCameraSensor -> left_camera
        Link_IMUSensor -> imu
        Link_RadarSensor -> radar
    """
    name = link_name
    if name.startswith("Link_"):
        name = name[5:]
    if name.endswith("Sensor"):
        name = name[:-6]

    # CamelCase to snake_case
    s1 = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    s2 = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s1)
    return s2.lower()


def _detect_sensor_type(link_name: str) -> Optional[str]:
    """Detect sensor type from link name."""
    lower = link_name.lower()
    if "imu" in lower:
        return "imu"
    elif "ultrasonic" in lower or "sonar" in lower or "sonic" in lower:
        # Returns 'sonar' so SENSOR_DEFAULTS['sonar'] applies single-beam ray config.
        # libgazebo_ros_sonar.so doesn't exist in ROS2; step_gazebo converts to ray SDF.
        return "sonar"
    elif "radar" in lower or "lidar" in lower or "laser" in lower:
        return "ray"
    elif "camera" in lower:
        return "camera"
    return None


def _generate_topic(link_name: str, sensor_type: str) -> str:
    """Generate a ROS topic name for the sensor."""
    core = _extract_core_name(link_name)
    if sensor_type == "imu":
        return f"/{core}_data" if core else "/imu_data"
    elif sensor_type == "ray":
        return f"/{core}/scan" if core else "/scan"
    elif sensor_type == "camera":
        return f"/{core}/image_raw" if core else "/camera/image_raw"
    elif sensor_type == "sonar":
        return f"/{core}/range" if core else "/sonar/range"
    return f"/{core}" if core else "/unknown"


def extract_sensors_to_yaml(
    urdf_path: str,
    output_yaml: str,
    sensor_descriptions_path: str = None,
):
    """Extract sensors from URDF or load from sensor_descriptions.yaml.

    Priority:
      1. sensor_descriptions_path (if provided and exists) — primary source
      2. Auto-detect from URDF link names — fallback

    Args:
        urdf_path: Path to the (cleaned) URDF file
        output_yaml: Output path for sensors.yaml
        sensor_descriptions_path: Optional path to sensor_descriptions.yaml
    """
    sensors = []

    if sensor_descriptions_path and os.path.exists(sensor_descriptions_path):
        # Load from sensor_descriptions.yaml (primary source)
        sensors = _load_sensor_descriptions(sensor_descriptions_path)
        logger.info(f"Loaded {len(sensors)} sensor(s) from {sensor_descriptions_path}")
    else:
        # Fallback: auto-detect from URDF
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
        except ET.ParseError as e:
            logger.error(f"Failed to parse URDF: {e}")
            return

        for link in root.findall("link"):
            link_name = link.get("name", "")
            is_sensor = link_name.endswith("Sensor")
            if not is_sensor:
                is_sensor = bool(
                    link.findall("visual/material[@name='Sensor']") or
                    link.findall("collision/material[@name='Sensor']")
                )
            if not is_sensor:
                continue

            sensor_type = _detect_sensor_type(link_name)
            if not sensor_type:
                logger.warning(f"Could not detect sensor type for '{link_name}', skipping")
                continue

            config = dict(SENSOR_DEFAULTS.get(sensor_type, {}))
            config["link_name"] = link_name
            config["sensor_name"] = f"{_extract_core_name(link_name)}_{sensor_type}"
            config["topic"] = _generate_topic(link_name, sensor_type)
            sensors.append(config)
            logger.info(f"Auto-detected sensor: {link_name} -> {sensor_type} ({config['topic']})")

    if not sensors:
        logger.info("No sensor links found")
        _write_yaml({"sensors": []}, output_yaml)
        return

    yaml_data = {"sensors": sensors}
    _write_yaml(yaml_data, output_yaml)
    logger.info(f"Extracted {len(sensors)} sensor(s) to {output_yaml}")

    # Also generate individual per-sensor config files for easy editing
    sensor_config_dir = os.path.join(os.path.dirname(output_yaml), "sensors")
    _write_per_sensor_configs(sensors, sensor_config_dir)


def _load_sensor_descriptions(desc_path: str) -> list:
    """Load sensors from sensor_descriptions.yaml, fill defaults for missing fields."""
    try:
        import yaml
    except ImportError:
        logger.error("PyYAML required to load sensor_descriptions.yaml")
        return []

    with open(desc_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    raw_sensors = data.get("sensors", [])
    sensors = []

    for raw in raw_sensors:
        sensor_type = raw.get("sensor_type", "")
        defaults = SENSOR_DEFAULTS.get(sensor_type, {})

        sensor = {}
        sensor["link_name"] = raw.get("link_name", "")
        sensor["sensor_name"] = raw.get("sensor_name", f"{sensor_type}_sensor")
        sensor["type"] = sensor_type
        sensor["update_rate"] = raw.get("update_rate", defaults.get("update_rate", 10))
        sensor["topic"] = raw.get("topic", defaults.get("topic", f"/{sensor_type}/data"))

        # Copy type-specific config from raw, fallback to defaults
        type_key = sensor_type  # camera, depth, imu, ray, sonar
        if sensor_type in ("camera", "depth"):
            raw_cam = raw.get("camera")
            def_cam = defaults.get("camera", {})
            if raw_cam or def_cam:
                cam = copy.deepcopy(def_cam)
                if raw_cam:
                    _deep_update(cam, raw_cam)
                sensor["camera"] = cam
        elif sensor_type == "ray":
            raw_ray = raw.get("ray")
            def_ray = defaults.get("ray", {})
            if raw_ray or def_ray:
                ray = copy.deepcopy(def_ray)
                if raw_ray:
                    _deep_update(ray, raw_ray)
                sensor["ray"] = ray
        elif sensor_type == "sonar":
            raw_sonar = raw.get("sonar")
            def_sonar = defaults.get("sonar", {})
            if raw_sonar or def_sonar:
                sonar = copy.deepcopy(def_sonar)
                if raw_sonar:
                    sonar.update(raw_sonar)
                sensor["sonar"] = sonar
        else:
            # imu or unknown: copy any remaining type-specific keys
            for key in ("noise",):
                if key in raw:
                    sensor[key] = dict(raw[key])

        # Also grab noise from defaults if not set
        if "noise" not in sensor and "noise" in defaults:
            sensor["noise"] = copy.deepcopy(defaults["noise"])

        sensors.append(sensor)

    return sensors


def _deep_update(base: dict, overlay: dict):
    """Recursively update nested dict (like dict.update but for nested keys)."""
    for key, val in overlay.items():
        if key in base and isinstance(base[key], dict) and isinstance(val, dict):
            _deep_update(base[key], val)
        else:
            base[key] = val


def _write_per_sensor_configs(sensors: list, config_dir: str):
    """Write individual YAML config files for each sensor.

    Creates one file per sensor in config_dir/, named as {sensor_name}.yaml.
    These files are for user convenience — each sensor can be edited independently.
    """
    os.makedirs(config_dir, exist_ok=True)

    count = 0
    for sensor in sensors:
        link_name = sensor.get("link_name", "")
        sensor_name = sensor.get("sensor_name", _extract_core_name(link_name))
        file_name = f"{sensor_name}.yaml"
        file_path = os.path.join(config_dir, file_name)

        lines = [
            f"# Sensor configuration: {sensor_name} (on {link_name})",
            f"# Generated automatically — edit this file to customize sensor parameters",
            f"",
            f"type: {sensor.get('type', 'unknown')}",
            f"link_name: {link_name}",
            f"update_rate: {sensor.get('update_rate', 10)}",
            f"topic: {sensor.get('topic', '/unknown')}",
        ]

        # Camera / Depth type-specific config
        if "camera" in sensor:
            cam = sensor["camera"]
            lines.append("camera:")
            lines.append(f"  horizontal_fov: {cam.get('horizontal_fov', 1.0472)}")
            lines.append("  image:")
            img = cam.get("image", {})
            lines.append(f"    width: {img.get('width', 640)}")
            lines.append(f"    height: {img.get('height', 480)}")
            if "format" in img:
                lines.append(f"    format: {img['format']}")
            lines.append("  clip:")
            lines.append(f"    near: {cam['clip'].get('near', 0.1)}")
            lines.append(f"    far: {cam['clip'].get('far', 100.0)}")

        # Ray (lidar/radar) type-specific config
        if "ray" in sensor:
            ray = sensor["ray"]
            lines.append("ray:")
            lines.append("  scan:")
            lines.append("    horizontal:")
            hz = ray.get("scan", {}).get("horizontal", {})
            lines.append(f"      samples: {hz.get('samples', 360)}")
            lines.append(f"      resolution: {hz.get('resolution', 1)}")
            lines.append(f"      min_angle: {hz.get('min_angle', -3.14159)}")
            lines.append(f"      max_angle: {hz.get('max_angle', 3.14159)}")
            lines.append("  range:")
            rng = ray.get("range", {})
            lines.append(f"    min: {rng.get('min', 0.1)}")
            lines.append(f"    max: {rng.get('max', 30.0)}")
            lines.append(f"    resolution: {rng.get('resolution', 0.01)}")

        # Sonar type-specific config
        if "sonar" in sensor:
            son = sensor["sonar"]
            lines.append("sonar:")
            lines.append(f"  min: {son.get('min', 0.25)}")
            lines.append(f"  max: {son.get('max', 4.5)}")
            lines.append(f"  radius: {son.get('radius', 0.5236)}")

        # Noise
        if "noise" in sensor:
            lines.append("noise:")
            lines.append(f"  type: {sensor['noise'].get('type', 'gaussian')}")
            lines.append(f"  mean: {sensor['noise'].get('mean', 0.0)}")
            lines.append(f"  stddev: {sensor['noise'].get('stddev', 0.01)}")

        lines.append("")  # trailing newline

        with open(file_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

        count += 1
        logger.info(f"  Sensor config: {file_name}")

    logger.info(f"Generated {count} per-sensor config file(s) in {config_dir}")


def _write_yaml(data: dict, path: str):
    """Write data to YAML file."""
    try:
        import yaml
    except ImportError:
        logger.warning("PyYAML not installed, writing YAML manually")
        _write_yaml_simple(data, path)
        return

    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
    logger.info(f"Written: {path}")


def _write_yaml_simple(data: dict, path: str):
    """Simple YAML writer without PyYAML dependency."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("sensors:\n")
        for sensor in data.get("sensors", []):
            f.write(f"  - link_name: {sensor.get('link_name', '')}\n")
            f.write(f"    type: {sensor.get('type', '')}\n")
            f.write(f"    update_rate: {sensor.get('update_rate', 10)}\n")
            f.write(f"    topic: {sensor.get('topic', '')}\n")

            # Camera / Depth
            if "camera" in sensor:
                f.write("    camera:\n")
                cam = sensor["camera"]
                f.write(f"      horizontal_fov: {cam.get('horizontal_fov', 1.0472)}\n")
                f.write("      image:\n")
                img = cam.get("image", {})
                f.write(f"        width: {img.get('width', 640)}\n")
                f.write(f"        height: {img.get('height', 480)}\n")
                if "format" in img:
                    f.write(f"        format: {img['format']}\n")
                f.write("      clip:\n")
                f.write(f"        near: {cam['clip'].get('near', 0.1)}\n")
                f.write(f"        far: {cam['clip'].get('far', 100.0)}\n")

            # Ray
            if "ray" in sensor:
                f.write("    ray:\n")
                ray = sensor["ray"]
                f.write("      scan:\n")
                f.write("        horizontal:\n")
                hz = ray.get("scan", {}).get("horizontal", {})
                f.write(f"          samples: {hz.get('samples', 360)}\n")
                f.write(f"          resolution: {hz.get('resolution', 1)}\n")
                f.write(f"          min_angle: {hz.get('min_angle', -3.14159)}\n")
                f.write(f"          max_angle: {hz.get('max_angle', 3.14159)}\n")
                f.write("      range:\n")
                rng = ray.get("range", {})
                f.write(f"        min: {rng.get('min', 0.1)}\n")
                f.write(f"        max: {rng.get('max', 30.0)}\n")
                f.write(f"        resolution: {rng.get('resolution', 0.01)}\n")

            # Sonar
            if "sonar" in sensor:
                f.write("    sonar:\n")
                son = sensor["sonar"]
                f.write(f"      min: {son.get('min', 0.25)}\n")
                f.write(f"      max: {son.get('max', 4.5)}\n")
                f.write(f"      radius: {son.get('radius', 0.5236)}\n")

            # Noise
            if "noise" in sensor:
                f.write("    noise:\n")
                f.write(f"      type: {sensor['noise'].get('type', 'gaussian')}\n")
                f.write(f"      mean: {sensor['noise'].get('mean', 0.0)}\n")
                f.write(f"      stddev: {sensor['noise'].get('stddev', 0.01)}\n")
