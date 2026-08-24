#!/usr/bin/env python3
"""
model_pipeline/pipeline.py

Main entry point for the SW-exported URDF to ROS2 package pipeline.

Usage:
    python -m model_pipeline --source-dir ./Robot --new-name my_robot
    python -m model_pipeline --source-dir ./Robot --new-name my_robot --output-dir ./src
"""

import argparse
import logging
import os
import shutil
import sys
from pathlib import Path

from model_pipeline.utils import find_urdf_file, find_model_name_from_urdf, logger
from model_pipeline.steps.step_clean import clean_sw_export
from model_pipeline.steps.step_rename import rename_package
from model_pipeline.steps.step_fix_urdf import fix_urdf
from model_pipeline.steps.step_sensors import extract_sensors_to_yaml
from model_pipeline.steps.step_config import extract_configs
from model_pipeline.steps.step_gazebo import add_gazebo_support
from model_pipeline.steps.step_urdf2xacro import urdf_to_xacro
from model_pipeline.steps.step_package import generate_ros2_package


def setup_logging(verbose: bool = False):
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="[%(levelname)s] %(message)s",
        handlers=[logging.StreamHandler(sys.stdout)],
    )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Convert SW-exported URDF model to ROS2 package"
    )
    parser.add_argument(
        "--source-dir", "-s",
        required=True,
        help="Source directory containing SW-exported model (e.g. ./Robot)",
    )
    parser.add_argument(
        "--new-name", "-n",
        required=True,
        help="New model/package name (e.g. ylr1d)",
    )
    parser.add_argument(
        "--output-dir", "-o",
        default="./src",
        help="Output directory for the generated ROS2 package (default: ./src)",
    )
    parser.add_argument(
        "--no-xacro",
        action="store_true",
        help="Skip Xacro generation (URDF only)",
    )
    parser.add_argument(
        "--sensor-config", "-sc",
        default=None,
        help="Path to sensors_description.yaml (manual sensor configuration, external to source dir)",
    )
    parser.add_argument(
        "--no-backup",
        action="store_true",
        help="Delete backup of the source directory after processing",
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable debug logging",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    setup_logging(args.verbose)

    source_dir = os.path.abspath(args.source_dir)
    new_name = args.new_name
    output_dir = os.path.abspath(args.output_dir)

    if not os.path.isdir(source_dir):
        logger.error(f"Source directory does not exist: {source_dir}")
        return 1

    # ---------------------------------------------------------------
    # Step 0: Detect current model name
    # ---------------------------------------------------------------
    urdf_path = find_urdf_file(source_dir)
    if not urdf_path:
        logger.error(f"No .urdf file found in {source_dir}")
        return 1

    old_name = find_model_name_from_urdf(urdf_path)
    if not old_name:
        logger.warning("Could not determine model name from URDF; using directory name")
        old_name = os.path.basename(source_dir)

    logger.info(f"Detected model name: '{old_name}' -> new name: '{new_name}'")
    logger.info(f"URDF file: {urdf_path}")
    logger.info(f"Output directory: {output_dir}")

    # ---------------------------------------------------------------
    # Step 0: Backup original source directory before any modification
    # ---------------------------------------------------------------
    backup_dir = os.path.join(os.path.dirname(source_dir), f"{old_name}_copy")
    if not os.path.isdir(backup_dir):
        logger.info(f"Creating backup of original source: {backup_dir}")
        shutil.copytree(source_dir, backup_dir)
    else:
        logger.info(f"Backup already exists: {backup_dir}")

    # ---------------------------------------------------------------
    # Step 1: Clean up SW export artifacts
    # ---------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("Step 1/8: Cleaning SW export artifacts")
    logger.info("=" * 60)
    clean_sw_export(source_dir)

    # ---------------------------------------------------------------
    # Step 2: Rename package from old_name to new_name
    # ---------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("Step 2/8: Renaming package")
    logger.info("=" * 60)
    rename_package(source_dir, old_name, new_name)

    # Update urdf_path after rename
    urdf_path = os.path.join(source_dir, f"{new_name}.urdf")
    if not os.path.exists(urdf_path):
        logger.error(f"URDF file not found after rename: {urdf_path}")
        return 1

    # ---------------------------------------------------------------
    # Step 3: Fix URDF issues
    # ---------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("Step 3/8: Fixing URDF issues")
    logger.info("=" * 60)
    fix_urdf(urdf_path)

    # ---------------------------------------------------------------
    # Step 4: Generate sensor config file (sensors.yaml)
    # ---------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("Step 4/8: Generating sensor configuration")
    logger.info("=" * 60)
    config_dir = os.path.join(source_dir, "config")
    os.makedirs(config_dir, exist_ok=True)

    # Check for external sensors_description.yaml (manual/expert config)
    sensor_desc_path = None
    if args.sensor_config:
        sensor_desc_path = os.path.abspath(args.sensor_config)
        if not os.path.exists(sensor_desc_path):
            logger.warning(f"Sensor config not found: {sensor_desc_path}, falling back to auto-detection")
            sensor_desc_path = None
        else:
            logger.info(f"Using sensor descriptions: {sensor_desc_path}")

    sensors_yaml = os.path.join(config_dir, "sensors.yaml")
    extract_sensors_to_yaml(urdf_path, sensors_yaml, sensor_descriptions_path=sensor_desc_path)

    # ---------------------------------------------------------------
    # Step 5: Extract URDF config data (links.yaml, colors.yaml, limits.yaml, scale.yaml)
    # ---------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("Step 5/8: Extracting URDF configuration data")
    logger.info("=" * 60)
    extract_configs(urdf_path, config_dir)

    # ---------------------------------------------------------------
    # Step 6 (optional): Convert URDF to Xacro
    # ---------------------------------------------------------------
    xacro_path = os.path.join(source_dir, f"{new_name}.xacro") if not args.no_xacro else None
    gazebo_xacro_path = os.path.join(
        source_dir, f"{new_name}_gazebo.xacro") if not args.no_xacro else None

    if not args.no_xacro:
        logger.info("=" * 60)
        logger.info("Step 6/8: Converting URDF to Xacro")
        logger.info("=" * 60)
        urdf_to_xacro(urdf_path, xacro_path, new_name)

    # ---------------------------------------------------------------
    # Step 7: Add Gazebo support (sensor tags + plugins)
    # ---------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("Step 7/8: Adding Gazebo support")
    logger.info("=" * 60)
    gazebo_urdf = os.path.join(source_dir, f"{new_name}_gazebo.urdf")

    add_gazebo_support(
        urdf_path=urdf_path,
        xacro_path=xacro_path,
        sensors_yaml=sensors_yaml,
        output_urdf=gazebo_urdf,
        output_xacro=gazebo_xacro_path,
    )

    # ---------------------------------------------------------------
    # Step 8: Generate ROS2 package
    # ---------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("Step 8/8: Generating ROS2 package")
    logger.info("=" * 60)
    pkg_dir = os.path.join(output_dir, f"{new_name}_description")
    sensors_yaml = os.path.join(source_dir, "config", "sensors.yaml")
    generate_ros2_package(
        source_dir=source_dir,
        pkg_dir=pkg_dir,
        model_name=new_name,
        has_xacro=not args.no_xacro,
        sensors_yaml=sensors_yaml,
    )

    # ---------------------------------------------------------------
    # Cleanup: remove backup only if explicitly requested
    # ---------------------------------------------------------------
    if args.no_backup and os.path.isdir(backup_dir):
        shutil.rmtree(backup_dir)
        logger.info(f"Removed backup: {backup_dir}")
    else:
        logger.info(f"Backup preserved: {backup_dir}")

    logger.info("=" * 60)
    logger.info(f"Pipeline complete! ROS2 package generated at:")
    logger.info(f"  {pkg_dir}")
    logger.info("=" * 60)

    return 0


if __name__ == "__main__":
    sys.exit(main())
