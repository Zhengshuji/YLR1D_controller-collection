"""Shared utility functions for the model pipeline."""

import os
import re
import shutil
import logging
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)


def find_urdf_file(directory: str) -> Optional[str]:
    """Find the main URDF file in a directory.

    Looks for .urdf files in the directory root first,
    then in urdf/ subdirectory.
    Returns the path to the file, or None if not found.
    """
    # First check root
    for f in Path(directory).glob("*.urdf"):
        return str(f)
    # Then check urdf/ subdirectory
    for f in Path(directory).glob("urdf/*.urdf"):
        return str(f)
    return None


def find_model_name_from_urdf(urdf_path: str) -> Optional[str]:
    """Extract the robot name from the <robot name="..."> tag."""
    try:
        import xml.etree.ElementTree as ET
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        return root.get("name")
    except Exception:
        return None


def find_stl_files(directory: str):
    """Yield all .STL file paths in a directory."""
    for f in Path(directory).rglob("*.STL"):
        yield str(f)
    for f in Path(directory).rglob("*.stl"):
        yield str(f)


def is_binary_file(file_path: str) -> bool:
    """Check if a file is binary by looking for null bytes."""
    try:
        with open(file_path, "rb") as f:
            chunk = f.read(1024)
            return b"\0" in chunk
    except Exception:
        return True  # If we can't read it, treat as binary


def safe_delete(path: str):
    """Delete a file or directory without raising if not found."""
    if os.path.isfile(path):
        os.remove(path)
        logger.info(f"Deleted file: {path}")
    elif os.path.isdir(path):
        shutil.rmtree(path)
        logger.info(f"Deleted directory: {path}")


def replace_in_file(file_path: str, pattern: str, replacement: str):
    """Replace regex pattern in a text file (skips binary files)."""
    if is_binary_file(file_path):
        logger.debug(f"Skipping binary file: {file_path}")
        return False

    try:
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()
    except Exception as e:
        logger.warning(f"Cannot read {file_path}: {e}")
        return False

    new_content = re.sub(pattern, replacement, content)
    if new_content != content:
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(new_content)
        return True
    return False


def rename_items(root_dir: str, pattern: str, replacement: str):
    """Replace pattern in filenames, directory names, and file contents."""
    # First, rename file contents (bottom-up to avoid path issues)
    for root, dirs, files in os.walk(root_dir, topdown=False):
        for name in files:
            file_path = os.path.join(root, name)
            if not name.lower().endswith(".stl"):
                replace_in_file(file_path, pattern, replacement)

            # Rename file
            new_name = re.sub(pattern, replacement, name)
            if new_name != name:
                new_path = os.path.join(root, new_name)
                os.rename(file_path, new_path)
                logger.info(f"Renamed file: {name} -> {new_name}")

    # Then rename directories (bottom-up)
    for root, dirs, files in os.walk(root_dir, topdown=False):
        for name in dirs:
            old_path = os.path.join(root, name)
            new_name = re.sub(pattern, replacement, name)
            if new_name != name:
                new_path = os.path.join(root, new_name)
                os.rename(old_path, new_path)
                logger.info(f"Renamed directory: {name} -> {new_name}")


def pretty_print_xml(element, encoding="unicode"):
    """Format an XML element tree with proper indentation."""
    from xml.dom import minidom
    rough = element if isinstance(element, str) else ET.tostring(element, encoding=encoding)
    dom = minidom.parseString(rough.encode() if isinstance(rough, str) else rough)
    lines = dom.toprettyxml(indent="  ").splitlines()
    # Filter out blank lines (but keep XML declaration)
    lines = [l for l in lines if l.strip() or "<?xml" in l]
    return "\n".join(lines)


import xml.etree.ElementTree as ET
