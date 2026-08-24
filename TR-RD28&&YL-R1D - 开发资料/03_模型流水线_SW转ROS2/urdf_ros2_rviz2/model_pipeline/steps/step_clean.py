"""Step 1: Clean SW export artifacts.

Removes ROS1 build files, textures, and reorganizes the URDF file location.
Mirrors the logic from the original bash/clear_SWurdf.sh.
"""

import logging
import os
import shutil
from pathlib import Path

logger = logging.getLogger(__name__)


def clean_sw_export(source_dir: str):
    """Clean up SW export artifacts from the source directory.

    Removes:
    - CMakeLists.txt (ROS1 catkin)
    - package.xml (ROS1)
    - export.log
    - textures/ directory
    - launch/ directory
    - config/ directory (all contents)

    Moves urdf/*.urdf files to the root directory,
    then removes the urdf/ directory.
    """
    source = Path(source_dir)

    # Remove specific files
    for filename in ["CMakeLists.txt", "package.xml", "export.log"]:
        filepath = source / filename
        if filepath.exists():
            filepath.unlink()
            logger.info(f"Removed: {filename}")

    # Remove directories
    for dirname in ["textures", "launch", "config"]:
        dirpath = source / dirname
        if dirpath.exists() and dirpath.is_dir():
            shutil.rmtree(str(dirpath))
            logger.info(f"Removed directory: {dirname}/")

    # Move urdf/*.urdf to root
    urdf_dir = source / "urdf"
    if urdf_dir.exists() and urdf_dir.is_dir():
        urdf_files = list(urdf_dir.glob("*.urdf"))
        for f in urdf_files:
            dest = source / f.name
            if not dest.exists():
                shutil.move(str(f), str(dest))
                logger.info(f"Moved: urdf/{f.name} -> {f.name}")
            else:
                logger.warning(f"Target exists, skipping: {dest}")

        # Remove the urdf/ directory
        shutil.rmtree(str(urdf_dir))
        logger.info("Removed urdf/ directory")

    logger.info("SW export cleanup complete.")
