"""Step 2: Rename package from old name to new name.

Replaces all occurrences of old_name with new_name in:
- File contents (except STL/binaries)
- File names
- Directory names
"""

import logging
import os
import re
from pathlib import Path

from model_pipeline.utils import rename_items, is_binary_file

logger = logging.getLogger(__name__)


def rename_package(source_dir: str, old_name: str, new_name: str):
    """Rename all occurrences of old_name to new_name in the source directory.
    """
    source = Path(source_dir)
    parent = source.parent

    # Escape old_name for use as regex pattern (it may contain special chars)
    # For SW names like "Robot" or "YLR1D", this should be fine as-is
    pattern = re.escape(old_name)
    replacement = new_name

    logger.info(f"Replacing '{old_name}' -> '{new_name}' in all files and names")
    rename_items(source_dir, pattern, replacement)

    # If the root directory itself was renamed, note it
    # rename_items handles bottom-up, so the root itself isn't renamed
    # We don't rename the source_dir itself since it's passed by reference

    logger.info("Renaming complete.")
