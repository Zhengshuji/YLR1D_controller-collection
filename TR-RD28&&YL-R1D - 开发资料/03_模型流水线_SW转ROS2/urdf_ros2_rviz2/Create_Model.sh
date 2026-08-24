#!/usr/bin/env bash
# ===================================================
# Create_Model.sh - Linux/macOS Shell Wrapper
#
# Usage:
#   ./Create_Model.sh <source_dir> <new_name> [options]
#
# Examples:
#   ./Create_Model.sh Robot ylr1d
#   ./Create_Model.sh Robot ylr1d -o ./src -v
#   ./Create_Model.sh Robot ylr1d --no-xacro
# ===================================================

SOURCE_DIR="${1:?Usage: $0 <source_dir> <new_name> [options]}"
NEW_NAME="${2:?Usage: $0 <source_dir> <new_name> [options]}"
shift 2

# Check Python availability
if ! command -v python3 &>/dev/null; then
    if command -v python &>/dev/null; then
        PYTHON=python
    else
        echo "Error: Python not found. Please install Python 3.8+."
        exit 1
    fi
else
    PYTHON=python3
fi

echo "==================================================="
echo "Model Pipeline"
echo "  Source:  $SOURCE_DIR"
echo "  New Name: $NEW_NAME"
echo "  Args: $*"
echo "==================================================="
echo ""

$PYTHON -m model_pipeline -s "$SOURCE_DIR" -n "$NEW_NAME" "$@"

exit_code=$?
if [ $exit_code -ne 0 ]; then
    echo ""
    echo "Pipeline failed with exit code $exit_code"
    exit $exit_code
fi

echo ""
echo "Pipeline completed successfully!"
