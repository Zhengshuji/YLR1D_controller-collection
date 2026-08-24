#!/usr/bin/env bash
# ===================================================
# Restore_Model.sh - Restore a model from its backup
#
# Usage:
#   ./Restore_Model.sh <model_name>
#
# Examples:
#   ./Restore_Model.sh Robot    (restores from Robot_copy)
#   ./Restore_Model.sh YLR1D    (restores from YLR1D_copy)
# ===================================================

MODEL_NAME="${1:?Usage: $0 <model_name>}"
BACKUP_DIR="${MODEL_NAME}_copy"

if [ ! -d "$BACKUP_DIR" ]; then
    echo "Error: Backup directory '$BACKUP_DIR' not found."
    echo "Nothing to restore."
    exit 1
fi

if [ -d "$MODEL_NAME" ]; then
    echo "Removing current model directory: $MODEL_NAME"
    rm -rf "$MODEL_NAME"
fi

echo "Restoring from backup: $BACKUP_DIR -> $MODEL_NAME"
cp -r "$BACKUP_DIR" "$MODEL_NAME"

echo "Restore complete! Model directory: $MODEL_NAME"
