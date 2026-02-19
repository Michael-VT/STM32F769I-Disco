#!/bin/bash
# STM32F769I-Discovery Project - Source File Restore Script
# Created: 2026-02-12
# Description: Restores source files from archive to correct locations

# Configuration
PROJECT_DIR="/Users/mich/work/Antigravity/STM32F769I-Disco"
BACKUP_DIR="${PROJECT_DIR}/Backups"

echo "======================================================"
echo "STM32F769I-Discovery Source File Restore"
echo "======================================================"
echo "Project: ${PROJECT_DIR}"
echo ""

# Check if archive path was provided
if [ -z "$1" ]; then
    echo "Usage: $0 <archive_path>"
    echo ""
    echo "Available backups in ${BACKUP_DIR}:"
    echo ""
    ls -lh "${BACKUP_DIR}"/*.tar.gz 2>/dev/null || echo "  No backups found"
    echo ""
    echo "======================================================"
    exit 1
fi

ARCHIVE_PATH="$1"

# Check if archive exists
if [ ! -f "${ARCHIVE_PATH}" ]; then
    echo "ERROR: Archive not found: ${ARCHIVE_PATH}"
    exit 1
fi

# Check if it's a valid tar.gz archive
if ! tar -tzf "${ARCHIVE_PATH}" >/dev/null 2>&1; then
    echo "ERROR: Invalid archive format: ${ARCHIVE_PATH}"
    exit 1
fi

echo "Archive: ${ARCHIVE_PATH}"
echo ""

# Show archive contents
echo "Archive contents:"
echo "------------------------------------------------------"
tar -tzf "${ARCHIVE_PATH}" | head -30
TOTAL_FILES=$(tar -tzf "${ARCHIVE_PATH}" | wc -l | tr -d ' ')
if [ $TOTAL_FILES -gt 30 ]; then
    echo "... and $((TOTAL_FILES - 30)) more files"
fi
echo "------------------------------------------------------"
echo "Total files: ${TOTAL_FILES}"
echo ""

# Confirm restore
echo "WARNING: This will overwrite existing files!"
echo ""
read -p "Continue with restore? (yes/no): " CONFIRM

if [ "$CONFIRM" != "yes" ]; then
    echo "Restore cancelled."
    exit 0
fi

echo ""
echo "Restoring files..."

# Change to project directory
cd "${PROJECT_DIR}" || exit 1

# Extract archive
tar -xzf "${ARCHIVE_PATH}"

# Check if restore was successful
if [ $? -eq 0 ]; then
    echo ""
    echo "======================================================"
    echo "Restore completed successfully!"
    echo "======================================================"
    echo "Restored ${TOTAL_FILES} files to:"
    echo "  ${PROJECT_DIR}"
    echo ""
    echo "Next steps:"
    echo "  1. Review the restored files"
    echo "  2. Run 'make clean' to clean old build artifacts"
    echo "  3. Run 'make' to rebuild the firmware"
    echo "  4. Run 'st-flash write build/TestF.bin 0x08000000' to flash"
    echo "======================================================"
else
    echo ""
    echo "ERROR: Restore failed!"
    exit 1
fi
