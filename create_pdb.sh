#!/bin/bash
################################################################################
# Script to create Portable Development Build (pdb) directory for
# STM32F769I-Discovery firmware project
################################################################################

PROJECT_ROOT="/Users/mich/work/Antigravity/STM32F769I-Disco"
PDB_DIR="$PROJECT_ROOT/pdb"

echo "Creating Portable Development Build directory..."
echo "Source: $PROJECT_ROOT"
echo "Target: $PDB_DIR"
echo ""

# Create directory structure
echo "Creating directory structure..."
mkdir -p "$PDB_DIR/Core/Src"
mkdir -p "$PDB_DIR/Core/Inc"
mkdir -p "$PDB_DIR/Core/Startup"

# Copy Core source files
echo "Copying Core source files..."
cp "$PROJECT_ROOT/Core/Src/"*.c "$PDB_DIR/Core/Src/" 2>/dev/null
cp "$PROJECT_ROOT/Core/Inc/"*.h "$PDB_DIR/Core/Inc/" 2>/dev/null
cp "$PROJECT_ROOT/Core/Startup/"*.s "$PDB_DIR/Core/Startup/" 2>/dev/null

# Copy Drivers
echo "Copying Drivers directory..."
cp -R "$PROJECT_ROOT/Drivers" "$PDB_DIR/"

# Copy Middlewares
echo "Copying Middlewares directory..."
cp -R "$PROJECT_ROOT/Middlewares" "$PDB_DIR/"

# Copy USB_DEVICE
echo "Copying USB_DEVICE directory..."
cp -R "$PROJECT_ROOT/USB_DEVICE" "$PDB_DIR/"

# Copy Utilities
echo "Copying Utilities directory..."
cp -R "$PROJECT_ROOT/Utilities" "$PDB_DIR/"

# Copy TouchGFX (for reference - mostly for simulator, but keep for completeness)
echo "Copying TouchGFX directory..."
cp -R "$PROJECT_ROOT/TouchGFX" "$PDB_DIR/"

# Copy linker scripts
echo "Copying linker scripts..."
cp "$PROJECT_ROOT/"*.ld "$PDB_DIR/" 2>/dev/null
cp "$PROJECT_ROOT/startup_stm32f769xx.s" "$PDB_DIR/" 2>/dev/null

# Copy Makefile
echo "Copying Makefile..."
cp "$PROJECT_ROOT/Makefile" "$PDB_DIR/"

# Copy User code if exists
if [ -d "$PROJECT_ROOT/User" ]; then
    echo "Copying User directory..."
    cp -R "$PROJECT_ROOT/User" "$PDB_DIR/"
fi

echo ""
echo "PDB directory created successfully!"
echo "Location: $PDB_DIR"
