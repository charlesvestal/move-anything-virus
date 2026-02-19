#!/bin/bash
# Install Virus module to Move
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$REPO_ROOT"

if [ ! -d "dist/virus" ]; then
    echo "Error: dist/virus not found. Run ./scripts/build.sh first."
    exit 1
fi

echo "=== Installing Virus Module ==="

# Deploy to Move
echo "Copying module to Move..."
ssh ableton@move.local "mkdir -p /data/UserData/move-anything/modules/sound_generators/virus/roms"
scp -r dist/virus/module.json dist/virus/ui.js dist/virus/dsp.so \
    ableton@move.local:/data/UserData/move-anything/modules/sound_generators/virus/

# Set permissions
echo "Setting permissions..."
ssh ableton@move.local "chmod -R a+rw /data/UserData/move-anything/modules/sound_generators/virus"

echo ""
echo "=== Install Complete ==="
echo "Module installed to: /data/UserData/move-anything/modules/sound_generators/virus/"
echo ""
echo "IMPORTANT: Place a Virus B/C ROM file (.bin) in:"
echo "  /data/UserData/move-anything/modules/sound_generators/virus/roms/"
echo ""
echo "Restart Move Anything to load the new module."
