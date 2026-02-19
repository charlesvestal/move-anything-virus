#!/bin/bash
# Install Osirus module to Move
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$REPO_ROOT"

if [ ! -d "dist/osirus" ]; then
    echo "Error: dist/osirus not found. Run ./scripts/build.sh first."
    exit 1
fi

echo "=== Installing Osirus Module ==="

# Deploy to Move
echo "Copying module to Move..."
ssh ableton@move.local "mkdir -p /data/UserData/move-anything/modules/sound_generators/osirus/roms"
scp -r dist/osirus/module.json dist/osirus/ui.js dist/osirus/dsp.so \
    ableton@move.local:/data/UserData/move-anything/modules/sound_generators/osirus/

# Set permissions
echo "Setting permissions..."
ssh ableton@move.local "chmod -R a+rw /data/UserData/move-anything/modules/sound_generators/osirus"

echo ""
echo "=== Install Complete ==="
echo "Module installed to: /data/UserData/move-anything/modules/sound_generators/osirus/"
echo ""
echo "IMPORTANT: Place a Virus A ROM file (.mid) in:"
echo "  /data/UserData/move-anything/modules/sound_generators/osirus/roms/"
echo ""
echo "Restart Move Anything to load the new module."
