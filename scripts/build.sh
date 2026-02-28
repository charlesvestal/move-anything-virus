#!/usr/bin/env bash
# Build Osirus module for Move Everything (ARM64)
#
# Uses CMake to build the gearmulator Virus library and plugin wrapper.
# Automatically uses Docker for cross-compilation if needed.
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
IMAGE_NAME="move-anything-virus-builder"

# Check if we need Docker
if [ -z "$CROSS_PREFIX" ] && [ ! -f "/.dockerenv" ]; then
    echo "=== Osirus Module Build (via Docker) ==="
    echo ""

    # Build Docker image if needed
    if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
        echo "Building Docker image (first time only)..."
        docker build -t "$IMAGE_NAME" -f "$SCRIPT_DIR/Dockerfile" "$REPO_ROOT"
        echo ""
    fi

    # Run build inside container
    echo "Running build..."
    docker run --rm \
        -v "$REPO_ROOT:/build" \
        -u "$(id -u):$(id -g)" \
        -w /build \
        "$IMAGE_NAME" \
        ./scripts/build.sh

    echo ""
    echo "=== Done ==="
    exit 0
fi

# === Actual build (runs in Docker or with cross-compiler) ===
cd "$REPO_ROOT"

echo "=== Building Osirus Module ==="

# Create build directory
mkdir -p build

# Run CMake configure with cross-compilation toolchain
echo "Configuring CMake..."
cmake -B build \
    -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -G Ninja \
    2>&1

# Build
echo "Building (this may take a while - JIT compiler is large)..."
cmake --build build --target virus-move-plugin -j$(nproc) 2>&1

# Package
echo "Packaging..."
mkdir -p dist/osirus

# Copy files to dist
cat src/module.json > dist/osirus/module.json
[ -f src/help.json ] && cat src/help.json > dist/osirus/help.json
cat src/ui.js > dist/osirus/ui.js
cat build/dsp.so > dist/osirus/dsp.so
chmod +x dist/osirus/dsp.so

# Create roms directory placeholder
mkdir -p dist/osirus/roms

# Create tarball for release
cd dist
tar -czvf osirus-module.tar.gz osirus/
cd ..

echo ""
echo "=== Build Complete ==="
echo "Output: dist/osirus/"
echo "Tarball: dist/osirus-module.tar.gz"
echo ""
echo "To install on Move:"
echo "  ./scripts/install.sh"
echo ""
echo "IMPORTANT: Place a Virus A ROM file (.mid) in the roms/ directory on device:"
echo "  /data/UserData/move-anything/modules/sound_generators/osirus/roms/"
