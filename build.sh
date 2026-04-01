#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== Radar Build ==="

# 构建目录
BUILD_DIR="$SCRIPT_DIR/build"
OUT_DIR="$SCRIPT_DIR/out"

# 清理旧构建
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

# 配置 CMake
echo "Configuring CMake..."
cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DRADAR_BUILD_TESTS=OFF

# 编译
echo "Building..."
cmake --build "$BUILD_DIR" --parallel

# 输出目录
echo ""
echo "=== Build Complete ==="
echo "Output directory: $OUT_DIR"
echo "  Executable: $OUT_DIR/bin/radar"
echo "  Libraries:  $OUT_DIR/lib/"
ls -la "$OUT_DIR/" 2>/dev/null || true
