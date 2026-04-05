#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== Radar Build ==="

# 输出目录
OUT_DIR="$SCRIPT_DIR/out"
BUILD_DIR="$OUT_DIR/build"

# 清理旧构建
echo "Cleaning old build..."
rm -rf "$BUILD_DIR"
mkdir -p "$OUT_DIR/bin" "$OUT_DIR/lib" "$BUILD_DIR"

# 配置 CMake
echo "Configuring CMake..."
cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DRADAR_BUILD_TESTS=ON

# 编译
echo "Building..."
cmake --build "$BUILD_DIR" --parallel

# 输出目录结构
echo ""
echo "=== Build Complete ==="
echo ""
echo "输出目录结构:"
echo "  $OUT_DIR/"
echo "  ├── bin/               # 可执行文件"
echo "  │   ├── radar"
echo "  │   ├── matlab_export_tool"
echo "  │   └── radar_tests"
echo "  ├── lib/               # 静态库"
echo "  │   └── libradar_*.a"
echo "  └── build/             # CMake 中间文件"
echo ""
echo "运行方式:"
echo "  ./out/bin/radar                  # 运行主程序"
echo "  ./out/bin/radar_tests            # 运行测试"
echo "  ./out/bin/matlab_export_tool     # MATLAB 导出工具"
