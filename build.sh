#!/bin/bash

# Radar 项目构建脚本
# 编译模式：
#   --all       编译全部 (默认)
#   --qt        只编译 Qt GUI 客户端 (radar_client)
#   --server    只编译独立 gRPC server (radar_server)
#   --clean     清理构建目录

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
BUILD_TYPE="Release"

# 默认编译全部
BUILD_MODE="all"

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --qt)
            BUILD_MODE="qt"
            shift
            ;;
        --server)
            BUILD_MODE="server"
            shift
            ;;
        --all)
            BUILD_MODE="all"
            shift
            ;;
        --clean)
            echo "清理构建目录..."
            rm -rf "${BUILD_DIR}"
            rm -rf "${SCRIPT_DIR}/out"
            echo "清理完成"
            exit 0
            ;;
        --debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        -h|--help)
            echo "用法: $0 [选项]"
            echo ""
            echo "编译模式 (默认：--all):"
            echo "  --all        编译全部目标"
            echo "  --qt         只编译 Qt GUI 客户端 (radar_client)"
            echo "  --server     只编译独立 gRPC server (radar_server)"
            echo ""
            echo "其他选项:"
            echo "  --clean      清理构建目录和输出"
            echo "  --debug      使用 Debug 模式编译 (默认：Release)"
            echo "  -h, --help   显示帮助"
            exit 0
            ;;
        *)
            echo "未知选项：$1"
            echo "使用 -h 或 --help 查看帮助"
            exit 1
            ;;
    esac
done

echo "========================================"
echo "Radar 项目构建"
echo "========================================"
echo "构建模式：${BUILD_MODE}"
echo "构建类型：${BUILD_TYPE}"
echo "构建目录：${BUILD_DIR}"
echo "========================================"

# CMake 配置选项
CMAKE_OPTS="-B ${BUILD_DIR} -DCMAKE_BUILD_TYPE=${BUILD_TYPE}"

# 根据模式设置 CMake 缓存变量
case $BUILD_MODE in
    qt)
        CMAKE_OPTS="${CMAKE_OPTS} -DRADAR_BUILD_CLIENT=ON -DRADAR_BUILD_SERVER=OFF -DRADAR_BUILD_TESTS=OFF"
        echo "→ 编译 Qt GUI 客户端..."
        ;;
    server)
        CMAKE_OPTS="${CMAKE_OPTS} -DRADAR_BUILD_CLIENT=OFF -DRADAR_BUILD_SERVER=ON -DRADAR_BUILD_TESTS=OFF"
        echo "→ 编译 gRPC Server..."
        ;;
    all)
        echo "→ 编译全部目标..."
        ;;
esac

# 配置和构建
echo ""
echo "Step 1: 配置 CMake..."
cmake ${CMAKE_OPTS}

echo ""
echo "Step 2: 编译..."
cmake --build "${BUILD_DIR}" --config "${BUILD_TYPE}" -j$(nproc)

echo ""
echo "========================================"
echo "构建完成!"
echo "输出目录：${SCRIPT_DIR}/out/"
echo "========================================"

# 列出已生成的可执行文件
echo ""
echo "已生成的可执行文件:"
if [ -d "${SCRIPT_DIR}/out/bin" ]; then
    find "${SCRIPT_DIR}/out/bin" -type f -executable -name "*" | sort | while read -r file; do
        echo "  - $(basename "${file}")"
    done
fi
