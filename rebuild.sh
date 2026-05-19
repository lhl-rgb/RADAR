#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

OUT_DIR="$SCRIPT_DIR/out"
BUILD_DIR="$SCRIPT_DIR/build"
BIN_DIR="$OUT_DIR/bin"

show_help() {
    echo "用法: ./rebuild.sh [选项]"
    echo ""
    echo "默认行为: 构建后显示交互式菜单"
    echo ""
    echo "选项:"
    echo "  -h, --help          显示帮助信息"
    echo "  -s, --server        构建后直接运行 gRPC 服务端(跳过菜单)"
    echo "  -t, --test          构建后直接运行测试(跳过菜单)"
    echo ""
    exit 0
}

# 解析参数
SKIP_MENU=false
RUN_TARGET=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            ;;
        -s|--server)
            SKIP_MENU=true
            RUN_TARGET="radar_server"
            shift
            ;;
        -t|--test)
            SKIP_MENU=true
            RUN_TARGET="radar_tests"
            shift
            ;;
        *)
            echo "未知选项: $1"
            echo "使用 -h 查看帮助"
            exit 1
            ;;
    esac
done

echo "=== Radar Build ==="

echo "Cleaning old build..."
rm -rf "$BUILD_DIR"
mkdir -p "$OUT_DIR/bin" "$OUT_DIR/lib" "$BUILD_DIR"

echo "Configuring CMake..."
cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DRADAR_BUILD_TESTS=ON

echo "Building..."
cmake --build "$BUILD_DIR" --parallel

echo ""
echo "=== Build Complete ==="
echo ""

run_program() {
    local prog_name=$1
    local prog_path="$BIN_DIR/$prog_name"
    if [[ -f "$prog_path" ]]; then
        echo ""
        echo ">>> 运行 $prog_name <<<"
        echo "----------------------------------------"
        "$prog_path"
        local exit_code=$?
        echo "----------------------------------------"
        if [[ $exit_code -eq 0 ]]; then
            echo "✓ $prog_name 运行成功"
        else
            echo "✗ $prog_name 退出码: $exit_code"
        fi
    else
        echo "✗ 未找到可执行文件: $prog_path"
    fi
}

# 如果指定了参数，直接运行后退出
if [[ "$SKIP_MENU" == true ]]; then
    run_program "$RUN_TARGET"
    exit 0
fi

# 交互式菜单
while true; do
    echo ""
    echo "===== 请选择要运行的程序 ====="
    echo "  1) gRPC 服务端 (radar_server)"
    echo "  2) Qt 客户端 (radar_client)"
    echo "  3) 测试 (radar_tests)"
    echo "  0) 退出"
    echo "================================"
    read -p "请输入选项 [0-3]: " choice

    case $choice in
        1)
            run_program "radar_server"
            break
            ;;
        2)
            run_program "radar_client"
            break
            ;;
        3)
            run_program "radar_tests"
            break
            ;;
        0)
            echo "退出"
            break
            ;;
        *)
            echo "无效选项，请重新输入"
            ;;
    esac
done
