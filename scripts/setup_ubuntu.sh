#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -eq 0 ]]; then
    SUDO=""
else
    SUDO="sudo"
fi

INSTALL_CUDA="${INSTALL_CUDA:-1}"
INSTALL_QT="${INSTALL_QT:-1}"
INSTALL_GRPC="${INSTALL_GRPC:-1}"
BUILD_AFTER_INSTALL="${BUILD_AFTER_INSTALL:-0}"
CUDA_ARCHITECTURES="${CUDA_ARCHITECTURES:-}"

usage() {
    cat <<'EOF'
Usage:
  scripts/setup_ubuntu.sh [options]

Options:
  --no-cuda        不安装 nvidia-cuda-toolkit，只安装 CPU 构建依赖
  --no-qt          不安装 Qt GUI 依赖
  --no-grpc        不安装 gRPC/Protobuf 依赖
  --build          安装后立即执行 cmake 配置和编译
  --cuda-arch X    构建时传入 CMAKE_CUDA_ARCHITECTURES，例如 65、72、87、89
  -h, --help       显示帮助

Environment:
  INSTALL_CUDA=0|1
  INSTALL_QT=0|1
  INSTALL_GRPC=0|1
  BUILD_AFTER_INSTALL=0|1
  CUDA_ARCHITECTURES=<arch-list>

Examples:
  scripts/setup_ubuntu.sh
  scripts/setup_ubuntu.sh --cuda-arch 87 --build
  scripts/setup_ubuntu.sh --no-qt --cuda-arch 65
  scripts/setup_ubuntu.sh --no-cuda --build

Notes:
  - Ubuntu apt 的 nvidia-cuda-toolkit 不一定是最新版本。
  - Jetson AGX Orin 一般使用 JetPack 自带 CUDA，不建议用 apt 装桌面版 CUDA。
    Jetson 上可用: scripts/setup_ubuntu.sh --no-cuda --cuda-arch 87
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-cuda)
            INSTALL_CUDA=0
            shift
            ;;
        --no-qt)
            INSTALL_QT=0
            shift
            ;;
        --no-grpc)
            INSTALL_GRPC=0
            shift
            ;;
        --build)
            BUILD_AFTER_INSTALL=1
            shift
            ;;
        --cuda-arch)
            CUDA_ARCHITECTURES="${2:-}"
            if [[ -z "${CUDA_ARCHITECTURES}" ]]; then
                echo "error: --cuda-arch requires a value" >&2
                exit 2
            fi
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "error: unknown option: $1" >&2
            usage
            exit 2
            ;;
    esac
done

if [[ ! -f /etc/os-release ]]; then
    echo "error: /etc/os-release not found; this script targets Ubuntu/Debian systems" >&2
    exit 1
fi

. /etc/os-release
echo "[setup] detected OS: ${PRETTY_NAME:-unknown}"

if ! command -v apt-get >/dev/null 2>&1; then
    echo "error: apt-get not found; this script targets Ubuntu/Debian systems" >&2
    exit 1
fi

echo "[setup] updating apt index"
${SUDO} apt-get update

BASE_PACKAGES=(
    build-essential
    cmake
    ninja-build
    pkg-config
    git
    gdb
    ca-certificates
    curl
    wget
    python3
    python3-pip
    libeigen3-dev
    nlohmann-json3-dev
    libspdlog-dev
    libboost-all-dev
    libgdal-dev
    gdal-bin
    libfftw3-dev
)

GRPC_PACKAGES=(
    protobuf-compiler
    libprotobuf-dev
    protobuf-compiler-grpc
    libgrpc++-dev
    libgrpc-dev
)

QT_PACKAGES=(
    qtbase5-dev
    qtbase5-dev-tools
)

CUDA_PACKAGES=(
    nvidia-cuda-toolkit
)

PACKAGES=("${BASE_PACKAGES[@]}")

if [[ "${INSTALL_GRPC}" == "1" ]]; then
    PACKAGES+=("${GRPC_PACKAGES[@]}")
fi

if [[ "${INSTALL_QT}" == "1" ]]; then
    PACKAGES+=("${QT_PACKAGES[@]}")
fi

if [[ "${INSTALL_CUDA}" == "1" ]]; then
    PACKAGES+=("${CUDA_PACKAGES[@]}")
fi

echo "[setup] installing packages"
${SUDO} apt-get install -y "${PACKAGES[@]}"

echo "[setup] tool versions"
cmake --version | head -n 1 || true
g++ --version | head -n 1 || true
protoc --version || true
if command -v grpc_cpp_plugin >/dev/null 2>&1; then
    echo "grpc_cpp_plugin: $(command -v grpc_cpp_plugin)"
else
    echo "warning: grpc_cpp_plugin not found in PATH"
fi
if command -v nvcc >/dev/null 2>&1; then
    nvcc --version | tail -n 1 || true
else
    echo "warning: nvcc not found. Use --no-cuda for CPU-only build, or install CUDA/JetPack."
fi
if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi || true
else
    echo "info: nvidia-smi not found. This is normal on Jetson; use tegrastats or deviceQuery instead."
fi

if [[ "${BUILD_AFTER_INSTALL}" == "1" ]]; then
    echo "[setup] configuring project"
    CMAKE_ARGS=(-S . -B out/build)
    if [[ "${INSTALL_CUDA}" == "0" ]]; then
        CMAKE_ARGS+=(-DRADAR_ENABLE_CUDA=OFF)
    fi
    if [[ -n "${CUDA_ARCHITECTURES}" ]]; then
        CMAKE_ARGS+=(-DCMAKE_CUDA_ARCHITECTURES="${CUDA_ARCHITECTURES}")
    fi
    cmake "${CMAKE_ARGS[@]}"

    echo "[setup] building project"
    cmake --build out/build -j"$(nproc)"
fi

cat <<EOF
[setup] done.

Next commands:
  cmake -S . -B out/build${CUDA_ARCHITECTURES:+ -DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCHITECTURES}}
  cmake --build out/build -j
  ./out/bin/radar_tests

CUDA benchmark example:
  ./out/bin/benchmark_clutter_gpu_stream \\
    2000 1 0.5 -40 25 1600 60 \\
    3 1.5 1.2 -0.5 0.5 1.0 2.0 4096
EOF
