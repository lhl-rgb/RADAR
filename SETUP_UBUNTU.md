# Ubuntu 工控机环境配置

本文档用于在新 Ubuntu 机器上配置 Radar 项目依赖并编译测试。

## 1. 一键安装

在仓库根目录执行：

```bash
cd ~/Radar
scripts/setup_ubuntu.sh
```

安装后编译：

```bash
cmake -S . -B out/build
cmake --build out/build -j
./out/bin/radar_tests
```

## 2. 常用安装方式

### 2.1 普通 Ubuntu + NVIDIA GPU

```bash
scripts/setup_ubuntu.sh --cuda-arch 65 --build
```

`--cuda-arch` 按显卡修改：

```text
GT 1030 / Pascal:       61 或 65
RTX 20 系 / Turing:     75
RTX 30 系 / Ampere:     86
RTX 40 系 / Ada:        89
Jetson AGX Orin:        87
```

如果不确定，先查：

```bash
nvidia-smi
nvcc --version
```

### 2.2 Jetson AGX Orin

Jetson 通常由 JetPack 提供 CUDA，不建议用 Ubuntu 的 `nvidia-cuda-toolkit` 覆盖。

```bash
scripts/setup_ubuntu.sh --no-cuda --cuda-arch 87 --build
```

如果 JetPack 已经提供 `nvcc`，也可以：

```bash
cmake -S . -B out/build -DCMAKE_CUDA_ARCHITECTURES=87
cmake --build out/build -j
```

### 2.3 不编译 Qt 客户端

工控机只做后端和 benchmark 时，可以关闭 GUI：

```bash
scripts/setup_ubuntu.sh --no-qt
cmake -S . -B out/build -DRADAR_BUILD_CLIENT=OFF
cmake --build out/build -j
```

### 2.4 CPU-only 临时编译

如果机器还没配好 CUDA：

```bash
scripts/setup_ubuntu.sh --no-cuda --build
```

或手动：

```bash
cmake -S . -B out/build -DRADAR_ENABLE_CUDA=OFF
cmake --build out/build -j
```

## 3. 主要依赖

脚本会安装：

```text
build-essential
cmake
ninja-build
pkg-config
git
Eigen3
nlohmann-json
spdlog
Boost
GDAL
FFTW3
Protobuf
gRPC
Qt5 Widgets
nvidia-cuda-toolkit
```

其中 Qt 和 CUDA 可通过 `--no-qt`、`--no-cuda` 跳过。

## 4. CUDA 运行检查

```bash
nvcc --version
nvidia-smi
```

项目配置时指定架构：

```bash
cmake -S . -B out/build -DCMAKE_CUDA_ARCHITECTURES=65
```

GT 1030 如果默认架构不支持，可以用：

```bash
cmake -S . -B out/build -DCMAKE_CUDA_ARCHITECTURES=61
```

## 5. 性能测试命令

K 分布 stream 双缓冲测试：

```bash
./out/bin/benchmark_clutter_gpu_stream \
  2000 1 0.5 -40 25 1600 60 \
  3 1.5 1.2 -0.5 0.5 1.0 2.0 4096
```

较大 active patch 压力测试：

```bash
./out/bin/benchmark_clutter_gpu_stream \
  2000 1 0.2 -40 37 1600 60 \
  3 1.5 1.2 -0.5 0.5 1.0 2.0 4096
```

1D / 2D / 2D-fast 对比：

```bash
./out/bin/benchmark_clutter_gpu_2d \
  2000 1 0.5 -40 25 1600 60 \
  3 1.5 1.2 -0.5 0.5 1.0 2.0 4096
```

## 6. 常见问题

### 6.1 找不到 `grpc_cpp_plugin`

安装：

```bash
sudo apt-get install protobuf-compiler-grpc libgrpc++-dev
```

确认：

```bash
which grpc_cpp_plugin
```

### 6.2 找不到 CUDA Toolkit

如果需要 GPU：

```bash
sudo apt-get install nvidia-cuda-toolkit
```

如果暂时只验证 CPU：

```bash
cmake -S . -B out/build -DRADAR_ENABLE_CUDA=OFF
```

### 6.3 CUDA driver/runtime 不匹配

现象一般是：

```text
CUDA driver version is insufficient for CUDA runtime version
```

处理方式：

```bash
nvidia-smi
nvcc --version
```

确认驱动版本支持当前 CUDA Toolkit。桌面 Ubuntu 可升级 NVIDIA 驱动；Jetson 应优先使用对应 JetPack 版本。
