# Radar 项目构建说明

## 快速开始

### 编译主程序
```bash
./build.sh --main
```

### 编译验证程序
```bash
./build.sh --validate
```

### 编译全部
```bash
./build.sh --all
```

### 清理构建
```bash
./build.sh --clean
```

## 编译模式

| 模式 | 参数 | 输出的可执行文件 |
|------|------|-----------------|
| 主程序 | `--main` | `radar`, `matlab_export_tool` |
| 验证程序 | `--validate` | `waveform_validation`, `antenna_validation`, `noise_validation`, `clutter_validation` |
| 全部 | `--all` (默认) | 全部可执行文件 |

## VSCode 集成

使用 `Ctrl+Shift+B` (或 `Cmd+Shift+B` on macOS) 打开构建任务菜单，可选择：
- **build - main (主程序)** - 编译主程序
- **build - validate (验证程序)** - 编译验证程序
- **build - all (全部)** - 编译全部
- **build - clean** - 清理构建
- **build - main (debug)** - Debug 模式编译主程序
- **build - validate (debug)** - Debug 模式编译验证程序

## CMake 选项

可以直接使用 CMake 命令进行构建：

```bash
# 只编译主程序
cmake -B build -DRADAR_BUILD_MAIN_ONLY=ON
cmake --build build

# 只编译验证程序
cmake -B build -DRADAR_BUILD_VALIDATE_ONLY=ON
cmake --build build

# 编译全部 (默认)
cmake -B build
cmake --build build
```

## 输出目录

所有编译产物位于 `out/` 目录：
- `out/bin/` - 可执行文件
- `out/lib/` - 静态库
- `build/` - CMake 构建中间文件
