```bash
Radar/
├── CMakeLists.txt      # 根配置（已配置输出目录）
├── cmake/              # cmake 配置文件（保留）
├── build/              # 构建目录
│   ├── lib/            # 库文件
│   │   ├── libradar_core.a
│   │   ├── libradar_clutter.a
│   │   ├── libradar_target.a
│   │   └── libradar_noise.a
│   └── bin/            # 可执行文件
│   │   ├── radar
│   │   └── radar_tests
├── src/                # 源代码（干净）
├── include/            # 头文件
├── app/                # 应用代码（干净）
├── test/               # 测试代码（干净）
```
```bash
cd /home/lhlj/Radar
rm -rf build && mkdir build && cd build && cmake .. && cmake --build .
```
