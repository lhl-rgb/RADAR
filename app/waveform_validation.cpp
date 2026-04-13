/**
 * @file waveform_validation.cpp
 * @brief 波形生成验证工具 - 对比C++实现与MATLAB参考
 *
 * 目的：
 *   验证三种波形（LFM、NLFM、相位编码）的C++实现与MATLAB的一致性
 *
 * 输出：
 *   - 参数信息（采样点数、时间轴等）
 *   - 波形数据（CSV格式，供MATLAB对比）
 *   - 瞬时频率和相位信息
 *
 * 使用方法：
 *   ./waveform_validation --output_dir=out/validation
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <complex>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <getopt.h>

#include "core/types.h"
#include "core/tools/math_utils.h"
#include "core/radar_system_params.h"
#include "waveform/waveform_generator.h"
#include "waveform/waveform_config.h"

using namespace radar;

namespace fs = std::filesystem;

struct ValidationOptions {
    std::string output_dir = "out/waveform_validation";
    bool export_all_waveforms = true;
    bool verbose = true;
};

// 导出复数数据为CSV
bool export_complex_csv(const std::string& filepath,
                        const ComplexVec& data,
                        const std::string& var_name = "signal") {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open: " << filepath << std::endl;
        return false;
    }

    file << std::scientific << std::setprecision(15);
    file << "index,real,imag\n";

    for (std::size_t i = 0; i < data.size(); ++i) {
        file << i << "," << data[i].real() << "," << data[i].imag() << "\n";
    }

    return true;
}

// 导出实数数据为CSV
bool export_scalar_csv(const std::string& filepath,
                       const std::vector<Scalar>& data,
                       const std::string& var_name = "data") {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open: " << filepath << std::endl;
        return false;
    }

    file << std::scientific << std::setprecision(15);
    file << "index,value\n";

    for (std::size_t i = 0; i < data.size(); ++i) {
        file << i << "," << data[i] << "\n";
    }

    return true;
}


int main(int argc, char* argv[]) {
    ValidationOptions opts;

    static struct option long_options[] = {
        {"output-dir", required_argument, nullptr, 'o'},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, 0, nullptr, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "o:h", long_options, nullptr)) != -1) {
        switch (opt) {
        case 'o': opts.output_dir = optarg; break;
        case 'h':
            std::cout << "Usage: " << argv[0] << " [OPTIONS]\n"
                      << "Options:\n"
                      << "  -o, --output-dir DIR  输出目录\n"
                      << "  -h, --help            帮助\n";
            return 0;
        }
    }

    fs::create_directories(opts.output_dir);

    std::cout << "╔══════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  波形验证工具 - C++ vs MATLAB 对比          ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════╝" << std::endl;

    // ===== 使用与MATLAB相同的参数 =====
    RadarSystemParams sys;
    sys.fs_hz = 40.0e6;          // 40 MHz (与MATLAB一致)
    sys.pulse_width_s = 20.0e-6; // 20 μs (与MATLAB一致)
    sys.bw_hz = 20.0e6;          // 20 MHz (与MATLAB一致)
    sys.compute_derived_params();

    const int N_cpp = sys.samples_per_tx;
    const int N_matlab = static_cast<int>(std::round(sys.pulse_width_s * sys.fs_hz));

    std::cout << "\n=== 采样点数对比 ===" << std::endl;
    std::cout << "参数: T = " << sys.pulse_width_s * 1e6 << " μs, fs = " << sys.fs_hz / 1e6 << " MHz" << std::endl;
    std::cout << "T * fs = " << sys.pulse_width_s * sys.fs_hz << std::endl;
    std::cout << "C++ (ceil):  N = " << N_cpp << std::endl;
    std::cout << "MATLAB (round): N = " << N_matlab << std::endl;

    // ===== 验证三种波形 =====

    // 1. LFM
    std::cout << "\n=== 验证 LFM 波形 ===" << std::endl;
    waveform::WaveformGenerator lfm_gen;
    waveform::WaveformConfig lfm_cfg;
    lfm_cfg.waveform_type = WaveformType::LFM;

    lfm_gen.set_config(lfm_cfg);
    lfm_gen.set_system_params(sys);
    lfm_gen.initialize();

    const ComplexVec& lfm_waveform = lfm_gen.get_waveform();
    std::cout << "LFM 波形长度: " << lfm_waveform.size() << " samples" << std::endl;

    // 导出LFM数据
    export_complex_csv(opts.output_dir + "/lfm_waveform.csv", lfm_waveform);

    // 导出匹配滤波器
    ComplexVec lfm_mf = lfm_gen.generate_matched_filter();
    export_complex_csv(opts.output_dir + "/lfm_matched_filter.csv", lfm_mf);

    // 2. NLFM
    std::cout << "\n=== 验证 NLFM 波形 ===" << std::endl;
    waveform::WaveformGenerator nlfm_gen;
    waveform::WaveformConfig nlfm_cfg;
    nlfm_cfg.waveform_type = WaveformType::NLFM;
    nlfm_cfg.nlfm_window_type = WindowType::Hamming;

    nlfm_gen.set_config(nlfm_cfg);
    nlfm_gen.set_system_params(sys);
    nlfm_gen.initialize();

    const ComplexVec& nlfm_waveform = nlfm_gen.get_waveform();
    std::cout << "NLFM 波形长度: " << nlfm_waveform.size() << " samples" << std::endl;

    export_complex_csv(opts.output_dir + "/nlfm_waveform.csv", nlfm_waveform);

    ComplexVec nlfm_mf = nlfm_gen.generate_matched_filter();
    export_complex_csv(opts.output_dir + "/nlfm_matched_filter.csv", nlfm_mf);

    // 3. Phase-coded (Barker11)
    std::cout << "\n=== 验证相位编码波形 (Barker11) ===" << std::endl;

    // 注意：相位编码的参数设置不同
    // RadarSystemParams pc_sys;
    // pc_sys.fs_hz = 40.0e6;          // 20 MHz (与MATLAB一致)
    // pc_sys.pulse_width_s = 20.0e-6; // 20 μs (与MATLAB一致)
    // pc_sys.bw_hz = 0;               // 相位编码不使用带宽参数
    // pc_sys.compute_derived_params();

    waveform::WaveformGenerator pc_gen;
    waveform::WaveformConfig pc_cfg;
    pc_cfg.waveform_type = WaveformType::PHASE_CODED;
    pc_cfg.phase_code_type = PhaseCodeType::Barker11;

    pc_gen.set_config(pc_cfg);
    pc_gen.set_system_params(sys);
    pc_gen.initialize();

    const ComplexVec& pc_waveform = pc_gen.get_waveform();
    std::cout << "相位编码波形长度: " << pc_waveform.size() << " samples" << std::endl;
    std::cout << "Barker11 码片数: 11" << std::endl;
    std::cout << "每码片采样数: " << pc_waveform.size() / 11 << std::endl;

    export_complex_csv(opts.output_dir + "/barker11_waveform.csv", pc_waveform);

    ComplexVec pc_mf = pc_gen.generate_matched_filter();
    export_complex_csv(opts.output_dir + "/barker11_matched_filter.csv", pc_mf);

    std::cout << "\n=== 输出文件 ===" << std::endl;
    std::cout << "目录: " << opts.output_dir << std::endl;
    std::cout << "  - lfm_waveform.csv" << std::endl;
    std::cout << "  - nlfm_waveform.csv" << std::endl;
    std::cout << "  - barker11_waveform.csv" << std::endl;
    std::cout << "  - compare_cpp_matlab.m" << std::endl;

    std::cout << "\n=== 下一步 ===" << std::endl;
    std::cout << "1. 编译此工具: cmake --build out --target waveform_validation" << std::endl;
    std::cout << "2. 运行: ./out/bin/waveform_validation --output_dir=out/validation" << std::endl;

    return 0;
}