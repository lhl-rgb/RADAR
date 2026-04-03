/**
 * @file matlab_export_tool.cpp
 * @brief 独立的 MATLAB 信号导出工具
 * @details
 * 生成 LFM、NLFM 信号、噪声、海杂波数据并导出为 MATLAB 可读格式
 *
 * 使用方法:
 *   ./matlab_export_tool --output_dir=out/matlab --waveform=lfm --duration=10
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
#include "core/math_utils.h"
#include "core/radar_system_params.h"
#include "waveform/waveform_generator.h"
#include "waveform/waveform_config.h"
#include "noise/noise_engine.h"
#include "noise/noise_config.h"
#include "clutter/sea_clutter_model.h"
#include "clutter/sea_clutter_config.h"
#include "antenna/antenna_model.h"

using namespace radar;

namespace fs = std::filesystem;

// 命令行参数结构
struct ExportOptions {
    std::string output_dir = "matlab";
    std::string waveform_type = "lfm";  // lfm, nlfm, barker, cw
    int num_pulses = 32;
    int samples_per_pulse = 512;
    Scalar snr_db = 10.0;
    bool export_clutter = true;
    bool export_noise = true;
    bool generate_matlab_script = true;
    uint64_t seed = 2026;
};

void print_usage(const char* program) {
    std::cout << "Usage: " << program << " [OPTIONS]\n"
              << "\nOptions:\n"
              << "  -o, --output-dir DIR    输出目录 (默认：matlab)\n"
              << "  -w, --waveform TYPE     波形类型：lfm, nlfm, barker, cw (默认：lfm)\n"
              << "  -n, --pulses NUM        脉冲数 (默认：32)\n"
              << "  -s, --samples NUM       每脉冲采样数 (默认：512)\n"
              << "  -S, --snr-db DB         信噪比 dB (默认：10)\n"
              << "  -c, --no-clutter        不生成杂波\n"
              << "  -N, --no-noise          不生成噪声\n"
              << "  -m, --no-matlab         不生成 MATLAB 脚本\n"
              << "  -S, --seed NUM          随机种子 (默认：2026)\n"
              << "  -h, --help              显示帮助\n";
}

bool parse_args(int argc, char* argv[], ExportOptions& opts) {
    static struct option long_options[] = {
        {"output-dir", required_argument, nullptr, 'o'},
        {"waveform",   required_argument, nullptr, 'w'},
        {"pulses",     required_argument, nullptr, 'n'},
        {"samples",    required_argument, nullptr, 's'},
        {"snr-db",     required_argument, nullptr, 'S'},
        {"no-clutter", no_argument,       nullptr, 'c'},
        {"no-noise",   no_argument,       nullptr, 'N'},
        {"no-matlab",  no_argument,       nullptr, 'm'},
        {"seed",       required_argument, nullptr, 'd'},
        {"help",       no_argument,       nullptr, 'h'},
        {nullptr,      0,                 nullptr, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "o:w:n:s:S:cNmd:h", long_options, nullptr)) != -1) {
        switch (opt) {
        case 'o': opts.output_dir = optarg; break;
        case 'w': opts.waveform_type = optarg; break;
        case 'n': opts.num_pulses = std::atoi(optarg); break;
        case 's': opts.samples_per_pulse = std::atoi(optarg); break;
        case 'S': opts.snr_db = std::atof(optarg); break;
        case 'c': opts.export_clutter = false; break;
        case 'N': opts.export_noise = false; break;
        case 'm': opts.generate_matlab_script = false; break;
        case 'd': opts.seed = std::stoull(optarg); break;
        case 'h':
        default:
            return false;
        }
    }
    return true;
}

// 导出为 CSV 格式 (MATLAB 可读)
template<typename T>
bool export_csv(const std::string& filepath, const std::vector<T>& data,
                const std::vector<std::string>& headers = {"real", "imag"}) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filepath << std::endl;
        return false;
    }

    file << std::scientific << std::setprecision(15);

    // 写入表头
    for (std::size_t i = 0; i < headers.size(); ++i) {
        file << headers[i];
        if (i < headers.size() - 1) file << ",";
    }
    file << "\n";

    // 写入数据
    for (const auto& val : data) {
        file << val.real() << "," << val.imag() << "\n";
    }

    file.close();
    return true;
}

// 导出为 MAT 格式（简单二进制）
bool export_mat_binary(const std::string& filepath, const std::vector<Complex>& data,
                       const std::string& var_name = "data") {
    std::ofstream file(filepath, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }

    // 简单格式：[var_name_len(1)] [var_name] [num_elements(8)] [data...]
    uint8_t name_len = static_cast<uint8_t>(var_name.size());
    uint64_t num_elements = static_cast<uint64_t>(data.size());

    file.write(reinterpret_cast<const char*>(&name_len), sizeof(name_len));
    file.write(var_name.c_str(), name_len);
    file.write(reinterpret_cast<const char*>(&num_elements), sizeof(num_elements));

    for (const auto& val : data) {
        double r = val.real();
        double i = val.imag();
        file.write(reinterpret_cast<const char*>(&r), sizeof(r));
        file.write(reinterpret_cast<const char*>(&i), sizeof(i));
    }

    file.close();
    return true;
}

int main(int argc, char* argv[]) {
    // 解析命令行参数
    ExportOptions opts;
    if (!parse_args(argc, argv, opts)) {
        print_usage(argv[0]);
        return 1;
    }

    // 创建输出目录
    fs::create_directories(opts.output_dir);
    fs::create_directories(opts.output_dir + "/csv");
    fs::create_directories(opts.output_dir + "/binary");

    std::cout << "╔══════════════════════════════════════════╗" << std::endl;
    std::cout << "║     MATLAB Export Tool for Radar         ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════╝" << std::endl;
    std::cout << "\nConfiguration:" << std::endl;
    std::cout << "  Output directory: " << opts.output_dir << std::endl;
    std::cout << "  Waveform type:    " << opts.waveform_type << std::endl;
    std::cout << "  Pulses:           " << opts.num_pulses << std::endl;
    std::cout << "  Samples/pulse:    " << opts.samples_per_pulse << std::endl;
    std::cout << "  SNR:              " << opts.snr_db << " dB" << std::endl;
    std::cout << "  Seed:             " << opts.seed << std::endl;
    std::cout << "  Export clutter:   " << (opts.export_clutter ? "yes" : "no") << std::endl;
    std::cout << "  Export noise:     " << (opts.export_noise ? "yes" : "no") << std::endl;
    std::cout << std::endl;

    // 初始化雷达系统参数
    RadarSystemParams sys;
    sys.fc_hz = 10.0e9;         // 10 GHz
    sys.prf_hz = 1600.0;        // 1.6 kHz PRF
    sys.fs_hz = 40.0e6;         // 40 MHz 采样率
    sys.bw_hz = 20.0e6;         // 20 MHz 带宽
    sys.pulse_width_s = 20.0e-6; // 20 μs 脉宽
    sys.peak_power_w = 5000.0;  // 5 kW 峰值功率
    sys.pulses_per_cpi = opts.num_pulses;
    sys.compute_derived_params();

    // 初始化波形生成器
    waveform::WaveformGenerator waveform_gen;
    waveform::WaveformConfig wf_cfg;

    if (opts.waveform_type == "lfm") {
        wf_cfg.waveform_type = WaveformType::LFM;
    } else if (opts.waveform_type == "nlfm") {
        wf_cfg.waveform_type = WaveformType::NLFM;
        wf_cfg.nlfm_window_type = WindowType::Hamming;
    } else if (opts.waveform_type == "barker") {
        wf_cfg.waveform_type = WaveformType::PHASE_CODED;
        wf_cfg.phase_code_type = PhaseCodeType::Barker13;
    } else if (opts.waveform_type == "cw") {
        wf_cfg.waveform_type = WaveformType::CW;
    } else {
        std::cerr << "Unknown waveform type: " << opts.waveform_type << std::endl;
        return 1;
    }

    waveform_gen.set_config(wf_cfg);
    waveform_gen.set_system_params(sys);

    if (!waveform_gen.initialize()) {
        std::cerr << "Failed to initialize waveform generator" << std::endl;
        return 1;
    }

    const ComplexVec& waveform = waveform_gen.get_waveform();
    std::cout << "Waveform generated: " << waveform.size() << " samples" << std::endl;

    // 导出波形
    std::string waveform_csv = opts.output_dir + "/csv/" + opts.waveform_type + "_waveform.csv";
    if (export_csv(waveform_csv, waveform, {"sample", "real", "imag"})) {
        std::cout << "  Exported: " << waveform_csv << std::endl;
    }

    // 导出匹配滤波器
    ComplexVec mf = waveform_gen.generate_matched_filter();
    std::string mf_csv = opts.output_dir + "/csv/" + opts.waveform_type + "_matched_filter.csv";
    if (export_csv(mf_csv, mf, {"sample", "real", "imag"})) {
        std::cout << "  Exported: " << mf_csv << std::endl;
    }

    // 生成并导出噪声
    if (opts.export_noise) {
        noise::NoiseEngine noise_engine;
        noise::NoiseConfig noise_cfg;
        noise_cfg.mode = NoiseLevelMode::ComplexSigma;

        // 根据 SNR 计算噪声强度
        Scalar signal_power = waveform.size();  // 假设单位幅度
        Scalar noise_power = signal_power / std::pow(10.0, opts.snr_db / 10.0);
        noise_cfg.sigma_complex = std::sqrt(noise_power);
        noise_cfg.seed = opts.seed;

        noise_engine.set_config(noise_cfg);
        noise_engine.set_system_params(sys);
        noise_engine.initialize();

        // 生成与波形相同长度的噪声
        ComplexVec noise_samples = noise_engine.generate(opts.samples_per_pulse);

        std::string noise_csv = opts.output_dir + "/csv/noise_samples.csv";
        if (export_csv(noise_csv, noise_samples, {"sample", "real", "imag"})) {
            std::cout << "  Exported: " << noise_csv << std::endl;
        }
    }

    // 生成并导出海杂波
    if (opts.export_clutter) {
        clutter::SeaClutterConfig clutter_cfg;
        clutter_cfg.enabled = true;
        clutter_cfg.ground_range_min_m = 1000.0;
        clutter_cfg.ground_range_max_m = 10000.0;
        clutter_cfg.range_step_m = 15.0;
        clutter_cfg.beam_az_width_deg = 5.0;
        clutter_cfg.az_step_deg = 1.0;
        clutter_cfg.k_shape_nu = 4.5;
        clutter_cfg.doppler_center_hz = 0.0;
        clutter_cfg.doppler_sigma_hz = 50.0;
        clutter_cfg.pool_length_factor = 2;
        clutter_cfg.seed = opts.seed;
        clutter_cfg.morchin_sea_state = 3;

        clutter::SeaClutterModel clutter_model(clutter_cfg);

        // 初始化天线模型
        antenna::AntennaConfig ant_cfg;
        ant_cfg.model_type = PhasedArrayModelType::UPA_2D;
        ant_cfg.num_elements_az = 16;
        ant_cfg.num_elements_el = 8;
        ant_cfg.peak_gain_db = 30.0;

        antenna::AntennaModel antenna;
        antenna.set_config(ant_cfg);
        antenna.initialize();

        // 生成海杂波 CPI
        BeamPoint beam;
        beam.azimuth_deg = 0.0;
        beam.elevation_deg = 0.0;

        // 使用简化波形进行杂波生成
        ComplexVec simple_waveform(opts.samples_per_pulse, Complex(1.0, 0.0));
        CpiEcho clutter_echo;

        if (clutter_model.generate_cpi(sys, antenna, beam, 0, simple_waveform, clutter_echo)) {
            // 导出第一个脉冲的杂波
            std::vector<Complex> clutter_pulse = clutter_echo.pulses[0];
            std::string clutter_csv = opts.output_dir + "/csv/sea_clutter_pulse0.csv";
            if (export_csv(clutter_csv, clutter_pulse, {"range_bin", "real", "imag"})) {
                std::cout << "  Exported: " << clutter_csv << std::endl;
            }
        }
    }

    // 生成 MATLAB 验证脚本
    if (opts.generate_matlab_script) {
        std::string matlab_script = opts.output_dir + "/validate_signals.m";
        std::ofstream script(matlab_script);
        if (script.is_open()) {
            script << "% Radar Signal Validation Script\n"
                   << "% Generated by matlab_export_tool\n\n"
                   << "clear; close all; clc;\n\n"
                   << "%% Load waveform data\n"
                   << "waveform_data = readtable('" << opts.output_dir << "/csv/" << opts.waveform_type << "_waveform.csv');\n"
                   << "waveform = waveform_data.real + 1i * waveform_data.imag;\n\n"
                   << "%% Load matched filter\n"
                   << "mf_data = readtable('" << opts.output_dir << "/csv/" << opts.waveform_type << "_matched_filter.csv');\n"
                   << "matched_filter = mf_data.real + 1i * mf_data.imag;\n\n";

            if (opts.export_noise) {
                script << "%% Load noise samples\n"
                       << "noise_data = readtable('" << opts.output_dir << "/csv/noise_samples.csv');\n"
                       << "noise = noise_data.real + 1i * noise_data.imag;\n\n";
            }

            if (opts.export_clutter) {
                script << "%% Load sea clutter\n"
                       << "clutter_data = readtable('" << opts.output_dir << "/csv/sea_clutter_pulse0.csv');\n"
                       << "clutter = clutter_data.real + 1i * clutter_data.imag;\n\n";
            }

            script << "%% Waveform spectrum\n"
                   << "figure('Name', 'Waveform Analysis');\n"
                   << "subplot(3,1,1);\n"
                   << "plot(abs(waveform));\n"
                   << "title('Waveform Magnitude');\n"
                   << "xlabel('Sample'); ylabel('Magnitude');\n"
                   << "grid on;\n\n"

                   << "subplot(3,1,2);\n"
                   << "plot(angle(waveform));\n"
                   << "title('Waveform Phase');\n"
                   << "xlabel('Sample'); ylabel('Phase (rad)');\n"
                   << "grid on;\n\n"

                   << "subplot(3,1,3);\n"
                   << "waveform_fft = fftshift(fft(waveform, 1024));\n"
                   << "plot(20*log10(abs(waveform_fft) + 1e-20));\n"
                   << "title('Waveform Spectrum');\n"
                   << "xlabel('Frequency Bin'); ylabel('Magnitude (dB)');\n"
                   << "grid on;\n\n";

            script << "%% Matched filter output (pulse compression)\n"
                   << "figure('Name', 'Pulse Compression');\n"
                   << "compressed = ifft(fft(waveform, 1024) .* fft(matched_filter, 1024));\n"
                   << "plot(abs(fftshift(compressed)));\n"
                   << "title('Matched Filter Output');\n"
                   << "xlabel('Range Bin'); ylabel('Magnitude');\n"
                   << "grid on;\n\n";

            if (opts.export_noise) {
                script << "%% Noise statistics\n"
                       << "figure('Name', 'Noise Statistics');\n"
                       << "subplot(2,1,1);\n"
                       << "histogram(real(noise), 100);\n"
                       << "title('Noise I-component Distribution');\n"
                       << "xlabel('Real'); ylabel('Count');\n"
                       << "grid on;\n\n"

                       << "subplot(2,1,2);\n"
                       << "histogram(imag(noise), 100);\n"
                       << "title('Noise Q-component Distribution');\n"
                       << "xlabel('Imag'); ylabel('Count');\n"
                       << "grid on;\n\n";
            }

            if (opts.export_clutter) {
                script << "%% Sea clutter analysis\n"
                       << "figure('Name', 'Sea Clutter');\n"
                       << "subplot(2,1,1);\n"
                       << "plot(abs(clutter));\n"
                       << "title('Sea Clutter Magnitude');\n"
                       << "xlabel('Range Bin'); ylabel('Magnitude');\n"
                       << "grid on;\n\n"

                       << "subplot(2,1,2);\n"
                       << "clutter_fft = fftshift(fft(clutter, 512));\n"
                       << "plot(20*log10(abs(clutter_fft) + 1e-20));\n"
                       << "title('Sea Clutter Spectrum');\n"
                       << "xlabel('Doppler Bin'); ylabel('Magnitude (dB)');\n"
                       << "grid on;\n\n";
            }

            script << "fprintf('Validation complete.\\n');\n";
            script.close();
            std::cout << "  Exported: " << matlab_script << std::endl;
        }
    }

    std::cout << "\nExport complete!" << std::endl;
    std::cout << "\nTo validate in MATLAB, run:" << std::endl;
    std::cout << "  cd " << opts.output_dir << std::endl;
    std::cout << "  validate_signals" << std::endl;

    return 0;
}
