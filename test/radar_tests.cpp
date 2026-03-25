#include <chrono>
#include <cmath>
#include <complex>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "core/antenna_set.h"
#include "core/radar_params.h"
#include "core/waveform_generator.h"
#include "noise/noise_engine.h"

namespace {

using radar::AntennaScanModel;
using radar::CpiEcho;
using radar::Complex;
using radar::ComplexVec;
using radar::NoiseEngine;
using radar::NoiseLevelMode;
using radar::NoiseParams;
using radar::PhaseCodeType;
using radar::PhasedArrayAntenna;
using radar::PhasedArrayAntennaConfig;
using radar::PhasedArrayModelType;
using radar::PulseEcho;
using radar::RadarParams;
using radar::Scalar;
using radar::WaveformGenerator;
using radar::WaveformType;
using radar::WindowType;

void require_true(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool approx_equal(Scalar lhs, Scalar rhs, Scalar tol = 1e-9) {
    return std::abs(lhs - rhs) <= tol;
}

bool approx_equal_complex(const Complex& lhs, const Complex& rhs, Scalar tol = 1e-9) {
    return std::abs(lhs.real() - rhs.real()) <= tol &&
           std::abs(lhs.imag() - rhs.imag()) <= tol;
}

struct IqStats {
    Scalar mean_i = 0.0;
    Scalar mean_q = 0.0;
    Scalar var_i = 0.0;
    Scalar var_q = 0.0;
};

Scalar mean_power(const ComplexVec& data) {
    if (data.empty()) {
        return 0.0;
    }
    Scalar sum = 0.0;
    for (const auto& x : data) {
        sum += std::norm(x);
    }
    return sum / static_cast<Scalar>(data.size());
}

IqStats compute_iq_stats(const ComplexVec& data) {
    IqStats stats{};
    if (data.empty()) {
        return stats;
    }

    const Scalar n = static_cast<Scalar>(data.size());
    for (const auto& x : data) {
        stats.mean_i += x.real();
        stats.mean_q += x.imag();
    }
    stats.mean_i /= n;
    stats.mean_q /= n;

    for (const auto& x : data) {
        const Scalar di = x.real() - stats.mean_i;
        const Scalar dq = x.imag() - stats.mean_q;
        stats.var_i += di * di;
        stats.var_q += dq * dq;
    }
    stats.var_i /= n;
    stats.var_q /= n;
    return stats;
}

std::filesystem::path write_temp_csv(const std::string& stem, const std::string& content) {
    const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("radar_" + stem + "_" + std::to_string(tick) + ".csv");

    std::ofstream out(path);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to create temp csv: " + path.string());
    }
    out << content;
    return path;
}

struct TempCsvGuard {
    std::filesystem::path path;
    ~TempCsvGuard() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
};

}  // namespace

int main() {
    int total = 0;
    int passed = 0;
    int failed = 0;

    auto run_case = [&](const std::string& tag, const std::string& name,
                        const std::function<void()>& fn) {
        ++total;
        try {
            fn();
            ++passed;
            std::cout << "[" << tag << "] " << name << " ... PASS\n";
        } catch (const std::exception& ex) {
            ++failed;
            std::cout << "[" << tag << "] " << name << " ... FAIL: " << ex.what() << "\n";
        } catch (...) {
            ++failed;
            std::cout << "[" << tag << "] " << name << " ... FAIL: unknown exception\n";
        }
    };

    std::cout << "=== MODULE TESTS ===\n";

    run_case("MODULE", "RadarParams.DefaultDerivedAndValidate", [] {
        RadarParams params;
        params.compute_derived_params();
        require_true(params.samples_per_pulse > 0, "samples_per_pulse should be positive.");
        require_true(params.wavelength_m > 0.0, "wavelength_m should be positive.");
        require_true(params.validate(), "default params should validate.");
    });

    run_case("MODULE", "RadarParams.InvalidPRIRejected", [] {
        RadarParams params;
        params.prf_hz = 1000.0;
        params.pulse_width_s = 2.0e-3;  // 2ms > PRI(1ms)
        params.compute_derived_params();
        require_true(!params.validate(), "pulse_width > PRI should be invalid.");
    });

    run_case("MODULE", "RadarParams.InvalidFsBwRejected", [] {
        RadarParams params;
        params.fs_hz = 1.0e6;
        params.bw_hz = 2.0e6;
        params.compute_derived_params();
        require_true(!params.validate(), "fs < bw should be invalid for current model.");
    });

    run_case("MODULE", "WaveformGenerator.LFMAndMatchedFilter", [] {
        RadarParams params;
        params.waveform_type = WaveformType::LFM;
        params.pulse_width_s = 20.0e-6;
        params.fs_hz = 10.0e6;
        params.compute_derived_params();

        WaveformGenerator generator(params);
        const auto& waveform = generator.get_waveform();
        const auto& matched = generator.get_matched_filter();

        const std::size_t expected =
            static_cast<std::size_t>(std::ceil(params.pulse_width_s * params.fs_hz));
        require_true(waveform.size() == expected, "LFM waveform size mismatch.");
        require_true(matched.size() == waveform.size(), "Matched filter size mismatch.");

        require_true(approx_equal_complex(matched.front(), std::conj(waveform.back()), 1e-8),
                     "Matched filter first sample should be conjugate of waveform last.");
    });

    run_case("MODULE", "WaveformGenerator.PhaseCoded", [] {
        RadarParams params;
        params.waveform_type = WaveformType::PHASE_CODED;
        params.phase_code_type = PhaseCodeType::Barker13;
        params.pulse_width_s = 13.0e-6;
        params.fs_hz = 13.0e6;
        params.compute_derived_params();

        WaveformGenerator generator(params);
        const auto& waveform = generator.get_waveform();
        require_true(!waveform.empty(), "Phase-coded waveform should not be empty.");

        for (const auto& s : waveform) {
            require_true(std::abs(std::abs(s.real()) - 1.0) < 1e-8,
                         "Phase-coded waveform real part should be +/-1.");
            require_true(std::abs(s.imag()) < 1e-8, "Phase-coded waveform imag should be 0.");
        }
    });

    run_case("MODULE", "WaveformGenerator.NLFMAndSpectrumStub", [] {
        RadarParams params;
        params.waveform_type = WaveformType::NLFM;
        params.nlfm_window_type = WindowType::Hamming;
        params.pulse_width_s = 20.0e-6;
        params.fs_hz = 10.0e6;
        params.bw_hz = 2.0e6;
        params.compute_derived_params();

        WaveformGenerator generator(params);
        const auto& waveform = generator.get_waveform();
        require_true(!waveform.empty(), "NLFM waveform should not be empty.");

        bool spectrum_throw = false;
        try {
            (void)generator.compute_spectrum(waveform);
        } catch (const std::logic_error&) {
            spectrum_throw = true;
        }
        require_true(spectrum_throw, "compute_spectrum should throw logic_error.");
    });

    run_case("MODULE", "PhasedArrayAntenna.ULAResponse", [] {
        PhasedArrayAntennaConfig cfg;
        cfg.model_type = PhasedArrayModelType::ULA_1D;
        cfg.num_elements_az = 16;
        cfg.spacing_az_lambda = 0.5;
        cfg.peak_gain_db = 30.0;

        PhasedArrayAntenna antenna(cfg);
        const Scalar center = antenna.normalized_power(0.0, 0.0, 0.0, 0.0);
        const Scalar off_axis = antenna.normalized_power(30.0, 0.0, 0.0, 0.0);

        require_true(approx_equal(center, 1.0, 1e-8), "ULA center response should be 1.");
        require_true(off_axis < center, "ULA off-axis response should be lower.");
        require_true(antenna.is_target_in_beam(0.0, 0.0, 0.0, 0.0, -3.0),
                     "ULA center target should be in beam.");
    });

    run_case("MODULE", "PhasedArrayAntenna.UPAResponse", [] {
        PhasedArrayAntennaConfig cfg;
        cfg.model_type = PhasedArrayModelType::UPA_2D;
        cfg.num_elements_az = 8;
        cfg.num_elements_el = 6;
        cfg.spacing_az_lambda = 0.5;
        cfg.spacing_el_lambda = 0.5;
        cfg.peak_gain_db = 28.0;

        PhasedArrayAntenna antenna(cfg);
        const Scalar center = antenna.normalized_power(10.0, 5.0, 10.0, 5.0);
        const Scalar off_axis = antenna.normalized_power(25.0, 20.0, 10.0, 5.0);
        require_true(approx_equal(center, 1.0, 1e-8), "UPA center response should be 1.");
        require_true(off_axis < center, "UPA off-axis response should be lower.");
    });

    run_case("MODULE", "AntennaScanModel.CsvAndLoopAndEmptyProtection", [] {
        const auto csv_path = write_temp_csv(
            "scan_module",
            "# az,el\n"
            "-10,0\n"
            "0,0\n"
            "10,5\n");
        TempCsvGuard guard{csv_path};

        AntennaScanModel scan;
        require_true(scan.load_beam_table_csv(csv_path.string()),
                     "load_beam_table_csv should succeed.");
        require_true(scan.beam_count() == 3, "beam count should be 3.");

        auto beam = scan.get_beam_pointing();
        require_true(approx_equal(beam.azimuth, -10.0), "first beam az mismatch.");
        require_true(approx_equal(beam.elevation, 0.0), "first beam el mismatch.");

        scan.advance_one_cpi();
        beam = scan.get_beam_pointing();
        require_true(approx_equal(beam.azimuth, 0.0), "second beam az mismatch.");
        scan.advance_one_cpi();
        scan.advance_one_cpi();
        beam = scan.get_beam_pointing();
        require_true(approx_equal(beam.azimuth, -10.0), "beam loop behavior mismatch.");

        PhasedArrayAntennaConfig cfg;
        cfg.model_type = PhasedArrayModelType::ULA_1D;
        cfg.num_elements_az = 8;
        scan.set_antenna(PhasedArrayAntenna(cfg));

        AntennaScanModel empty_scan;
        empty_scan.set_antenna(PhasedArrayAntenna(cfg));
        bool threw = false;
        try {
            (void)empty_scan.get_gain_to_target(0.0, 0.0);
        } catch (const std::runtime_error&) {
            threw = true;
        }
        require_true(threw, "empty beam table should throw for gain query.");
    });

    run_case("MODULE", "NoiseEngine.PowerStats.AllModes", [] {
        const auto check_mode = [](const NoiseParams& params, Scalar expected_power,
                                   const std::string& mode_name) {
            NoiseEngine engine;
            require_true(engine.set_params(params),
                         "set_params should succeed in " + mode_name + " mode.");

            const auto noise = engine.generate(200000);
            const Scalar estimated_power = mean_power(noise);
            const Scalar relative_error =
                std::abs(estimated_power - expected_power) / expected_power;
            require_true(relative_error < 0.08,
                         mode_name + " power stats relative error is too large.");
        };

        NoiseParams sigma_params;
        sigma_params.mode = NoiseLevelMode::ComplexSigma;
        sigma_params.sigma_complex = 0.2;
        sigma_params.seed = 1001;
        check_mode(sigma_params, sigma_params.sigma_complex * sigma_params.sigma_complex,
                   "ComplexSigma");

        NoiseParams power_params;
        power_params.mode = NoiseLevelMode::NoisePower;
        power_params.noise_power_w = 0.25;
        power_params.seed = 1002;
        check_mode(power_params, power_params.noise_power_w, "NoisePower");

        NoiseParams thermal_params;
        thermal_params.mode = NoiseLevelMode::ThermalKTB;
        thermal_params.noise_figure_db = 3.0;
        thermal_params.system_temperature_k = 300.0;
        thermal_params.noise_bandwidth_hz = 2.0e6;
        thermal_params.seed = 1003;
        const Scalar thermal_power =
            1.380649e-23 * thermal_params.system_temperature_k * thermal_params.noise_bandwidth_hz *
            std::pow(10.0, thermal_params.noise_figure_db / 10.0);
        check_mode(thermal_params, thermal_power, "ThermalKTB");
    });

    run_case("MODULE", "NoiseEngine.IQVariance.AllModes", [] {
        const auto check_mode = [](const NoiseParams& params, Scalar expected_power,
                                   const std::string& mode_name) {
            NoiseEngine engine;
            require_true(engine.set_params(params),
                         "set_params should succeed in " + mode_name + " mode.");

            const auto noise = engine.generate(200000);
            const IqStats stats = compute_iq_stats(noise);
            const Scalar expected_var = expected_power / 2.0;

            const Scalar rel_err_i = std::abs(stats.var_i - expected_var) / expected_var;
            const Scalar rel_err_q = std::abs(stats.var_q - expected_var) / expected_var;

            require_true(rel_err_i < 0.10,
                         mode_name + " I variance relative error is too large.");
            require_true(rel_err_q < 0.10,
                         mode_name + " Q variance relative error is too large.");
        };

        NoiseParams sigma_params;
        sigma_params.mode = NoiseLevelMode::ComplexSigma;
        sigma_params.sigma_complex = 0.2;
        sigma_params.seed = 2001;
        check_mode(sigma_params, sigma_params.sigma_complex * sigma_params.sigma_complex,
                   "ComplexSigma");

        NoiseParams power_params;
        power_params.mode = NoiseLevelMode::NoisePower;
        power_params.noise_power_w = 0.25;
        power_params.seed = 2002;
        check_mode(power_params, power_params.noise_power_w, "NoisePower");

        NoiseParams thermal_params;
        thermal_params.mode = NoiseLevelMode::ThermalKTB;
        thermal_params.noise_figure_db = 3.0;
        thermal_params.system_temperature_k = 300.0;
        thermal_params.noise_bandwidth_hz = 2.0e6;
        thermal_params.seed = 2003;
        const Scalar thermal_power =
            1.380649e-23 * thermal_params.system_temperature_k * thermal_params.noise_bandwidth_hz *
            std::pow(10.0, thermal_params.noise_figure_db / 10.0);
        check_mode(thermal_params, thermal_power, "ThermalKTB");
    });

    run_case("MODULE", "NoiseEngine.InvalidParamsRollback", [] {
        NoiseParams valid;
        valid.mode = NoiseLevelMode::NoisePower;
        valid.noise_power_w = 1.0e-3;
        valid.seed = 99;

        NoiseEngine engine;
        require_true(engine.set_params(valid), "valid noise params should apply.");
        const Scalar sigma_before = engine.noise_sigma();
        const Scalar power_before = engine.noise_power_w();

        NoiseParams invalid = valid;
        invalid.mode = NoiseLevelMode::ComplexSigma;
        invalid.sigma_complex = 0.0;

        require_true(!engine.set_params(invalid), "invalid noise params should fail.");
        require_true(!engine.last_error().empty(), "invalid set should provide diagnostic error.");
        require_true(approx_equal(engine.noise_sigma(), sigma_before, 1e-12),
                     "sigma should stay unchanged after failed set.");
        require_true(approx_equal(engine.noise_power_w(), power_before, 1e-12),
                     "noise power should stay unchanged after failed set.");
    });

    run_case("MODULE", "NoiseEngine.SeedRepeatability", [] {
        NoiseParams params;
        params.mode = NoiseLevelMode::ComplexSigma;
        params.sigma_complex = 0.1;
        params.seed = 42;

        NoiseEngine engine_a(params);
        NoiseEngine engine_b(params);

        const auto noise_a = engine_a.generate(128);
        const auto noise_b = engine_b.generate(128);
        require_true(noise_a.size() == noise_b.size(), "generated size mismatch.");
        for (std::size_t i = 0; i < noise_a.size(); ++i) {
            require_true(approx_equal_complex(noise_a[i], noise_b[i], 0.0),
                         "same seed should generate identical sequence.");
        }

        NoiseEngine engine_c(params);
        const Complex first = engine_c.sample();
        engine_c.reseed(params.seed);
        const Complex reseeded_first = engine_c.sample();
        require_true(approx_equal_complex(first, reseeded_first, 0.0),
                     "reseed should restart sequence.");
    });

    run_case("MODULE", "NoiseEngine.AddNoiseCpiEcho", [] {
        NoiseParams params;
        params.mode = NoiseLevelMode::NoisePower;
        params.noise_power_w = 0.04;
        params.seed = 2026;

        NoiseEngine engine(params);

        CpiEcho cpi;
        cpi.pulses.emplace_back(4096, Complex(0.0, 0.0));
        cpi.pulses.emplace_back(4096, Complex(0.0, 0.0));
        cpi.pulses.emplace_back(4096, Complex(0.0, 0.0));
        engine.add_noise(cpi);

        Scalar total_power = 0.0;
        for (const auto& p : cpi.pulses) {
            total_power += mean_power(p);
        }
        const Scalar avg_power = total_power / static_cast<Scalar>(cpi.pulses.size());
        const Scalar relative_error = std::abs(avg_power - params.noise_power_w) / params.noise_power_w;
        require_true(relative_error < 0.10,
                     "add_noise(CpiEcho) average noise power should match configured power.");
    });

    std::cout << "=== INTEGRATION TESTS ===\n";

    run_case("INTEGRATION", "1DScanGainTrendWithWaveform", [] {
        RadarParams params;
        params.waveform_type = WaveformType::LFM;
        params.fc_hz = 10.0e9;
        params.bw_hz = 2.0e6;
        params.fs_hz = 10.0e6;
        params.pulse_width_s = 20.0e-6;
        params.prf_hz = 1600.0;
        params.pulses_per_cpi = 32;
        params.antenna_config.model_type = PhasedArrayModelType::ULA_1D;
        params.antenna_config.num_elements_az = 16;
        params.antenna_config.spacing_az_lambda = 0.5;
        params.compute_derived_params();
        require_true(params.validate(), "integration params should validate.");

        WaveformGenerator generator(params);
        require_true(!generator.get_waveform().empty(), "integration waveform should exist.");

        const auto csv_path = write_temp_csv(
            "scan_1d",
            "-10,0\n"
            "0,0\n"
            "10,0\n");
        TempCsvGuard guard{csv_path};

        AntennaScanModel scan;
        require_true(scan.load_beam_table_csv(csv_path.string()), "integration csv should load.");
        scan.set_antenna(PhasedArrayAntenna(params.antenna_config));

        const Scalar g_left = scan.get_gain_to_target(0.0, 0.0);
        scan.advance_one_cpi();
        const Scalar g_center = scan.get_gain_to_target(0.0, 0.0);
        scan.advance_one_cpi();
        const Scalar g_right = scan.get_gain_to_target(0.0, 0.0);

        require_true(g_center > g_left, "center beam gain should be greater than left.");
        require_true(g_center > g_right, "center beam gain should be greater than right.");
    });

    run_case("INTEGRATION", "2DScanGainTrendAndFailurePath", [] {
        RadarParams params;
        params.waveform_type = WaveformType::PHASE_CODED;
        params.phase_code_type = PhaseCodeType::Barker7;
        params.fc_hz = 9.5e9;
        params.bw_hz = 1.0e6;
        params.fs_hz = 8.0e6;
        params.pulse_width_s = 14.0e-6;
        params.prf_hz = 1200.0;
        params.pulses_per_cpi = 24;
        params.antenna_config.model_type = PhasedArrayModelType::UPA_2D;
        params.antenna_config.num_elements_az = 10;
        params.antenna_config.num_elements_el = 8;
        params.compute_derived_params();
        require_true(params.validate(), "integration params should validate.");

        WaveformGenerator generator(params);
        require_true(!generator.get_waveform().empty(), "integration waveform should exist.");

        const auto csv_path = write_temp_csv(
            "scan_2d",
            "0,0\n"
            "15,10\n"
            "30,20\n");
        TempCsvGuard guard{csv_path};

        AntennaScanModel scan;
        require_true(scan.load_beam_table_csv(csv_path.string()), "integration csv should load.");
        scan.set_antenna(PhasedArrayAntenna(params.antenna_config));

        const Scalar g0 = scan.get_gain_to_target(15.0, 10.0);
        scan.advance_one_cpi();
        const Scalar g1 = scan.get_gain_to_target(15.0, 10.0);
        require_true(g1 > g0, "beam steered to target should increase gain.");

        AntennaScanModel bad_scan;
        require_true(!bad_scan.load_beam_table_csv("/tmp/nonexistent_beam_table.csv"),
                     "bad csv path should fail.");
        require_true(!bad_scan.last_error().empty(), "bad csv path should provide error message.");
    });

    run_case("INTEGRATION", "WaveformPlusNoisePowerTrend", [] {
        RadarParams params;
        params.waveform_type = WaveformType::LFM;
        params.pulse_width_s = 64.0e-6;
        params.fs_hz = 8.0e6;
        params.bw_hz = 2.0e6;
        params.compute_derived_params();

        WaveformGenerator generator(params);
        ComplexVec noisy_waveform = generator.get_waveform();
        require_true(!noisy_waveform.empty(), "waveform should not be empty.");
        const Scalar clean_power = mean_power(noisy_waveform);

        NoiseParams noise_params;
        noise_params.mode = NoiseLevelMode::NoisePower;
        noise_params.noise_power_w = 0.4;
        noise_params.seed = 314159;
        NoiseEngine engine(noise_params);

        engine.add_noise(noisy_waveform);
        const Scalar noisy_power = mean_power(noisy_waveform);
        require_true(noisy_power > clean_power, "noise injection should increase waveform power.");

        CpiEcho cpi;
        cpi.pulses.push_back(generator.get_waveform());
        cpi.pulses.push_back(generator.get_waveform());
        const Scalar cpi_before = mean_power(cpi.pulses.front());
        engine.add_noise(cpi);
        const Scalar cpi_after = mean_power(cpi.pulses.front());
        require_true(cpi_after > cpi_before, "noise injection should increase CPI pulse power.");
    });

    std::cout << "[SUMMARY] total=" << total << " pass=" << passed << " fail=" << failed << "\n";
    return failed == 0 ? 0 : 1;
}
