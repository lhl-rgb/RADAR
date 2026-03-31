#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "clutter/clutter_engine.h"
#include "clutter/sea_clutter_model.h"
#include "core/antenna_set.h"
#include "core/radar_params.h"
#include "core/radar_system_params.h"
#include "core/waveform_generator.h"
#include "noise/noise_engine.h"
#include "target/target_engine.h"
#include "target/target_manager.h"

namespace {

using radar::AntennaScanModel;
using radar::AzEl;
using radar::ClutterEngine;
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
using radar::SeaClutterModel;
using radar::SeaClutterParams;
using radar::SeaClutterSequenceMode;
using radar::SwerlingType;
using radar::MotionModel;
using radar::TargetEngine;
using radar::TargetManager;
using radar::TargetState;
using radar::BeamView;
using radar::TargetList;
using radar::WaveformGenerator;
using radar::WaveformType;
using radar::WindowType;
using radar::Vec3;
using radar::RadarSystemParams;

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

struct SpectrumStats {
    Scalar center_hz = 0.0;
    Scalar sigma_hz = 0.0;
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

Scalar mean_power_cpi(const CpiEcho& cpi) {
    Scalar total_power = 0.0;
    std::size_t total_samples = 0;
    for (const auto& pulse : cpi.pulses) {
        for (const auto& sample : pulse) {
            total_power += std::norm(sample);
            ++total_samples;
        }
    }
    if (total_samples == 0) {
        return 0.0;
    }
    return total_power / static_cast<Scalar>(total_samples);
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

Scalar relative_error(Scalar actual, Scalar expected) {
    const Scalar denom = std::max(std::abs(expected), 1e-18);
    return std::abs(actual - expected) / denom;
}

ComplexVec extract_slow_time_sequence(const CpiEcho& cpi, std::size_t sample_index) {
    ComplexVec seq;
    seq.reserve(cpi.pulses.size());
    for (const auto& pulse : cpi.pulses) {
        require_true(sample_index < pulse.size(), "sample index out of pulse bound.");
        seq.push_back(pulse[sample_index]);
    }
    return seq;
}

std::size_t dominant_range_bin(const CpiEcho& cpi) {
    require_true(!cpi.pulses.empty(), "CPI should contain at least one pulse.");
    require_true(!cpi.pulses.front().empty(), "Pulse should contain at least one sample.");

    const std::size_t num_bins = cpi.pulses.front().size();
    std::vector<Scalar> power(num_bins, 0.0);
    for (const auto& pulse : cpi.pulses) {
        require_true(pulse.size() == num_bins, "Pulse length mismatch in CPI.");
        for (std::size_t i = 0; i < num_bins; ++i) {
            power[i] += std::norm(pulse[i]);
        }
    }
    return static_cast<std::size_t>(
        std::distance(power.begin(), std::max_element(power.begin(), power.end())));
}

std::vector<std::size_t> top_k_range_bins(const CpiEcho& cpi, std::size_t top_k) {
    require_true(!cpi.pulses.empty(), "CPI should contain at least one pulse.");
    const std::size_t num_bins = cpi.pulses.front().size();
    std::vector<std::pair<Scalar, std::size_t>> ranking;
    ranking.reserve(num_bins);

    for (std::size_t bin = 0; bin < num_bins; ++bin) {
        Scalar energy = 0.0;
        for (const auto& pulse : cpi.pulses) {
            energy += std::norm(pulse[bin]);
        }
        ranking.emplace_back(energy, bin);
    }

    std::sort(ranking.begin(), ranking.end(),
              [](const auto& lhs, const auto& rhs) { return lhs.first > rhs.first; });

    top_k = std::min(top_k, ranking.size());
    std::vector<std::size_t> bins;
    bins.reserve(top_k);
    for (std::size_t i = 0; i < top_k; ++i) {
        bins.push_back(ranking[i].second);
    }
    return bins;
}

SpectrumStats estimate_spectrum_stats(const ComplexVec& sequence, Scalar prf_hz) {
    SpectrumStats stats{};
    if (sequence.empty()) {
        return stats;
    }

    const std::size_t n = sequence.size();
    std::vector<Scalar> power(n, 0.0);
    Scalar power_sum = 0.0;

    for (std::size_t k = 0; k < n; ++k) {
        Complex accum(0.0, 0.0);
        for (std::size_t m = 0; m < n; ++m) {
            const Scalar phase =
                -2.0 * radar::PI * static_cast<Scalar>(k) * static_cast<Scalar>(m) /
                static_cast<Scalar>(n);
            accum += sequence[m] * Complex(std::cos(phase), std::sin(phase));
        }
        power[k] = std::norm(accum) / static_cast<Scalar>(n);
        power_sum += power[k];
    }

    if (power_sum <= 1e-18) {
        return stats;
    }

    for (std::size_t k = 0; k < n; ++k) {
        const Scalar centered_bin =
            (k <= n / 2) ? static_cast<Scalar>(k) : static_cast<Scalar>(k) - static_cast<Scalar>(n);
        const Scalar freq = centered_bin * prf_hz / static_cast<Scalar>(n);
        stats.center_hz += freq * power[k];
    }
    stats.center_hz /= power_sum;

    Scalar second_moment = 0.0;
    for (std::size_t k = 0; k < n; ++k) {
        const Scalar centered_bin =
            (k <= n / 2) ? static_cast<Scalar>(k) : static_cast<Scalar>(k) - static_cast<Scalar>(n);
        const Scalar freq = centered_bin * prf_hz / static_cast<Scalar>(n);
        const Scalar diff = freq - stats.center_hz;
        second_moment += diff * diff * power[k];
    }
    second_moment /= power_sum;
    stats.sigma_hz = std::sqrt(std::max(second_moment, 0.0));
    return stats;
}

Scalar normalized_fourth_moment(const ComplexVec& sequence) {
    if (sequence.empty()) {
        return 0.0;
    }

    Scalar second = 0.0;
    Scalar fourth = 0.0;
    for (const Complex& x : sequence) {
        const Scalar p = std::norm(x);
        second += p;
        fourth += p * p;
    }
    second /= static_cast<Scalar>(sequence.size());
    fourth /= static_cast<Scalar>(sequence.size());
    if (second <= 1e-18) {
        return 0.0;
    }
    return fourth / (second * second);
}

Scalar compute_single_cell_expected_power(const RadarParams& params,
                                          const PhasedArrayAntenna& antenna,
                                          const AzEl& beam_pointing,
                                          Scalar range_min_m,
                                          Scalar range_step_m,
                                          Scalar az_step_deg,
                                          const SeaClutterParams& clutter_params) {
    const Scalar rg = range_min_m + 0.5 * range_step_m;
    const Scalar h = std::max(params.antenna_height_m, 0.0);
    const Scalar slant = std::sqrt(h * h + rg * rg);
    const Scalar grazing = std::atan2(h, rg);
    const Scalar cell_az = beam_pointing.azimuth;
    const Scalar cell_el = -grazing * 180.0 / radar::PI;

    const Scalar sin_psi = std::max(std::sin(grazing), clutter_params.morchin.sin_psi_floor);
    const Scalar fc_ghz = std::max(params.fc_hz * 1e-9, radar::EPSILON);
    const Scalar sigma0_db = clutter_params.morchin.a0_db +
                             clutter_params.morchin.a_g * std::log10(sin_psi) +
                             clutter_params.morchin.a_f * std::log10(fc_ghz) +
                             clutter_params.morchin.a_s * clutter_params.morchin.sea_state;
    const Scalar sigma0 = std::pow(10.0, sigma0_db / 10.0);

    const Scalar cell_area = rg * range_step_m * az_step_deg * radar::PI / 180.0;
    const Scalar sigma_cell = sigma0 * cell_area;

    const Scalar g = antenna.gain(cell_az, cell_el, beam_pointing.azimuth, beam_pointing.elevation);
    const Scalar numerator = params.peak_power_w * g * g * params.wavelength_m * params.wavelength_m *
                             sigma_cell;
    const Scalar denominator = std::pow(4.0 * radar::PI, 3.0) * std::pow(slant, 4.0) *
                               std::max(params.system_loss_linear, radar::EPSILON);
    return numerator / denominator;
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

    // ==================== RadarSystemParams Tests ====================

    run_case("MODULE", "RadarSystemParams.DefaultConstructionComputesDerived", [] {
        RadarSystemParams params;

        // Check derived parameters are computed
        require_true(params.wavelength_m > 0.0, "wavelength_m should be positive.");
        require_true(params.pri_s > 0.0, "pri_s should be positive.");
        require_true(params.range_resolution_m > 0.0, "range_resolution_m should be positive.");
        require_true(params.samples_per_pulse > 0, "samples_per_pulse should be positive.");

        // Check wavelength calculation (C / fc)
        require_true(approx_equal(params.wavelength_m, radar::C / 10.0e9, 1e-6),
                     "wavelength should equal C/fc.");

        // Check PRI calculation (1 / prf)
        require_true(approx_equal(params.pri_s, 1.0 / 1600.0, 1e-9),
                     "pri should equal 1/prf.");
    });

    run_case("MODULE", "RadarSystemParams.ValidateDetectsInvalidParams", [] {
        RadarSystemParams params;
        std::string error;

        // Default params should be valid
        require_true(params.validate(error), "default params should validate.");

        // Invalid carrier frequency
        params.fc_hz = -1.0;
        require_true(!params.validate(error), "negative fc should be invalid.");
        require_true(!error.empty(), "error message should not be empty.");

        // Restore and test other invalid cases
        params.fc_hz = 10.0e9;
        params.pulse_width_s = 1.0;  // Exceeds PRI
        require_true(!params.validate(error), "pulse_width > PRI should be invalid.");
    });

    run_case("MODULE", "RadarSystemParams.DerivedParamsFormulas", [] {
        RadarSystemParams params;
        params.fc_hz = 5.0e9;
        params.bw_hz = 10.0e6;
        params.prf_hz = 2000.0;
        params.pulses_per_cpi = 64;
        params.compute_derived_params();

        // Wavelength = C / fc
        require_true(approx_equal(params.wavelength_m, radar::C / 5.0e9, 1e-6),
                     "wavelength formula check.");

        // Range resolution = C / (2 * bw)
        require_true(approx_equal(params.range_resolution_m, radar::C / (2.0 * 10.0e6), 1e-4),
                     "range resolution formula check.");

        // Max unambiguous range = C / (2 * prf)
        require_true(approx_equal(params.max_unambiguous_range_m, radar::C / (2.0 * 2000.0), 1e-2),
                     "max unambiguous range formula check.");

        // Bandwidth-time product = bw * pulse_width
        require_true(approx_equal(params.bandwidth_time_product, 10.0e6 * 20.0e-6, 1.0),
                     "bandwidth-time product formula check.");
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

    run_case("MODULE", "SeaClutter.ParamsValidation", [] {
        RadarParams params;
        params.compute_derived_params();
        require_true(params.validate(), "default radar params should validate.");

        params.sea_clutter.range_step_m = 0.0;
        require_true(!params.validate(), "range_step_m=0 should fail radar params validation.");

        SeaClutterModel model;
        SeaClutterParams invalid = params.sea_clutter;
        invalid.range_step_m = -10.0;
        require_true(!model.set_params(invalid), "invalid clutter params should be rejected.");
    });

    run_case("MODULE", "SeaClutter.DopplerNyquistConstraint", [] {
        RadarParams params;
        params.prf_hz = 1200.0;
        params.fs_hz = 4.0e6;
        params.bw_hz = 1.0e6;
        params.pulse_width_s = 1.5e-6;
        params.min_range_m = 1000.0;
        params.max_range_m = 1100.0;
        params.pulses_per_cpi = 64;
        params.antenna_height_m = 15.0;
        params.compute_derived_params();
        require_true(params.samples_per_pulse > 0, "nyquist test should produce valid samples.");

        params.sea_clutter.ground_range_min_m = 1000.0;
        params.sea_clutter.ground_range_max_m = 1100.0;
        params.sea_clutter.range_step_m = 100.0;
        params.sea_clutter.beam_az_width_deg = 0.1;
        params.sea_clutter.az_step_deg = 0.1;
        params.sea_clutter.k_shape_nu = 0.8;
        params.sea_clutter.doppler_sigma_hz = 20.0;
        params.sea_clutter.sequence_mode = SeaClutterSequenceMode::InTimeMode;
        params.sea_clutter.seed = 5002;

        params.sea_clutter.doppler_center_hz = 0.5 * params.prf_hz;
        require_true(!params.validate(), "doppler_center_hz = +PRF/2 should fail validate().");

        SeaClutterModel model;
        require_true(model.set_params(params.sea_clutter),
                     "set_params should not reject nyquist constraint without PRF context.");
        PhasedArrayAntenna antenna(params.antenna_config);
        CpiEcho invalid_echo;
        require_true(!model.generate_cpi(params, antenna, AzEl(0.0, 0.0), 0,
                                         ComplexVec{Complex(1.0, 0.0)}, invalid_echo),
                     "generate_cpi should reject out-of-nyquist doppler center.");
        require_true(model.last_error().find("[-prf_hz/2, prf_hz/2)") != std::string::npos,
                     "generate_cpi nyquist error message should be explicit.");

        SeaClutterParams valid_edge = params.sea_clutter;
        valid_edge.doppler_center_hz = -0.5 * params.prf_hz;
        require_true(model.set_params(valid_edge), "set_params should accept -PRF/2 edge.");
        CpiEcho valid_echo;
        require_true(model.generate_cpi(params, antenna, AzEl(0.0, 0.0), 1,
                                        ComplexVec{Complex(1.0, 0.0)}, valid_echo),
                     "generate_cpi should accept doppler_center_hz = -PRF/2.");
    });

    run_case("MODULE", "SeaClutter.SingleCellPowerNumeric", [] {
        RadarParams params;
        params.fc_hz = 10.0e9;
        params.prf_hz = 1000.0;
        params.fs_hz = 4.0e6;
        params.bw_hz = 1.0e6;
        params.pulse_width_s = 2.0e-6;
        params.peak_power_w = 2000.0;
        params.system_loss_db = 6.0;
        params.min_range_m = 1000.0;
        params.max_range_m = 1100.0;
        params.pulses_per_cpi = 128;
        params.antenna_height_m = 20.0;
        params.antenna_config.model_type = PhasedArrayModelType::UPA_2D;
        params.compute_derived_params();
        require_true(params.samples_per_pulse > 0,
                     "single-cell test should produce valid fast-time sample count.");

        SeaClutterParams clutter = params.sea_clutter;
        clutter.ground_range_min_m = 1000.0;
        clutter.ground_range_max_m = 1100.0;
        clutter.range_step_m = 100.0;
        clutter.beam_az_width_deg = 0.1;
        clutter.az_step_deg = 0.1;
        clutter.doppler_center_hz = 0.0;
        clutter.doppler_sigma_hz = 25.0;
        clutter.k_shape_nu = 0.8;
        clutter.sequence_mode = SeaClutterSequenceMode::InTimeMode;
        clutter.seed = 2026;

        SeaClutterModel model;
        require_true(model.set_params(clutter), "set_params should succeed for single-cell case.");

        PhasedArrayAntenna antenna(params.antenna_config);
        CpiEcho clutter_echo;
        const AzEl beam(0.0, 0.0);
        const ComplexVec tx = {Complex(1.0, 0.0)};

        require_true(model.generate_cpi(params, antenna, beam, 0, tx, clutter_echo),
                     "single-cell clutter generation should succeed.");
        const std::size_t bin = dominant_range_bin(clutter_echo);
        const ComplexVec slow_time = extract_slow_time_sequence(clutter_echo, bin);

        const Scalar measured_power = mean_power(slow_time);
        const Scalar expected_power =
            compute_single_cell_expected_power(params, antenna, beam, 1000.0, 100.0, 0.1, clutter);
        const Scalar rel_err = relative_error(measured_power, expected_power);

        std::cout << "    [SeaClutter.SingleCellPower] expected=" << expected_power
                  << " measured=" << measured_power
                  << " rel_err=" << rel_err << "\n";
        require_true(rel_err < 0.20, "single-cell average power mismatch is too large.");
    });

    run_case("MODULE", "SeaClutter.GaussianDopplerStatsNumeric", [] {
        RadarParams params;
        params.prf_hz = 1200.0;
        params.fs_hz = 4.0e6;
        params.bw_hz = 1.0e6;
        params.pulse_width_s = 1.5e-6;
        params.min_range_m = 1000.0;
        params.max_range_m = 1100.0;
        params.pulses_per_cpi = 256;
        params.antenna_height_m = 15.0;
        params.compute_derived_params();
        require_true(params.samples_per_pulse > 0,
                     "doppler test should produce valid fast-time sample count.");

        SeaClutterParams clutter = params.sea_clutter;
        clutter.ground_range_min_m = 1000.0;
        clutter.ground_range_max_m = 1100.0;
        clutter.range_step_m = 100.0;
        clutter.beam_az_width_deg = 0.1;
        clutter.az_step_deg = 0.1;
        clutter.doppler_center_hz = 45.0;
        clutter.doppler_sigma_hz = 18.0;
        clutter.k_shape_nu = 100.0;
        clutter.sequence_mode = SeaClutterSequenceMode::InTimeMode;
        clutter.seed = 901;

        SeaClutterModel model;
        require_true(model.set_params(clutter), "set_params should succeed for doppler stats test.");

        PhasedArrayAntenna antenna(params.antenna_config);
        CpiEcho clutter_echo;
        require_true(model.generate_cpi(params, antenna, AzEl(0.0, 0.0), 1,
                                        ComplexVec{Complex(1.0, 0.0)}, clutter_echo),
                     "doppler clutter generation should succeed.");

        const std::size_t bin = dominant_range_bin(clutter_echo);
        const ComplexVec slow_time = extract_slow_time_sequence(clutter_echo, bin);
        const SpectrumStats stats = estimate_spectrum_stats(slow_time, params.prf_hz);

        const Scalar center_err = std::abs(stats.center_hz - clutter.doppler_center_hz);
        const Scalar sigma_rel_err = relative_error(stats.sigma_hz, clutter.doppler_sigma_hz);
        std::cout << "    [SeaClutter.DopplerStats] cfg_center=" << clutter.doppler_center_hz
                  << " est_center=" << stats.center_hz
                  << " cfg_sigma=" << clutter.doppler_sigma_hz
                  << " est_sigma=" << stats.sigma_hz
                  << " center_err=" << center_err
                  << " sigma_rel_err=" << sigma_rel_err << "\n";

        require_true(center_err < 12.0, "estimated doppler center deviates too much.");
        require_true(sigma_rel_err < 0.50, "estimated doppler sigma deviates too much.");
    });

    run_case("MODULE", "SeaClutter.KDistributionTailBehavior", [] {
        RadarParams params;
        params.prf_hz = 1000.0;
        params.fs_hz = 4.0e6;
        params.bw_hz = 1.0e6;
        params.pulse_width_s = 1.5e-6;
        params.min_range_m = 1000.0;
        params.max_range_m = 1100.0;
        params.pulses_per_cpi = 256;
        params.compute_derived_params();
        require_true(params.samples_per_pulse > 0,
                     "k-tail test should produce valid fast-time sample count.");

        auto make_sequence = [&](Scalar nu, uint64_t seed) {
            SeaClutterParams clutter = params.sea_clutter;
            clutter.ground_range_min_m = 1000.0;
            clutter.ground_range_max_m = 1100.0;
            clutter.range_step_m = 100.0;
            clutter.beam_az_width_deg = 0.1;
            clutter.az_step_deg = 0.1;
            clutter.k_shape_nu = nu;
            clutter.doppler_center_hz = 0.0;
            clutter.doppler_sigma_hz = 20.0;
            clutter.sequence_mode = SeaClutterSequenceMode::InTimeMode;
            clutter.seed = seed;

            SeaClutterModel model;
            require_true(model.set_params(clutter), "set_params should succeed.");

            CpiEcho clutter_echo;
            PhasedArrayAntenna antenna(params.antenna_config);
            require_true(model.generate_cpi(params, antenna, AzEl(0.0, 0.0), 3,
                                            ComplexVec{Complex(1.0, 0.0)}, clutter_echo),
                         "k-tail clutter generation should succeed.");
            return extract_slow_time_sequence(clutter_echo, dominant_range_bin(clutter_echo));
        };

        const ComplexVec seq_heavy = make_sequence(0.5, 1001);
        const ComplexVec seq_light = make_sequence(100.0, 1002);
        const Scalar m4_heavy = normalized_fourth_moment(seq_heavy);
        const Scalar m4_light = normalized_fourth_moment(seq_light);

        std::cout << "    [SeaClutter.KTail] m4_heavy=" << m4_heavy
                  << " m4_light=" << m4_light << "\n";
        require_true(m4_heavy > m4_light + 0.15,
                     "low-nu K distribution should have heavier tail than high-nu baseline.");
    });

    run_case("MODULE", "SeaClutter.SequenceModesAndDelayMapping", [] {
        RadarParams params;
        params.prf_hz = 1000.0;
        params.fs_hz = 3.0e6;
        params.bw_hz = 1.0e6;
        params.pulse_width_s = 1.0e-6;
        params.min_range_m = 1000.0;
        params.max_range_m = 1200.0;
        params.pulses_per_cpi = 64;
        params.antenna_height_m = 10.0;
        params.compute_derived_params();
        require_true(params.samples_per_pulse > 0,
                     "sequence-mode test should produce valid fast-time sample count.");

        SeaClutterParams clutter = params.sea_clutter;
        clutter.ground_range_min_m = 1000.0;
        clutter.ground_range_max_m = 1200.0;
        clutter.range_step_m = 100.0;
        clutter.beam_az_width_deg = 0.1;
        clutter.az_step_deg = 0.1;
        clutter.doppler_center_hz = 0.0;
        clutter.doppler_sigma_hz = 18.0;
        clutter.k_shape_nu = 0.9;
        clutter.seed = 4040;
        clutter.sequence_mode = SeaClutterSequenceMode::InTimeMode;

        PhasedArrayAntenna antenna(params.antenna_config);
        SeaClutterModel model;
        require_true(model.set_params(clutter), "set_params should succeed.");

        CpiEcho a, b;
        const ComplexVec tx = {Complex(1.0, 0.0)};
        require_true(model.generate_cpi(params, antenna, AzEl(0.0, 0.0), 9, tx, a),
                     "deterministic sequence generation A should succeed.");
        require_true(model.generate_cpi(params, antenna, AzEl(0.0, 0.0), 9, tx, b),
                     "deterministic sequence generation B should succeed.");

        require_true(a.pulses.size() == b.pulses.size(), "deterministic CPI pulse count mismatch.");
        for (std::size_t p = 0; p < a.pulses.size(); ++p) {
            require_true(a.pulses[p].size() == b.pulses[p].size(),
                         "deterministic CPI pulse size mismatch.");
            for (std::size_t n = 0; n < a.pulses[p].size(); ++n) {
                require_true(approx_equal_complex(a.pulses[p][n], b.pulses[p][n], 0.0),
                             "deterministic mode should be fully reproducible.");
            }
        }

        const Scalar h = params.antenna_height_m;
        const Scalar rg_min = 1000.0;
        const Scalar rg_center = rg_min + 0.5 * clutter.range_step_m;
        const Scalar tau_ref = 2.0 * std::sqrt(h * h + rg_min * rg_min) / radar::C;
        const Scalar tau_cell = 2.0 * std::sqrt(h * h + rg_center * rg_center) / radar::C;
        const int expected_n0 = static_cast<int>(std::llround((tau_cell - tau_ref) * params.fs_hz));
        const std::size_t observed_n0 = dominant_range_bin(a);
        std::cout << "    [SeaClutter.DelayMap] expected_n0=" << expected_n0
                  << " observed_n0=" << observed_n0 << "\n";
        require_true(expected_n0 == static_cast<int>(observed_n0),
                     "nearest delay mapping index mismatch.");

        clutter.sequence_mode = SeaClutterSequenceMode::SequencePoolMode;
        clutter.pool_length_factor = 64;
        require_true(model.set_params(clutter), "pool mode set_params should succeed.");

        CpiEcho pool_a, pool_b;
        require_true(model.generate_cpi(params, antenna, AzEl(0.0, 0.0), 11, tx, pool_a),
                     "pool mode generation A should succeed.");
        require_true(model.generate_cpi(params, antenna, AzEl(0.0, 0.0), 11, tx, pool_b),
                     "pool mode generation B should succeed.");

        for (std::size_t p = 0; p < pool_a.pulses.size(); ++p) {
            for (std::size_t n = 0; n < pool_a.pulses[p].size(); ++n) {
                require_true(approx_equal_complex(pool_a.pulses[p][n], pool_b.pulses[p][n], 0.0),
                             "pool mode should be reproducible under fixed seed.");
            }
        }

        const auto bins = top_k_range_bins(pool_a, 2);
        require_true(bins.size() >= 2, "pool mode should contain at least two active bins.");
        const ComplexVec seq0 = extract_slow_time_sequence(pool_a, bins[0]);
        const ComplexVec seq1 = extract_slow_time_sequence(pool_a, bins[1]);

        bool exactly_same = true;
        for (std::size_t i = 0; i < seq0.size(); ++i) {
            if (!approx_equal_complex(seq0[i], seq1[i], 0.0)) {
                exactly_same = false;
                break;
            }
        }
        require_true(!exactly_same, "different cells should use different random start segments.");
    });

    // ==================== TargetManager Tests ====================

    run_case("MODULE", "TargetManager.StationaryTargetNoMotion", [] {
        TargetManager mgr;
        std::vector<TargetState> targets{
            TargetState{1, Vec3(1000.0, 0.0, 0.0), Vec3::Zero(), Vec3::Zero(),
                        MotionModel::Stationary, 5.0, SwerlingType::Swerling0, true}
        };
        mgr.set_targets(targets);

        // 时间推进后位置应该不变
        mgr.update_to_time(0.0);
        require_true(mgr.get_current_targets()[0].position_m.x() == 1000.0,
                     "stationary target position at t=0 should be initial position.");

        mgr.update_to_time(10.0);
        require_true(mgr.get_current_targets()[0].position_m.x() == 1000.0,
                     "stationary target position at t=10 should still be initial position.");
    });

    run_case("MODULE", "TargetManager.ConstantVelocityMotion", [] {
        TargetManager mgr;
        const Scalar v = 50.0;  // 50 m/s
        std::vector<TargetState> targets{
            TargetState{2, Vec3(0.0, 0.0, 0.0), Vec3(v, 0.0, 0.0), Vec3::Zero(),
                        MotionModel::ConstantVelocity, 5.0, SwerlingType::Swerling0, true}
        };
        mgr.set_targets(targets);

        // t=0 位置应该是初始位置
        mgr.update_to_time(0.0);
        require_true(mgr.get_current_targets()[0].position_m.x() == 0.0,
                     "CV target at t=0 should be at initial position.");

        // t=10 位置应该是 0 + 50*10 = 500m
        mgr.update_to_time(10.0);
        require_true(mgr.get_current_targets()[0].position_m.x() == 500.0,
                     "CV target at t=10 should have moved 500m.");

        // t=20 位置应该是 1000m
        mgr.update_to_time(20.0);
        require_true(mgr.get_current_targets()[0].position_m.x() == 1000.0,
                     "CV target at t=20 should have moved 1000m.");
    });

    run_case("MODULE", "TargetManager.ConstantAccelerationMotion", [] {
        TargetManager mgr;
        const Scalar v0 = 0.0;
        const Scalar a = 10.0;  // 10 m/s^2
        std::vector<TargetState> targets{
            TargetState{3, Vec3(0.0, 0.0, 0.0), Vec3(v0, 0.0, 0.0), Vec3(a, 0.0, 0.0),
                        MotionModel::ConstantAcceleration, 5.0, SwerlingType::Swerling0, true}
        };
        mgr.set_targets(targets);

        // t=0: x = 0, v = 0
        mgr.update_to_time(0.0);
        require_true(mgr.get_current_targets()[0].position_m.x() == 0.0,
                     "CA target at t=0 position should be 0.");
        require_true(mgr.get_current_targets()[0].velocity_mps.x() == 0.0,
                     "CA target at t=0 velocity should be 0.");

        // t=10: x = 0.5 * 10 * 100 = 500m, v = 10 * 10 = 100 m/s
        mgr.update_to_time(10.0);
        require_true(mgr.get_current_targets()[0].position_m.x() == 500.0,
                     "CA target at t=10 position should be 500m.");
        require_true(mgr.get_current_targets()[0].velocity_mps.x() == 100.0,
                     "CA target at t=10 velocity should be 100 m/s.");

        // t=20: x = 0.5 * 10 * 400 = 2000m, v = 10 * 20 = 200 m/s
        mgr.update_to_time(20.0);
        require_true(mgr.get_current_targets()[0].position_m.x() == 2000.0,
                     "CA target at t=20 position should be 2000m.");
        require_true(mgr.get_current_targets()[0].velocity_mps.x() == 200.0,
                     "CA target at t=20 velocity should be 200 m/s.");
    });

    run_case("MODULE", "TargetManager.MultipleTargets", [] {
        TargetManager mgr;
        std::vector<TargetState> targets{
            TargetState{1, Vec3(1000.0, 0.0, 0.0), Vec3::Zero(), Vec3::Zero(),
                        MotionModel::Stationary, 5.0, SwerlingType::Swerling0, true},
            TargetState{2, Vec3(0.0, 0.0, 0.0), Vec3(50.0, 0.0, 0.0), Vec3::Zero(),
                        MotionModel::ConstantVelocity, 5.0, SwerlingType::Swerling0, true},
            TargetState{3, Vec3(0.0, 0.0, 0.0), Vec3::Zero(), Vec3(10.0, 0.0, 0.0),
                        MotionModel::ConstantAcceleration, 5.0, SwerlingType::Swerling0, true}
        };
        mgr.set_targets(targets);

        mgr.update_to_time(10.0);

        const auto& current = mgr.get_current_targets();
        require_true(current.size() == 3, "should have 3 targets.");

        // 目标1：静止，位置不变
        require_true(current[0].position_m.x() == 1000.0,
                     "target 1 (stationary) position should not change.");

        // 目标2：匀速，x = 0 + 50*10 = 500m
        require_true(current[1].position_m.x() == 500.0,
                     "target 2 (CV) should be at 500m.");

        // 目标3：匀加速，x = 0.5 * 10 * 100 = 500m
        require_true(current[2].position_m.x() == 500.0,
                     "target 3 (CA) should be at 500m.");
    });

    // ==================== TargetEngine Tests ====================

    run_case("MODULE", "TargetEngine.EmptyTargetsReturnZeroEcho", [] {
        RadarParams params;
        params.fc_hz = 10.0e9;
        params.prf_hz = 1000.0;
        params.fs_hz = 5.0e6;
        params.bw_hz = 1.0e6;
        params.pulse_width_s = 2.0e-6;
        params.pulses_per_cpi = 16;
        params.min_range_m = 1000.0;
        params.max_range_m = 3000.0;
        params.compute_derived_params();
        require_true(params.validate(), "target empty test radar params should validate.");
        WaveformGenerator wf(params);

        TargetEngine engine;
        CpiEcho echo;
        require_true(engine.generate({}, BeamView{AzEl(0.0, 0.0), 0, nullptr},
                                     params, wf.get_waveform(), echo),
                     "empty target list should generate a valid zero echo.");
        require_true(echo.pulses.size() == static_cast<std::size_t>(params.pulses_per_cpi),
                     "pulse count mismatch.");
        require_true(!echo.pulses.empty(), "echo should contain pulses.");
        require_true(echo.pulses.front().size() == static_cast<std::size_t>(params.samples_per_pulse),
                     "sample count mismatch.");
        require_true(mean_power_cpi(echo) == 0.0, "empty target output should be all zeros.");
    });

    run_case("MODULE", "TargetEngine.GenerateFromRadarParamsTargets", [] {
        RadarParams params;
        params.fc_hz = 10.0e9;
        params.prf_hz = 1000.0;
        params.fs_hz = 6.0e6;
        params.bw_hz = 1.0e6;
        params.pulse_width_s = 2.0e-6;
        params.pulses_per_cpi = 16;
        params.min_range_m = 1000.0;
        params.max_range_m = 3000.0;
        params.target_params.enable_phase = false;
        params.target_params.enable_two_way_propagation_loss = false;
        params.target_params.enable_swerling = false;
        params.compute_derived_params();
        require_true(params.validate(), "radar params should validate.");
        WaveformGenerator wf(params);

        const int expected_bin = 20;
        const Scalar range_bin_size_m = radar::C / (2.0 * params.fs_hz);
        const Scalar target_range_m = params.min_range_m + expected_bin * range_bin_size_m;

        TargetList targets = {
            TargetState{99, Vec3(target_range_m, 0.0, 0.0), Vec3::Zero(), Vec3::Zero(),
                        radar::MotionModel::Stationary, 10.0, SwerlingType::Swerling0, true}
        };

        TargetEngine engine;
        CpiEcho echo;
        require_true(engine.generate(targets,
                                     BeamView{AzEl(0.0, 0.0), 0, nullptr},
                                     params, wf.get_waveform(), echo),
                     "generation with active target list should succeed.");

        const std::size_t peak = dominant_range_bin(echo);
        require_true(static_cast<int>(peak) == expected_bin,
                     "dominant range bin should match configured inlined target.");
    });

    run_case("MODULE", "TargetEngine.SingleStationaryTargetRangeBin", [] {
        RadarParams params;
        params.fc_hz = 10.0e9;
        params.prf_hz = 1000.0;
        params.fs_hz = 6.0e6;
        params.bw_hz = 1.0e6;
        params.pulse_width_s = 2.0e-6;
        params.pulses_per_cpi = 16;
        params.min_range_m = 1000.0;
        params.max_range_m = 3000.0;
        params.target_params.enable_phase = false;
        params.target_params.enable_two_way_propagation_loss = false;
        params.target_params.enable_swerling = false;
        params.compute_derived_params();
        require_true(params.validate(), "stationary test radar params should validate.");
        WaveformGenerator wf(params);

        TargetEngine engine;

        const int expected_bin = 20;
        const Scalar range_bin_size_m = radar::C / (2.0 * params.fs_hz);
        const Scalar target_range_m = params.min_range_m + expected_bin * range_bin_size_m;

        TargetList targets{
            TargetState{1, Vec3(target_range_m, 0.0, 0.0), Vec3::Zero(), Vec3::Zero(),
                        radar::MotionModel::Stationary, 10.0, SwerlingType::Swerling0, true}
        };

        CpiEcho echo;
        require_true(engine.generate(targets, BeamView{AzEl(0.0, 0.0), 0, nullptr},
                                     params, wf.get_waveform(), echo),
                     "stationary single-target generation should succeed.");
        const std::size_t peak = dominant_range_bin(echo);
        require_true(static_cast<int>(peak) == expected_bin,
                     "dominant range bin should match theoretical delay mapping.");
    });

    run_case("MODULE", "TargetEngine.SingleMovingTargetDoppler", [] {
        RadarParams params;
        params.fc_hz = 10.0e9;
        params.prf_hz = 1200.0;
        params.fs_hz = 6.0e6;
        params.bw_hz = 1.0e6;
        params.pulse_width_s = 2.0e-6;
        params.pulses_per_cpi = 128;
        params.min_range_m = 1000.0;
        params.max_range_m = 4000.0;
        params.target_params.enable_phase = true;
        params.target_params.enable_two_way_propagation_loss = false;
        params.target_params.enable_swerling = false;
        params.compute_derived_params();
        require_true(params.validate(), "moving target radar params should validate.");
        WaveformGenerator wf(params);

        TargetEngine engine;

        const Scalar vr_mps = 3.0;
        TargetList targets{
            TargetState{2, Vec3(2000.0, 0.0, 0.0), Vec3(vr_mps, 0.0, 0.0), Vec3::Zero(),
                        radar::MotionModel::ConstantVelocity, 8.0, SwerlingType::Swerling0, true}
        };

        CpiEcho echo;
        require_true(engine.generate(targets, BeamView{AzEl(0.0, 0.0), 3, nullptr},
                                     params, wf.get_waveform(), echo),
                     "moving single-target generation should succeed.");
        const std::size_t peak = dominant_range_bin(echo);
        const ComplexVec slow_time = extract_slow_time_sequence(echo, peak);
        const SpectrumStats stats = estimate_spectrum_stats(slow_time, params.prf_hz);

        const Scalar expected_fd_hz = 2.0 * vr_mps / params.wavelength_m;
        const Scalar err_hz = std::abs(std::abs(stats.center_hz) - expected_fd_hz);
        require_true(err_hz < 20.0, "estimated doppler center should match theoretical value.");
    });

    run_case("MODULE", "TargetEngine.SwerlingSlowFastBehavior", [] {
        RadarParams params;
        params.fc_hz = 10.0e9;
        params.prf_hz = 1000.0;
        params.fs_hz = 6.0e6;
        params.bw_hz = 1.0e6;
        params.pulse_width_s = 2.0e-6;
        params.pulses_per_cpi = 64;
        params.min_range_m = 1000.0;
        params.max_range_m = 4000.0;
        params.target_params.enable_phase = false;
        params.target_params.enable_two_way_propagation_loss = false;
        params.target_params.enable_swerling = true;
        params.target_params.seed = 42;
        params.compute_derived_params();
        require_true(params.validate(), "swerling test radar params should validate.");
        WaveformGenerator wf(params);

        TargetEngine engine;

        auto pulse_mag_span = [&](const SwerlingType model) {
            TargetList targets{
                TargetState{10, Vec3(1800.0, 0.0, 0.0), Vec3::Zero(), Vec3::Zero(),
                            radar::MotionModel::Stationary, 5.0, model, true}
            };
            CpiEcho echo;
            require_true(engine.generate(targets, BeamView{AzEl(0.0, 0.0), 5, nullptr},
                                         params, wf.get_waveform(), echo),
                         "swerling target generation should succeed.");
            const std::size_t peak = dominant_range_bin(echo);
            Scalar min_mag = std::numeric_limits<Scalar>::infinity();
            Scalar max_mag = 0.0;
            for (const auto& pulse : echo.pulses) {
                const Scalar mag = std::abs(pulse[peak]);
                min_mag = std::min(min_mag, mag);
                max_mag = std::max(max_mag, mag);
            }
            return max_mag - min_mag;
        };

        const Scalar span_slow = pulse_mag_span(SwerlingType::Swerling1);
        const Scalar span_fast = pulse_mag_span(SwerlingType::Swerling2);

        require_true(span_slow < 1e-12, "Swerling I should be constant inside one CPI.");
        require_true(span_fast > 1e-6, "Swerling II should fluctuate pulse-by-pulse.");
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

    run_case("INTEGRATION", "AntennaScanWithSeaClutter.1DAnd2D", [] {
        const auto csv_1d = write_temp_csv(
            "sea_scan_1d",
            "-40,0\n"
            "0,0\n"
            "40,0\n");
        TempCsvGuard guard_1d{csv_1d};

        RadarParams p1;
        p1.waveform_type = WaveformType::LFM;
        p1.fc_hz = 10.0e9;
        p1.prf_hz = 1000.0;
        p1.fs_hz = 4.0e6;
        p1.bw_hz = 1.0e6;
        p1.pulse_width_s = 2.0e-6;
        p1.pulses_per_cpi = 64;
        p1.min_range_m = 1000.0;
        p1.max_range_m = 4000.0;
        p1.antenna_config.model_type = PhasedArrayModelType::ULA_1D;
        p1.antenna_config.num_elements_az = 16;
        p1.compute_derived_params();
        require_true(p1.validate(), "1D sea clutter integration params should validate.");

        p1.sea_clutter.ground_range_min_m = 1000.0;
        p1.sea_clutter.ground_range_max_m = 2500.0;
        p1.sea_clutter.range_step_m = 200.0;
        p1.sea_clutter.beam_az_width_deg = 3.0;
        p1.sea_clutter.az_step_deg = 0.1;
        p1.sea_clutter.doppler_center_hz = 0.0;
        p1.sea_clutter.doppler_sigma_hz = 20.0;
        p1.sea_clutter.k_shape_nu = 0.8;
        p1.sea_clutter.sequence_mode = SeaClutterSequenceMode::InTimeMode;
        p1.sea_clutter.seed = 5001;

        AntennaScanModel scan_1d;
        require_true(scan_1d.load_beam_table_csv(csv_1d.string()), "1D scan csv should load.");
        scan_1d.set_antenna(PhasedArrayAntenna(p1.antenna_config));

        WaveformGenerator wf_1d(p1);
        ClutterEngine clutter_engine_1d;
        require_true(clutter_engine_1d.set_sea_params(p1.sea_clutter),
                     "clutter engine 1D set_sea_params should succeed.");

        std::vector<Scalar> power_1d;
        for (int i = 0; i < 3; ++i) {
            CpiEcho clutter;
            const AzEl beam = scan_1d.get_beam_pointing();
            require_true(clutter_engine_1d.generate_sea_clutter_cpi(
                             p1, PhasedArrayAntenna(p1.antenna_config), beam, i, wf_1d.get_waveform(),
                             clutter),
                         "1D clutter generation should succeed.");
            power_1d.push_back(mean_power_cpi(clutter));
            scan_1d.advance_one_cpi();
        }

        const Scalar pmin = *std::min_element(power_1d.begin(), power_1d.end());
        const Scalar pmax = *std::max_element(power_1d.begin(), power_1d.end());
        const Scalar variation = relative_error(pmax, pmin);
        std::cout << "    [SeaClutter.1DScan] p0=" << power_1d[0]
                  << " p1=" << power_1d[1]
                  << " p2=" << power_1d[2]
                  << " variation=" << variation << "\n";
        require_true(variation > 0.03, "1D steering should change clutter power trend.");

        const auto csv_2d = write_temp_csv(
            "sea_scan_2d",
            "0,-1\n"
            "0,8\n"
            "0,16\n");
        TempCsvGuard guard_2d{csv_2d};

        RadarParams p2 = p1;
        p2.antenna_config.model_type = PhasedArrayModelType::UPA_2D;
        p2.antenna_config.num_elements_az = 12;
        p2.antenna_config.num_elements_el = 10;
        p2.compute_derived_params();
        require_true(p2.validate(), "2D sea clutter integration params should validate.");

        AntennaScanModel scan_2d;
        require_true(scan_2d.load_beam_table_csv(csv_2d.string()), "2D scan csv should load.");
        scan_2d.set_antenna(PhasedArrayAntenna(p2.antenna_config));

        WaveformGenerator wf_2d(p2);
        ClutterEngine clutter_engine_2d;
        require_true(clutter_engine_2d.set_sea_params(p2.sea_clutter),
                     "clutter engine 2D set_sea_params should succeed.");

        std::vector<Scalar> power_2d;
        for (int i = 0; i < 3; ++i) {
            CpiEcho clutter;
            const AzEl beam = scan_2d.get_beam_pointing();
            require_true(clutter_engine_2d.generate_sea_clutter_cpi(
                             p2, PhasedArrayAntenna(p2.antenna_config), beam, i, wf_2d.get_waveform(),
                             clutter),
                         "2D clutter generation should succeed.");
            power_2d.push_back(mean_power_cpi(clutter));
            scan_2d.advance_one_cpi();
        }
        std::cout << "    [SeaClutter.2DScan] p(-1deg)=" << power_2d[0]
                  << " p(8deg)=" << power_2d[1]
                  << " p(16deg)=" << power_2d[2] << "\n";
        require_true(power_2d[0] > power_2d[1] && power_2d[1] > power_2d[2],
                     "2D scan elevation should show monotonic sea clutter attenuation.");
    });

    run_case("INTEGRATION", "WaveformSeaNoiseChain", [] {
        RadarParams params;
        params.waveform_type = WaveformType::LFM;
        params.fc_hz = 9.5e9;
        params.prf_hz = 1000.0;
        params.fs_hz = 8.0e6;
        params.bw_hz = 2.0e6;
        params.pulse_width_s = 10.0e-6;
        params.pulses_per_cpi = 64;
        params.min_range_m = 800.0;
        params.max_range_m = 6000.0;
        params.compute_derived_params();
        require_true(params.validate(), "chain integration params should validate.");

        params.sea_clutter.ground_range_min_m = 800.0;
        params.sea_clutter.ground_range_max_m = 3000.0;
        params.sea_clutter.range_step_m = 200.0;
        params.sea_clutter.beam_az_width_deg = 3.0;
        params.sea_clutter.az_step_deg = 0.1;
        params.sea_clutter.k_shape_nu = 0.8;
        params.sea_clutter.doppler_center_hz = 0.0;
        params.sea_clutter.doppler_sigma_hz = 20.0;
        params.sea_clutter.sequence_mode = SeaClutterSequenceMode::InTimeMode;
        params.sea_clutter.seed = 7007;

        WaveformGenerator wf(params);
        PhasedArrayAntenna antenna(params.antenna_config);
        ClutterEngine clutter_engine;
        require_true(clutter_engine.set_sea_params(params.sea_clutter),
                     "chain clutter set_sea_params should succeed.");

        CpiEcho clutter;
        require_true(clutter_engine.generate_sea_clutter_cpi(
                         params, antenna, AzEl(0.0, 0.0), 0, wf.get_waveform(), clutter),
                     "chain sea clutter generation should succeed.");
        const Scalar clutter_power = mean_power_cpi(clutter);

        NoiseParams noise_params;
        noise_params.mode = NoiseLevelMode::NoisePower;
        noise_params.noise_power_w = 1.0e-7;
        noise_params.seed = 777;
        NoiseEngine noise(noise_params);

        CpiEcho noisy = clutter;
        noise.add_noise(noisy);
        const Scalar noisy_power = mean_power_cpi(noisy);

        std::cout << "    [SeaClutter.NoiseChain] clutter_power=" << clutter_power
                  << " noisy_power=" << noisy_power << "\n";
        require_true(noisy_power > clutter_power, "adding noise should increase total CPI power.");
    });

    std::cout << "[SUMMARY] total=" << total << " pass=" << passed << " fail=" << failed << "\n";
    return failed == 0 ? 0 : 1;
}
