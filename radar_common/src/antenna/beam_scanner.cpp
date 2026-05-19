/**
 * @file beam_scanner.cpp
 * @brief 波束扫描控制器实现
 */

#include "antenna/beam_scanner.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace radar::antenna {

namespace {

std::string trim(const std::string& value) {
    const std::size_t begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const std::size_t end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

bool parse_double_strict(const std::string& text, Scalar& output) {
    std::size_t consumed = 0;
    try {
        output = std::stod(text, &consumed);
    } catch (...) {
        return false;
    }
    return consumed == text.size();
}

constexpr const char* kEmptyBeamTableError =
    "BeamScanner: beam table is empty. Load or set beam table before gain query.";

}  // namespace

bool BeamScanner::initialize() {
    // 1. 初始化天线模型
    antenna_model_.set_config(antenna_config_);
    if (!antenna_model_.initialize()) {
        last_error_ = "AntennaModel initialization failed";
        return false;
    }

    // 2. 如果配置了波位表，根据配置加载
    if (!beam_table_config_.type.empty() || !beam_table_config_.file_path.empty() ||
        !beam_table_config_.beams.empty()) {

        // 1. 如果 type="file"，从 CSV 文件加载
        if (beam_table_config_.type == "file" && !beam_table_config_.file_path.empty()) {
            if (!load_beam_table_csv(beam_table_config_.file_path)) {
                return false;
            }
        }
        // 2. 如果有直接定义的波位列表，直接使用
        else if (!beam_table_config_.beams.empty()) {
            BeamTable table;
            for (const auto& beam : beam_table_config_.beams) {
                table.push_back({beam.azimuth_deg, beam.elevation_deg});
            }
            set_beam_table(table);
        }
        // 3. 方位扫描模式：根据参数生成波位表
        else if (beam_table_config_.type == "azimuth_scan") {
            BeamTable table;
            for (Scalar az = beam_table_config_.az_start_deg;
                 az <= beam_table_config_.az_end_deg + 0.0001;
                 az += beam_table_config_.az_step_deg) {
                table.emplace_back(az, beam_table_config_.elevation_deg);
            }
            set_beam_table(table);
        }
    }

    initialized_ = true;
    return true;
}

bool BeamScanner::load_beam_table_csv(const std::string& filepath) {
    last_error_.clear();

    std::ifstream input(filepath);
    if (!input.is_open()) {
        last_error_ = "Failed to open beam table CSV: " + filepath;
        return false;
    }

    BeamTable parsed_table;
    std::string line;
    std::size_t line_number = 0;

    while (std::getline(input, line)) {
        ++line_number;

        const std::string trimmed_line = trim(line);
        if (trimmed_line.empty() || trimmed_line[0] == '#') {
            continue;
        }

        std::stringstream ss(trimmed_line);
        std::string az_token;
        std::string el_token;
        std::string extra_token;

        if (!std::getline(ss, az_token, ',') || !std::getline(ss, el_token, ',')) {
            last_error_ = "Invalid CSV format at line " + std::to_string(line_number) +
                          ": expected 'az_deg,el_deg'";
            return false;
        }

        if (std::getline(ss, extra_token, ',')) {
            if (!trim(extra_token).empty()) {
                last_error_ = "Invalid CSV format at line " + std::to_string(line_number) +
                              ": too many columns";
                return false;
            }
        }

        Scalar az_deg = 0.0;
        Scalar el_deg = 0.0;
        if (!parse_double_strict(trim(az_token), az_deg) ||
            !parse_double_strict(trim(el_token), el_deg)) {
            last_error_ = "Invalid numeric value at line " + std::to_string(line_number);
            return false;
        }

        parsed_table.push_back({az_deg, el_deg});
    }

    if (parsed_table.empty()) {
        last_error_ = "Beam table CSV is empty: " + filepath;
        return false;
    }

    set_beam_table(parsed_table);
    return true;
}

void BeamScanner::set_beam_table(const BeamTable& beam_table) {
    beam_table_ = beam_table;
    current_beam_index_ = 0;
}

bool BeamScanner::advance_one_cpi() {
    if (beam_table_.empty()) {
        return false;
    }
    current_beam_index_ = (current_beam_index_ + 1) % beam_table_.size();
    return true;
}

void BeamScanner::reset() {
    current_beam_index_ = 0;
}

BeamPoint BeamScanner::get_beam_pointing() const {
    if (beam_table_.empty()) {
        return {0.0, 0.0};
    }
    const BeamPoint& beam = beam_table_[current_beam_index_];
    return {beam.azimuth_deg, beam.elevation_deg};
}

Scalar BeamScanner::get_gain_to_target(Scalar target_az_deg, Scalar target_el_deg) const {
    if (beam_table_.empty()) {
        throw std::runtime_error(kEmptyBeamTableError);
    }
    const BeamPoint beam = get_beam_pointing();
    return antenna_model_.gain(target_az_deg, target_el_deg, beam.azimuth_deg, beam.elevation_deg);
}

Scalar BeamScanner::get_gain_db_to_target(Scalar target_az_deg,
                                          Scalar target_el_deg) const {
    if (beam_table_.empty()) {
        throw std::runtime_error(kEmptyBeamTableError);
    }
    const BeamPoint beam = get_beam_pointing();
    return antenna_model_.gain_db(target_az_deg, target_el_deg, beam.azimuth_deg, beam.elevation_deg);
}

Scalar BeamScanner::get_normalized_power_to_target(Scalar target_az_deg,
                                                   Scalar target_el_deg) const {
    if (beam_table_.empty()) {
        throw std::runtime_error(kEmptyBeamTableError);
    }
    const BeamPoint beam = get_beam_pointing();
    return antenna_model_.normalized_power(target_az_deg, target_el_deg,
                                           beam.azimuth_deg, beam.elevation_deg);
}

bool BeamScanner::is_target_in_beam(Scalar target_az_deg, Scalar target_el_deg,
                                    Scalar threshold_db) const {
    if (beam_table_.empty()) {
        throw std::runtime_error(kEmptyBeamTableError);
    }
    const BeamPoint beam = get_beam_pointing();
    return antenna_model_.is_target_in_beam(target_az_deg, target_el_deg,
                                            beam.azimuth_deg, beam.elevation_deg, threshold_db);
}

}  // namespace radar::antenna
