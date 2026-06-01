#include "output/file_echo_sink.h"

#include <exception>
#include <filesystem>
#include <nlohmann/json.hpp>

namespace radar::output
{
    namespace fs = std::filesystem;

    namespace
    {
        constexpr std::uint32_t kMagic = 0x45434846; // ECHF
        constexpr std::uint32_t kVersion = 1;
    }

    bool FileEchoSink::open(const OutputConfig &config)
    {
        close();
        config_ = config;
        frames_written_ = 0;
        samples_per_frame_ = 0;
        last_error_.clear();

        try
        {
            fs::create_directories(config_.output_dir);
        }
        catch (const std::exception &e)
        {
            last_error_ = std::string("Failed to create output directory: ") + e.what();
            return false;
        }

        if (!write_metadata(config_))
        {
            return false;
        }

        const std::string path = config_.output_dir + "/" + config_.file_prefix + ".dat";
        data_file_.open(path, std::ios::binary | std::ios::trunc);
        if (!data_file_.is_open())
        {
            last_error_ = "Failed to open output file: " + path;
            return false;
        }

        data_file_.write(reinterpret_cast<const char *>(&kMagic), sizeof(kMagic));
        data_file_.write(reinterpret_cast<const char *>(&kVersion), sizeof(kVersion));
        const std::uint64_t reserved = 0;
        data_file_.write(reinterpret_cast<const char *>(&reserved), sizeof(reserved));
        opened_ = true;
        return true;
    }

    bool FileEchoSink::write_frame(const core::EchoFrame &frame)
    {
        if (!opened_ || !data_file_.is_open())
        {
            last_error_ = "FileEchoSink is not open";
            return false;
        }

        const std::uint32_t sample_count = static_cast<std::uint32_t>(frame.iq.size());
        if (frames_written_ == 0)
        {
            samples_per_frame_ = sample_count;
        }
        else if (sample_count != samples_per_frame_)
        {
            last_error_ = "EchoFrame sample count changed within one file";
            return false;
        }

        data_file_.write(reinterpret_cast<const char *>(&frame.pulse_index), sizeof(frame.pulse_index));
        data_file_.write(reinterpret_cast<const char *>(&frame.scan_index), sizeof(frame.scan_index));
        data_file_.write(reinterpret_cast<const char *>(&sample_count), sizeof(sample_count));
        data_file_.write(reinterpret_cast<const char *>(&frame.timestamp_s), sizeof(frame.timestamp_s));
        data_file_.write(reinterpret_cast<const char *>(&frame.beam_az_deg), sizeof(frame.beam_az_deg));
        data_file_.write(reinterpret_cast<const char *>(&frame.beam_el_deg), sizeof(frame.beam_el_deg));
        data_file_.write(reinterpret_cast<const char *>(&frame.range_bin_size_m), sizeof(frame.range_bin_size_m));

        for (const auto &sample : frame.iq)
        {
            const float real = static_cast<float>(sample.real());
            const float imag = static_cast<float>(sample.imag());
            data_file_.write(reinterpret_cast<const char *>(&real), sizeof(real));
            data_file_.write(reinterpret_cast<const char *>(&imag), sizeof(imag));
        }

        if (!data_file_)
        {
            last_error_ = "Failed while writing EchoFrame";
            return false;
        }

        ++frames_written_;
        return true;
    }

    void FileEchoSink::close()
    {
        if (data_file_.is_open())
        {
            data_file_.close();
        }
        opened_ = false;
    }

    bool FileEchoSink::write_metadata(const OutputConfig &config)
    {
        const std::string path = config.output_dir + "/" + config.file_prefix + "_metadata.json";
        try
        {
            nlohmann::json j;
            j["format"] = "Radar EchoFrame stream";
            j["magic"] = "ECHF";
            j["version"] = kVersion;
            j["layout"] = "file_header_then_repeated_frame_header_then_iq_float32_real_imag";
            j["file"] = config.file_prefix + ".dat";

            std::ofstream meta(path);
            if (!meta.is_open())
            {
                last_error_ = "Failed to open metadata file: " + path;
                return false;
            }
            meta << j.dump(2);
            return true;
        }
        catch (const std::exception &e)
        {
            last_error_ = std::string("Failed to write metadata: ") + e.what();
            return false;
        }
    }
} // namespace radar::output
