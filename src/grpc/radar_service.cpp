#include "grpc/radar_service.h"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace radar {

// RadarProtoConfig is the proto-generated message type (see header for the
// macro rename).  ::radar::RadarConfig is the C++ struct from radar_config.h.
using ProtoConfig = RadarProtoConfig;

// ============================================================================
// SetConfig
// ============================================================================
grpc::Status RadarServiceImpl::SetConfig(
    grpc::ServerContext*, const ProtoConfig* request, Status* reply) {

    std::lock_guard<std::mutex> lock(mutex_);

    if (engine_ && engine_->is_running()) {
        reply->set_success(false);
        reply->set_message("Cannot change config while simulation is running");
        return grpc::Status::OK;
    }

    try {
        const std::string& json_str = request->json_content();
        nlohmann::json j = nlohmann::json::parse(json_str);

        ::radar::RadarConfig new_config = j.get<::radar::RadarConfig>();
        std::string error;
        if (!new_config.validate(error)) {
            reply->set_success(false);
            reply->set_message("Validation failed: " + error);
            return grpc::Status::OK;
        }

        current_config_ = new_config;

        engine_ = std::make_unique<SimulationEngine>(current_config_);
        if (!engine_->initialize()) {
            reply->set_success(false);
            reply->set_message("Engine init failed: " + engine_->last_error());
            engine_.reset();
            return grpc::Status::OK;
        }

        SPDLOG_INFO("gRPC: config updated successfully");
        reply->set_success(true);
        reply->set_message("Config updated and engine initialized");
        return grpc::Status::OK;

    } catch (const std::exception& e) {
        reply->set_success(false);
        reply->set_message(std::string("JSON parse error: ") + e.what());
        return grpc::Status::OK;
    }
}

// ============================================================================
// GetConfig
// ============================================================================
grpc::Status RadarServiceImpl::GetConfig(
    grpc::ServerContext*, const Empty*, ProtoConfig* reply) {

    std::lock_guard<std::mutex> lock(mutex_);

    nlohmann::json j;
    j["simulation"] = current_config_.simulation;
    j["system"] = current_config_.system;
    j["waveform"] = current_config_.waveform;
    j["antenna"] = current_config_.antenna;
    j["beam_table"] = current_config_.beam_table;
    j["noise"] = current_config_.noise;
    j["clutter"] = current_config_.clutter;
    j["target"] = current_config_.target;
    j["data_export"] = current_config_.data_export;
    j["udp_output"] = current_config_.udp_output;
    j["initial_targets"] = nlohmann::json::array();
    for (const auto& t : current_config_.initial_targets) {
        nlohmann::json tj;
        to_json(tj, t);
        j["initial_targets"].push_back(tj);
    }

    reply->set_json_content(j.dump(2));
    return grpc::Status::OK;
}

// ============================================================================
// ValidateConfig
// ============================================================================
grpc::Status RadarServiceImpl::ValidateConfig(
    grpc::ServerContext*, const ProtoConfig* request, ValidationResult* reply) {

    try {
        const std::string& json_str = request->json_content();
        nlohmann::json j = nlohmann::json::parse(json_str);
        ::radar::RadarConfig cfg = j.get<::radar::RadarConfig>();

        std::string error;
        if (cfg.validate(error)) {
            reply->set_valid(true);
        } else {
            reply->set_valid(false);
            reply->add_errors(error);
        }
    } catch (const std::exception& e) {
        reply->set_valid(false);
        reply->add_errors(std::string("JSON parse error: ") + e.what());
    }

    return grpc::Status::OK;
}

// ============================================================================
// StartSimulation
// ============================================================================
grpc::Status RadarServiceImpl::StartSimulation(
    grpc::ServerContext*, const Empty*, Status* reply) {

    std::lock_guard<std::mutex> lock(mutex_);

    if (!engine_) {
        reply->set_success(false);
        reply->set_message("No config set - call SetConfig first");
        return grpc::Status::OK;
    }

    if (engine_->is_running()) {
        reply->set_success(false);
        reply->set_message("Simulation already running");
        return grpc::Status::OK;
    }

    engine_->run_async(0);
    SPDLOG_INFO("gRPC: simulation started");
    reply->set_success(true);
    reply->set_message("Simulation started");
    return grpc::Status::OK;
}

// ============================================================================
// StopSimulation
// ============================================================================
grpc::Status RadarServiceImpl::StopSimulation(
    grpc::ServerContext*, const Empty*, Status* reply) {

    std::lock_guard<std::mutex> lock(mutex_);

    if (!engine_ || !engine_->is_running()) {
        reply->set_success(false);
        reply->set_message("No simulation running");
        return grpc::Status::OK;
    }

    engine_->stop();
    SPDLOG_INFO("gRPC: simulation stop requested");
    reply->set_success(true);
    reply->set_message("Stop requested");
    return grpc::Status::OK;
}

// ============================================================================
// GetStatus
// ============================================================================
grpc::Status RadarServiceImpl::GetStatus(
    grpc::ServerContext*, const Empty*, SimStatus* reply) {

    std::lock_guard<std::mutex> lock(mutex_);

    if (!engine_) {
        reply->set_state(SimStatus::IDLE);
        reply->set_progress(0.0);
        reply->set_current_scan(0);
        reply->set_total_scans(0);
        return grpc::Status::OK;
    }

    if (engine_->is_running()) {
        reply->set_state(SimStatus::RUNNING);
    } else if (engine_->stop_requested()) {
        reply->set_state(SimStatus::STOPPED);
    } else {
        reply->set_state(SimStatus::FINISHED);
    }

    reply->set_progress(engine_->progress_percent() / 100.0);
    reply->set_current_scan(engine_->current_scan_index());
    reply->set_total_scans(engine_->total_scan_count());
    return grpc::Status::OK;
}

// ============================================================================
// StartGrpcServer
// ============================================================================
std::unique_ptr<grpc::Server> StartGrpcServer(
    const std::string& server_address,
    RadarServiceImpl* service) {

    grpc::ServerBuilder builder;
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(service);
    return builder.BuildAndStart();
}

}  // namespace radar