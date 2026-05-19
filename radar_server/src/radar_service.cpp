#include "grpc/radar_service.h"

#include <cmath>
#include <spdlog/spdlog.h>

namespace radar {

grpc::Status RadarServiceImpl::SetSystemParams(grpc::ServerContext *,
                                               const SystemParams *request,
                                               Status *reply) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_config_.system.fc_hz = request->fc_hz();
  current_config_.system.prf_hz = request->prf_hz();
  current_config_.system.bw_hz = request->bandwidth_hz();
  current_config_.system.pulse_width_s = request->pulse_width_s();
  current_config_.system.fs_hz = request->fs_hz();
  current_config_.system.min_range_m = request->min_range_m();
  current_config_.system.max_range_m = request->max_range_m();
  current_config_.simulation.scan_count = request->scan_count();
  current_config_.compute_derived_params();
  reply->set_success(true);
  reply->set_message("System params updated");
  return grpc::Status::OK;
}

grpc::Status RadarServiceImpl::GetSystemParams(grpc::ServerContext *,
                                               const Empty *,
                                               SystemParams *reply) {
  std::lock_guard<std::mutex> lock(mutex_);
  reply->set_fc_hz(current_config_.system.fc_hz);
  reply->set_prf_hz(current_config_.system.prf_hz);
  reply->set_bandwidth_hz(current_config_.system.bw_hz);
  reply->set_pulse_width_s(current_config_.system.pulse_width_s);
  reply->set_fs_hz(current_config_.system.fs_hz);
  reply->set_min_range_m(current_config_.system.min_range_m);
  reply->set_max_range_m(current_config_.system.max_range_m);
  reply->set_scan_count(current_config_.simulation.scan_count);
  return grpc::Status::OK;
}

grpc::Status RadarServiceImpl::SetWaveformParams(grpc::ServerContext *,
                                                 const WaveformParams *request,
                                                 Status *reply) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_config_.waveform.waveform_type =
      static_cast<WaveformType>(request->waveform_type());
  current_config_.waveform.phase_code_type =
      static_cast<PhaseCodeType>(request->phase_code_type());
  current_config_.waveform.nlfm_window_type =
      static_cast<WindowType>(request->nlfm_window_type());
  current_config_.waveform.polarization =
      static_cast<PolarizationType>(request->polarization());
  reply->set_success(true);
  reply->set_message("Waveform params updated");
  return grpc::Status::OK;
}

grpc::Status RadarServiceImpl::GetWaveformParams(grpc::ServerContext *,
                                                 const Empty *,
                                                 WaveformParams *reply) {
  std::lock_guard<std::mutex> lock(mutex_);
  reply->set_waveform_type(static_cast<WaveformParams::WaveformType>(
      current_config_.waveform.waveform_type));
  reply->set_phase_code_type(
      static_cast<int32_t>(current_config_.waveform.phase_code_type));
  reply->set_nlfm_window_type(
      static_cast<int32_t>(current_config_.waveform.nlfm_window_type));
  reply->set_polarization(
      static_cast<int32_t>(current_config_.waveform.polarization));
  return grpc::Status::OK;
}

grpc::Status RadarServiceImpl::SetAntennaParams(grpc::ServerContext *,
                                                const AntennaParams *request,
                                                Status *reply) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_config_.antenna.model_type =
      static_cast<PhasedArrayModelType>(request->model_type());
  current_config_.antenna.num_elements_az = request->num_elements_az();
  current_config_.antenna.num_elements_el = request->num_elements_el();
  current_config_.antenna.peak_gain_db = request->peak_gain_db();
  reply->set_success(true);
  reply->set_message("Antenna params updated");
  return grpc::Status::OK;
}

grpc::Status RadarServiceImpl::GetAntennaParams(grpc::ServerContext *,
                                                const Empty *,
                                                AntennaParams *reply) {
  std::lock_guard<std::mutex> lock(mutex_);
  reply->set_model_type(static_cast<AntennaParams::AntennaModelType>(
      current_config_.antenna.model_type));
  reply->set_num_elements_az(current_config_.antenna.num_elements_az);
  reply->set_num_elements_el(current_config_.antenna.num_elements_el);
  reply->set_peak_gain_db(current_config_.antenna.peak_gain_db);
  return grpc::Status::OK;
}

grpc::Status RadarServiceImpl::SetNoiseParams(grpc::ServerContext *,
                                              const NoiseParams *request,
                                              Status *reply) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_config_.noise.mode = static_cast<NoiseLevelMode>(request->mode());
  current_config_.noise.sigma_complex = request->sigma_complex();
  current_config_.noise.system_temperature_k = request->temperature_k();
  current_config_.noise.noise_power_w = request->noise_power_w();
  current_config_.noise.noise_bandwidth_hz = request->noise_bandwidth_hz();
  current_config_.noise.seed = request->seed();
  reply->set_success(true);
  reply->set_message("Noise params updated");
  return grpc::Status::OK;
}

grpc::Status RadarServiceImpl::GetNoiseParams(grpc::ServerContext *,
                                              const Empty *,
                                              NoiseParams *reply) {
  std::lock_guard<std::mutex> lock(mutex_);
  reply->set_mode(
      static_cast<NoiseParams::NoiseMode>(current_config_.noise.mode));
  reply->set_sigma_complex(current_config_.noise.sigma_complex);
  reply->set_temperature_k(current_config_.noise.system_temperature_k);
  reply->set_noise_power_w(current_config_.noise.noise_power_w);
  reply->set_noise_bandwidth_hz(current_config_.noise.noise_bandwidth_hz);
  reply->set_seed(current_config_.noise.seed);
  return grpc::Status::OK;
}

grpc::Status RadarServiceImpl::SetClutterParams(grpc::ServerContext *,
                                                const ClutterParams *request,
                                                Status *reply) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_config_.clutter.enabled = request->enabled();
  current_config_.clutter.k_shape_nu = request->k_shape_nu();
  current_config_.clutter.morchin_sea_state = request->morchin_sea_state();
  current_config_.clutter.doppler_center_hz = request->doppler_center_hz();
  current_config_.clutter.doppler_sigma_hz = request->doppler_sigma_hz();
  current_config_.clutter.ground_range_min_m = request->ground_range_min_m();
  current_config_.clutter.ground_range_max_m = request->ground_range_max_m();
  current_config_.clutter.seed = request->seed();
  reply->set_success(true);
  reply->set_message("Clutter params updated");
  return grpc::Status::OK;
}

grpc::Status RadarServiceImpl::GetClutterParams(grpc::ServerContext *,
                                                const Empty *,
                                                ClutterParams *reply) {
  std::lock_guard<std::mutex> lock(mutex_);
  reply->set_enabled(current_config_.clutter.enabled);
  reply->set_k_shape_nu(current_config_.clutter.k_shape_nu);
  reply->set_morchin_sea_state(current_config_.clutter.morchin_sea_state);
  reply->set_doppler_center_hz(current_config_.clutter.doppler_center_hz);
  reply->set_doppler_sigma_hz(current_config_.clutter.doppler_sigma_hz);
  reply->set_ground_range_min_m(current_config_.clutter.ground_range_min_m);
  reply->set_ground_range_max_m(current_config_.clutter.ground_range_max_m);
  reply->set_seed(current_config_.clutter.seed);
  return grpc::Status::OK;
}

grpc::Status RadarServiceImpl::AddTarget(grpc::ServerContext *,
                                         const TargetParams *request,
                                         Status *reply) {
  std::lock_guard<std::mutex> lock(mutex_);
  TargetState ts;
  ts.id = request->id();
  double range = request->range_m();
  double azimuth_rad = request->azimuth_deg() * M_PI / 180.0;
  double elevation_rad = request->elevation_deg() * M_PI / 180.0;
  double horizontal_dist = range * std::cos(elevation_rad);
  ts.position_m.x() = horizontal_dist * std::cos(azimuth_rad);
  ts.position_m.y() = horizontal_dist * std::sin(azimuth_rad);
  ts.position_m.z() = range * std::sin(elevation_rad);
  double velocity = request->radial_velocity_ms();
  ts.velocity_mps.x() =
      velocity * std::cos(elevation_rad) * std::cos(azimuth_rad);
  ts.velocity_mps.y() =
      velocity * std::cos(elevation_rad) * std::sin(azimuth_rad);
  ts.velocity_mps.z() = velocity * std::sin(elevation_rad);
  ts.rcs_mean_m2 = std::pow(10.0, request->rcs_db() / 10.0);
  ts.swerling = static_cast<SwerlingType>(request->swerling_type());
  ts.enabled = true;
  current_config_.initial_targets.push_back(ts);
  reply->set_success(true);
  reply->set_message("Target added");
  return grpc::Status::OK;
}

grpc::Status RadarServiceImpl::RemoveTarget(grpc::ServerContext *,
                                            const IntValue *request,
                                            Status *reply) {
  std::lock_guard<std::mutex> lock(mutex_);
  int id_to_remove = request->value();
  auto it = std::remove_if(
      current_config_.initial_targets.begin(),
      current_config_.initial_targets.end(),
      [id_to_remove](const TargetState &t) { return t.id == id_to_remove; });
  if (it != current_config_.initial_targets.end()) {
    current_config_.initial_targets.erase(
        it, current_config_.initial_targets.end());
    reply->set_success(true);
    reply->set_message("Target removed");
  } else {
    reply->set_success(false);
    reply->set_message("Target not found");
  }
  return grpc::Status::OK;
}

grpc::Status RadarServiceImpl::ClearTargets(grpc::ServerContext *,
                                            const Empty *, Status *reply) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_config_.initial_targets.clear();
  reply->set_success(true);
  reply->set_message("All targets cleared");
  return grpc::Status::OK;
}

grpc::Status RadarServiceImpl::GetTargets(grpc::ServerContext *, const Empty *,
                                          TargetListResponse *reply) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto &t : current_config_.initial_targets) {
    auto *target = reply->add_targets();
    target->set_id(t.id);
    double range = t.position_m.norm();
    target->set_range_m(range);
    double azimuth_deg =
        std::atan2(t.position_m.y(), t.position_m.x()) * 180.0 / M_PI;
    target->set_azimuth_deg(azimuth_deg);
    double elevation_deg =
        std::atan2(t.position_m.z(),
                   std::sqrt(t.position_m.x() * t.position_m.x() +
                             t.position_m.y() * t.position_m.y())) *
        180.0 / M_PI;
    target->set_elevation_deg(elevation_deg);
    double radial_velocity = t.velocity_mps.norm();
    target->set_radial_velocity_ms(radial_velocity);
    double rcs_db = 10.0 * std::log10(t.rcs_mean_m2);
    target->set_rcs_db(rcs_db);
    target->set_swerling_type(static_cast<int>(t.swerling));
  }
  return grpc::Status::OK;
}

grpc::Status RadarServiceImpl::ValidateAll(grpc::ServerContext *, const Empty *,
                                           ValidationResult *reply) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::string error;
  bool valid = current_config_.validate(error);
  reply->set_valid(valid);
  if (!valid) {
    reply->add_errors(error);
  }
  return grpc::Status::OK;
}

grpc::Status RadarServiceImpl::StartSimulation(grpc::ServerContext *,
                                               const Empty *, Status *reply) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!engine_) {
    engine_ = std::make_unique<SimulationEngine>(current_config_);
    if (!engine_->initialize()) {
      reply->set_success(false);
      reply->set_message("Engine init failed: " + engine_->last_error());
      engine_.reset();
      return grpc::Status::OK;
    }
  }

  if (engine_->is_running()) {
    reply->set_success(false);
    reply->set_message("Simulation already running");
    return grpc::Status::OK;
  }

  started_ = true;
  engine_->run_async(0);
  reply->set_success(true);
  reply->set_message("Simulation started");
  return grpc::Status::OK;
}

grpc::Status RadarServiceImpl::StopSimulation(grpc::ServerContext *,
                                              const Empty *, Status *reply) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (engine_ && engine_->is_running()) {
    engine_->stop();
    reply->set_success(true);
    reply->set_message("Simulation stopped");
  } else {
    reply->set_success(false);
    reply->set_message("No simulation running");
  }
  started_ = false;
  return grpc::Status::OK;
}

grpc::Status RadarServiceImpl::GetStatus(grpc::ServerContext *, const Empty *,
                                         SimStatus *reply) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!engine_) {
    reply->set_state(SimStatus::IDLE);
    reply->set_progress(0);
    reply->set_current_scan(0);
    reply->set_total_scans(current_config_.simulation.scan_count);
    return grpc::Status::OK;
  }

  if (engine_->is_running()) {
    reply->set_state(SimStatus::RUNNING);
    double progress = 0.0;
    if (engine_->total_scan_count() > 0) {
      progress = static_cast<double>(engine_->current_scan_index()) /
                 engine_->total_scan_count() * 100.0;
    }
    reply->set_progress(progress);
  } else if (started_) {
    reply->set_state(SimStatus::FINISHED);
    reply->set_progress(100.0);
  } else {
    reply->set_state(SimStatus::IDLE);
    reply->set_progress(0);
  }

  reply->set_current_scan(engine_->current_scan_index());
  reply->set_total_scans(engine_->total_scan_count());
  return grpc::Status::OK;
}

std::unique_ptr<grpc::Server> StartGrpcServer(const std::string &server_address,
                                              grpc::Service *service) {
  grpc::ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(service);
  return builder.BuildAndStart();
}

} // namespace radar