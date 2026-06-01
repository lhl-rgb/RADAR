#pragma once

#include "core/radar_config.h"
#include "core/simulation_engine.h"
#include "radar_service.grpc.pb.h"

#include <grpcpp/grpcpp.h>

#include <memory>
#include <mutex>
#include <string>

namespace radar
{

  class RadarServiceImpl final : public RadarService::Service
  {
  public:
    RadarServiceImpl() = default;

    grpc::Status SetSystemParams(grpc::ServerContext *context,
                                 const SystemParams *request,
                                 Status *reply) override;

    grpc::Status GetSystemParams(grpc::ServerContext *context,
                                 const Empty *request,
                                 SystemParams *reply) override;

    grpc::Status SetWaveformParams(grpc::ServerContext *context,
                                   const WaveformParams *request,
                                   Status *reply) override;

    grpc::Status GetWaveformParams(grpc::ServerContext *context,
                                   const Empty *request,
                                   WaveformParams *reply) override;

    grpc::Status SetAntennaParams(grpc::ServerContext *context,
                                  const AntennaParams *request,
                                  Status *reply) override;

    grpc::Status GetAntennaParams(grpc::ServerContext *context,
                                  const Empty *request,
                                  AntennaParams *reply) override;

    grpc::Status SetMechScanParams(grpc::ServerContext *context,
                                   const MechScanParams *request,
                                   Status *reply) override;

    grpc::Status GetMechScanParams(grpc::ServerContext *context,
                                   const Empty *request,
                                   MechScanParams *reply) override;

    grpc::Status SetNoiseParams(grpc::ServerContext *context,
                                const NoiseParams *request,
                                Status *reply) override;

    grpc::Status GetNoiseParams(grpc::ServerContext *context,
                                const Empty *request,
                                NoiseParams *reply) override;

    grpc::Status SetClutterParams(grpc::ServerContext *context,
                                  const ClutterParams *request,
                                  Status *reply) override;

    grpc::Status GetClutterParams(grpc::ServerContext *context,
                                  const Empty *request,
                                  ClutterParams *reply) override;

    grpc::Status AddTarget(grpc::ServerContext *context,
                           const TargetParams *request, Status *reply) override;

    grpc::Status RemoveTarget(grpc::ServerContext *context,
                              const IntValue *request, Status *reply) override;

    grpc::Status ClearTargets(grpc::ServerContext *context, const Empty *request,
                              Status *reply) override;

    grpc::Status GetTargets(grpc::ServerContext *context, const Empty *request,
                            TargetListResponse *reply) override;

    grpc::Status ValidateAll(grpc::ServerContext *context, const Empty *request,
                             ValidationResult *reply) override;

    grpc::Status StartSimulation(grpc::ServerContext *context,
                                 const Empty *request, Status *reply) override;

    grpc::Status StopSimulation(grpc::ServerContext *context,
                                const Empty *request, Status *reply) override;

    grpc::Status GetStatus(grpc::ServerContext *context, const Empty *request,
                           SimStatus *reply) override;

    grpc::Status GetMetrics(grpc::ServerContext *context, const Empty *request,
                            SimMetrics *reply) override;

    grpc::Status StreamMetrics(grpc::ServerContext *context, const Empty *request,
                               grpc::ServerWriter<SimMetrics> *writer) override;

    grpc::Status StreamRangeProfile(grpc::ServerContext *context, const Empty *request,
                                    grpc::ServerWriter<RangeProfilePreviewMsg> *writer) override;

    grpc::Status StreamPpiPreview(grpc::ServerContext *context, const Empty *request,
                                  grpc::ServerWriter<PpiPreviewMsg> *writer) override;

  private:
    std::mutex mutex_;
    ::radar::RadarConfig current_config_;
    std::unique_ptr<SimulationEngine> engine_;
    bool started_ = false;
  };

  std::unique_ptr<grpc::Server> StartGrpcServer(const std::string &server_address,
                                                grpc::Service *service);

} // namespace radar
