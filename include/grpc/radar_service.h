#pragma once

// ---------------------------------------------------------------------------
// Naming conflict resolution
//
// The proto-generated header defines  radar::RadarConfig  (a protobuf Message)
// The C++  radar_config.h        defines  radar::RadarConfig  (a C++   struct)
// Including both in the same TU causes an ODR violation because they share the
// same fully qualified name.  We work around this by renaming the proto type
// during the proto header include, then undefining the macro before including
// the C++ headers.
//
// Inside this header (and in radar_service.cpp):
//   RadarProtoConfig  == the proto message type (used in gRPC handler params)
//   RadarConfig       == the C++ struct         (from radar_config.h)
// ---------------------------------------------------------------------------
#define RadarConfig RadarProtoConfig

#include "grpc/radar_service.grpc.pb.h"

#undef RadarConfig

#include "core/radar_config.h"
#include "core/simulation_engine.h"

#include <grpcpp/grpcpp.h>

#include <memory>
#include <mutex>
#include <string>

namespace radar {

class RadarServiceImpl final : public RadarService::Service {
public:
    RadarServiceImpl() = default;

    grpc::Status SetConfig(grpc::ServerContext* context,
                           const RadarProtoConfig* request,
                           Status* reply) override;

    grpc::Status GetConfig(grpc::ServerContext* context,
                           const Empty* request,
                           RadarProtoConfig* reply) override;

    grpc::Status ValidateConfig(grpc::ServerContext* context,
                                const RadarProtoConfig* request,
                                ValidationResult* reply) override;

    grpc::Status StartSimulation(grpc::ServerContext* context,
                                 const Empty* request,
                                 Status* reply) override;

    grpc::Status StopSimulation(grpc::ServerContext* context,
                                const Empty* request,
                                Status* reply) override;

    grpc::Status GetStatus(grpc::ServerContext* context,
                           const Empty* request,
                           SimStatus* reply) override;

private:
    std::mutex mutex_;
    ::radar::RadarConfig current_config_;  // C++ struct
    std::unique_ptr<SimulationEngine> engine_;
};

/// Helper: start a gRPC server on a given address.
/// The caller must keep `service` alive as long as the server runs.
std::unique_ptr<grpc::Server> StartGrpcServer(
    const std::string& server_address,
    RadarServiceImpl* service);

}  // namespace radar