#include "grpc/embedded_server.h"
#include "grpc/radar_service.h"

#include <spdlog/spdlog.h>

namespace radar {

struct EmbeddedServerImpl {
  std::unique_ptr<grpc::Server> server;
  std::unique_ptr<RadarServiceImpl> service;
};

EmbeddedServer *CreateEmbeddedServer(const std::string &address) {
  auto *es = new EmbeddedServer();
  es->impl = std::make_unique<EmbeddedServerImpl>();
  es->impl->service = std::make_unique<RadarServiceImpl>();
  es->impl->server = StartGrpcServer(
      address, static_cast<grpc::Service *>(es->impl->service.get()));
  SPDLOG_INFO("Embedded gRPC server started on {}", address);
  return es;
}

void DestroyEmbeddedServer(EmbeddedServer *server) {
  if (!server)
    return;
  if (server->impl->server) {
    server->impl->server->Shutdown();
  }
  delete server;
}

} // namespace radar
