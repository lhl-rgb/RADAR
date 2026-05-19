#pragma once

#include <memory>
#include <string>

namespace radar {

class RadarServiceImpl;

struct EmbeddedServer {
    std::unique_ptr<struct EmbeddedServerImpl> impl;
};

EmbeddedServer* CreateEmbeddedServer(const std::string& address);
void DestroyEmbeddedServer(EmbeddedServer* server);

}  // namespace radar
