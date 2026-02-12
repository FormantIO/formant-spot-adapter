#include <csignal>
#include <iostream>

#include "formant_spot_adapter/adapter.hpp"
#include "formant_spot_adapter/config.hpp"

static fsa::Adapter* g_adapter = nullptr;

void handle_signal(int) {
  if (g_adapter) g_adapter->Stop();
}

int main() {
  auto cfg = fsa::load_config();
  fsa::Adapter adapter(cfg);
  g_adapter = &adapter;

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  if (!adapter.Init()) {
    std::cerr << "Adapter initialization failed" << std::endl;
    return 1;
  }

  adapter.Run();
  return 0;
}
