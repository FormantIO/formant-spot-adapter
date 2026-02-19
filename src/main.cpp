#include <csignal>
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>

#include "formant_spot_adapter/adapter.hpp"
#include "formant_spot_adapter/config.hpp"

static volatile std::sig_atomic_t g_stop_requested = 0;

void handle_signal(int) {
  g_stop_requested = 1;
}

int main() {
  auto cfg = fsa::load_config();
  fsa::Adapter adapter(cfg);

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  if (!adapter.Init()) {
    std::cerr << "Adapter initialization failed" << std::endl;
    return 1;
  }

  std::atomic<bool> run_done{false};
  std::thread signal_watcher([&adapter, &run_done]() {
    while (!run_done.load(std::memory_order_relaxed)) {
      if (g_stop_requested) {
        adapter.Stop();
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  });

  adapter.Run();
  run_done = true;
  if (signal_watcher.joinable()) signal_watcher.join();
  return 0;
}
