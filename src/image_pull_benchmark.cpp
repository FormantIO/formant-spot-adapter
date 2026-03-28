#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "formant_spot_adapter/config.hpp"
#include "formant_spot_adapter/spot_client.hpp"

namespace {

constexpr int kImageStatusOk = 1;

volatile std::sig_atomic_t g_stop_requested = 0;

void handle_signal(int) {
  g_stop_requested = 1;
}

struct Options {
  std::vector<std::string> sources;
  double duration_sec{15.0};
  double target_hz{0.0};
  int warmup_requests{2};
};

struct SourceStats {
  int frames_returned{0};
  int ok_frames{0};
  int error_frames{0};
  int missing_acquisition_time{0};
  int unique_frames{0};
  int duplicate_frames{0};
  bool has_last_acquisition_time{false};
  int64_t last_acquisition_time_ns{0};
  int64_t first_acquisition_time_ns{0};
  int64_t newest_acquisition_time_ns{0};
  std::vector<double> interframe_ms;
  std::vector<double> age_ms;
  std::vector<double> payload_kib;
};

struct StatsSummary {
  double min{0.0};
  double avg{0.0};
  double p50{0.0};
  double p95{0.0};
  double max{0.0};
};

std::string trim(std::string value) {
  auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

std::vector<std::string> split_csv(const std::string& csv) {
  std::vector<std::string> values;
  std::stringstream ss(csv);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item = trim(item);
    if (!item.empty()) values.push_back(item);
  }
  return values;
}

void print_usage(const char* argv0, const fsa::Config& cfg) {
  std::cout
      << "Usage: " << argv0 << " [--duration-sec N] [--target-hz N] [--sources a,b]\n"
      << "Defaults to the configured front fisheyes:\n"
      << "  " << cfg.front_left_camera_source << "," << cfg.front_right_camera_source << "\n"
      << "Use --target-hz 0 to pull as fast as possible.\n";
}

bool parse_args(int argc, char** argv, const fsa::Config& cfg, Options* out) {
  if (!out) return false;
  Options options;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg(argv[i]);
      if (arg == "--help" || arg == "-h") {
        print_usage(argv[0], cfg);
        std::exit(0);
      }
      if (arg == "--duration-sec" && i + 1 < argc) {
        options.duration_sec = std::stod(argv[++i]);
        continue;
      }
      if (arg == "--target-hz" && i + 1 < argc) {
        options.target_hz = std::stod(argv[++i]);
        continue;
      }
      if (arg == "--sources" && i + 1 < argc) {
        options.sources = split_csv(argv[++i]);
        continue;
      }
      if (arg == "--warmup-requests" && i + 1 < argc) {
        options.warmup_requests = std::max(0, std::stoi(argv[++i]));
        continue;
      }
      std::cerr << "Unknown argument: " << arg << '\n';
      print_usage(argv[0], cfg);
      return false;
    }
  } catch (const std::exception& e) {
    std::cerr << "Invalid arguments: " << e.what() << '\n';
    print_usage(argv[0], cfg);
    return false;
  }

  if (options.sources.empty()) {
    if (!cfg.front_left_camera_source.empty()) {
      options.sources.push_back(cfg.front_left_camera_source);
    }
    if (!cfg.front_right_camera_source.empty() &&
        cfg.front_right_camera_source != cfg.front_left_camera_source) {
      options.sources.push_back(cfg.front_right_camera_source);
    }
  }
  if (options.sources.empty()) {
    std::cerr << "No image sources configured or provided." << std::endl;
    return false;
  }
  if (!(options.duration_sec > 0.0)) {
    std::cerr << "--duration-sec must be > 0" << std::endl;
    return false;
  }
  if (options.target_hz < 0.0) {
    std::cerr << "--target-hz must be >= 0" << std::endl;
    return false;
  }

  *out = options;
  return true;
}

StatsSummary summarize_samples(const std::vector<double>& samples) {
  StatsSummary summary;
  if (samples.empty()) return summary;

  std::vector<double> sorted = samples;
  std::sort(sorted.begin(), sorted.end());
  const auto percentile = [&](double p) {
    const double idx = (static_cast<double>(sorted.size() - 1) * p);
    return sorted[static_cast<size_t>(idx + 0.5)];
  };
  summary.min = sorted.front();
  summary.max = sorted.back();
  summary.avg = std::accumulate(sorted.begin(), sorted.end(), 0.0) /
                static_cast<double>(sorted.size());
  summary.p50 = percentile(0.50);
  summary.p95 = percentile(0.95);
  return summary;
}

void print_summary_line(const std::string& label, const std::vector<double>& samples,
                        const std::string& units) {
  const auto summary = summarize_samples(samples);
  if (samples.empty()) {
    std::cout << "  " << label << ": n/a\n";
    return;
  }

  std::cout << std::fixed << std::setprecision(2)
            << "  " << label
            << ": min=" << summary.min << units
            << " avg=" << summary.avg << units
            << " p50=" << summary.p50 << units
            << " p95=" << summary.p95 << units
            << " max=" << summary.max << units
            << '\n';
}

int64_t acquisition_time_to_ns(const fsa::SpotClient::ImageFrame& frame) {
  return frame.acquisition_time_sec * 1000000000LL +
         static_cast<int64_t>(frame.acquisition_time_nanos);
}

}  // namespace

int main(int argc, char** argv) {
  const fsa::Config cfg = fsa::load_config();
  Options options;
  if (!parse_args(argc, argv, cfg, &options)) return 2;

  if (cfg.spot_host.empty() || cfg.spot_username.empty() || cfg.spot_password.empty()) {
    std::cerr << "Spot host/credentials are not configured." << std::endl;
    return 2;
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  fsa::SpotClient client;
  if (!client.Connect(cfg.spot_host, cfg.spot_username, cfg.spot_password)) {
    std::cerr << "Connect failed: " << client.LastError() << std::endl;
    return 1;
  }

  int64_t robot_clock_skew_ns = 0;
  const bool has_robot_clock_skew = client.GetRobotClockSkewNanos(&robot_clock_skew_ns);
  if (!has_robot_clock_skew) {
    std::cerr << "Warning: timesync unavailable, frame age at receipt will be omitted: "
              << client.LastError() << std::endl;
  }

  std::cout << "Spot image pull benchmark\n"
            << "  host: " << cfg.spot_host << '\n'
            << "  sources:";
  for (const auto& source : options.sources) std::cout << ' ' << source;
  std::cout << '\n'
            << "  duration_sec: " << options.duration_sec << '\n'
            << "  target_hz: " << options.target_hz << '\n'
            << "  warmup_requests: " << options.warmup_requests << '\n';
  if (has_robot_clock_skew) {
    std::cout << "  robot_clock_skew_ms: " << std::fixed << std::setprecision(3)
              << (static_cast<double>(robot_clock_skew_ns) / 1.0e6) << '\n';
  }
  std::cout << std::defaultfloat;

  std::vector<fsa::SpotClient::ImageFrame> frames;
  for (int i = 0; i < options.warmup_requests && !g_stop_requested; ++i) {
    client.GetImageFrames(options.sources, &frames);
  }

  std::unordered_map<std::string, SourceStats> per_source;
  for (const auto& source : options.sources) per_source.emplace(source, SourceStats{});

  int request_count = 0;
  int request_failures = 0;
  size_t total_frames = 0;
  std::vector<double> request_latency_ms;
  std::vector<double> response_payload_kib;
  const auto benchmark_start = std::chrono::steady_clock::now();
  auto next_request_time = benchmark_start;
  const auto request_period = (options.target_hz > 0.0)
                                  ? std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                        std::chrono::duration<double>(1.0 / options.target_hz))
                                  : std::chrono::steady_clock::duration::zero();

  while (!g_stop_requested) {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration<double>(now - benchmark_start).count();
    if (elapsed >= options.duration_sec) break;

    if (options.target_hz > 0.0 && now < next_request_time) {
      std::this_thread::sleep_until(next_request_time);
      continue;
    }

    const auto request_start_steady = std::chrono::steady_clock::now();
    const bool ok = client.GetImageFrames(options.sources, &frames);
    const auto request_end_steady = std::chrono::steady_clock::now();
    const auto request_end_wall = std::chrono::system_clock::now();
    if (options.target_hz > 0.0) next_request_time = request_start_steady + request_period;

    const double latency_ms =
        std::chrono::duration<double, std::milli>(request_end_steady - request_start_steady).count();
    request_latency_ms.push_back(latency_ms);
    ++request_count;

    if (!ok) {
      ++request_failures;
      continue;
    }

    total_frames += frames.size();
    double response_kib = 0.0;

    for (const auto& frame : frames) {
      auto it = per_source.find(frame.source_name);
      if (it == per_source.end()) {
        it = per_source.emplace(frame.source_name, SourceStats{}).first;
      }
      SourceStats& stats = it->second;
      ++stats.frames_returned;
      response_kib += static_cast<double>(frame.encoded_image.size()) / 1024.0;
      stats.payload_kib.push_back(static_cast<double>(frame.encoded_image.size()) / 1024.0);

      if (frame.status == kImageStatusOk) {
        ++stats.ok_frames;
      } else {
        ++stats.error_frames;
      }

      if (!frame.has_acquisition_time) {
        ++stats.missing_acquisition_time;
        continue;
      }

      const int64_t acquisition_time_ns = acquisition_time_to_ns(frame);
      if (!stats.has_last_acquisition_time) {
        stats.has_last_acquisition_time = true;
        stats.last_acquisition_time_ns = acquisition_time_ns;
        stats.first_acquisition_time_ns = acquisition_time_ns;
        stats.newest_acquisition_time_ns = acquisition_time_ns;
        ++stats.unique_frames;
      } else if (acquisition_time_ns == stats.last_acquisition_time_ns) {
        ++stats.duplicate_frames;
      } else {
        ++stats.unique_frames;
        stats.interframe_ms.push_back(
            static_cast<double>(acquisition_time_ns - stats.last_acquisition_time_ns) / 1.0e6);
        stats.last_acquisition_time_ns = acquisition_time_ns;
        stats.newest_acquisition_time_ns = acquisition_time_ns;
      }

      if (has_robot_clock_skew) {
        const int64_t request_end_wall_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                request_end_wall.time_since_epoch()).count();
        const double age_ms =
            static_cast<double>((request_end_wall_ns + robot_clock_skew_ns) - acquisition_time_ns) /
            1.0e6;
        stats.age_ms.push_back(age_ms);
      }
    }

    response_payload_kib.push_back(response_kib);
  }

  const auto benchmark_end = std::chrono::steady_clock::now();
  const auto wall_elapsed_sec =
      std::chrono::duration<double>(benchmark_end - benchmark_start).count();
  const auto wall_elapsed_ms =
      std::chrono::duration<double, std::milli>(benchmark_end - benchmark_start).count();

  std::cout << '\n'
            << "Request summary\n"
            << "  elapsed_ms: " << std::fixed << std::setprecision(2) << wall_elapsed_ms << '\n'
            << "  requests: " << request_count << '\n'
            << "  request_failures: " << request_failures << '\n'
            << "  achieved_request_hz: "
            << ((wall_elapsed_sec > 0.0) ? (static_cast<double>(request_count) / wall_elapsed_sec) : 0.0)
            << '\n'
            << "  returned_frames: " << total_frames << '\n';
  print_summary_line("request_latency", request_latency_ms, " ms");
  print_summary_line("response_payload", response_payload_kib, " KiB");

  std::map<std::string, SourceStats> ordered_sources(per_source.begin(), per_source.end());
  for (const auto& entry : ordered_sources) {
    const auto& source = entry.first;
    const auto& stats = entry.second;
    std::cout << '\n' << "Source: " << source << '\n'
              << "  frames_returned: " << stats.frames_returned << '\n'
              << "  ok_frames: " << stats.ok_frames << '\n'
              << "  error_frames: " << stats.error_frames << '\n'
              << "  unique_acquisition_timestamps: " << stats.unique_frames << '\n'
              << "  duplicate_acquisition_timestamps: " << stats.duplicate_frames << '\n'
              << "  missing_acquisition_time: " << stats.missing_acquisition_time << '\n';
    if (wall_elapsed_sec > 0.0) {
      std::cout << "  observed_unique_frame_hz: "
                << (static_cast<double>(stats.unique_frames) / wall_elapsed_sec) << '\n'
                << "  duplicate_ratio: "
                << ((stats.frames_returned > 0)
                        ? (static_cast<double>(stats.duplicate_frames) /
                           static_cast<double>(stats.frames_returned))
                        : 0.0)
                << '\n';
    }
    print_summary_line("payload", stats.payload_kib, " KiB");
    print_summary_line("interframe", stats.interframe_ms, " ms");
    print_summary_line("age_at_receipt", stats.age_ms, " ms");
  }

  return 0;
}
