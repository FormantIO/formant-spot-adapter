#include "formant_spot_adapter/adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <unordered_map>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace fsa {

namespace {
enum MotionModeState {
  kMotionModeWalk = 0,
  kMotionModeStairs = 1,
  kMotionModeCrawl = 2,
};

long long now_ms() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

bool bitset_has_true(const v1::model::Bitset& bitset) {
  for (const auto& b : bitset.bits()) {
    if (b.value()) return true;
  }
  return false;
}

struct CameraStreamSpec {
  std::string source;
  std::string stream;
  bool apply_rotation{false};
};
}  // namespace

Adapter::Adapter(const Config& config)
    : cfg_(config), agent_(config.formant_agent_target) {}

bool Adapter::Init() {
  if (cfg_.spot_host.empty() || cfg_.spot_username.empty() || cfg_.spot_password.empty()) {
    std::cerr << "Missing Spot config. Require spot_host in config.json and SPOT_USERNAME/SPOT_PASSWORD in env."
              << std::endl;
    return false;
  }

  if (!spot_.Connect(cfg_.spot_host, cfg_.spot_username, cfg_.spot_password)) {
    std::cerr << "Failed to connect/authenticate with Spot: " << spot_.LastError() << std::endl;
    return false;
  }

  std::vector<SpotClient::ImageSourceInfo> sources;
  if (spot_.ListImageSources(&sources)) {
    std::unordered_map<std::string, SpotClient::ImageSourceInfo> source_map;
    for (const auto& src : sources) {
      source_map[src.name] = src;
    }
    std::cerr << "[camera] available image sources (" << sources.size() << "):" << std::endl;
    for (const auto& src : sources) {
      std::cerr << "  - " << src.name << " (" << src.cols << "x" << src.rows << ")" << std::endl;
    }

    const auto log_configured_source = [&source_map](const std::string& source, const std::string& stream) {
      if (source.empty() || stream.empty()) return;
      auto it = source_map.find(source);
      if (it == source_map.end()) {
        std::cerr << "[camera] configured source missing: source=" << source
                  << " stream=" << stream << std::endl;
        return;
      }
      std::cerr << "[camera] stream config: stream=" << stream << " source=" << source
                << " size=" << it->second.cols << "x" << it->second.rows << std::endl;
    };
    log_configured_source(cfg_.camera_source, cfg_.camera_stream_name);
    log_configured_source(cfg_.left_camera_source, cfg_.left_camera_stream_name);
    log_configured_source(cfg_.right_camera_source, cfg_.right_camera_stream_name);
    log_configured_source(cfg_.back_camera_source, cfg_.back_camera_stream_name);
  } else {
    std::cerr << "[camera] failed to list image sources: " << spot_.LastError() << std::endl;
  }

  lease_owned_ = false;
  heartbeat_seen_ = false;
  control_seen_ = false;
  last_twist_ms_ = now_ms();
  last_control_ms_ = now_ms();
  moving_ = false;
  desired_motion_mode_ = kMotionModeWalk;
  last_arm_hold_cmd_ms_ = 0;
  last_lease_attempt_ms_ = 0;
  last_teleop_diag_log_ms_ = 0;
  last_control_log_ms_ = 0;
  last_logged_rotate_camera_ccw_ = rotate_camera_ccw_.load();
  last_rotate_error_log_ms_ = 0;
  return true;
}

void Adapter::Run() {
  running_ = true;
  std::cerr << "[adapter] starting teleop loops" << std::endl;

  std::vector<std::string> stream_filter{cfg_.teleop_twist_stream, cfg_.teleop_buttons_stream,
                                         cfg_.stand_button_stream, cfg_.sit_button_stream,
                                         cfg_.estop_button_stream, cfg_.recover_button_stream,
                                         cfg_.walk_button_stream, cfg_.stairs_button_stream,
                                         cfg_.crawl_button_stream, cfg_.reset_arm_button_stream,
                                         cfg_.dock_button_stream};
  stream_filter.erase(std::remove_if(stream_filter.begin(), stream_filter.end(),
                                     [](const std::string& s) { return s.empty(); }),
                      stream_filter.end());
  std::sort(stream_filter.begin(), stream_filter.end());
  stream_filter.erase(std::unique(stream_filter.begin(), stream_filter.end()), stream_filter.end());

  agent_.StartTeleopLoop(stream_filter,
                         [this](const v1::model::ControlDatapoint& dp) { HandleTeleop(dp); });
  agent_.StartCommandLoop({"spot.jetson.reboot", "spot.robot.reboot"},
                          [this](const v1::model::CommandRequest& request) { HandleCommand(request); });
  agent_.StartHeartbeatLoop(
      [this](const v1::agent::GetTeleopHeartbeatStreamResponse& hb) { HandleHeartbeat(hb); });

  camera_threads_.clear();
  auto start_camera = [this](const std::string& source, const std::string& stream, bool apply_rotation,
                             int fps) {
    if (source.empty() || stream.empty()) return;
    camera_threads_.emplace_back(&Adapter::CameraLoop, this, source, stream, apply_rotation, fps);
  };
  const int hand_fps = std::max(1, cfg_.camera_fps);
  const int side_fps = std::max(1, std::min(10, cfg_.camera_fps));
  start_camera(cfg_.camera_source, cfg_.camera_stream_name, true, hand_fps);
  start_camera(cfg_.left_camera_source, cfg_.left_camera_stream_name, false, side_fps);
  start_camera(cfg_.right_camera_source, cfg_.right_camera_stream_name, false, side_fps);
  start_camera(cfg_.back_camera_source, cfg_.back_camera_stream_name, false, side_fps);
  lease_thread_ = std::thread(&Adapter::LeaseRetainLoop, this);
  dock_thread_ = std::thread(&Adapter::DockLoop, this);

  while (running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (HeartbeatExpired() && lease_owned_.load() && moving_.load()) {
      spot_.ZeroVelocity(cfg_.zero_velocity_repeats);
      moving_ = false;
    }

    const bool teleop_active = TeleopSessionActive();
    const long long now = now_ms();
    if (!teleop_active && (now - last_teleop_diag_log_ms_.load()) >= 5000) {
      std::cerr << "[teleop] inactive heartbeat_seen=" << (heartbeat_seen_.load() ? "true" : "false")
                << " control_seen=" << (control_seen_.load() ? "true" : "false")
                << " heartbeat_timeout_ms=" << cfg_.heartbeat_timeout_ms << std::endl;
      last_teleop_diag_log_ms_ = now;
    }
    if (teleop_active && !lease_owned_) {
      if ((now - last_lease_attempt_ms_.load()) >= 1000) {
        last_lease_attempt_ms_ = now;
        lease_owned_ = spot_.AcquireBodyLease();
        if (lease_owned_) {
          std::cerr << "[adapter] lease acquired" << std::endl;
          ApplyDesiredArmMode(true);
        } else {
          std::cerr << "[adapter] failed to acquire lease: " << spot_.LastError() << std::endl;
        }
      }
    } else if (!teleop_active && lease_owned_) {
      std::cerr << "[adapter] teleop inactive: stopping motion, stowing arm, returning lease" << std::endl;
      spot_.ZeroVelocity(cfg_.zero_velocity_repeats);
      moving_ = false;
      if (!WaitForArmStow(3000)) {
        std::cerr << "[adapter] arm did not reach stow before lease return (timeout)" << std::endl;
      }
      if (!spot_.ReturnBodyLease()) {
        std::cerr << "[adapter] failed returning lease: " << spot_.LastError() << std::endl;
      }
      lease_owned_ = false;
    }

    const long long age_ms = now_ms() - last_twist_ms_.load();
    if (moving_.load() && age_ms > cfg_.teleop_idle_timeout_ms) {
      spot_.ZeroVelocity(cfg_.zero_velocity_repeats);
      moving_ = false;
    }
  }

  spot_.RequestDockCancel();
  agent_.StopLoops();
  for (auto& t : camera_threads_) {
    if (t.joinable()) t.join();
  }
  camera_threads_.clear();
  if (lease_thread_.joinable()) lease_thread_.join();
  if (dock_thread_.joinable()) dock_thread_.join();
  spot_.ZeroVelocity(cfg_.zero_velocity_repeats);
  if (lease_owned_) {
    (void)WaitForArmStow(3000);
    spot_.ReturnBodyLease();
    lease_owned_ = false;
  }
}

void Adapter::Stop() {
  running_ = false;
  spot_.RequestDockCancel();
  agent_.StopLoops();
}

void Adapter::CameraLoop(const std::string& source, const std::string& stream, bool apply_rotation, int fps) {
  const int period_ms = std::max(1, 1000 / std::max(1, fps));
  long long last_error_log_ms = 0;
  while (running_) {
    std::string jpg;
    if (spot_.GetImageJpeg(source, &jpg)) {
      if (apply_rotation && rotate_camera_ccw_) {
        std::string rotated;
        if (RotateJpeg90CCW(jpg, &rotated)) {
          agent_.PostImage(stream, "image/jpeg", rotated);
        } else {
          const long long now = now_ms();
          if ((now - last_rotate_error_log_ms_.load()) >= 2000) {
            std::cerr << "[camera] rotate requested but JPEG rotate failed; publishing unrotated frame"
                      << std::endl;
            last_rotate_error_log_ms_ = now;
          }
          agent_.PostImage(stream, "image/jpeg", jpg);
        }
      } else {
        agent_.PostImage(stream, "image/jpeg", jpg);
      }
    } else {
      const long long now = now_ms();
      if ((now - last_error_log_ms) >= 2000) {
        std::cerr << "[camera] failed source=" << source
                  << " stream=" << stream
                  << " error=" << spot_.LastError() << std::endl;
        last_error_log_ms = now;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }
}

void Adapter::LeaseRetainLoop() {
  const int period_ms = 100;
  const int periodic_publish_ms = 5000;  // 0.2 Hz
  long long last_retain_ms = 0;
  long long last_can_dock_pub_ms = 0;
  long long last_status_pub_ms = 0;
  long long last_mode_pub_ms = 0;
  long long last_rotate_poll_ms = 0;
  while (running_) {
    const long long now = now_ms();
    if (lease_owned_ && (now - last_retain_ms) >= std::max(100, 1000 / std::max(1, cfg_.lease_retain_hz))) {
      spot_.RetainLease();
      last_retain_ms = now;
    }

    if ((now - last_can_dock_pub_ms) >= periodic_publish_ms) {
      int dock_id = resolved_dock_id_.load();
      if (dock_id <= 0) dock_id = ResolveDockId();
      bool can_dock = false;
      if (dock_id > 0) {
        bool ok = spot_.CanDock(dock_id, &can_dock);
        if (!ok) can_dock = false;
      }
      can_dock_ = can_dock;
      agent_.PostBitset(cfg_.can_dock_stream, {{"Can dock", can_dock_.load()}});
      last_can_dock_pub_ms = now;
    }

    if (cfg_.camera_auto_rotate && (now - last_rotate_poll_ms) >= 250) {
      double wr1 = 0.0;
      rotate_camera_ccw_ = ShouldRotateCamera90CCW(&wr1);
      const bool cur = rotate_camera_ccw_.load();
      const bool prev = last_logged_rotate_camera_ccw_.load();
      if (cur != prev) {
        std::cerr << "[camera] auto-rotate state changed: rotate_ccw=" << (cur ? "true" : "false")
                  << " wr1=" << wr1
                  << " threshold=" << cfg_.camera_rotate_wr1_threshold_rad << std::endl;
        last_logged_rotate_camera_ccw_ = cur;
      }
      last_rotate_poll_ms = now;
    }

    if ((now - last_status_pub_ms) >= periodic_publish_ms) {
      agent_.PostBitset("spot.status", {
        {"Has lease", lease_owned_.load()},
        {"Teleop running", running_.load()},
        {"Teleop active", TeleopSessionActive()},
        {"Docking", docking_in_progress_.load()},
      });
      last_status_pub_ms = now;
    }

    if ((now - last_mode_pub_ms) >= periodic_publish_ms) {
      agent_.PostBitset(cfg_.stateful_mode_stream, {
        {"Walk", desired_motion_mode_.load() == kMotionModeWalk},
        {"Stairs", desired_motion_mode_.load() == kMotionModeStairs},
        {"Crawl", desired_motion_mode_.load() == kMotionModeCrawl},
      });
      last_mode_pub_ms = now;
    }

    if (lease_owned_ && TeleopSessionActive() && !docking_in_progress_) {
      ApplyDesiredArmMode(false);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }
}

void Adapter::HandleTeleop(const v1::model::ControlDatapoint& dp) {
  control_seen_ = true;
  const long long now = now_ms();
  last_control_ms_ = now;
  if ((now - last_control_log_ms_.load()) >= 1000) {
    std::cerr << "[teleop] control stream=" << dp.stream() << std::endl;
    last_control_log_ms_ = now;
  }

  if (dp.stream() == cfg_.stand_button_stream && dp.has_bitset()) {
    if (TeleopSessionActive() && bitset_has_true(dp.bitset()) && lease_owned_) spot_.Stand();
    return;
  }

  if (dp.stream() == cfg_.sit_button_stream && dp.has_bitset()) {
    if (TeleopSessionActive() && bitset_has_true(dp.bitset()) && lease_owned_) spot_.Sit();
    return;
  }

  if (dp.stream() == cfg_.estop_button_stream && dp.has_bitset()) {
    if (bitset_has_true(dp.bitset()) && lease_owned_) {
      spot_.EmergencyStop();
      moving_ = false;
    }
    return;
  }

  if (dp.stream() == cfg_.recover_button_stream && dp.has_bitset()) {
    if (TeleopSessionActive() && bitset_has_true(dp.bitset()) && lease_owned_) {
      spot_.RecoverSelfRight();
      moving_ = false;
    }
    return;
  }

  if (dp.stream() == cfg_.walk_button_stream && dp.has_bitset()) {
    if (bitset_has_true(dp.bitset())) {
      desired_motion_mode_ = kMotionModeWalk;
      std::cerr << "[mode] walk selected" << std::endl;
    }
    return;
  }

  if (dp.stream() == cfg_.stairs_button_stream && dp.has_bitset()) {
    if (bitset_has_true(dp.bitset())) {
      desired_motion_mode_ = kMotionModeStairs;
      std::cerr << "[mode] stairs selected" << std::endl;
    }
    return;
  }

  if (dp.stream() == cfg_.crawl_button_stream && dp.has_bitset()) {
    if (bitset_has_true(dp.bitset())) {
      desired_motion_mode_ = kMotionModeCrawl;
      std::cerr << "[mode] crawl selected" << std::endl;
    }
    return;
  }

  if (dp.stream() == cfg_.dock_button_stream && dp.has_bitset()) {
    if (TeleopSessionActive() && bitset_has_true(dp.bitset())) {
      std::cerr << "[dock] request received on stream=" << cfg_.dock_button_stream << std::endl;
      RequestDock();
    }
    return;
  }

  if (dp.stream() == cfg_.reset_arm_button_stream && dp.has_bitset()) {
    if (bitset_has_true(dp.bitset())) {
      std::cerr << "[arm] reset requested (stream)" << std::endl;
    }
    if (TeleopSessionActive() && bitset_has_true(dp.bitset()) && lease_owned_) {
      ApplyDesiredArmMode(true);
    } else if (bitset_has_true(dp.bitset()) && !lease_owned_) {
      std::cerr << "[arm] reset ignored: no lease" << std::endl;
    }
    return;
  }

  if (dp.stream() == cfg_.teleop_buttons_stream && dp.has_bitset()) {
    HandleButtons(dp.bitset());
    return;
  }

  if (dp.stream() == cfg_.teleop_twist_stream && dp.has_twist()) {
    HandleTwist(dp.twist());
    return;
  }
}

void Adapter::HandleHeartbeat(const v1::agent::GetTeleopHeartbeatStreamResponse& hb) {
  (void)hb;
  heartbeat_seen_ = true;
  MarkHeartbeat();
}

void Adapter::HandleCommand(const v1::model::CommandRequest& request) {
  if (request.command() == "spot.jetson.reboot") {
    const bool accepted = !reboot_requested_.exchange(true);
    agent_.SendCommandResponse(request.id(), accepted);
    if (!accepted) return;

    std::thread([]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      const int rc = std::system("/bin/systemctl reboot");
      (void)rc;
    }).detach();
    return;
  }

  if (request.command() == "spot.robot.reboot") {
    std::cerr << "spot.robot.reboot requested but not implemented yet" << std::endl;
    agent_.SendCommandResponse(request.id(), false);
    return;
  }

  agent_.SendCommandResponse(request.id(), false);
}

void Adapter::HandleButtons(const v1::model::Bitset& bitset) {
  if (!TeleopSessionActive()) return;
  for (const auto& b : bitset.bits()) {
    if (!b.value()) continue;

    if (b.key() == "Stand") {
      if (lease_owned_) spot_.Stand();
    } else if (b.key() == "Sit") {
      if (lease_owned_) spot_.Sit();
    } else if (b.key() == "E-Stop") {
      if (lease_owned_) {
        spot_.EmergencyStop();
        moving_ = false;
      }
    } else if (b.key() == "Recover") {
      if (lease_owned_ && TeleopSessionActive()) {
        spot_.RecoverSelfRight();
        moving_ = false;
      }
    } else if (b.key() == "Dock") {
      std::cerr << "[dock] request received from Buttons bit key=Dock" << std::endl;
      RequestDock();
    } else if (b.key() == "Walk") {
      desired_motion_mode_ = kMotionModeWalk;
      std::cerr << "[mode] walk selected" << std::endl;
    } else if (b.key() == "Stairs") {
      desired_motion_mode_ = kMotionModeStairs;
      std::cerr << "[mode] stairs selected" << std::endl;
    } else if (b.key() == "Crawl") {
      desired_motion_mode_ = kMotionModeCrawl;
      std::cerr << "[mode] crawl selected" << std::endl;
    } else if (b.key() == "Reset Arm") {
      std::cerr << "[arm] reset requested (Buttons)" << std::endl;
      if (lease_owned_) {
        ApplyDesiredArmMode(true);
      } else {
        std::cerr << "[arm] reset ignored: no lease" << std::endl;
      }
    }
  }
}

void Adapter::ApplyDesiredArmMode(bool force) {
  const long long now = now_ms();
  const int min_interval_ms = std::max(250, cfg_.arm_hold_interval_ms);
  if (!force && (now - last_arm_hold_cmd_ms_.load()) < min_interval_ms) return;
  if (!lease_owned_) return;

  const bool ok = spot_.ResetArmToStow();
  if (ok) {
    std::cerr << "[arm] set mode=stow" << std::endl;
    LogArmSnapshot("stow");
  } else {
    std::cerr << "[arm] failed mode=stow: " << spot_.LastError() << std::endl;
  }
  if (ok) last_arm_hold_cmd_ms_ = now;
}

void Adapter::LogArmSnapshot(const std::string& reason) {
  static const char* joints[] = {"arm0.sh0", "arm0.sh1", "arm0.el0", "arm0.el1",
                                 "arm0.wr0", "arm0.wr1", "arm0.f1x"};
  std::cerr << "[arm] snapshot reason=" << reason;
  for (const char* jname : joints) {
    double v = 0.0;
    if (spot_.GetArmJointPosition(jname, &v)) {
      std::cerr << " " << jname << "=" << v;
    }
  }
  std::cerr << std::endl;
}

bool Adapter::IsArmLikelyStowed() {
  double sh1 = 0.0;
  double el0 = 0.0;
  double el1 = 0.0;
  double wr1 = 0.0;
  if (!spot_.GetArmJointPosition("arm0.sh1", &sh1)) return false;
  if (!spot_.GetArmJointPosition("arm0.el0", &el0)) return false;
  if (!spot_.GetArmJointPosition("arm0.el1", &el1)) return false;
  if (!spot_.GetArmJointPosition("arm0.wr1", &wr1)) return false;

  // Heuristic stow envelope based on observed stow posture.
  if (sh1 > -2.6) return false;
  if (el0 < 2.6) return false;
  if (el1 < 1.0) return false;
  if (std::abs(wr1 - (-1.57)) > 0.7) return false;
  return true;
}

bool Adapter::WaitForArmStow(int timeout_ms) {
  const auto start = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::milliseconds(std::max(500, timeout_ms));
  while (lease_owned_) {
    if (IsArmLikelyStowed()) return true;
    ApplyDesiredArmMode(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if ((std::chrono::steady_clock::now() - start) >= timeout) break;
  }
  return IsArmLikelyStowed();
}

bool Adapter::ShouldRotateCamera90CCW(double* out_wr1) {
  double wr1 = 0.0;
  if (!spot_.GetArmJointPosition("arm0.wr1", &wr1)) return false;
  if (out_wr1) *out_wr1 = wr1;
  // Stow is near wr1 ~= -pi/2 and should be unrotated. Raised poses are closer to 0 and should rotate.
  return std::abs(wr1) <= std::abs(cfg_.camera_rotate_wr1_threshold_rad);
}

bool Adapter::RotateJpeg90CCW(const std::string& in_jpeg, std::string* out_jpeg) {
  if (!out_jpeg) return false;
  const std::vector<unsigned char> input(in_jpeg.begin(), in_jpeg.end());
  cv::Mat src = cv::imdecode(input, cv::IMREAD_COLOR);
  if (src.empty()) return false;
  cv::Mat rotated;
  cv::rotate(src, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
  std::vector<unsigned char> output;
  if (!cv::imencode(".jpg", rotated, output)) return false;
  out_jpeg->assign(reinterpret_cast<const char*>(output.data()), output.size());
  return true;
}

void Adapter::HandleTwist(const v1::model::Twist& twist) {
  if (!TeleopSessionActive()) return;
  if (docking_in_progress_) return;
  if (!lease_owned_) return;
  last_twist_ms_ = now_ms();

  double nx = twist.linear().x();
  double ny = twist.linear().y();
  double nz = twist.angular().z();
  double np = twist.angular().y();
  if (std::abs(nx) < cfg_.twist_deadband) nx = 0.0;
  if (std::abs(ny) < cfg_.twist_deadband) ny = 0.0;
  if (std::abs(nz) < cfg_.twist_deadband) nz = 0.0;
  if (std::abs(np) < cfg_.twist_deadband) np = 0.0;

  const double vx = std::max(-cfg_.max_vx_mps, std::min(cfg_.max_vx_mps, nx * cfg_.max_vx_mps));
  const double vy = std::max(-cfg_.max_vy_mps, std::min(cfg_.max_vy_mps, ny * cfg_.max_vy_mps));
  const double wz = std::max(-cfg_.max_wz_rps, std::min(cfg_.max_wz_rps, nz * cfg_.max_wz_rps));
  const double body_pitch =
      std::max(-cfg_.max_body_pitch_rad, std::min(cfg_.max_body_pitch_rad, np * cfg_.max_body_pitch_rad));

  if (vx == 0.0 && vy == 0.0 && wz == 0.0 && body_pitch == 0.0) {
    if (moving_.load()) {
      spot_.ZeroVelocity(1);
      moving_ = false;
    }
    return;
  }

  MotionMode mode = MotionMode::kWalk;
  switch (desired_motion_mode_.load()) {
    case kMotionModeStairs:
      mode = MotionMode::kStairs;
      break;
    case kMotionModeCrawl:
      mode = MotionMode::kCrawl;
      break;
    case kMotionModeWalk:
    default:
      mode = MotionMode::kWalk;
      break;
  }
  spot_.Velocity(vx, vy, wz, cfg_.teleop_idle_timeout_ms + 150, mode, body_pitch);
  moving_ = true;
}

void Adapter::MarkHeartbeat() {
  std::lock_guard<std::mutex> lk(hb_mu_);
  last_heartbeat_ms_ = now_ms();
}

bool Adapter::HeartbeatExpired() const {
  if (!heartbeat_seen_) return true;
  std::lock_guard<std::mutex> lk(hb_mu_);
  return (now_ms() - last_heartbeat_ms_) > cfg_.heartbeat_timeout_ms;
}

bool Adapter::TeleopSessionActive() const {
  if (!heartbeat_seen_) return false;
  if (HeartbeatExpired()) return false;
  return true;
}

void Adapter::RequestDock() { dock_requested_ = true; }

int Adapter::ResolveDockId() {
  int current = resolved_dock_id_.load();
  if (current > 0) return current;
  if (cfg_.dock_station_id > 0) {
    resolved_dock_id_ = cfg_.dock_station_id;
    return cfg_.dock_station_id;
  }
  int discovered = -1;
  if (spot_.DiscoverSingleDockId(&discovered) && discovered > 0) {
    resolved_dock_id_ = discovered;
    return discovered;
  }
  return -1;
}

void Adapter::DockLoop() {
  while (running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!dock_requested_.exchange(false)) continue;
    if (docking_in_progress_) {
      std::cerr << "[dock] request ignored: dock already in progress" << std::endl;
      continue;
    }
    if (!TeleopSessionActive()) {
      std::cerr << "[dock] request ignored: teleop session not active" << std::endl;
      continue;
    }

    int dock_id = ResolveDockId();
    if (dock_id <= 0) {
      std::cerr << "[dock] no dock id resolved: " << spot_.LastError() << std::endl;
      continue;
    }

    if (!lease_owned_) {
      lease_owned_ = spot_.AcquireBodyLease();
      if (!lease_owned_) {
        std::cerr << "[dock] failed to acquire lease for dock: " << spot_.LastError() << std::endl;
        continue;
      }
    }

    docking_in_progress_ = true;
    moving_ = false;
    spot_.ZeroVelocity(1);
    std::cerr << "[dock] starting dock id=" << dock_id << std::endl;
    bool ok = spot_.AutoDock(dock_id, cfg_.dock_attempts, cfg_.dock_poll_ms, cfg_.dock_command_timeout_sec);
    if (!ok) {
      std::cerr << "[dock] failed: " << spot_.LastError() << std::endl;
    } else {
      std::cerr << "[dock] success" << std::endl;
    }
    docking_in_progress_ = false;
  }
}

}  // namespace fsa
