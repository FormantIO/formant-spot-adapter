#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include "formant_spot_adapter/config.hpp"
#include "formant_spot_adapter/formant_agent_client.hpp"
#include "formant_spot_adapter/spot_client.hpp"

namespace fsa {

class Adapter {
 public:
  explicit Adapter(const Config& config);
  bool Init();
  void Run();
  void Stop();

 private:
  void CameraLoop(const std::string& source, const std::string& stream, bool apply_rotation, int fps);
  void LeaseRetainLoop();
  void HandleTeleop(const v1::model::ControlDatapoint& dp);
  void HandleHeartbeat(const v1::agent::GetTeleopHeartbeatStreamResponse& hb);
  void HandleCommand(const v1::model::CommandRequest& request);
  void HandleButtons(const v1::model::Bitset& bitset);
  void HandleTwist(const v1::model::Twist& twist);
  void DockLoop();
  void ApplyDesiredArmMode(bool force = false);
  void LogArmSnapshot(const std::string& reason);
  bool IsArmLikelyStowed();
  bool WaitForArmStow(int timeout_ms);
  bool ShouldRotateCamera90CCW(double* out_wr1 = nullptr);
  static bool RotateJpeg90CCW(const std::string& in_jpeg, std::string* out_jpeg);
  void MarkHeartbeat();
  bool HeartbeatExpired() const;
  bool TeleopSessionActive() const;
  void RequestDock();
  int ResolveDockId();

  Config cfg_;
  FormantAgentClient agent_;
  SpotClient spot_;

  std::atomic<bool> running_{false};
  std::atomic<bool> lease_owned_{false};
  std::atomic<long long> last_twist_ms_{0};
  std::atomic<bool> moving_{false};
  std::atomic<long long> last_control_ms_{0};
  std::atomic<bool> control_seen_{false};
  std::atomic<bool> heartbeat_seen_{false};
  std::atomic<bool> docking_in_progress_{false};
  std::atomic<bool> dock_requested_{false};
  std::atomic<bool> can_dock_{false};
  std::atomic<bool> reboot_requested_{false};
  std::atomic<bool> rotate_camera_ccw_{false};
  std::atomic<bool> last_logged_rotate_camera_ccw_{false};
  std::atomic<long long> last_rotate_error_log_ms_{0};
  std::atomic<int> desired_motion_mode_{0};
  std::atomic<long long> last_arm_hold_cmd_ms_{0};
  std::atomic<long long> last_lease_attempt_ms_{0};
  std::atomic<long long> last_teleop_diag_log_ms_{0};
  std::atomic<long long> last_control_log_ms_{0};
  std::atomic<int> resolved_dock_id_{-1};

  std::vector<std::thread> camera_threads_;
  std::thread lease_thread_;
  std::thread dock_thread_;

  mutable std::mutex hb_mu_;
  long long last_heartbeat_ms_{0};
};

}  // namespace fsa
