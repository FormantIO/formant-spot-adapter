#pragma once

#include <atomic>
#include <deque>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
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
  void CameraLoop(const std::string& source, const std::string& stream, int fps);
  void LeaseRetainLoop();
  void HandleTeleop(const v1::model::ControlDatapoint& dp);
  void HandleHeartbeat(const v1::agent::GetTeleopHeartbeatStreamResponse& hb);
  void HandleCommand(const v1::model::CommandRequest& request);
  void HandleButtons(const v1::model::Bitset& bitset, const std::string& source_stream);
  void HandleButtonPress(const std::string& button_key, const std::string& source_stream);
  void HandleTwist(const v1::model::Twist& twist);
  void DockLoop();
  void ApplyDesiredArmMode(bool force = false);
  bool IsArmLikelyStowed();
  bool WaitForArmStow(int timeout_ms);
  void MarkHeartbeat();
  bool HeartbeatExpired() const;
  bool TeleopSessionActive() const;
  void RequestDock();
  int ResolveDockId();
  void QueueStatusText(const std::string& stream, const std::string& value);
  void QueueStatusNumeric(const std::string& stream, double value);
  void QueueStatusBitset(const std::string& stream, const std::vector<std::pair<std::string, bool>>& bits);
  void FlushQueuedStatus(long long now_ms);
  void EmitLog(const std::string& msg);
  void FlushAdapterLogStream(long long now_ms);
  bool HandleMapCommand(const v1::model::CommandRequest& request);
  bool HandleWaypointCommand(const v1::model::CommandRequest& request);
  bool ExtractCommandText(const v1::model::CommandRequest& request, std::string* out) const;
  std::string ExtractParam(const std::string& input,
                           const std::vector<std::string>& keys) const;
  bool LoadAdapterMapState();
  bool SaveAdapterMapState();
  bool SaveCurrentMapToDisk(const std::string& map_id);
  bool LoadMapFromDisk(const std::string& map_id, SpotClient::StoredMap* out_map);
  bool DeleteMapFromDisk(const std::string& map_id);
  std::string MapDirectoryFor(const std::string& map_id) const;
  void RefreshGraphWaypointIndex();
  void PublishWaypointsText(bool force);
  void PublishMapsText(bool force);
  void MaybeRestoreActiveMap(long long now);
  void PollGraphNavNavigation(long long now);
  bool IsGraphNavNavigationActive() const;
  bool SpotConnected() const;
  void ConnectionLoop();
  void SetSpotConnected();
  void SetSpotDisconnected(const std::string& reason);
  std::string BuildSpotConnectionJson() const;
  void ApplySoftRecoveryForRobotState(const SpotClient::RobotStateSnapshot& snap);
  void PublishFaultEvents(const SpotClient::RobotStateSnapshot& snap, long long now_ms);
  void ResetFaultEventsState();
  bool ResolveWaypointNameLocked(const std::string& name, std::string* out_waypoint_id) const;
  void PruneAliasesForLoadedGraphLocked();
  std::string BuildWaypointTextLocked() const;
  std::string BuildMapsTextLocked() const;
  std::string EnsureActiveMapIdLocked();
  static std::string Trim(const std::string& in);
  static std::string ToLower(const std::string& in);

  Config cfg_;
  FormantAgentClient agent_;
  SpotClient spot_;

  std::atomic<bool> running_{false};
  std::atomic<bool> lease_owned_{false};
  std::atomic<long long> last_twist_ms_{0};
  std::atomic<long long> last_twist_timeout_log_ms_{0};
  std::atomic<long long> last_nonzero_cmd_ms_{0};
  std::atomic<bool> moving_{false};
  mutable std::mutex twist_cmd_mu_;
  double desired_vx_{0.0};
  double desired_vy_{0.0};
  double desired_wz_{0.0};
  double desired_body_pitch_{0.0};
  std::atomic<bool> desired_twist_valid_{false};
  std::atomic<long long> last_control_ms_{0};
  std::atomic<bool> control_seen_{false};
  std::atomic<bool> heartbeat_seen_{false};
  std::atomic<bool> docking_in_progress_{false};
  std::atomic<bool> dock_requested_{false};
  std::atomic<bool> can_dock_{false};
  std::atomic<bool> reboot_requested_{false};
  std::atomic<int> desired_motion_mode_{0};
  std::atomic<long long> last_arm_hold_cmd_ms_{0};
  std::atomic<long long> last_arm_error_log_ms_{0};
  std::atomic<long long> last_lease_attempt_ms_{0};
  std::atomic<long long> dock_cooldown_until_ms_{0};
  std::atomic<int> resolved_dock_id_{-1};
  std::string last_logged_control_stream_;

  std::vector<std::thread> camera_threads_;
  std::thread lease_thread_;
  std::thread dock_thread_;
  std::thread connection_thread_;

  mutable std::mutex hb_mu_;
  long long last_heartbeat_ms_{0};

  std::mutex adapter_log_mu_;
  std::deque<std::string> adapter_log_buffer_;
  std::string adapter_log_pending_payload_;
  std::atomic<long long> last_adapter_log_pub_ms_{0};
  std::atomic<long long> adapter_log_retry_after_ms_{0};
  std::atomic<int> adapter_log_backoff_ms_{1000};

  struct PendingStatusDatapoint {
    enum class Kind {
      kText,
      kNumeric,
      kBitset
    };
    Kind kind{Kind::kText};
    std::string text_value;
    double numeric_value{0.0};
    std::vector<std::pair<std::string, bool>> bitset_value;
    bool pending{false};
    long long retry_after_ms{0};
    int backoff_ms{1000};
  };
  std::mutex status_queue_mu_;
  std::unordered_map<std::string, PendingStatusDatapoint> status_queue_;
  std::atomic<long long> last_status_flush_ms_{0};
  std::atomic<int> spot_connection_state_{0};  // 0=disconnected, 1=connecting, 2=connected
  std::atomic<long long> last_spot_connect_success_ms_{0};
  std::atomic<long long> last_spot_connect_attempt_ms_{0};
  std::atomic<long long> next_spot_connect_attempt_ms_{0};
  std::atomic<long long> last_spot_healthcheck_ms_{0};
  std::atomic<int> spot_reconnect_attempt_{0};
  std::atomic<int> spot_health_failures_{0};
  std::atomic<bool> spot_degraded_non_estop_{false};
  std::atomic<long long> last_soft_recovery_log_ms_{0};
  mutable std::mutex spot_connection_mu_;
  std::string last_spot_error_;
  std::string spot_degraded_reason_;
  mutable std::mutex fault_events_mu_;
  std::unordered_map<std::string, SpotClient::FaultInfo> last_fault_events_;
  std::atomic<long long> last_fault_summary_pub_ms_{0};

  std::mutex map_mu_;
  std::string active_map_id_;
  std::string default_map_id_;
  std::unordered_map<std::string, std::unordered_map<std::string, std::string>> waypoint_aliases_by_map_;
  std::unordered_map<std::string, std::string> loaded_waypoint_id_to_label_;
  std::unordered_map<std::string, std::string> loaded_waypoint_label_to_id_;
  std::string last_waypoint_text_payload_;
  std::string last_maps_text_payload_;
  std::string last_map_progress_signature_;
  std::string pending_restore_map_id_;
  std::string nav_target_waypoint_id_;
  std::string nav_target_waypoint_name_;
  std::string last_nav_feedback_signature_;
  int last_nav_status_{0};
  double last_nav_remaining_route_m_{0.0};
  long long last_nav_progress_change_ms_{0};
  std::atomic<long long> last_waypoint_pub_ms_{0};
  std::atomic<long long> last_maps_pub_ms_{0};
  std::atomic<long long> next_map_restore_attempt_ms_{0};
  std::atomic<long long> last_nav_feedback_poll_ms_{0};
  std::atomic<long long> last_nav_hold_log_ms_{0};
  std::atomic<long long> last_nav_diag_log_ms_{0};
  std::atomic<bool> force_waypoint_publish_{true};
  std::atomic<bool> force_maps_publish_{true};
  std::atomic<bool> map_recording_active_{false};
  std::atomic<bool> graphnav_navigation_active_{false};
  std::atomic<bool> nav_auto_recovered_{false};
  std::atomic<uint32_t> last_graph_nav_command_id_{0};
};

}  // namespace fsa
