#pragma once

#include <atomic>
#include <deque>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <memory>
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
  void CameraLoop(const std::string& source, const std::string& stream, int output_fps,
                  int poll_hz, bool rotate_180, bool normalize_jpeg, int post_timeout_ms);
  void SurroundImageLoop(int output_fps, int poll_hz);
  void FrontImageLoop(const std::string& stream, int fps, int poll_hz);
  void LocalizationImageLoop(const std::string& stream, int fps, int poll_hz);
  void GraphNavMapImageLoop(const std::string& stream, int fps, int poll_hz);
  void LeaseRetainLoop();
  void HandleTeleop(const v1::model::ControlDatapoint& dp);
  void HandleHeartbeat(const v1::agent::GetTeleopHeartbeatStreamResponse& hb);
  void HandleCommand(const v1::model::CommandRequest& request);
  void HandleJoy(const v1::model::Joy& joy);
  void HandleButtons(const v1::model::Bitset& bitset, const std::string& source_stream);
  void HandleButtonPress(const std::string& button_key, const std::string& source_stream);
  bool EnsureLeaseForCommand(const std::string& action_name);
  bool ExecuteStandAction(bool require_teleop, bool auto_acquire_lease);
  bool ExecuteSitAction(bool require_teleop, bool auto_acquire_lease);
  bool ExecuteRecoverAction(bool require_teleop, bool auto_acquire_lease);
  bool ExecuteDockAction(bool require_teleop);
  bool ExecuteResetArmAction(bool require_teleop, bool auto_acquire_lease);
  bool ExecuteRotateCommand(const v1::model::CommandRequest& request, bool left);
  bool ExecuteQueuedCommand(const v1::model::CommandRequest& request);
  void CommandExecutionLoop();
  bool ExecuteCameraCalibrateCommand();
  bool ExecuteWaypointGotoCommand(const v1::model::CommandRequest& request);
  bool ExecuteWaypointGotoStraightCommand(const v1::model::CommandRequest& request);
  bool ExecuteGraphNavGotoPoseCommand(const v1::model::CommandRequest& request, bool straight);
  bool ExecuteGraphNavCancelCommand();
  bool ExecuteDockSequence(bool return_and_dock);
  bool ExecuteUndockCommand();
  bool WaitForGraphNavCommandResult(uint32_t command_id, long long timeout_ms);
  bool EnsureLocalizedForNavigation(const std::string& action_name);
  bool StartNavigateWithRecovery(const std::string& action_name,
                                 const std::string& waypoint_id,
                                 uint32_t* out_command_id,
                                 bool straight = false,
                                 bool has_waypoint_goal = false,
                                 double waypoint_goal_x = 0.0,
                                 double waypoint_goal_y = 0.0,
                                 double waypoint_goal_yaw_rad = 0.0);
  bool RetryActiveGraphNavCommandWithRecovery(const std::string& action_name,
                                              uint32_t* out_command_id);
  void HandleTwist(const v1::model::Twist& twist);
  void ApplyTeleopTwist(const v1::model::Twist& twist, int source_id, const char* source_name);
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
  bool SaveMapToDisk(const std::string& map_id, const SpotClient::StoredMap& map_data);
  bool LoadMapFromDisk(const std::string& map_id, SpotClient::StoredMap* out_map);
  bool DeleteMapFromDisk(const std::string& map_id);
  std::string MapDirectoryFor(const std::string& map_id) const;
  void RefreshGraphWaypointIndex();
  void ClearGraphNavMapArtifacts();
  bool RefreshGraphNavMapArtifacts(const std::string& map_id, const SpotClient::StoredMap& map_data);
  bool RefreshGraphNavMapArtifactsFromDisk(const std::string& map_id);
  void RefreshGraphNavMetadataForMap(const std::string& map_id);
  void PublishWaypointsText(bool force);
  void PublishMapsText(bool force);
  void PublishGraphNavMetadataText(bool force);
  void PublishGraphNavOverlayText(bool force);
  void MaybePublishGraphNavMap(long long now_ms);
  void PublishCurrentMapText(bool force);
  void PublishDefaultMapText(bool force);
  void PollCurrentWaypointAtStatus(long long now_ms);
  void PublishCurrentWaypointText(long long now_ms);
  void PublishGraphNavNavState(long long now_ms);
  void MaybeRestoreActiveMap(long long now);
  void PollGraphNavNavigation(long long now);
  bool IsGraphNavNavigationActive() const;
  bool SpotConnected() const;
  void ConnectionLoop();
  void SetSpotConnected();
  void SetSpotDisconnected(const std::string& reason);
  std::string BuildSpotConnectionJson() const;
  void ApplySoftRecoveryForRobotState(const SpotClient::RobotStateSnapshot& snap);
  void PublishFaultEvents(const SpotClient::RobotStateSnapshot& snap);
  void ResetFaultEventsState();
  bool ResolveWaypointNameLocked(const std::string& name, std::string* out_waypoint_id) const;
  bool ResolveSavedDockHomeLocked(std::string* out_map_id, std::string* out_waypoint_id) const;
  bool LookupWaypointSeedPoseLocked(const std::string& waypoint_id,
                                    SpotClient::Pose3D* out_pose,
                                    std::string* out_map_uuid = nullptr) const;
  void PruneAliasesForLoadedGraphLocked();
  void CaptureDockWaypointCandidate();
  void CommitDockWaypointCandidateOnSuccess();
  void ClearDockWaypointCandidateLocked();
  void ResetGraphNavNavStateLocked();
  void MarkGraphNavNavigationTerminalLocked(uint32_t command_id,
                                            int status,
                                            double remaining_route_m,
                                            const std::string& result,
                                            const std::string& reason,
                                            long long updated_ms);
  void ClearGraphNavNavTargetLocked();
  void SetGraphNavNavTarget(const std::string& mode,
                            const std::string& waypoint_id,
                            const std::string& waypoint_name,
                            const std::string& map_id,
                            const std::string& map_uuid,
                            bool has_seed_goal = false,
                            double seed_x = 0.0,
                            double seed_y = 0.0,
                            double seed_yaw_rad = 0.0,
                            bool has_waypoint_goal = false,
                            double waypoint_goal_x = 0.0,
                            double waypoint_goal_y = 0.0,
                            double waypoint_goal_yaw_rad = 0.0);
  bool ValidateGraphNavCommandMapUuid(const std::string& command_name,
                                      const std::string& requested_map_uuid,
                                      bool require_map_uuid,
                                      std::string* out_map_id = nullptr,
                                      std::string* out_map_uuid = nullptr);
  bool IsStreamEnabled(const std::string& stream) const;
  bool AnyStreamEnabled(std::initializer_list<const char*> streams) const;
  void DisableConfiguredStreamIfDisabled(std::string* stream, const char* label);
  std::string BuildWaypointTextLocked() const;
  std::string BuildMapsText(const std::string& active_map_id,
                            const std::string& default_map_id) const;
  std::string ResolveWaypointNameForIdLocked(const std::string& waypoint_id) const;
  std::string BuildCurrentMapTextLocked() const;
  std::string BuildDefaultMapTextLocked() const;
  std::string BuildGraphNavNavStateJson(
      const SpotClient::LocalizationSnapshot* localization = nullptr) const;
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
  std::atomic<bool> return_and_dock_requested_{false};
  std::atomic<bool> can_dock_{false};
  std::atomic<bool> reboot_requested_{false};
  std::atomic<bool> command_action_in_progress_{false};
  std::atomic<int> desired_motion_mode_{0};
  std::atomic<long long> last_arm_hold_cmd_ms_{0};
  std::atomic<long long> last_arm_error_log_ms_{0};
  std::atomic<long long> last_lease_attempt_ms_{0};
  std::atomic<long long> dock_cooldown_until_ms_{0};
  std::atomic<int> resolved_dock_id_{-1};
  std::atomic<int> active_motion_source_{0};
  std::string last_logged_control_stream_;
  std::vector<int> last_joy_buttons_;

  std::vector<std::thread> camera_threads_;
  std::thread surround_image_thread_;
  std::thread front_image_thread_;
  std::thread localization_image_thread_;
  std::thread graphnav_map_image_thread_;
  std::thread lease_thread_;
  std::thread dock_thread_;
  std::thread connection_thread_;
  std::thread command_exec_thread_;

  std::mutex command_queue_mu_;
  std::condition_variable command_queue_cv_;
  std::deque<v1::model::CommandRequest> command_queue_;

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

  std::mutex map_mu_;
  std::string active_map_id_;
  std::string default_map_id_;
  std::unordered_map<std::string, std::unordered_map<std::string, std::string>> waypoint_aliases_by_map_;
  std::unordered_map<std::string, std::string> dock_waypoint_by_map_;
  std::unordered_map<std::string, std::string> loaded_waypoint_id_to_label_;
  std::unordered_map<std::string, std::vector<std::string>> loaded_waypoint_label_to_ids_;
  std::unordered_map<std::string, std::vector<std::string>> loaded_waypoint_lower_label_to_ids_;
  struct GraphNavMapArtifacts {
    std::string map_id;
    std::string map_uuid;
    SpotClient::GraphNavMapSnapshot snapshot;
    std::string metadata_json;
  };
  std::shared_ptr<GraphNavMapArtifacts> graphnav_map_artifacts_;
  uint64_t graphnav_map_artifacts_version_{0};
  std::string last_waypoint_text_payload_;
  std::string last_maps_text_payload_;
  std::string last_graphnav_metadata_payload_;
  std::string last_graphnav_overlay_payload_;
  std::string last_current_map_text_payload_;
  std::string last_default_map_text_payload_;
  std::string last_map_progress_signature_;
  std::string pending_restore_map_id_;
  std::string pending_dock_candidate_map_id_;
  std::string pending_dock_candidate_waypoint_id_;
  std::string last_nav_feedback_signature_;
  int last_nav_status_{0};
  double last_nav_remaining_route_m_{0.0};
  long long last_nav_progress_change_ms_{0};
  std::string last_nav_state_payload_;
  std::string nav_phase_{"idle"};
  std::string nav_terminal_result_;
  std::string nav_terminal_reason_;
  uint32_t nav_last_completed_command_id_{0};
  long long nav_last_updated_ms_{0};
  std::string nav_target_mode_;
  std::string nav_target_waypoint_id_;
  std::string nav_target_waypoint_name_;
  std::string nav_target_map_id_;
  std::string nav_target_map_uuid_;
  bool nav_target_has_seed_goal_{false};
  double nav_target_seed_x_{0.0};
  double nav_target_seed_y_{0.0};
  double nav_target_seed_yaw_rad_{0.0};
  bool nav_target_has_waypoint_goal_{false};
  double nav_target_waypoint_goal_x_{0.0};
  double nav_target_waypoint_goal_y_{0.0};
  double nav_target_waypoint_goal_yaw_rad_{0.0};
  std::atomic<long long> last_waypoint_pub_ms_{0};
  std::atomic<long long> last_waypoint_at_poll_ms_{0};
  std::atomic<long long> last_waypoint_at_pub_ms_{0};
  std::atomic<long long> last_maps_pub_ms_{0};
  std::atomic<long long> last_graphnav_metadata_pub_ms_{0};
  std::atomic<long long> last_graphnav_overlay_pub_ms_{0};
  std::atomic<long long> last_current_map_pub_ms_{0};
  std::atomic<long long> last_default_map_pub_ms_{0};
  std::atomic<long long> next_graphnav_map_post_ms_{0};
  std::atomic<long long> next_map_restore_attempt_ms_{0};
  std::atomic<long long> last_nav_feedback_poll_ms_{0};
  std::atomic<long long> last_nav_state_pub_ms_{0};
  std::atomic<long long> last_nav_hold_log_ms_{0};
  std::atomic<long long> last_nav_diag_log_ms_{0};
  std::atomic<bool> force_waypoint_publish_{true};
  std::atomic<bool> force_maps_publish_{true};
  std::atomic<bool> force_graphnav_metadata_publish_{true};
  std::atomic<bool> force_graphnav_overlay_publish_{true};
  std::string current_waypoint_at_text_;
  std::atomic<bool> map_recording_active_{false};
  std::atomic<bool> graphnav_navigation_active_{false};
  std::atomic<bool> nav_auto_recovered_{false};
  std::atomic<uint32_t> last_graph_nav_command_id_{0};
  int graphnav_map_post_backoff_ms_{1000};
  uint64_t last_graphnav_map_posted_version_{0};
  mutable std::mutex nav_state_mu_;

  static constexpr size_t kMaxCommandQueueDepth = 10;
};

}  // namespace fsa
