#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "bosdyn/client/docking/docking_client.h"
#include "bosdyn/client/graph_nav/graph_nav_client.h"
#include "bosdyn/client/graph_nav/recording_client.h"
#include "bosdyn/client/docking/docking_helpers.h"
#include "bosdyn/client/image/image_client.h"
#include "bosdyn/client/lease/lease_client.h"
#include "bosdyn/client/power/power_client.h"
#include "bosdyn/client/robot_command/robot_command_builder.h"
#include "bosdyn/client/robot_command/robot_command_client.h"
#include "bosdyn/client/robot_state/robot_state_client.h"
#include "bosdyn/client/sdk/client_sdk.h"
#include "bosdyn/client/time_sync/time_sync_helpers.h"
#include "bosdyn/client/world_objects/world_object_client.h"
#include "formant_spot_adapter/spot_check_client.hpp"

namespace fsa {

enum class MotionMode {
  kWalk = 0,
  kStairs = 1,
  kCrawl = 2,
};

class SpotClient {
 public:
  struct FaultInfo {
    std::string id;
    std::string severity;
    std::string message;
  };

  struct ImageSourceInfo {
    std::string name;
    int cols{0};
    int rows{0};
  };

  struct StoredMap {
    ::bosdyn::api::graph_nav::Graph graph;
    std::vector<::bosdyn::api::graph_nav::WaypointSnapshot> waypoint_snapshots;
    std::vector<::bosdyn::api::graph_nav::EdgeSnapshot> edge_snapshots;
  };

  struct MappingStatus {
    bool is_recording{false};
    int status{0};
    int map_state{0};
    bool impaired{false};
    int waypoint_count{0};
    int edge_count{0};
    int waypoint_snapshot_count{0};
    int edge_snapshot_count{0};
    double total_path_length_m{0.0};
    std::vector<int> visible_fiducial_ids;
  };

  struct NavigationFeedbackSnapshot {
    int status{0};
    double remaining_route_length{0.0};
    int body_movement_status{0};
    int goal_status{0};
    int route_following_status{0};
    int blockage_status{0};
    int stuck_reason{0};
    int waiting_region_count{0};
    int callback_in_control_region_count{0};
  };

  struct RobotStateSnapshot {
    std::string motor_power_state;
    std::string shore_power_state;
    std::string behavior_state;
    bool any_estopped{false};
    bool has_battery_pct{false};
    double battery_pct{0.0};
    bool has_body_pitch_rad{false};
    double body_pitch_rad{0.0};
    std::string body_pitch_parent_frame;
    std::vector<FaultInfo> system_faults;
    std::vector<FaultInfo> behavior_faults;
    std::vector<FaultInfo> service_faults;
  };

  struct LocalizationSnapshot {
    std::string waypoint_id;
    bool has_waypoint_tform_body{false};
    double waypoint_tform_body_x{0.0};
    double waypoint_tform_body_y{0.0};
    double waypoint_tform_body_z{0.0};
  };

  SpotClient() = default;

  bool Connect(const std::string& host, const std::string& username, const std::string& password);

  bool AcquireBodyLease();
  bool RetainLease();
  bool ReturnBodyLease();

  bool Stand();
  bool Sit();
  bool EmergencyStop();
  bool RecoverSelfRight();
  bool RebootRobotBody();
  bool StartCameraCalibration();
  bool CancelCameraCalibration();
  bool GetCameraCalibrationFeedback(int* out_status, float* out_progress);
  bool ResetArmToStow();
  bool MoveArmToPoseBody(double x, double y, double z, double qw, double qx, double qy, double qz,
                         double move_sec);
  bool GetArmJointPosition(const std::string& joint_name, double* out_position);
  bool GetRobotStateSnapshot(RobotStateSnapshot* out_snapshot);
  bool Velocity(double vx, double vy, double wz, int end_after_ms = 600,
                MotionMode mode = MotionMode::kWalk, double body_pitch_rad = 0.0);
  bool ZeroVelocity(int repeats = 1);

  bool GetImageJpeg(const std::string& source, std::string* out_bytes);
  bool ListImageSources(std::vector<ImageSourceInfo>* out_sources);
  bool StartGraphRecording(std::string* out_created_waypoint_id = nullptr);
  bool StopGraphRecording();
  bool CreateGraphWaypoint(const std::string& waypoint_name, std::string* out_waypoint_id);
  bool DownloadCurrentGraph(::bosdyn::api::graph_nav::Graph* out_graph);
  bool DownloadCurrentMap(StoredMap* out_map);
  bool GetMappingStatus(MappingStatus* out_status);
  bool UploadGraphMap(const StoredMap& map_data);
  bool ClearGraph();
  bool SetLocalizationFiducial();
  bool GetLocalizationWaypointId(std::string* out_waypoint_id);
  bool GetLocalizationSnapshot(LocalizationSnapshot* out_snapshot);
  bool NavigateToWaypoint(const std::string& waypoint_id, int command_timeout_sec,
                          uint32_t* out_command_id = nullptr);
  bool NavigateToWaypointStraight(const std::string& waypoint_id, int command_timeout_sec,
                                  uint32_t* out_command_id = nullptr);
  bool GetNavigationFeedback(uint32_t command_id, int* out_status);
  bool GetNavigationFeedbackSnapshot(uint32_t command_id, NavigationFeedbackSnapshot* out_feedback);
  bool DiscoverSingleDockId(int* out_dock_id);
  bool CanDock(int dock_id, bool* out_can_dock);
  bool AutoDock(int dock_id, int attempts, int poll_ms, int command_timeout_sec);
  void RequestDockCancel();
  std::string LastError() const;

 private:
  bool EnsureTimeSync();
  bool EnsureDockingClient();
  bool EnsurePowerClient();
  bool EnsureGraphNavClient();
  bool EnsureGraphNavRecordingClient();
  bool EnsureSpotCheckClient();
  bool EnsureRobotStateClient();
  bool EnsureWorldObjectClient();
  bool EnsurePowered();
  void SetLastError(const std::string& msg);

  std::unique_ptr<::bosdyn::client::ClientSdk> sdk_;
  std::unique_ptr<::bosdyn::client::Robot> robot_;

  ::bosdyn::client::RobotCommandClient* robot_command_client_{nullptr};
  ::bosdyn::client::LeaseClient* lease_client_{nullptr};
  ::bosdyn::client::TimeSyncEndpoint* time_sync_endpoint_{nullptr};
  ::bosdyn::client::ImageClient* image_client_{nullptr};
  ::bosdyn::client::PowerClient* power_client_{nullptr};
  ::bosdyn::client::GraphNavClient* graph_nav_client_{nullptr};
  ::bosdyn::client::GraphNavRecordingClient* graph_nav_recording_client_{nullptr};
  ::bosdyn::client::SpotCheckClient* spot_check_client_{nullptr};
  ::bosdyn::client::DockingClient* docking_client_{nullptr};
  ::bosdyn::client::RobotStateClient* robot_state_client_{nullptr};
  ::bosdyn::client::WorldObjectClient* world_object_client_{nullptr};
  ::bosdyn::api::Lease body_lease_;
  std::atomic<bool> cancel_docking_{false};
  mutable std::recursive_mutex api_mu_;
  mutable std::mutex err_mu_;
  std::string last_error_;
};

}  // namespace fsa
