#pragma once

#include "formant_spot_adapter/localize_recovery.hpp"
#include <atomic>
#include <functional>
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
#include "formant_spot_adapter/dock_preflight.hpp"
#include "formant_spot_adapter/map_processing_client.hpp"
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

  struct Pose3D {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double qx{0.0};
    double qy{0.0};
    double qz{0.0};
    double qw{1.0};
  };

  struct CameraIntrinsics {
    enum class ModelType {
      kUnknown = 0,
      kPinhole = 1,
      kKannalaBrandt = 2,
    };

    ModelType model_type{ModelType::kUnknown};
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    double skew_x{0.0};
    double skew_y{0.0};
    double k1{0.0};
    double k2{0.0};
    double k3{0.0};
    double k4{0.0};
  };

  struct ImageFrame {
    std::string source_name;
    std::string frame_name_image_sensor;
    int cols{0};
    int rows{0};
    int status{0};
    bool has_acquisition_time{false};
    int64_t acquisition_time_sec{0};
    int32_t acquisition_time_nanos{0};
    bool has_body_tform_sensor{false};
    Pose3D body_tform_sensor;
    CameraIntrinsics intrinsics;
    std::string encoded_image;
  };

  struct StoredMap {
    ::bosdyn::api::graph_nav::Graph graph;
    std::vector<::bosdyn::api::graph_nav::WaypointSnapshot> waypoint_snapshots;
    std::vector<::bosdyn::api::graph_nav::EdgeSnapshot> edge_snapshots;
  };

  struct TopologyProcessResult {
    int new_edge_count{0};
    int new_waypoint_count{0};
    bool map_modified{false};
    bool timed_out{false};
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

  // Why GraphNav said ROBOT_IMPAIRED (its RobotImpairedState). fault_ids use
  // the same ids as RobotStateSnapshot's FaultInfo, so the adapter can tell
  // whether a named cause is still listed.
  struct ImpairedCause {
    std::string status;  // e.g. "IMPAIRED_STATUS_SYSTEM_FAULT"
    std::vector<std::string> fault_ids;
    std::string detail;  // e.g. "camera_server: right camera failed to initialize"
  };

  struct NavigationFeedbackSnapshot {
    int status{0};
    ImpairedCause impaired;  // filled when status is STATUS_ROBOT_IMPAIRED
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
    double waypoint_tform_body_qx{0.0};
    double waypoint_tform_body_qy{0.0};
    double waypoint_tform_body_qz{0.0};
    double waypoint_tform_body_qw{1.0};
    bool has_seed_tform_body{false};
    double seed_tform_body_x{0.0};
    double seed_tform_body_y{0.0};
    double seed_tform_body_z{0.0};
    double seed_tform_body_qx{0.0};
    double seed_tform_body_qy{0.0};
    double seed_tform_body_qz{0.0};
    double seed_tform_body_qw{1.0};
  };

  // Localize against the loaded map from a guess of "the robot is here relative
  // to this waypoint", without using fiducials. See localize_recovery.hpp.
  using WaypointLocalizeRequest = WaypointLocalizeSpec;

  struct WaypointLocalizeResult {
    // STATUS_OK, QUALITY_CHECK_SUCCESS and no suspected ambiguity.
    bool ok{false};
    // GraphNav applied a localization (STATUS_OK). `applied && !ok` means the
    // robot is now localized to a result that was rejected.
    bool applied{false};
    // Why it was rejected, empty when ok.
    std::string reason;
    // SetLocalizationResponse status name, e.g. STATUS_OK or STATUS_FAILED.
    std::string status;
    // QualityCheckResult name; GraphNav applies the localization even when poor.
    std::string quality_check;
    bool suspected_ambiguity{false};
    // Which check accepted the result (see LocalizeAcceptance::basis).
    std::string basis;
    // How far GraphNav moved the pose from the guess, when it kept the guessed
    // waypoint as the reference.
    bool has_correction{false};
    double correction_m{0.0};  // horizontal (x, y)
    double correction_z_m{0.0};
    double correction_yaw_rad{0.0};
    // The REQUEST asked for visual refinement and for its quality to be
    // verified. Not a response fact: SetLocalizationResponse has no field
    // saying the refinement ran, only STATUS_VISUAL_ALIGNMENT_FAILED when it
    // did not work. Named for what it is, so acceptance rule 2 cannot be read
    // as "the robot confirmed the alignment".
    bool visual_verification_requested{false};
    // Waypoint GraphNav ended up localized to.
    std::string localized_waypoint_id;
  };

  struct OccupancyGridMapSnapshot {
    std::string map_type;
    double resolution_m{0.0};
    int width{0};
    int height{0};
    Pose3D seed_tform_grid;
    std::vector<int32_t> occupancy;
  };

  struct LocalizationMapSnapshot {
    bool localized{false};
    std::string waypoint_id;
    bool has_waypoint_tform_body{false};
    Pose3D waypoint_tform_body;
    bool has_seed_tform_body{false};
    Pose3D seed_tform_body;
    bool has_map{false};
    OccupancyGridMapSnapshot map;
  };

  struct GraphNavWaypoint {
    std::string id;
    std::string snapshot_id;
    std::string label;
    Pose3D seed_tform_waypoint;
  };

  struct GraphNavEdge {
    std::string from_waypoint_id;
    std::string to_waypoint_id;
    std::string snapshot_id;
    Pose3D from_tform_to;
  };

  struct GraphNavObject {
    std::string id;
    Pose3D seed_tform_object;
  };

  struct GraphNavMapSnapshot {
    bool has_anchoring{false};
    bool has_map{false};
    OccupancyGridMapSnapshot map;
    std::vector<GraphNavWaypoint> waypoints;
    std::vector<GraphNavEdge> edges;
    std::vector<GraphNavObject> objects;
  };

  SpotClient() = default;

  bool Connect(const std::string& host, const std::string& username, const std::string& password);
  void SetArmPresentOverride(int override_state);
  bool HasArm();
  bool ArmPresenceKnown() const;

  bool AcquireBodyLease();
  bool TryAcquireBodyLeaseNoTakeover();
  bool RetainLease();
  bool ReturnBodyLease();
  void InvalidateBodyLease();

  bool Stand();
  bool Sit();
  bool EmergencyStop();
  // A Spot stop command: the robot stops in place, preempting GraphNav and
  // docking. Does not power the motors on.
  bool StopMotion();
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
  bool GetImageFrames(const std::vector<std::string>& sources,
                      std::vector<ImageFrame>* out_frames);
  bool GetRobotClockSkewNanos(int64_t* out_skew_nanos);
  bool ListImageSources(std::vector<ImageSourceInfo>* out_sources);
  bool StartGraphRecording(std::string* out_created_waypoint_id = nullptr);
  bool StopGraphRecording();
  bool CreateGraphWaypoint(const std::string& waypoint_name, std::string* out_waypoint_id);
  bool DownloadCurrentGraph(::bosdyn::api::graph_nav::Graph* out_graph);
  bool DownloadCurrentMap(StoredMap* out_map);
  bool GetMappingStatus(MappingStatus* out_status);
  // Runs the robot's map processing service (odometry + fiducial loop closure)
  // on the graph currently loaded on the robot and writes the new edges into
  // it (modify_map_on_server). Returns false on an RPC error or a non-OK
  // processing status; a processing timeout is a partial success.
  bool ProcessMapTopology(TopologyProcessResult* out_result);
  bool BuildGraphNavMapSnapshot(const StoredMap& map_data, GraphNavMapSnapshot* out_snapshot);
  bool UploadGraphMap(const StoredMap& map_data);
  // Whether the last NavigateTo* call made on the calling thread was refused
  // with STATUS_ROBOT_IMPAIRED, and GraphNav's reason when it was. Per thread,
  // so a navigate from another thread cannot overwrite it.
  bool LastNavigateImpaired(ImpairedCause* out_cause) const;

  // True once Spot's directory lists the GraphNav service and a client for it
  // exists. Right after a power-on it is not registered yet
  // (UnregisteredService); LastError() then says so.
  bool GraphNavServiceReady();
  bool ClearGraph();
  // Number of waypoints in the graph currently loaded on the robot. Reads the
  // recording service's map stats first (no graph download). A zero or missing
  // count is cross-checked: a localization to a waypoint means the graph is not
  // empty (count reported as 1); otherwise one DownloadGraph decides.
  // `out_signal` names the check that decided: record_status,
  // localization_state or download_graph.
  bool GetGraphWaypointCount(int* out_count, std::string* out_signal = nullptr);
  // Who holds the body lease right now; empty client name when nobody does.
  bool GetBodyLeaseOwner(std::string* out_client_name);
  bool SetLocalizationFiducial();
  // No lease needed. Returns true only for an accepted result (see
  // EvaluateWaypointLocalization). On failure LastError() gives the reason.
  bool SetLocalizationAtWaypoint(const WaypointLocalizeRequest& request,
                                 const LocalizeAcceptancePolicy& policy,
                                 WaypointLocalizeResult* out_result);
  // Height of the body above Spot's ground plane estimate ("gpe" frame), meters.
  // About 0.5 standing and lower sitting. False when the robot state has no
  // ground plane estimate.
  bool GetBodyHeightAboveGround(double* out_height_m);
  bool GetLocalizationWaypointId(std::string* out_waypoint_id);
  bool GetLocalizationSnapshot(LocalizationSnapshot* out_snapshot);
  bool GetLocalizationMapSnapshot(LocalizationMapSnapshot* out_snapshot);
  bool NavigateToWaypoint(const std::string& waypoint_id, int command_timeout_sec,
                          uint32_t* out_command_id = nullptr);
  bool NavigateToWaypointStraight(const std::string& waypoint_id, int command_timeout_sec,
                                  uint32_t* out_command_id = nullptr);
  bool NavigateToWaypointPose(const std::string& waypoint_id,
                              double waypoint_tform_body_goal_x,
                              double waypoint_tform_body_goal_y,
                              double waypoint_tform_body_goal_yaw_rad,
                              int command_timeout_sec,
                              uint32_t* out_command_id = nullptr);
  bool NavigateToWaypointPoseStraight(const std::string& waypoint_id,
                                      double waypoint_tform_body_goal_x,
                                      double waypoint_tform_body_goal_y,
                                      double waypoint_tform_body_goal_yaw_rad,
                                      int command_timeout_sec,
                                      uint32_t* out_command_id = nullptr);
  bool GetNavigationFeedback(uint32_t command_id, int* out_status);
  bool GetNavigationFeedbackSnapshot(uint32_t command_id, NavigationFeedbackSnapshot* out_feedback);
  bool DiscoverSingleDockId(int* out_dock_id);
  bool GetDockingStatus(int* out_status);
  bool CanDock(int dock_id, bool* out_can_dock);
  // Looks up the dock's AprilTag (tag id == dock id) in the world-object list.
  // Returns false only if the query itself failed; a tag that is not listed is
  // reported as found=false.
  bool GetDockFiducialSighting(int dock_id, DockFiducialSighting* out_sighting);
  // should_cancel, when given, is checked with the dock cancel flag on every
  // attempt and feedback poll (a spot.graphnav.cancel for this dock).
  bool AutoDock(int dock_id, int attempts, int poll_ms, int command_timeout_sec,
                const std::function<bool()>& should_cancel = nullptr);
  bool Undock(int poll_ms, int command_timeout_sec);
  void RequestDockCancel();
  std::string LastError() const;
  // True when the last Connect() was refused because a map processing RPC was
  // in flight (not a connection failure).
  bool LastConnectDeferred() const { return last_connect_deferred_.load(); }

 private:
  bool EnsureTimeSync();
  bool EnsureDockingClient();
  bool EnsurePowerClient();
  bool EnsureGraphNavClient();
  bool EnsureGraphNavRecordingClient();
  bool EnsureSpotCheckClient();
  bool EnsureMapProcessingClient();
  bool EnsureRobotStateClient();
  bool EnsureWorldObjectClient();
  bool EnsurePowered();
  bool DetectArmPresenceLocked(bool* out_has_arm);
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
  ::bosdyn::client::MapProcessingClient* map_processing_client_{nullptr};
  ::bosdyn::client::DockingClient* docking_client_{nullptr};
  ::bosdyn::client::RobotStateClient* robot_state_client_{nullptr};
  ::bosdyn::client::WorldObjectClient* world_object_client_{nullptr};
  ::bosdyn::api::Lease body_lease_;
  std::atomic<bool> cancel_docking_{false};
  std::atomic<int> arm_present_override_{-1};
  std::atomic<int> arm_presence_state_{-1};
  mutable std::recursive_mutex api_mu_;
  // ProcessMapTopology runs its long RPC outside api_mu_ so E-stop, velocity
  // and lease calls never wait on it. This mutex serializes map processing,
  // and the in-flight flag makes Connect refuse (instead of destroying the
  // robot and its clients) while the RPC is running.
  std::mutex map_processing_mu_;
  std::atomic<bool> map_processing_in_flight_{false};
  std::atomic<bool> last_connect_deferred_{false};
  mutable std::mutex err_mu_;
  std::string last_error_;
};

}  // namespace fsa
