#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "bosdyn/client/docking/docking_client.h"
#include "bosdyn/client/docking/docking_helpers.h"
#include "bosdyn/client/image/image_client.h"
#include "bosdyn/client/lease/lease_client.h"
#include "bosdyn/client/robot_command/robot_command_builder.h"
#include "bosdyn/client/robot_command/robot_command_client.h"
#include "bosdyn/client/robot_state/robot_state_client.h"
#include "bosdyn/client/sdk/client_sdk.h"
#include "bosdyn/client/time_sync/time_sync_helpers.h"
#include "bosdyn/client/world_objects/world_object_client.h"

namespace fsa {

enum class MotionMode {
  kWalk = 0,
  kStairs = 1,
  kCrawl = 2,
};

class SpotClient {
 public:
  struct ImageSourceInfo {
    std::string name;
    int cols{0};
    int rows{0};
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
  bool ResetArmToStow();
  bool MoveArmToPoseBody(double x, double y, double z, double qw, double qx, double qy, double qz,
                         double move_sec);
  bool GetArmJointPosition(const std::string& joint_name, double* out_position);
  bool Velocity(double vx, double vy, double wz, int end_after_ms = 600,
                MotionMode mode = MotionMode::kWalk, double body_pitch_rad = 0.0);
  bool ZeroVelocity(int repeats = 1);

  bool GetImageJpeg(const std::string& source, std::string* out_bytes);
  bool ListImageSources(std::vector<ImageSourceInfo>* out_sources);
  bool DiscoverSingleDockId(int* out_dock_id);
  bool CanDock(int dock_id, bool* out_can_dock);
  bool AutoDock(int dock_id, int attempts, int poll_ms, int command_timeout_sec);
  void RequestDockCancel();
  std::string LastError() const;

 private:
  bool EnsureTimeSync();
  bool EnsureDockingClient();
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
