#include "formant_spot_adapter/spot_client.hpp"

#include <chrono>
#include <cmath>

#include "bosdyn/client/docking/docking_helpers.h"
#include "bosdyn/client/image/image_client.h"
#include "bosdyn/client/power/power_client_helper.h"
#include "bosdyn/api/world_object.pb.h"

namespace fsa {

void SpotClient::SetLastError(const std::string& msg) {
  std::lock_guard<std::mutex> lk(err_mu_);
  last_error_ = msg;
}

std::string SpotClient::LastError() const {
  std::lock_guard<std::mutex> lk(err_mu_);
  return last_error_;
}

bool SpotClient::Connect(const std::string& host, const std::string& username, const std::string& password) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  sdk_ = ::bosdyn::client::CreateStandardSDK("formant-spot-adapter");

  auto robot_result = sdk_->CreateRobot(host);
  if (!robot_result) {
    SetLastError(robot_result.status.DebugString());
    return false;
  }
  robot_ = robot_result.move();

  auto auth_status = robot_->Authenticate(username, password);
  if (!auth_status) {
    SetLastError(auth_status.DebugString());
    return false;
  }

  auto cmd_client = robot_->EnsureServiceClient<::bosdyn::client::RobotCommandClient>(
      ::bosdyn::client::RobotCommandClient::GetDefaultServiceName());
  if (!cmd_client.status) {
    SetLastError(cmd_client.status.DebugString());
    return false;
  }
  robot_command_client_ = cmd_client.response;

  auto lease_client = robot_->EnsureServiceClient<::bosdyn::client::LeaseClient>(
      ::bosdyn::client::LeaseClient::GetDefaultServiceName());
  if (!lease_client.status) {
    SetLastError(lease_client.status.DebugString());
    return false;
  }
  lease_client_ = lease_client.response;

  auto image_client = robot_->EnsureServiceClient<::bosdyn::client::ImageClient>(
      ::bosdyn::client::ImageClient::GetDefaultServiceName());
  if (!image_client.status) {
    SetLastError(image_client.status.DebugString());
    return false;
  }
  image_client_ = image_client.response;

  return true;
}

bool SpotClient::AcquireBodyLease() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!lease_client_) return false;
  auto r = lease_client_->AcquireLease("body");
  if (r.status) {
    body_lease_ = r.response.lease();
    return true;
  }

  // Common after crashes/restarts: previous owner still holds body lease.
  // During active teleop we prefer explicit takeover to recover service.
  auto t = lease_client_->TakeLease("body");
  if (!t.status) {
    SetLastError("Acquire failed: " + r.status.DebugString() +
                 " ; Take failed: " + t.status.DebugString());
    return false;
  }
  body_lease_ = t.response.lease();
  return true;
}

bool SpotClient::RetainLease() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!lease_client_) return false;
  if (body_lease_.resource().empty()) return false;
  ::bosdyn::api::RetainLeaseRequest req;
  req.mutable_lease()->CopyFrom(body_lease_);
  auto r = lease_client_->RetainLease(req);
  return r.status;
}

bool SpotClient::ReturnBodyLease() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!lease_client_) return false;
  if (body_lease_.resource().empty()) return false;
  ::bosdyn::api::ReturnLeaseRequest req;
  req.mutable_lease()->CopyFrom(body_lease_);
  auto r = lease_client_->ReturnLease(req);
  if (!r.status) SetLastError(r.status.DebugString());
  body_lease_.Clear();
  return r.status;
}

bool SpotClient::Stand() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!robot_command_client_) return false;
  if (!EnsureTimeSync()) return false;
  if (!EnsurePowered()) return false;
  auto cmd = ::bosdyn::client::StandCommand();
  auto r = robot_command_client_->RobotCommand(cmd);
  if (!r.status) SetLastError(r.status.DebugString());
  return r.status;
}

bool SpotClient::Sit() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!robot_command_client_) return false;
  if (!EnsureTimeSync()) return false;
  if (!EnsurePowered()) return false;
  auto cmd = ::bosdyn::client::SitCommand();
  auto r = robot_command_client_->RobotCommand(cmd);
  if (!r.status) SetLastError(r.status.DebugString());
  return r.status;
}

bool SpotClient::EmergencyStop() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!robot_command_client_) return false;
  if (!EnsureTimeSync()) return false;
  auto cmd = ::bosdyn::client::SafePowerOffCommand();
  auto r = robot_command_client_->RobotCommand(cmd);
  if (!r.status) SetLastError(r.status.DebugString());
  return r.status;
}

bool SpotClient::RecoverSelfRight() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!robot_command_client_) return false;
  if (!EnsureTimeSync()) return false;
  if (!EnsurePowered()) return false;
  auto cmd = ::bosdyn::client::SelfrightCommand();
  auto r = robot_command_client_->RobotCommand(cmd);
  if (!r.status) SetLastError(r.status.DebugString());
  return r.status;
}

bool SpotClient::ResetArmToStow() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!robot_command_client_) return false;
  if (!EnsureTimeSync()) return false;
  if (!EnsurePowered()) return false;
  auto cmd = ::bosdyn::client::ArmStowCommand();
  auto r = robot_command_client_->RobotCommand(cmd);
  if (!r.status) SetLastError(r.status.DebugString());
  return r.status;
}

bool SpotClient::MoveArmToPoseBody(double x, double y, double z, double qw, double qx, double qy,
                                   double qz, double move_sec) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!robot_command_client_) return false;
  if (!EnsureTimeSync()) return false;
  if (!EnsurePowered()) return false;
  auto cmd = ::bosdyn::client::ArmPoseCommand(
      x, y, z, qw, qx, qy, qz, ::bosdyn::api::kBodyFrame, std::max(0.5, move_sec));
  auto r = robot_command_client_->RobotCommand(cmd);
  return r.status;
}

bool SpotClient::GetArmJointPosition(const std::string& joint_name, double* out_position) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!out_position) return false;
  if (!EnsureRobotStateClient()) return false;
  auto state = robot_state_client_->GetRobotState();
  if (!state.status) {
    SetLastError(state.status.DebugString());
    return false;
  }
  const auto& joints = state.response.robot_state().kinematic_state().joint_states();
  for (const auto& j : joints) {
    if (j.name() == joint_name) {
      *out_position = j.position().value();
      return true;
    }
  }
  return false;
}

bool SpotClient::Velocity(double vx, double vy, double wz, int end_after_ms, MotionMode mode,
                          double body_pitch_rad) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!robot_command_client_) return false;
  if (!EnsureTimeSync()) return false;
  if (!EnsurePowered()) return false;
  ::bosdyn::api::spot::MobilityParams params;
  switch (mode) {
    case MotionMode::kStairs:
      params.set_locomotion_hint(::bosdyn::api::spot::LocomotionHint::HINT_TROT);
      params.set_stairs_mode(::bosdyn::api::spot::MobilityParams::STAIRS_MODE_ON);
      break;
    case MotionMode::kCrawl:
      params.set_locomotion_hint(::bosdyn::api::spot::LocomotionHint::HINT_CRAWL);
      params.set_stairs_mode(::bosdyn::api::spot::MobilityParams::STAIRS_MODE_OFF);
      break;
    case MotionMode::kWalk:
    default:
      params.set_locomotion_hint(::bosdyn::api::spot::LocomotionHint::HINT_AUTO);
      params.set_stairs_mode(::bosdyn::api::spot::MobilityParams::STAIRS_MODE_OFF);
      break;
  }
  // Body pitch offset in footprint frame. Keep other offsets/orientations neutral.
  auto* body_ctrl = params.mutable_body_control();
  auto* base_traj = body_ctrl->mutable_base_offset_rt_footprint();
  auto* point = base_traj->add_points();
  auto* pose = point->mutable_pose();
  pose->mutable_position()->set_x(0.0);
  pose->mutable_position()->set_y(0.0);
  pose->mutable_position()->set_z(0.0);
  const double half_pitch = 0.5 * body_pitch_rad;
  pose->mutable_rotation()->set_w(std::cos(half_pitch));
  pose->mutable_rotation()->set_x(0.0);
  pose->mutable_rotation()->set_y(std::sin(half_pitch));
  pose->mutable_rotation()->set_z(0.0);

  auto cmd = ::bosdyn::client::VelocityCommand(vx, vy, wz, "body", params);
  auto r = robot_command_client_->RobotCommand(
      cmd, nullptr, nullptr,
      std::chrono::system_clock::now() + std::chrono::milliseconds(end_after_ms));
  if (!r.status) SetLastError(r.status.DebugString());
  return r.status;
}

bool SpotClient::ZeroVelocity(int repeats) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  bool ok = true;
  for (int i = 0; i < repeats; ++i) {
    ok = Velocity(0.0, 0.0, 0.0) && ok;
  }
  return ok;
}

bool SpotClient::GetImageJpeg(const std::string& source, std::string* out_bytes) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!image_client_ || !out_bytes) return false;
  auto r = image_client_->GetImage({source});
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }
  if (r.response.image_responses_size() <= 0) return false;

  *out_bytes = r.response.image_responses(0).shot().image().data();
  return !out_bytes->empty();
}

bool SpotClient::ListImageSources(std::vector<ImageSourceInfo>* out_sources) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!out_sources) return false;
  out_sources->clear();
  if (!image_client_) return false;
  auto r = image_client_->ListImageSources();
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }
  out_sources->reserve(static_cast<size_t>(r.response.image_sources_size()));
  for (const auto& source : r.response.image_sources()) {
    ImageSourceInfo info;
    info.name = source.name();
    info.cols = source.cols();
    info.rows = source.rows();
    out_sources->push_back(std::move(info));
  }
  return true;
}

bool SpotClient::DiscoverSingleDockId(int* out_dock_id) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!out_dock_id) return false;
  if (!EnsureDockingClient()) return false;

  auto cfg = docking_client_->GetDockingConfig();
  if (!cfg.status) {
    SetLastError(cfg.status.DebugString());
    return false;
  }

  int candidate = -1;
  long long count = 0;
  for (const auto& r : cfg.response.dock_configs()) {
    const long long start = r.id_start();
    const long long end = r.id_end();
    if (end < start) continue;
    const long long span = end - start + 1;
    if (count == 0) candidate = static_cast<int>(start);
    count += span;
    if (count > 1) break;
  }

  if (count == 1 && candidate > 0) {
    *out_dock_id = candidate;
    return true;
  }

  // Fallback: look for visible dock world-objects (similar intent as legacy integration).
  if (!EnsureWorldObjectClient()) {
    SetLastError("Dock config ambiguous and world-object client unavailable: " + LastError());
    return false;
  }

  ::bosdyn::api::ListWorldObjectRequest req;
  req.add_object_type(::bosdyn::api::WORLD_OBJECT_DOCK);
  auto wo = world_object_client_->ListWorldObjects(req);
  if (!wo.status) {
    SetLastError("Dock config ambiguous and world-object query failed: " + wo.status.DebugString());
    return false;
  }

  int found = -1;
  int found_count = 0;
  for (const auto& obj : wo.response.world_objects()) {
    if (!obj.has_dock_properties()) continue;
    const int dock_id = static_cast<int>(obj.dock_properties().dock_id());
    if (dock_id <= 0) continue;
    if (found_count == 0 || dock_id != found) {
      found = dock_id;
      ++found_count;
    }
    if (found_count > 1) break;
  }

  if (found_count == 1 && found > 0) {
    *out_dock_id = found;
    return true;
  }

  SetLastError("No unique dock id resolved (config ranges and visible docks are ambiguous/empty)");
  return false;
}

bool SpotClient::CanDock(int dock_id, bool* out_can_dock) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!out_can_dock) return false;
  *out_can_dock = false;
  if (dock_id <= 0) return false;
  if (!EnsureDockingClient()) return false;

  auto state = docking_client_->GetDockingState();
  if (!state.status) {
    SetLastError(state.status.DebugString());
    return false;
  }

  const auto s = state.response.dock_state().status();
  *out_can_dock = (s == ::bosdyn::api::docking::DockState_DockedStatus_DOCK_STATUS_UNDOCKED);
  return true;
}

bool SpotClient::AutoDock(int dock_id, int attempts, int poll_ms, int command_timeout_sec) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (dock_id <= 0) return false;
  if (!robot_) return false;
  if (!EnsureDockingClient()) return false;
  if (!EnsureTimeSync()) return false;
  if (!EnsurePowered()) return false;

  cancel_docking_ = false;
  auto result = ::bosdyn::client::BlockingDock(
      robot_.get(), static_cast<unsigned int>(dock_id), std::max(1, attempts),
      std::chrono::milliseconds(std::max(100, poll_ms)),
      std::chrono::seconds(std::max(5, command_timeout_sec)),
      [this]() { return cancel_docking_.load(); },
      nullptr);
  if (!result.status) SetLastError(result.status.DebugString());
  return result.status;
}

void SpotClient::RequestDockCancel() { cancel_docking_ = true; }

bool SpotClient::EnsureTimeSync() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (time_sync_endpoint_) return true;
  if (!robot_) return false;
  auto ts = robot_->StartTimeSyncAndGetEndpoint();
  if (!ts.status) {
    SetLastError(ts.status.DebugString());
    return false;
  }
  time_sync_endpoint_ = ts.move();
  if (robot_command_client_) robot_command_client_->AddTimeSyncEndpoint(time_sync_endpoint_);
  if (docking_client_) docking_client_->AddTimeSyncEndpoint(time_sync_endpoint_);
  return true;
}

bool SpotClient::EnsureDockingClient() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (docking_client_) return true;
  if (!robot_) return false;
  auto dock_client = robot_->EnsureServiceClient<::bosdyn::client::DockingClient>(
      ::bosdyn::client::DockingClient::GetDefaultServiceName());
  if (!dock_client.status) {
    SetLastError(dock_client.status.DebugString());
    return false;
  }
  docking_client_ = dock_client.response;
  if (time_sync_endpoint_) docking_client_->AddTimeSyncEndpoint(time_sync_endpoint_);
  return true;
}

bool SpotClient::EnsureRobotStateClient() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (robot_state_client_) return true;
  if (!robot_) return false;
  auto state_client = robot_->EnsureServiceClient<::bosdyn::client::RobotStateClient>(
      ::bosdyn::client::RobotStateClient::GetDefaultServiceName());
  if (!state_client.status) {
    SetLastError(state_client.status.DebugString());
    return false;
  }
  robot_state_client_ = state_client.response;
  return true;
}

bool SpotClient::EnsureWorldObjectClient() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (world_object_client_) return true;
  if (!robot_) return false;
  auto wo_client = robot_->EnsureServiceClient<::bosdyn::client::WorldObjectClient>(
      ::bosdyn::client::WorldObjectClient::GetDefaultServiceName());
  if (!wo_client.status) {
    SetLastError(wo_client.status.DebugString());
    return false;
  }
  world_object_client_ = wo_client.response;
  return true;
}

bool SpotClient::EnsurePowered() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!robot_) return false;
  auto state = robot_->IsPoweredOn();
  if (!state.status) {
    SetLastError(state.status.DebugString());
    return false;
  }
  if (state.response) return true;

  auto power = robot_->PowerOnMotors(std::chrono::seconds(20), 1.0);
  if (!power) SetLastError(power.DebugString());
  return power;
}

}  // namespace fsa
