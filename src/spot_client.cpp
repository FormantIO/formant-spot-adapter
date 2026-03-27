#include "formant_spot_adapter/spot_client.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "bosdyn/client/docking/docking_helpers.h"
#include "bosdyn/client/graph_nav/graph_nav_client.h"
#include "bosdyn/client/graph_nav/recording_client.h"
#include "bosdyn/client/image/image_client.h"
#include "bosdyn/client/power/power_client_helper.h"
#include "bosdyn/api/world_object.pb.h"
#include "bosdyn/math/frame_helpers.h"
#include "bosdyn/math/proto_math.h"

namespace fsa {

namespace {

std::string motor_power_state_to_string(::bosdyn::api::PowerState::MotorPowerState state) {
  switch (state) {
    case ::bosdyn::api::PowerState::MOTOR_POWER_STATE_OFF:
      return "off";
    case ::bosdyn::api::PowerState::MOTOR_POWER_STATE_ON:
      return "on";
    case ::bosdyn::api::PowerState::MOTOR_POWER_STATE_POWERING_ON:
      return "powering_on";
    case ::bosdyn::api::PowerState::MOTOR_POWER_STATE_POWERING_OFF:
      return "powering_off";
    case ::bosdyn::api::PowerState::MOTOR_POWER_STATE_ERROR:
      return "error";
    case ::bosdyn::api::PowerState::MOTOR_POWER_STATE_UNKNOWN:
    default:
      return "unknown";
  }
}

std::string shore_power_state_to_string(::bosdyn::api::PowerState::ShorePowerState state) {
  switch (state) {
    case ::bosdyn::api::PowerState::SHORE_POWER_STATE_ON:
      return "on";
    case ::bosdyn::api::PowerState::SHORE_POWER_STATE_OFF:
      return "off";
    case ::bosdyn::api::PowerState::SHORE_POWER_STATE_UNKNOWN:
    default:
      return "unknown";
  }
}

std::string behavior_state_to_string(::bosdyn::api::BehaviorState::State state) {
  switch (state) {
    case ::bosdyn::api::BehaviorState::STATE_NOT_READY:
      return "not_ready";
    case ::bosdyn::api::BehaviorState::STATE_TRANSITION:
      return "transition";
    case ::bosdyn::api::BehaviorState::STATE_STANDING:
      return "standing";
    case ::bosdyn::api::BehaviorState::STATE_STEPPING:
      return "stepping";
    case ::bosdyn::api::BehaviorState::STATE_UNKNOWN:
    default:
      return "unknown";
  }
}

std::string system_severity_to_string(::bosdyn::api::SystemFault::Severity severity) {
  switch (severity) {
    case ::bosdyn::api::SystemFault::SEVERITY_INFO:
      return "info";
    case ::bosdyn::api::SystemFault::SEVERITY_WARN:
      return "warn";
    case ::bosdyn::api::SystemFault::SEVERITY_CRITICAL:
      return "critical";
    case ::bosdyn::api::SystemFault::SEVERITY_UNKNOWN:
    default:
      return "unknown";
  }
}

std::string service_severity_to_string(::bosdyn::api::ServiceFault::Severity severity) {
  switch (severity) {
    case ::bosdyn::api::ServiceFault::SEVERITY_INFO:
      return "info";
    case ::bosdyn::api::ServiceFault::SEVERITY_WARN:
      return "warn";
    case ::bosdyn::api::ServiceFault::SEVERITY_CRITICAL:
      return "critical";
    case ::bosdyn::api::ServiceFault::SEVERITY_UNKNOWN:
    default:
      return "unknown";
  }
}

std::string behavior_cause_to_string(::bosdyn::api::BehaviorFault::Cause cause) {
  switch (cause) {
    case ::bosdyn::api::BehaviorFault::CAUSE_FALL:
      return "fall";
    case ::bosdyn::api::BehaviorFault::CAUSE_HARDWARE:
      return "hardware";
    case ::bosdyn::api::BehaviorFault::CAUSE_LEASE_TIMEOUT:
      return "lease_timeout";
    case ::bosdyn::api::BehaviorFault::CAUSE_UNKNOWN:
    default:
      return "unknown";
  }
}

std::string behavior_status_to_string(::bosdyn::api::BehaviorFault::Status status) {
  switch (status) {
    case ::bosdyn::api::BehaviorFault::STATUS_CLEARABLE:
      return "clearable";
    case ::bosdyn::api::BehaviorFault::STATUS_UNCLEARABLE:
      return "unclearable";
    case ::bosdyn::api::BehaviorFault::STATUS_UNKNOWN:
    default:
      return "unknown";
  }
}

std::string lowercase_ascii(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

void fill_pose3d(const ::bosdyn::api::SE3Pose& pose, SpotClient::Pose3D* out_pose) {
  if (!out_pose) return;
  out_pose->x = pose.position().x();
  out_pose->y = pose.position().y();
  out_pose->z = pose.position().z();
  out_pose->qx = pose.rotation().x();
  out_pose->qy = pose.rotation().y();
  out_pose->qz = pose.rotation().z();
  out_pose->qw = pose.rotation().w();
}

int local_grid_cell_width_bytes(::bosdyn::api::LocalGrid::CellFormat format) {
  switch (format) {
    case ::bosdyn::api::LocalGrid::CELL_FORMAT_FLOAT32:
      return 4;
    case ::bosdyn::api::LocalGrid::CELL_FORMAT_FLOAT64:
      return 8;
    case ::bosdyn::api::LocalGrid::CELL_FORMAT_INT8:
    case ::bosdyn::api::LocalGrid::CELL_FORMAT_UINT8:
      return 1;
    case ::bosdyn::api::LocalGrid::CELL_FORMAT_INT16:
    case ::bosdyn::api::LocalGrid::CELL_FORMAT_UINT16:
      return 2;
    case ::bosdyn::api::LocalGrid::CELL_FORMAT_UNKNOWN:
    default:
      return 0;
  }
}

template <typename T>
double read_local_grid_scalar(const std::string& bytes, size_t offset) {
  T value{};
  std::memcpy(&value, bytes.data() + offset, sizeof(T));
  return static_cast<double>(value);
}

bool decode_local_grid_cells(const ::bosdyn::api::LocalGrid& grid,
                             std::vector<double>* out_values,
                             std::vector<uint8_t>* out_unknown_mask,
                             std::string* out_error) {
  if (!out_values || !out_unknown_mask) return false;
  const int width = grid.extent().num_cells_x();
  const int height = grid.extent().num_cells_y();
  if (width <= 0 || height <= 0) {
    if (out_error) *out_error = "local grid extent is empty";
    return false;
  }
  const size_t cell_count = static_cast<size_t>(width) * static_cast<size_t>(height);
  const int cell_width = local_grid_cell_width_bytes(grid.cell_format());
  if (cell_width <= 0) {
    if (out_error) *out_error = "unsupported local grid cell format";
    return false;
  }
  if ((grid.data().size() % cell_width) != 0) {
    if (out_error) *out_error = "local grid byte payload is not aligned to cell width";
    return false;
  }

  auto decode_scalar = [&grid](const std::string& bytes, size_t offset) -> double {
    switch (grid.cell_format()) {
      case ::bosdyn::api::LocalGrid::CELL_FORMAT_FLOAT32:
        return read_local_grid_scalar<float>(bytes, offset);
      case ::bosdyn::api::LocalGrid::CELL_FORMAT_FLOAT64:
        return read_local_grid_scalar<double>(bytes, offset);
      case ::bosdyn::api::LocalGrid::CELL_FORMAT_INT8:
        return read_local_grid_scalar<int8_t>(bytes, offset);
      case ::bosdyn::api::LocalGrid::CELL_FORMAT_UINT8:
        return read_local_grid_scalar<uint8_t>(bytes, offset);
      case ::bosdyn::api::LocalGrid::CELL_FORMAT_INT16:
        return read_local_grid_scalar<int16_t>(bytes, offset);
      case ::bosdyn::api::LocalGrid::CELL_FORMAT_UINT16:
        return read_local_grid_scalar<uint16_t>(bytes, offset);
      case ::bosdyn::api::LocalGrid::CELL_FORMAT_UNKNOWN:
      default:
        return 0.0;
    }
  };

  std::vector<double> base_values;
  base_values.reserve(grid.data().size() / static_cast<size_t>(cell_width));
  for (size_t offset = 0; offset < grid.data().size(); offset += static_cast<size_t>(cell_width)) {
    base_values.push_back(decode_scalar(grid.data(), offset));
  }

  const double scale = (grid.cell_value_scale() == 0.0) ? 1.0 : grid.cell_value_scale();
  const double offset = grid.cell_value_offset();
  auto apply_scale_and_offset = [scale, offset](double raw_value) {
    return raw_value * scale + offset;
  };

  std::vector<double> decoded_values;
  decoded_values.reserve(cell_count);
  if (grid.encoding() == ::bosdyn::api::LocalGrid::ENCODING_RAW) {
    if (base_values.size() < cell_count) {
      if (out_error) *out_error = "local grid payload is shorter than declared extent";
      return false;
    }
    for (size_t i = 0; i < cell_count; ++i) {
      decoded_values.push_back(apply_scale_and_offset(base_values[i]));
    }
  } else if (grid.encoding() == ::bosdyn::api::LocalGrid::ENCODING_RLE) {
    if (base_values.size() != static_cast<size_t>(grid.rle_counts_size())) {
      if (out_error) *out_error = "local grid RLE counts do not match encoded value count";
      return false;
    }
    for (size_t i = 0; i < base_values.size(); ++i) {
      const int repeat = grid.rle_counts(static_cast<int>(i));
      if (repeat < 0) {
        if (out_error) *out_error = "local grid RLE count is negative";
        return false;
      }
      for (int j = 0; j < repeat; ++j) {
        decoded_values.push_back(apply_scale_and_offset(base_values[i]));
      }
    }
    if (decoded_values.size() != cell_count) {
      if (out_error) *out_error = "local grid RLE expansion did not match declared extent";
      return false;
    }
  } else {
    if (out_error) *out_error = "unsupported local grid encoding";
    return false;
  }

  std::vector<uint8_t> unknown_mask(cell_count, 0);
  if (!grid.unknown_cells().empty()) {
    if (grid.unknown_cells().size() < static_cast<int>(cell_count)) {
      if (out_error) *out_error = "local grid unknown mask is shorter than declared extent";
      return false;
    }
    for (size_t i = 0; i < cell_count; ++i) {
      unknown_mask[i] = static_cast<uint8_t>(grid.unknown_cells()[static_cast<int>(i)] != 0);
    }
  }

  *out_values = std::move(decoded_values);
  *out_unknown_mask = std::move(unknown_mask);
  return true;
}

bool local_grid_has_matching_layout(const ::bosdyn::api::LocalGrid& a,
                                    const ::bosdyn::api::LocalGrid& b) {
  return a.extent().num_cells_x() == b.extent().num_cells_x() &&
         a.extent().num_cells_y() == b.extent().num_cells_y() &&
         std::fabs(a.extent().cell_size() - b.extent().cell_size()) < 1e-9;
}

::bosdyn::api::Vec3 make_vec3(double x, double y, double z) {
  ::bosdyn::api::Vec3 out;
  out.set_x(x);
  out.set_y(y);
  out.set_z(z);
  return out;
}

struct DecodedLocalGrid {
  std::string map_type;
  double resolution_m{0.0};
  int width{0};
  int height{0};
  ::bosdyn::api::FrameTreeSnapshot transforms_snapshot;
  ::bosdyn::api::SE3Pose body_tform_grid;
  std::vector<int32_t> occupancy;
};

bool build_decoded_local_grid(
    const google::protobuf::RepeatedPtrField<::bosdyn::api::LocalGrid>& local_grids,
    DecodedLocalGrid* out_grid,
    std::string* out_error) {
  if (!out_grid) return false;

  const ::bosdyn::api::LocalGrid* no_step_grid = nullptr;
  const ::bosdyn::api::LocalGrid* obstacle_distance_grid = nullptr;
  const ::bosdyn::api::LocalGrid* terrain_valid_grid = nullptr;
  const ::bosdyn::api::LocalGrid* terrain_grid = nullptr;
  for (const auto& grid : local_grids) {
    const std::string label = lowercase_ascii(grid.local_grid_type_name() + " " +
                                              grid.frame_name_local_grid_data());
    if (!no_step_grid && label.find("no_step") != std::string::npos) {
      no_step_grid = &grid;
    } else if (!obstacle_distance_grid &&
               label.find("obstacle_distance") != std::string::npos) {
      obstacle_distance_grid = &grid;
    } else if (!terrain_valid_grid &&
               label.find("terrain_valid") != std::string::npos) {
      terrain_valid_grid = &grid;
    } else if (!terrain_grid && label.find("terrain") != std::string::npos &&
               label.find("terrain_valid") == std::string::npos &&
               label.find("terrain_intensity") == std::string::npos) {
      terrain_grid = &grid;
    }
  }

  const ::bosdyn::api::LocalGrid* primary_grid = no_step_grid;
  if (!primary_grid) primary_grid = obstacle_distance_grid;
  if (!primary_grid) primary_grid = terrain_valid_grid;
  if (!primary_grid) primary_grid = terrain_grid;
  if (!primary_grid) {
    if (out_error) *out_error = "no supported local grid was returned";
    return false;
  }

  std::vector<double> primary_values;
  std::vector<uint8_t> primary_unknown;
  if (!decode_local_grid_cells(*primary_grid, &primary_values, &primary_unknown, out_error)) {
    return false;
  }

  std::vector<double> valid_values;
  std::vector<uint8_t> valid_unknown;
  const bool have_valid_grid = terrain_valid_grid != nullptr &&
                               local_grid_has_matching_layout(*primary_grid, *terrain_valid_grid) &&
                               decode_local_grid_cells(*terrain_valid_grid, &valid_values,
                                                       &valid_unknown, out_error);

  const size_t cell_count = primary_values.size();
  std::vector<int32_t> occupancy(cell_count, -1);
  const bool use_no_step = primary_grid == no_step_grid;
  const bool use_obstacle_distance = primary_grid == obstacle_distance_grid;
  for (size_t i = 0; i < cell_count; ++i) {
    bool unknown = primary_unknown[i] != 0;
    if (have_valid_grid && !unknown) {
      if (valid_unknown[i] != 0 || valid_values[i] <= 0.0) unknown = true;
    }
    if (unknown) {
      occupancy[i] = -1;
      continue;
    }
    if (use_no_step) {
      occupancy[i] = primary_values[i] > 0.0 ? 100 : 0;
    } else if (use_obstacle_distance) {
      occupancy[i] = primary_values[i] <= 0.0 ? 100 : 0;
    } else {
      occupancy[i] = 0;
    }
  }

  ::bosdyn::api::SE3Pose body_tform_grid;
  if (!::bosdyn::api::get_a_tform_b(primary_grid->transforms_snapshot(),
                                    ::bosdyn::api::kBodyFrame,
                                    primary_grid->frame_name_local_grid_data(),
                                    &body_tform_grid)) {
    if (out_error) {
      *out_error = "failed to resolve body->local_grid transform for frame '" +
                   primary_grid->frame_name_local_grid_data() + "'";
    }
    return false;
  }

  DecodedLocalGrid decoded;
  decoded.map_type = primary_grid->local_grid_type_name().empty()
                         ? primary_grid->frame_name_local_grid_data()
                         : primary_grid->local_grid_type_name();
  decoded.resolution_m = primary_grid->extent().cell_size();
  decoded.width = primary_grid->extent().num_cells_x();
  decoded.height = primary_grid->extent().num_cells_y();
  decoded.transforms_snapshot = primary_grid->transforms_snapshot();
  decoded.body_tform_grid = body_tform_grid;
  decoded.occupancy = std::move(occupancy);
  *out_grid = std::move(decoded);
  return true;
}

bool build_occupancy_grid_from_live_data(
    const ::bosdyn::api::graph_nav::Localization& localization,
    const google::protobuf::RepeatedPtrField<::bosdyn::api::LocalGrid>& local_grids,
    SpotClient::OccupancyGridMapSnapshot* out_map,
    std::string* out_error) {
  if (!out_map) return false;
  if (!localization.has_seed_tform_body()) {
    if (out_error) *out_error = "localization missing seed_tform_body";
    return false;
  }

  DecodedLocalGrid decoded;
  if (!build_decoded_local_grid(local_grids, &decoded, out_error)) return false;

  SpotClient::OccupancyGridMapSnapshot map_snapshot;
  map_snapshot.map_type = decoded.map_type;
  map_snapshot.resolution_m = decoded.resolution_m;
  map_snapshot.width = decoded.width;
  map_snapshot.height = decoded.height;
  fill_pose3d(localization.seed_tform_body() * decoded.body_tform_grid,
              &map_snapshot.seed_tform_grid);
  map_snapshot.occupancy = std::move(decoded.occupancy);
  *out_map = std::move(map_snapshot);
  return true;
}

struct AccumulatedOccupancyCell {
  double known_weight{0.0};
  double occupied_weight{0.0};
};

void accumulate_occupancy_sample(double seed_x,
                                 double seed_y,
                                 double min_x,
                                 double min_y,
                                 double resolution_m,
                                 int width,
                                 int height,
                                 bool occupied,
                                 std::vector<AccumulatedOccupancyCell>* accum) {
  if (!accum || resolution_m <= 0.0 || width <= 0 || height <= 0) return;

  const double grid_x = ((seed_x - min_x) / resolution_m) - 0.5;
  const double grid_y = ((seed_y - min_y) / resolution_m) - 0.5;
  const int x0 = static_cast<int>(std::floor(grid_x));
  const int y0 = static_cast<int>(std::floor(grid_y));
  const double tx = grid_x - static_cast<double>(x0);
  const double ty = grid_y - static_cast<double>(y0);

  const auto apply = [&](int cell_x, int cell_y, double weight) {
    if (weight <= 0.0 || cell_x < 0 || cell_y < 0 || cell_x >= width || cell_y >= height) return;
    auto& cell = (*accum)[static_cast<size_t>(cell_x) +
                          static_cast<size_t>(width) * static_cast<size_t>(cell_y)];
    cell.known_weight += weight;
    if (occupied) cell.occupied_weight += weight;
  };

  apply(x0, y0, (1.0 - tx) * (1.0 - ty));
  apply(x0 + 1, y0, tx * (1.0 - ty));
  apply(x0, y0 + 1, (1.0 - tx) * ty);
  apply(x0 + 1, y0 + 1, tx * ty);
}

}  // namespace

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
  body_lease_.Clear();
  robot_command_client_ = nullptr;
  lease_client_ = nullptr;
  time_sync_endpoint_ = nullptr;
  image_client_ = nullptr;
  power_client_ = nullptr;
  graph_nav_client_ = nullptr;
  graph_nav_recording_client_ = nullptr;
  spot_check_client_ = nullptr;
  docking_client_ = nullptr;
  robot_state_client_ = nullptr;
  world_object_client_ = nullptr;
  robot_.reset();
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

bool SpotClient::TryAcquireBodyLeaseNoTakeover() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!lease_client_) return false;
  auto r = lease_client_->AcquireLease("body");
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }
  body_lease_ = r.response.lease();
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

bool SpotClient::RebootRobotBody() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!EnsureTimeSync()) return false;
  if (!EnsurePowerClient()) return false;
  if (!EnsureRobotStateClient()) return false;
  if (!lease_client_) return false;
  if (!robot_command_client_) return false;

  const bool had_lease = !body_lease_.resource().empty();
  if (!had_lease) {
    auto acquire = lease_client_->AcquireLease("body");
    if (acquire.status) {
      body_lease_ = acquire.response.lease();
    } else {
      auto take = lease_client_->TakeLease("body");
      if (!take.status) {
        SetLastError("Reboot requires body lease; acquire failed: " + acquire.status.DebugString() +
                     " ; take failed: " + take.status.DebugString());
        return false;
      }
      body_lease_ = take.response.lease();
    }
  }

  const auto status = ::bosdyn::client::power_client_helper::SafePowerCycleRobot(
      robot_command_client_, robot_state_client_, power_client_, std::chrono::seconds(30), 1.0);
  if (!status) {
    std::string reboot_err = status.DebugString();
    const bool no_timesync = reboot_err.find("STATUS_NO_TIMESYNC") != std::string::npos;
    if (no_timesync) {
      const auto fallback = ::bosdyn::client::power_client_helper::PowerCycleRobot(
          power_client_, std::chrono::seconds(30), 1.0);
      if (fallback) {
        if (!had_lease && !body_lease_.resource().empty()) {
          ::bosdyn::api::ReturnLeaseRequest req;
          req.mutable_lease()->CopyFrom(body_lease_);
          const auto ret = lease_client_->ReturnLease(req);
          (void)ret;
          body_lease_.Clear();
        }
        return true;
      }
      reboot_err += " ; fallback PowerCycleRobot failed: " + fallback.DebugString();
    }
    if (!had_lease && !body_lease_.resource().empty()) {
      ::bosdyn::api::ReturnLeaseRequest req;
      req.mutable_lease()->CopyFrom(body_lease_);
      const auto ret = lease_client_->ReturnLease(req);
      if (!ret.status) {
        reboot_err += " ; return temporary lease failed: " + ret.status.DebugString();
      }
      body_lease_.Clear();
    }
    SetLastError(reboot_err);
    return false;
  }
  if (!had_lease && !body_lease_.resource().empty()) {
    ::bosdyn::api::ReturnLeaseRequest req;
    req.mutable_lease()->CopyFrom(body_lease_);
    const auto ret = lease_client_->ReturnLease(req);
    if (!ret.status) {
      SetLastError("Reboot succeeded but return temporary lease failed: " + ret.status.DebugString());
      body_lease_.Clear();
      return false;
    }
    body_lease_.Clear();
  }
  return true;
}

bool SpotClient::StartCameraCalibration() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!EnsureSpotCheckClient()) return false;
  if (!EnsureTimeSync()) return false;
  if (body_lease_.resource().empty()) {
    SetLastError("Camera calibration requires an active body lease.");
    return false;
  }

  ::bosdyn::api::spot::CameraCalibrationCommandRequest req;
  req.set_command(::bosdyn::api::spot::CameraCalibrationCommandRequest::COMMAND_START);
  auto r = spot_check_client_->CameraCalibrationCommand(req);
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }
  return true;
}

bool SpotClient::CancelCameraCalibration() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!EnsureSpotCheckClient()) return false;
  if (!EnsureTimeSync()) return false;
  if (body_lease_.resource().empty()) {
    SetLastError("Camera calibration cancel requires an active body lease.");
    return false;
  }

  ::bosdyn::api::spot::CameraCalibrationCommandRequest req;
  req.set_command(::bosdyn::api::spot::CameraCalibrationCommandRequest::COMMAND_CANCEL);
  auto r = spot_check_client_->CameraCalibrationCommand(req);
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }
  return true;
}

bool SpotClient::GetCameraCalibrationFeedback(int* out_status, float* out_progress) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!out_status || !out_progress) return false;
  if (!EnsureSpotCheckClient()) return false;
  if (!EnsureTimeSync()) return false;

  auto r = spot_check_client_->CameraCalibrationFeedback();
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }

  *out_status = static_cast<int>(r.response.status());
  *out_progress = r.response.progress();
  return true;
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

bool SpotClient::GetRobotStateSnapshot(RobotStateSnapshot* out_snapshot) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!out_snapshot) return false;
  if (!EnsureRobotStateClient()) return false;
  auto state = robot_state_client_->GetRobotState();
  if (!state.status) {
    SetLastError(state.status.DebugString());
    return false;
  }
  const auto& robot_state = state.response.robot_state();

  RobotStateSnapshot snap;
  snap.motor_power_state = motor_power_state_to_string(robot_state.power_state().motor_power_state());
  snap.shore_power_state = shore_power_state_to_string(robot_state.power_state().shore_power_state());
  snap.behavior_state = behavior_state_to_string(robot_state.behavior_state().state());

  for (const auto& estop : robot_state.estop_states()) {
    if (estop.state() == ::bosdyn::api::EStopState::STATE_ESTOPPED) {
      snap.any_estopped = true;
      break;
    }
  }

  if (robot_state.power_state().has_locomotion_charge_percentage()) {
    snap.has_battery_pct = true;
    snap.battery_pct = robot_state.power_state().locomotion_charge_percentage().value();
  } else {
    for (const auto& battery : robot_state.battery_states()) {
      if (!battery.has_charge_percentage()) continue;
      snap.has_battery_pct = true;
      snap.battery_pct = battery.charge_percentage().value();
      break;
    }
  }

  for (const auto& fault : robot_state.system_fault_state().faults()) {
    FaultInfo item;
    item.id = !fault.name().empty() ? fault.name() : fault.uuid();
    item.severity = system_severity_to_string(fault.severity());
    item.message = fault.error_message();
    snap.system_faults.push_back(std::move(item));
  }

  for (const auto& fault : robot_state.behavior_fault_state().faults()) {
    FaultInfo item;
    item.id = "behavior_fault_id:" + std::to_string(fault.behavior_fault_id());
    item.severity = behavior_status_to_string(fault.status());
    item.message = "cause=" + behavior_cause_to_string(fault.cause());
    snap.behavior_faults.push_back(std::move(item));
  }

  for (const auto& fault : robot_state.service_fault_state().faults()) {
    FaultInfo item;
    const auto& id = fault.fault_id();
    if (!id.fault_name().empty()) {
      item.id = id.fault_name();
    } else if (!id.service_name().empty()) {
      item.id = id.service_name();
    } else {
      item.id = "service_fault";
    }
    item.severity = service_severity_to_string(fault.severity());
    item.message = fault.error_message();
    snap.service_faults.push_back(std::move(item));
  }

  ::bosdyn::api::SE3Pose odom_tform_body;
  if (::bosdyn::api::GetOdomTformBody(robot_state.kinematic_state().transforms_snapshot(),
                                      &odom_tform_body)) {
    snap.has_body_pitch_rad = true;
    snap.body_pitch_parent_frame = "odom";
    snap.body_pitch_rad = ::bosdyn::api::ToPitch(odom_tform_body);
  }

  *out_snapshot = std::move(snap);
  return true;
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

bool SpotClient::StartGraphRecording(std::string* out_created_waypoint_id) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!EnsureGraphNavRecordingClient()) return false;
  ::bosdyn::api::graph_nav::StartRecordingRequest req;
  auto r = graph_nav_recording_client_->StartRecording(req);
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }
  if (r.response.status() != ::bosdyn::api::graph_nav::StartRecordingResponse::STATUS_OK) {
    SetLastError("StartRecording failed status=" + std::to_string(static_cast<int>(r.response.status())));
    return false;
  }
  if (out_created_waypoint_id) *out_created_waypoint_id = r.response.created_waypoint().id();
  return true;
}

bool SpotClient::StopGraphRecording() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!EnsureGraphNavRecordingClient()) return false;
  constexpr int kRetryDelayMs = 300;
  constexpr int kMaxStopWaitMs = 30000;
  constexpr int kMaxAttempts = (kMaxStopWaitMs + kRetryDelayMs - 1) / kRetryDelayMs;
  for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
    ::bosdyn::api::graph_nav::StopRecordingRequest req;
    auto r = graph_nav_recording_client_->StopRecording(req);
    if (!r.status) {
      SetLastError(r.status.DebugString());
      return false;
    }
    const auto status = r.response.status();
    if (status == ::bosdyn::api::graph_nav::StopRecordingResponse::STATUS_OK) {
      return true;
    }
    if (status == ::bosdyn::api::graph_nav::StopRecordingResponse::STATUS_NOT_READY_YET) {
      std::this_thread::sleep_for(std::chrono::milliseconds(kRetryDelayMs));
      continue;
    }
    if (status == ::bosdyn::api::graph_nav::StopRecordingResponse::STATUS_NOT_LOCALIZED_TO_END) {
      auto record_status = graph_nav_recording_client_->GetRecordStatus();
      if (record_status.status && !record_status.response.is_recording()) {
        return true;
      }
      SetLastError("StopRecording failed status=" + std::to_string(static_cast<int>(status)) +
                   " localized_id=" + r.response.error_waypoint_localized_id());
      return false;
    }
    SetLastError("StopRecording failed status=" + std::to_string(static_cast<int>(status)));
    return false;
  }
  SetLastError("StopRecording failed status=" +
               std::to_string(static_cast<int>(
                   ::bosdyn::api::graph_nav::StopRecordingResponse::STATUS_NOT_READY_YET)) +
               " after retries");
  return false;
}

bool SpotClient::CreateGraphWaypoint(const std::string& waypoint_name, std::string* out_waypoint_id) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!EnsureGraphNavRecordingClient()) return false;
  ::bosdyn::api::graph_nav::CreateWaypointRequest req;
  if (!waypoint_name.empty()) req.set_waypoint_name(waypoint_name);
  auto r = graph_nav_recording_client_->CreateWaypoint(req);
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }
  if (r.response.status() != ::bosdyn::api::graph_nav::CreateWaypointResponse::STATUS_OK) {
    SetLastError("CreateWaypoint failed status=" + std::to_string(static_cast<int>(r.response.status())));
    return false;
  }
  if (out_waypoint_id) {
    *out_waypoint_id = r.response.created_waypoint().id();
  }
  return true;
}

bool SpotClient::DownloadCurrentGraph(::bosdyn::api::graph_nav::Graph* out_graph) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!out_graph) return false;
  if (!EnsureGraphNavClient()) return false;
  auto r = graph_nav_client_->DownloadGraph();
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }
  *out_graph = r.response.graph();
  return true;
}

bool SpotClient::DownloadCurrentMap(StoredMap* out_map) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!out_map) return false;
  if (!EnsureGraphNavClient()) return false;

  auto graph_result = graph_nav_client_->DownloadGraph();
  if (!graph_result.status) {
    SetLastError(graph_result.status.DebugString());
    return false;
  }

  StoredMap map_data;
  map_data.graph = graph_result.response.graph();

  std::unordered_set<std::string> waypoint_snapshot_ids;
  waypoint_snapshot_ids.reserve(static_cast<size_t>(map_data.graph.waypoints_size()));
  for (const auto& waypoint : map_data.graph.waypoints()) {
    if (!waypoint.snapshot_id().empty()) waypoint_snapshot_ids.insert(waypoint.snapshot_id());
  }
  for (const auto& snapshot_id : waypoint_snapshot_ids) {
    ::bosdyn::api::graph_nav::DownloadWaypointSnapshotRequest req;
    req.set_waypoint_snapshot_id(snapshot_id);
    auto snapshot_result = graph_nav_client_->DownloadWaypointSnapshot(req);
    if (!snapshot_result.status) {
      SetLastError(snapshot_result.status.DebugString());
      return false;
    }
    map_data.waypoint_snapshots.push_back(snapshot_result.response);
  }

  std::unordered_set<std::string> edge_snapshot_ids;
  edge_snapshot_ids.reserve(static_cast<size_t>(map_data.graph.edges_size()));
  for (const auto& edge : map_data.graph.edges()) {
    if (!edge.snapshot_id().empty()) edge_snapshot_ids.insert(edge.snapshot_id());
  }
  for (const auto& snapshot_id : edge_snapshot_ids) {
    ::bosdyn::api::graph_nav::DownloadEdgeSnapshotRequest req;
    req.set_edge_snapshot_id(snapshot_id);
    auto snapshot_result = graph_nav_client_->DownloadEdgeSnapshot(req);
    if (!snapshot_result.status) {
      SetLastError(snapshot_result.status.DebugString());
      return false;
    }
    map_data.edge_snapshots.push_back(snapshot_result.response);
  }

  *out_map = std::move(map_data);
  return true;
}

bool SpotClient::GetMappingStatus(MappingStatus* out_status) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!out_status) return false;
  if (!EnsureGraphNavRecordingClient()) return false;
  if (!EnsureWorldObjectClient()) return false;

  auto record_result = graph_nav_recording_client_->GetRecordStatus();
  if (!record_result.status) {
    SetLastError(record_result.status.DebugString());
    return false;
  }

  MappingStatus status;
  status.is_recording = record_result.response.is_recording();
  status.status = static_cast<int>(record_result.response.status());
  status.map_state = static_cast<int>(record_result.response.map_state());
  status.impaired = (record_result.response.status() ==
                     ::bosdyn::api::graph_nav::GetRecordStatusResponse::STATUS_ROBOT_IMPAIRED);
  status.waypoint_count = record_result.response.map_stats().waypoints().count();
  status.edge_count = record_result.response.map_stats().edges().count();
  status.waypoint_snapshot_count = record_result.response.map_stats().waypoint_snapshots().count();
  status.edge_snapshot_count = record_result.response.map_stats().edge_snapshots().count();
  status.total_path_length_m = record_result.response.map_stats().total_path_length();

  ::bosdyn::api::ListWorldObjectRequest wo_req;
  wo_req.add_object_type(::bosdyn::api::WORLD_OBJECT_APRILTAG);
  auto wo = world_object_client_->ListWorldObjects(wo_req);
  if (!wo.status) {
    SetLastError("GetRecordStatus ok but ListWorldObjects failed: " + wo.status.DebugString());
    return false;
  }
  status.visible_fiducial_ids.reserve(static_cast<size_t>(wo.response.world_objects_size()));
  for (const auto& obj : wo.response.world_objects()) {
    if (!obj.has_apriltag_properties()) continue;
    status.visible_fiducial_ids.push_back(obj.apriltag_properties().tag_id());
  }
  std::sort(status.visible_fiducial_ids.begin(), status.visible_fiducial_ids.end());
  status.visible_fiducial_ids.erase(
      std::unique(status.visible_fiducial_ids.begin(), status.visible_fiducial_ids.end()),
      status.visible_fiducial_ids.end());

  *out_status = std::move(status);
  return true;
}

bool SpotClient::BuildGraphNavMapSnapshot(const StoredMap& map_data,
                                          GraphNavMapSnapshot* out_snapshot) {
  if (!out_snapshot) return false;

  GraphNavMapSnapshot snapshot;
  const auto& graph = map_data.graph;

  std::unordered_map<std::string, ::bosdyn::api::SE3Pose> anchors_by_waypoint;
  anchors_by_waypoint.reserve(static_cast<size_t>(graph.anchoring().anchors_size()));
  for (const auto& anchor : graph.anchoring().anchors()) {
    if (anchor.id().empty()) continue;
    anchors_by_waypoint[anchor.id()] = anchor.seed_tform_waypoint();
  }
  snapshot.has_anchoring = !anchors_by_waypoint.empty();

  snapshot.waypoints.reserve(static_cast<size_t>(graph.waypoints_size()));
  for (const auto& waypoint : graph.waypoints()) {
    const auto anchor_it = anchors_by_waypoint.find(waypoint.id());
    if (anchor_it == anchors_by_waypoint.end()) continue;

    GraphNavWaypoint out_waypoint;
    out_waypoint.id = waypoint.id();
    out_waypoint.snapshot_id = waypoint.snapshot_id();
    out_waypoint.label = waypoint.annotations().name().empty() ? waypoint.id()
                                                               : waypoint.annotations().name();
    fill_pose3d(anchor_it->second, &out_waypoint.seed_tform_waypoint);
    snapshot.waypoints.push_back(std::move(out_waypoint));
  }

  snapshot.edges.reserve(static_cast<size_t>(graph.edges_size()));
  for (const auto& edge : graph.edges()) {
    if (edge.id().from_waypoint().empty() || edge.id().to_waypoint().empty()) continue;
    GraphNavEdge out_edge;
    out_edge.from_waypoint_id = edge.id().from_waypoint();
    out_edge.to_waypoint_id = edge.id().to_waypoint();
    out_edge.snapshot_id = edge.snapshot_id();
    fill_pose3d(edge.from_tform_to(), &out_edge.from_tform_to);
    snapshot.edges.push_back(std::move(out_edge));
  }

  snapshot.objects.reserve(static_cast<size_t>(graph.anchoring().objects_size()));
  for (const auto& object : graph.anchoring().objects()) {
    if (object.id().empty()) continue;
    GraphNavObject out_object;
    out_object.id = object.id();
    fill_pose3d(object.seed_tform_object(), &out_object.seed_tform_object);
    snapshot.objects.push_back(std::move(out_object));
  }

  if (anchors_by_waypoint.empty()) {
    *out_snapshot = std::move(snapshot);
    return true;
  }

  std::unordered_map<std::string, const ::bosdyn::api::graph_nav::WaypointSnapshot*> snapshots_by_id;
  snapshots_by_id.reserve(map_data.waypoint_snapshots.size());
  for (const auto& waypoint_snapshot : map_data.waypoint_snapshots) {
    if (waypoint_snapshot.id().empty()) continue;
    snapshots_by_id[waypoint_snapshot.id()] = &waypoint_snapshot;
  }

  struct AnchoredLocalGrid {
    DecodedLocalGrid grid;
    ::bosdyn::api::SE3Pose seed_tform_grid;
  };

  std::vector<AnchoredLocalGrid> anchored_grids;
  anchored_grids.reserve(snapshot.waypoints.size());
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double min_z = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  double base_resolution_m = std::numeric_limits<double>::infinity();

  for (const auto& waypoint : graph.waypoints()) {
    const auto anchor_it = anchors_by_waypoint.find(waypoint.id());
    if (anchor_it == anchors_by_waypoint.end()) continue;
    if (waypoint.snapshot_id().empty()) continue;

    const auto snapshot_it = snapshots_by_id.find(waypoint.snapshot_id());
    if (snapshot_it == snapshots_by_id.end() || !snapshot_it->second) continue;

    DecodedLocalGrid decoded;
    std::string decode_error;
    if (!build_decoded_local_grid(snapshot_it->second->robot_local_grids(), &decoded, &decode_error)) {
      continue;
    }

    ::bosdyn::api::SE3Pose ko_tform_body;
    if (!::bosdyn::api::get_a_tform_b(decoded.transforms_snapshot,
                                      ::bosdyn::api::kOdomFrame,
                                      ::bosdyn::api::kBodyFrame,
                                      &ko_tform_body)) {
      continue;
    }

    const ::bosdyn::api::SE3Pose seed_tform_body =
        anchor_it->second * waypoint.waypoint_tform_ko() * ko_tform_body;
    const ::bosdyn::api::SE3Pose seed_tform_grid = seed_tform_body * decoded.body_tform_grid;

    const double span_x_m = static_cast<double>(decoded.width) * decoded.resolution_m;
    const double span_y_m = static_cast<double>(decoded.height) * decoded.resolution_m;
    const std::array<::bosdyn::api::Vec3, 4> corners = {
        make_vec3(0.0, 0.0, 0.0),
        make_vec3(span_x_m, 0.0, 0.0),
        make_vec3(0.0, span_y_m, 0.0),
        make_vec3(span_x_m, span_y_m, 0.0),
    };
    for (const auto& corner : corners) {
      const ::bosdyn::api::Vec3 seed_corner = seed_tform_grid * corner;
      min_x = std::min(min_x, seed_corner.x());
      min_y = std::min(min_y, seed_corner.y());
      min_z = std::min(min_z, seed_corner.z());
      max_x = std::max(max_x, seed_corner.x());
      max_y = std::max(max_y, seed_corner.y());
    }
    base_resolution_m = std::min(base_resolution_m, decoded.resolution_m);

    AnchoredLocalGrid anchored_grid;
    anchored_grid.grid = std::move(decoded);
    anchored_grid.seed_tform_grid = seed_tform_grid;
    anchored_grids.push_back(std::move(anchored_grid));
  }

  if (anchored_grids.empty() || !std::isfinite(base_resolution_m) || base_resolution_m <= 0.0 ||
      !std::isfinite(min_x) || !std::isfinite(min_y) || !std::isfinite(max_x) ||
      !std::isfinite(max_y)) {
    *out_snapshot = std::move(snapshot);
    return true;
  }

  const double span_x_m = std::max(base_resolution_m, max_x - min_x);
  const double span_y_m = std::max(base_resolution_m, max_y - min_y);
  double output_resolution_m = base_resolution_m;
  constexpr double kMaxGlobalMapCells = 1000000.0;
  const double estimated_cells =
      (span_x_m / output_resolution_m) * (span_y_m / output_resolution_m);
  if (estimated_cells > kMaxGlobalMapCells) {
    output_resolution_m =
        std::max(output_resolution_m, std::sqrt((span_x_m * span_y_m) / kMaxGlobalMapCells));
  }

  const int width =
      std::max(1, static_cast<int>(std::ceil(span_x_m / output_resolution_m)));
  const int height =
      std::max(1, static_cast<int>(std::ceil(span_y_m / output_resolution_m)));
  std::vector<AccumulatedOccupancyCell> accum(
      static_cast<size_t>(width) * static_cast<size_t>(height));

  for (const auto& anchored_grid : anchored_grids) {
    const auto& grid = anchored_grid.grid;
    const ::bosdyn::api::Vec3 grid_origin = anchored_grid.seed_tform_grid * make_vec3(0.0, 0.0, 0.0);
    const ::bosdyn::api::Vec3 step_x =
        (anchored_grid.seed_tform_grid * make_vec3(grid.resolution_m, 0.0, 0.0)) - grid_origin;
    const ::bosdyn::api::Vec3 step_y =
        (anchored_grid.seed_tform_grid * make_vec3(0.0, grid.resolution_m, 0.0)) - grid_origin;
    const ::bosdyn::api::Vec3 first_center =
        anchored_grid.seed_tform_grid * make_vec3(grid.resolution_m * 0.5,
                                                  grid.resolution_m * 0.5,
                                                  0.0);

    for (int y = 0; y < grid.height; ++y) {
      const ::bosdyn::api::Vec3 row_center =
          first_center + (static_cast<double>(y) * step_y);
      for (int x = 0; x < grid.width; ++x) {
        const int32_t cell = grid.occupancy[static_cast<size_t>(x) +
                                            static_cast<size_t>(grid.width) * static_cast<size_t>(y)];
        if (cell < 0) continue;
        const ::bosdyn::api::Vec3 seed_center =
            row_center + (static_cast<double>(x) * step_x);
        accumulate_occupancy_sample(seed_center.x(), seed_center.y(), min_x, min_y,
                                    output_resolution_m, width, height, cell >= 50, &accum);
      }
    }
  }

  SpotClient::OccupancyGridMapSnapshot stitched_map;
  stitched_map.map_type = "graphnav_stitched";
  stitched_map.resolution_m = output_resolution_m;
  stitched_map.width = width;
  stitched_map.height = height;
  stitched_map.seed_tform_grid.x = min_x;
  stitched_map.seed_tform_grid.y = min_y;
  stitched_map.seed_tform_grid.z = std::isfinite(min_z) ? min_z : 0.0;
  stitched_map.seed_tform_grid.qx = 0.0;
  stitched_map.seed_tform_grid.qy = 0.0;
  stitched_map.seed_tform_grid.qz = 0.0;
  stitched_map.seed_tform_grid.qw = 1.0;
  stitched_map.occupancy.resize(accum.size(), -1);
  for (size_t i = 0; i < accum.size(); ++i) {
    if (accum[i].known_weight <= 1e-9) continue;
    const double occupied_ratio = accum[i].occupied_weight / accum[i].known_weight;
    stitched_map.occupancy[i] = occupied_ratio >= 0.5 ? 100 : 0;
  }

  snapshot.has_map = true;
  snapshot.map = std::move(stitched_map);
  *out_snapshot = std::move(snapshot);
  return true;
}

bool SpotClient::UploadGraphMap(const StoredMap& map_data) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!EnsureGraphNavClient()) return false;

  auto upload_graph = [this](const ::bosdyn::api::graph_nav::Graph& graph,
                             bool replace_graph,
                             ::bosdyn::api::graph_nav::UploadGraphResponse* out_response) -> bool {
    ::bosdyn::api::graph_nav::UploadGraphRequest upload_req;
    *upload_req.mutable_graph() = graph;
    upload_req.set_replace_graph(replace_graph);
    auto upload_result = graph_nav_client_->UploadGraph(upload_req);
    if (!upload_result.status) {
      SetLastError(upload_result.status.DebugString());
      return false;
    }
    if (upload_result.response.status() != ::bosdyn::api::graph_nav::UploadGraphResponse::STATUS_OK) {
      SetLastError("UploadGraph failed status=" +
                   std::to_string(static_cast<int>(upload_result.response.status())));
      return false;
    }
    if (out_response) *out_response = upload_result.response;
    return true;
  };

  ::bosdyn::api::graph_nav::UploadGraphResponse upload_response;
  if (!upload_graph(map_data.graph, true, &upload_response)) return false;

  if (!upload_response.replaced_graph()) {
    auto clear_result = graph_nav_client_->ClearGraph();
    if (!clear_result.status) {
      SetLastError("UploadGraph fallback clear failed: " + clear_result.status.DebugString());
      return false;
    }
    if (clear_result.response.status() != ::bosdyn::api::graph_nav::ClearGraphResponse::STATUS_OK &&
        clear_result.response.status() != ::bosdyn::api::graph_nav::ClearGraphResponse::STATUS_UNKNOWN) {
      SetLastError("UploadGraph fallback clear failed status=" +
                   std::to_string(static_cast<int>(clear_result.response.status())));
      return false;
    }
    if (!upload_graph(map_data.graph, false, nullptr)) return false;
  }

  for (auto snapshot : map_data.waypoint_snapshots) {
    auto up = graph_nav_client_->UploadWaypointSnapshot(snapshot);
    if (!up.status) {
      SetLastError(up.status.DebugString());
      return false;
    }
    if (up.response.status() != ::bosdyn::api::graph_nav::UploadWaypointSnapshotResponse::STATUS_OK) {
      SetLastError("UploadWaypointSnapshot failed status=" +
                   std::to_string(static_cast<int>(up.response.status())));
      return false;
    }
  }

  for (auto snapshot : map_data.edge_snapshots) {
    auto up = graph_nav_client_->UploadEdgeSnapshot(snapshot);
    if (!up.status) {
      SetLastError(up.status.DebugString());
      return false;
    }
  }
  return true;
}

bool SpotClient::ClearGraph() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!EnsureGraphNavClient()) return false;
  auto r = graph_nav_client_->ClearGraph();
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }
  if (r.response.status() != ::bosdyn::api::graph_nav::ClearGraphResponse::STATUS_OK) {
    SetLastError("ClearGraph failed status=" + std::to_string(static_cast<int>(r.response.status())));
    return false;
  }
  return true;
}

bool SpotClient::SetLocalizationFiducial() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!EnsureGraphNavClient()) return false;
  if (!EnsureTimeSync()) return false;
  ::bosdyn::api::graph_nav::SetLocalizationRequest req;
  req.set_fiducial_init(::bosdyn::api::graph_nav::SetLocalizationRequest::FIDUCIAL_INIT_NEAREST);
  auto r = graph_nav_client_->SetLocalization(req);
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }
  if (r.response.status() != ::bosdyn::api::graph_nav::SetLocalizationResponse::STATUS_OK) {
    SetLastError("SetLocalization failed status=" + std::to_string(static_cast<int>(r.response.status())));
    return false;
  }
  return true;
}

bool SpotClient::GetLocalizationWaypointId(std::string* out_waypoint_id) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!out_waypoint_id) return false;
  if (!EnsureGraphNavClient()) return false;
  ::bosdyn::api::graph_nav::GetLocalizationStateRequest req;
  auto r = graph_nav_client_->GetLocalizationState(req);
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }
  const auto& localization = r.response.localization();
  if (localization.waypoint_id().empty()) {
    SetLastError("GetLocalizationState returned empty waypoint_id");
    return false;
  }
  *out_waypoint_id = localization.waypoint_id();
  return true;
}

bool SpotClient::GetLocalizationSnapshot(LocalizationSnapshot* out_snapshot) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!out_snapshot) return false;
  if (!EnsureGraphNavClient()) return false;
  ::bosdyn::api::graph_nav::GetLocalizationStateRequest req;
  auto r = graph_nav_client_->GetLocalizationState(req);
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }
  const auto& localization = r.response.localization();
  if (localization.waypoint_id().empty()) {
    SetLastError("GetLocalizationState returned empty waypoint_id");
    return false;
  }

  LocalizationSnapshot snapshot;
  snapshot.waypoint_id = localization.waypoint_id();
  if (localization.has_waypoint_tform_body()) {
    snapshot.has_waypoint_tform_body = true;
    snapshot.waypoint_tform_body_x = localization.waypoint_tform_body().position().x();
    snapshot.waypoint_tform_body_y = localization.waypoint_tform_body().position().y();
    snapshot.waypoint_tform_body_z = localization.waypoint_tform_body().position().z();
  }
  if (localization.has_seed_tform_body()) {
    snapshot.has_seed_tform_body = true;
    snapshot.seed_tform_body_x = localization.seed_tform_body().position().x();
    snapshot.seed_tform_body_y = localization.seed_tform_body().position().y();
    snapshot.seed_tform_body_z = localization.seed_tform_body().position().z();
    snapshot.seed_tform_body_qx = localization.seed_tform_body().rotation().x();
    snapshot.seed_tform_body_qy = localization.seed_tform_body().rotation().y();
    snapshot.seed_tform_body_qz = localization.seed_tform_body().rotation().z();
    snapshot.seed_tform_body_qw = localization.seed_tform_body().rotation().w();
  }
  *out_snapshot = std::move(snapshot);
  return true;
}

bool SpotClient::GetLocalizationMapSnapshot(LocalizationMapSnapshot* out_snapshot) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (!out_snapshot) return false;
  if (!EnsureGraphNavClient()) return false;

  ::bosdyn::api::graph_nav::GetLocalizationStateRequest req;
  req.set_request_live_terrain_maps(true);
  auto r = graph_nav_client_->GetLocalizationState(req);
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }

  const auto& localization = r.response.localization();
  if (localization.waypoint_id().empty()) {
    SetLastError("GetLocalizationState returned empty waypoint_id");
    return false;
  }

  LocalizationMapSnapshot snapshot;
  snapshot.localized = true;
  snapshot.waypoint_id = localization.waypoint_id();
  if (localization.has_waypoint_tform_body()) {
    snapshot.has_waypoint_tform_body = true;
    fill_pose3d(localization.waypoint_tform_body(), &snapshot.waypoint_tform_body);
  }
  if (localization.has_seed_tform_body()) {
    snapshot.has_seed_tform_body = true;
    fill_pose3d(localization.seed_tform_body(), &snapshot.seed_tform_body);
  }

  std::string map_error;
  if (build_occupancy_grid_from_live_data(localization, r.response.live_data().robot_local_grids(),
                                          &snapshot.map, &map_error)) {
    snapshot.has_map = true;
  }

  *out_snapshot = std::move(snapshot);
  return true;
}

bool SpotClient::NavigateToWaypoint(const std::string& waypoint_id, int command_timeout_sec,
                                    uint32_t* out_command_id) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (waypoint_id.empty()) return false;
  if (!EnsureGraphNavClient()) return false;
  if (!EnsureTimeSync()) return false;

  ::bosdyn::api::graph_nav::NavigateToRequest req;
  req.set_destination_waypoint_id(waypoint_id);
  const auto end_time =
      time_sync_endpoint_->RobotTimestampFromLocal(std::chrono::system_clock::now() +
                                                   std::chrono::seconds(std::max(5, command_timeout_sec)));
  *req.mutable_end_time() = end_time;
  auto clock_id = time_sync_endpoint_->GetClockIdentifier();
  if (!clock_id.status) {
    SetLastError(clock_id.status.DebugString());
    return false;
  }
  req.set_clock_identifier(*clock_id.response);

  auto r = graph_nav_client_->NavigateTo(req);
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }
  if (r.response.status() != ::bosdyn::api::graph_nav::NavigateToResponse::STATUS_OK) {
    SetLastError("NavigateTo failed status=" + std::to_string(static_cast<int>(r.response.status())));
    return false;
  }
  if (out_command_id) *out_command_id = r.response.command_id();
  return true;
}

bool SpotClient::NavigateToWaypointStraight(const std::string& waypoint_id, int command_timeout_sec,
                                            uint32_t* out_command_id) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (waypoint_id.empty()) return false;
  if (!EnsureGraphNavClient()) return false;
  if (!EnsureTimeSync()) return false;

  ::bosdyn::api::graph_nav::NavigateToRequest req;
  req.set_destination_waypoint_id(waypoint_id);
  const auto end_time =
      time_sync_endpoint_->RobotTimestampFromLocal(std::chrono::system_clock::now() +
                                                   std::chrono::seconds(std::max(5, command_timeout_sec)));
  *req.mutable_end_time() = end_time;
  auto clock_id = time_sync_endpoint_->GetClockIdentifier();
  if (!clock_id.status) {
    SetLastError(clock_id.status.DebugString());
    return false;
  }
  req.set_clock_identifier(*clock_id.response);

  auto* travel = req.mutable_travel_params();
  travel->set_ignore_final_yaw(true);
  travel->set_disable_directed_exploration(true);
  travel->set_disable_alternate_route_finding(true);
  travel->set_max_corridor_distance(5.0);
  req.set_route_blocked_behavior(
      ::bosdyn::api::graph_nav::RouteFollowingParams::ROUTE_BLOCKED_FAIL);

  auto r = graph_nav_client_->NavigateTo(req);
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }
  if (r.response.status() != ::bosdyn::api::graph_nav::NavigateToResponse::STATUS_OK) {
    SetLastError("NavigateTo(straight) failed status=" +
                 std::to_string(static_cast<int>(r.response.status())));
    return false;
  }
  if (out_command_id) *out_command_id = r.response.command_id();
  return true;
}

bool SpotClient::GetNavigationFeedback(uint32_t command_id, int* out_status) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (command_id == 0 || !out_status) return false;
  if (!EnsureGraphNavClient()) return false;
  auto r = graph_nav_client_->NavigationFeedback(command_id);
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }
  *out_status = static_cast<int>(r.response.status());
  return true;
}

bool SpotClient::GetNavigationFeedbackSnapshot(uint32_t command_id,
                                               NavigationFeedbackSnapshot* out_feedback) {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (command_id == 0 || !out_feedback) return false;
  if (!EnsureGraphNavClient()) return false;
  auto r = graph_nav_client_->NavigationFeedback(command_id);
  if (!r.status) {
    SetLastError(r.status.DebugString());
    return false;
  }

  NavigationFeedbackSnapshot fb;
  fb.status = static_cast<int>(r.response.status());
  fb.remaining_route_length = r.response.remaining_route_length();
  fb.body_movement_status = static_cast<int>(r.response.body_movement_status());
  fb.goal_status = static_cast<int>(r.response.goal_status());
  fb.route_following_status = static_cast<int>(r.response.route_following_status());
  fb.blockage_status = static_cast<int>(r.response.blockage_status());
  fb.stuck_reason = static_cast<int>(r.response.stuck_reason());
  for (const auto& entry : r.response.active_region_information()) {
    const int region_status = static_cast<int>(entry.second.region_status());
    if (region_status == 2) ++fb.waiting_region_count;
    if (region_status == 3) ++fb.callback_in_control_region_count;
  }
  *out_feedback = fb;
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
  if (dock_id <= 0) return false;
  if (!EnsureDockingClient()) return false;
  if (!EnsureTimeSync()) return false;
  if (!EnsurePowered()) return false;

  cancel_docking_ = false;
  const int max_attempts = std::max(1, attempts);
  const int feedback_sleep_ms = std::max(100, poll_ms);
  const int command_timeout_ms = std::max(5000, command_timeout_sec * 1000);
  std::string last_err;

  for (int attempt = 1; attempt <= max_attempts; ++attempt) {
    if (cancel_docking_.load()) {
      SetLastError("Docking canceled");
      return false;
    }

    const auto end_time = std::chrono::system_clock::now() + std::chrono::milliseconds(command_timeout_ms);
    decltype(docking_client_->DockingCommandBuilder(static_cast<unsigned int>(dock_id), end_time,
                                                    time_sync_endpoint_)) request;
    {
      std::lock_guard<std::recursive_mutex> lk(api_mu_);
      if (!docking_client_ || !time_sync_endpoint_) {
        SetLastError("Docking client or time sync endpoint unavailable");
        return false;
      }
      request = docking_client_->DockingCommandBuilder(static_cast<unsigned int>(dock_id), end_time,
                                                       time_sync_endpoint_);
    }
    if (!request.status) {
      last_err = std::string("DockingCommandBuilder failed: ") + request.status.DebugString();
      continue;
    }

    request.response.set_prep_pose_behavior(
        (attempt % 2 == 1) ? ::bosdyn::api::docking::PREP_POSE_USE_POSE
                           : ::bosdyn::api::docking::PREP_POSE_SKIP_POSE);
    decltype(docking_client_->DockingCommand(request.response)) start;
    {
      std::lock_guard<std::recursive_mutex> lk(api_mu_);
      if (!docking_client_) {
        SetLastError("Docking client unavailable");
        return false;
      }
      start = docking_client_->DockingCommand(request.response);
    }
    if (!start.status) {
      last_err = std::string("DockingCommand failed: ") + start.status.DebugString();
      continue;
    }

    const uint32_t cmd_id = start.response.docking_command_id();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(command_timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
      if (cancel_docking_.load()) {
        SetLastError("Docking canceled");
        return false;
      }

      decltype(docking_client_->DockingCommandFeedback(cmd_id)) feedback;
      {
        std::lock_guard<std::recursive_mutex> lk(api_mu_);
        if (!docking_client_) {
          SetLastError("Docking client unavailable");
          return false;
        }
        feedback = docking_client_->DockingCommandFeedback(cmd_id);
      }
      if (!feedback.status) {
        last_err = std::string("DockingCommandFeedback failed: ") + feedback.status.DebugString();
        break;
      }

      const auto status = feedback.response.status();
      if (status == ::bosdyn::api::docking::DockingCommandFeedbackResponse::STATUS_DOCKED) {
        return true;
      }
      if (status == ::bosdyn::api::docking::DockingCommandFeedbackResponse::STATUS_IN_PROGRESS) {
        std::this_thread::sleep_for(std::chrono::milliseconds(feedback_sleep_ms));
        continue;
      }

      last_err = std::string("Docking feedback status=") + std::to_string(static_cast<int>(status));
      break;
    }
  }

  if (last_err.empty()) last_err = "Docking failed after retries";
  SetLastError(last_err);
  return false;
}

void SpotClient::RequestDockCancel() { cancel_docking_ = true; }

bool SpotClient::EnsureTimeSync() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (time_sync_endpoint_ && time_sync_endpoint_->HasEstablishedTimeSync()) {
    if (robot_command_client_) robot_command_client_->AddTimeSyncEndpoint(time_sync_endpoint_);
    if (docking_client_) docking_client_->AddTimeSyncEndpoint(time_sync_endpoint_);
    return true;
  }
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

bool SpotClient::EnsurePowerClient() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (power_client_) return true;
  if (!robot_) return false;
  auto power_client = robot_->EnsureServiceClient<::bosdyn::client::PowerClient>(
      ::bosdyn::client::PowerClient::GetDefaultServiceName());
  if (!power_client.status) {
    SetLastError(power_client.status.DebugString());
    return false;
  }
  power_client_ = power_client.response;
  return true;
}

bool SpotClient::EnsureGraphNavClient() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (graph_nav_client_) return true;
  if (!robot_) return false;
  auto client = robot_->EnsureServiceClient<::bosdyn::client::GraphNavClient>(
      ::bosdyn::client::GraphNavClient::GetDefaultServiceName());
  if (!client.status) {
    SetLastError(client.status.DebugString());
    return false;
  }
  graph_nav_client_ = client.response;
  return true;
}

bool SpotClient::EnsureGraphNavRecordingClient() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (graph_nav_recording_client_) return true;
  if (!robot_) return false;
  auto client = robot_->EnsureServiceClient<::bosdyn::client::GraphNavRecordingClient>(
      ::bosdyn::client::GraphNavRecordingClient::GetDefaultServiceName());
  if (!client.status) {
    SetLastError(client.status.DebugString());
    return false;
  }
  graph_nav_recording_client_ = client.response;
  return true;
}

bool SpotClient::EnsureSpotCheckClient() {
  std::lock_guard<std::recursive_mutex> lk(api_mu_);
  if (spot_check_client_) return true;
  if (!robot_) return false;

  auto client = robot_->EnsureServiceClient<::bosdyn::client::SpotCheckClient>(
      ::bosdyn::client::SpotCheckClient::GetDefaultServiceName());
  if (!client.status) {
    auto fallback = robot_->EnsureServiceClient<::bosdyn::client::SpotCheckClient>("spot-check-service");
    if (!fallback.status) {
      SetLastError("Failed to create SpotCheck client: " + client.status.DebugString() +
                   " ; fallback failed: " + fallback.status.DebugString());
      return false;
    }
    spot_check_client_ = fallback.response;
    return true;
  }
  spot_check_client_ = client.response;
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
