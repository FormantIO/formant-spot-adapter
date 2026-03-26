#include "formant_spot_adapter/config.hpp"

#include <cstdlib>
#include <cctype>
#include <fstream>
#include <iostream>
#include <sstream>

#include <google/protobuf/util/json_util.h>

#include "adapter_config.pb.h"

namespace fsa {

namespace {

std::string getenv_or(const char* k, const std::string& d) {
  const char* v = std::getenv(k);
  return (v && *v) ? std::string(v) : d;
}

int getenv_int_or(const char* k, int d) {
  const char* v = std::getenv(k);
  if (!v || !*v) return d;
  try {
    return std::stoi(v);
  } catch (...) {
    return d;
  }
}

bool getenv_bool_or(const char* k, bool d) {
  const char* v = std::getenv(k);
  if (!v || !*v) return d;
  std::string value(v);
  for (char& ch : value) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  if (value == "1" || value == "true" || value == "yes" || value == "on") return true;
  if (value == "0" || value == "false" || value == "no" || value == "off") return false;
  return d;
}

double getenv_double_or(const char* k, double d) {
  const char* v = std::getenv(k);
  if (!v || !*v) return d;
  try {
    return std::stod(v);
  } catch (...) {
    return d;
  }
}

std::string read_file_or_empty(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) return "";
  std::ostringstream oss;
  oss << ifs.rdbuf();
  return oss.str();
}

void apply_json_config(const config::AdapterConfig& j, Config* c) {
  if (j.has_spot_host()) c->spot_host = j.spot_host().value();
  if (j.has_formant_agent_target()) c->formant_agent_target = j.formant_agent_target().value();
  if (j.has_teleop_twist_stream()) c->teleop_twist_stream = j.teleop_twist_stream().value();
  if (j.has_teleop_buttons_stream()) c->teleop_buttons_stream = j.teleop_buttons_stream().value();
  if (j.has_stand_button_stream()) c->stand_button_stream = j.stand_button_stream().value();
  if (j.has_sit_button_stream()) c->sit_button_stream = j.sit_button_stream().value();
  if (j.has_estop_button_stream()) c->estop_button_stream = j.estop_button_stream().value();
  if (j.has_recover_button_stream()) c->recover_button_stream = j.recover_button_stream().value();
  if (j.has_walk_button_stream()) c->walk_button_stream = j.walk_button_stream().value();
  if (j.has_stairs_button_stream()) c->stairs_button_stream = j.stairs_button_stream().value();
  if (j.has_crawl_button_stream()) c->crawl_button_stream = j.crawl_button_stream().value();
  if (j.has_reset_arm_button_stream()) c->reset_arm_button_stream = j.reset_arm_button_stream().value();
  if (j.has_arm_raise_button_stream()) c->arm_raise_button_stream = j.arm_raise_button_stream().value();
  if (j.has_dock_button_stream()) c->dock_button_stream = j.dock_button_stream().value();
  if (j.has_can_dock_stream()) c->can_dock_stream = j.can_dock_stream().value();
  if (j.has_stateful_mode_stream()) c->stateful_mode_stream = j.stateful_mode_stream().value();
  if (j.has_camera_source()) c->camera_source = j.camera_source().value();
  if (j.has_camera_stream_name()) c->camera_stream_name = j.camera_stream_name().value();
  if (j.has_left_camera_source()) c->left_camera_source = j.left_camera_source().value();
  if (j.has_left_camera_stream_name()) c->left_camera_stream_name = j.left_camera_stream_name().value();
  if (j.has_right_camera_source()) c->right_camera_source = j.right_camera_source().value();
  if (j.has_right_camera_stream_name()) c->right_camera_stream_name = j.right_camera_stream_name().value();
  if (j.has_back_camera_source()) c->back_camera_source = j.back_camera_source().value();
  if (j.has_back_camera_stream_name()) c->back_camera_stream_name = j.back_camera_stream_name().value();
  if (j.has_localization_image_stream_name()) {
    c->localization_image_stream_name = j.localization_image_stream_name().value();
  }

  if (j.has_camera_fps()) c->camera_fps = j.camera_fps().value();
  if (j.has_surround_camera_fps()) c->surround_camera_fps = j.surround_camera_fps().value();
  if (j.has_surround_camera_poll_hz()) {
    c->surround_camera_poll_hz = j.surround_camera_poll_hz().value();
  }
  if (j.has_right_camera_rotate_180()) {
    c->right_camera_rotate_180 = j.right_camera_rotate_180().value();
  }
  if (j.has_localization_image_fps()) c->localization_image_fps = j.localization_image_fps().value();
  if (j.has_localization_image_poll_hz()) {
    c->localization_image_poll_hz = j.localization_image_poll_hz().value();
  }
  if (j.has_twist_deadband()) c->twist_deadband = j.twist_deadband().value();
  if (j.has_teleop_idle_timeout_ms()) c->teleop_idle_timeout_ms = j.teleop_idle_timeout_ms().value();
  if (j.has_max_vx_mps()) c->max_vx_mps = j.max_vx_mps().value();
  if (j.has_max_vy_mps()) c->max_vy_mps = j.max_vy_mps().value();
  if (j.has_max_wz_rps()) c->max_wz_rps = j.max_wz_rps().value();
  if (j.has_max_body_pitch_rad()) c->max_body_pitch_rad = j.max_body_pitch_rad().value();
  if (j.has_lease_retain_hz()) c->lease_retain_hz = j.lease_retain_hz().value();
  if (j.has_heartbeat_timeout_ms()) c->heartbeat_timeout_ms = j.heartbeat_timeout_ms().value();
  if (j.has_zero_velocity_repeats()) c->zero_velocity_repeats = j.zero_velocity_repeats().value();
  if (j.has_dock_station_id()) c->dock_station_id = j.dock_station_id().value();
  if (j.has_dock_attempts()) c->dock_attempts = j.dock_attempts().value();
  if (j.has_dock_poll_ms()) c->dock_poll_ms = j.dock_poll_ms().value();
  if (j.has_dock_command_timeout_sec()) c->dock_command_timeout_sec = j.dock_command_timeout_sec().value();
  if (j.has_arm_raise_x()) c->arm_raise_x = j.arm_raise_x().value();
  if (j.has_arm_raise_y()) c->arm_raise_y = j.arm_raise_y().value();
  if (j.has_arm_raise_z()) c->arm_raise_z = j.arm_raise_z().value();
  if (j.has_arm_raise_qw()) c->arm_raise_qw = j.arm_raise_qw().value();
  if (j.has_arm_raise_qx()) c->arm_raise_qx = j.arm_raise_qx().value();
  if (j.has_arm_raise_qy()) c->arm_raise_qy = j.arm_raise_qy().value();
  if (j.has_arm_raise_qz()) c->arm_raise_qz = j.arm_raise_qz().value();
  if (j.has_arm_raise_move_sec()) c->arm_raise_move_sec = j.arm_raise_move_sec().value();
  if (j.has_arm_hold_interval_ms()) c->arm_hold_interval_ms = j.arm_hold_interval_ms().value();
  if (j.has_graphnav_store_dir()) c->graphnav_store_dir = j.graphnav_store_dir().value();
  if (j.has_waypoint_text_stream()) c->waypoint_text_stream = j.waypoint_text_stream().value();
  if (j.has_maps_text_stream()) c->maps_text_stream = j.maps_text_stream().value();
  if (j.has_graphnav_command_timeout_sec()) {
    c->graphnav_command_timeout_sec = j.graphnav_command_timeout_sec().value();
  }
}

}  // namespace

Config load_config_from_env() {
  Config c;
  c.spot_host = getenv_or("SPOT_HOST", c.spot_host);
  c.spot_username = getenv_or("SPOT_USERNAME", c.spot_username);
  c.spot_password = getenv_or("SPOT_PASSWORD", c.spot_password);
  c.formant_agent_target = getenv_or("FORMANT_AGENT_TARGET", c.formant_agent_target);

  c.teleop_twist_stream = getenv_or("TELEOP_TWIST_STREAM", c.teleop_twist_stream);
  c.teleop_buttons_stream = getenv_or("TELEOP_BUTTONS_STREAM", c.teleop_buttons_stream);
  c.stand_button_stream = getenv_or("STAND_BUTTON_STREAM", c.stand_button_stream);
  c.sit_button_stream = getenv_or("SIT_BUTTON_STREAM", c.sit_button_stream);
  c.estop_button_stream = getenv_or("ESTOP_BUTTON_STREAM", c.estop_button_stream);
  c.recover_button_stream = getenv_or("RECOVER_BUTTON_STREAM", c.recover_button_stream);
  c.walk_button_stream = getenv_or("WALK_BUTTON_STREAM", c.walk_button_stream);
  c.stairs_button_stream = getenv_or("STAIRS_BUTTON_STREAM", c.stairs_button_stream);
  c.crawl_button_stream = getenv_or("CRAWL_BUTTON_STREAM", c.crawl_button_stream);
  c.reset_arm_button_stream = getenv_or("RESET_ARM_BUTTON_STREAM", c.reset_arm_button_stream);
  c.arm_raise_button_stream = getenv_or("ARM_RAISE_BUTTON_STREAM", c.arm_raise_button_stream);
  c.dock_button_stream = getenv_or("DOCK_BUTTON_STREAM", c.dock_button_stream);
  c.can_dock_stream = getenv_or("CAN_DOCK_STREAM", c.can_dock_stream);
  c.stateful_mode_stream = getenv_or("STATEFUL_MODE_STREAM", c.stateful_mode_stream);

  c.camera_source = getenv_or("CAMERA_SOURCE", c.camera_source);
  c.camera_stream_name = getenv_or("CAMERA_STREAM_NAME", c.camera_stream_name);
  c.left_camera_source = getenv_or("LEFT_CAMERA_SOURCE", c.left_camera_source);
  c.left_camera_stream_name = getenv_or("LEFT_CAMERA_STREAM_NAME", c.left_camera_stream_name);
  c.right_camera_source = getenv_or("RIGHT_CAMERA_SOURCE", c.right_camera_source);
  c.right_camera_stream_name = getenv_or("RIGHT_CAMERA_STREAM_NAME", c.right_camera_stream_name);
  c.back_camera_source = getenv_or("BACK_CAMERA_SOURCE", c.back_camera_source);
  c.back_camera_stream_name = getenv_or("BACK_CAMERA_STREAM_NAME", c.back_camera_stream_name);
  c.camera_fps = getenv_int_or("CAMERA_FPS", c.camera_fps);
  c.surround_camera_fps = getenv_int_or("SURROUND_CAMERA_FPS", c.surround_camera_fps);
  c.surround_camera_poll_hz =
      getenv_int_or("SURROUND_CAMERA_POLL_HZ", c.surround_camera_poll_hz);
  c.right_camera_rotate_180 =
      getenv_bool_or("RIGHT_CAMERA_ROTATE_180", c.right_camera_rotate_180);
  c.localization_image_stream_name =
      getenv_or("LOCALIZATION_IMAGE_STREAM_NAME", c.localization_image_stream_name);
  c.localization_image_fps = getenv_int_or("LOCALIZATION_IMAGE_FPS", c.localization_image_fps);
  c.localization_image_poll_hz =
      getenv_int_or("LOCALIZATION_IMAGE_POLL_HZ", c.localization_image_poll_hz);
  c.max_body_pitch_rad = getenv_double_or("MAX_BODY_PITCH_RAD", c.max_body_pitch_rad);
  c.twist_deadband = getenv_double_or("TWIST_DEADBAND", c.twist_deadband);
  c.teleop_idle_timeout_ms = getenv_int_or("TELEOP_IDLE_TIMEOUT_MS", c.teleop_idle_timeout_ms);

  c.max_vx_mps = getenv_double_or("MAX_VX_MPS", c.max_vx_mps);
  c.max_vy_mps = getenv_double_or("MAX_VY_MPS", c.max_vy_mps);
  c.max_wz_rps = getenv_double_or("MAX_WZ_RPS", c.max_wz_rps);
  c.max_body_pitch_rad = getenv_double_or("MAX_BODY_PITCH_RAD", c.max_body_pitch_rad);

  c.lease_retain_hz = getenv_int_or("LEASE_RETAIN_HZ", c.lease_retain_hz);
  c.heartbeat_timeout_ms = getenv_int_or("HEARTBEAT_TIMEOUT_MS", c.heartbeat_timeout_ms);
  c.zero_velocity_repeats = getenv_int_or("ZERO_VELOCITY_REPEATS", c.zero_velocity_repeats);
  c.dock_station_id = getenv_int_or("DOCK_STATION_ID", c.dock_station_id);
  c.dock_attempts = getenv_int_or("DOCK_ATTEMPTS", c.dock_attempts);
  c.dock_poll_ms = getenv_int_or("DOCK_POLL_MS", c.dock_poll_ms);
  c.dock_command_timeout_sec = getenv_int_or("DOCK_COMMAND_TIMEOUT_SEC", c.dock_command_timeout_sec);
  c.arm_raise_x = getenv_double_or("ARM_RAISE_X", c.arm_raise_x);
  c.arm_raise_y = getenv_double_or("ARM_RAISE_Y", c.arm_raise_y);
  c.arm_raise_z = getenv_double_or("ARM_RAISE_Z", c.arm_raise_z);
  c.arm_raise_qw = getenv_double_or("ARM_RAISE_QW", c.arm_raise_qw);
  c.arm_raise_qx = getenv_double_or("ARM_RAISE_QX", c.arm_raise_qx);
  c.arm_raise_qy = getenv_double_or("ARM_RAISE_QY", c.arm_raise_qy);
  c.arm_raise_qz = getenv_double_or("ARM_RAISE_QZ", c.arm_raise_qz);
  c.arm_raise_move_sec = getenv_double_or("ARM_RAISE_MOVE_SEC", c.arm_raise_move_sec);
  c.arm_hold_interval_ms = getenv_int_or("ARM_HOLD_INTERVAL_MS", c.arm_hold_interval_ms);
  c.graphnav_store_dir = getenv_or("GRAPHNAV_STORE_DIR", c.graphnav_store_dir);
  c.waypoint_text_stream = getenv_or("WAYPOINT_TEXT_STREAM", c.waypoint_text_stream);
  c.maps_text_stream = getenv_or("MAPS_TEXT_STREAM", c.maps_text_stream);
  c.graphnav_command_timeout_sec =
      getenv_int_or("GRAPHNAV_COMMAND_TIMEOUT_SEC", c.graphnav_command_timeout_sec);
  return c;
}

Config load_config() {
  Config c;

  const std::string json_path =
      getenv_or("CONFIG_PATH", "config/formant-spot-adapter.json");
  const std::string json_str = read_file_or_empty(json_path);
  if (!json_str.empty()) {
    config::AdapterConfig j;
    google::protobuf::util::JsonParseOptions opts;
    opts.ignore_unknown_fields = true;
    auto status = google::protobuf::util::JsonStringToMessage(json_str, &j, opts);
    if (!status.ok()) {
      std::cerr << "Failed to parse config JSON " << json_path << ": " << status.ToString() << std::endl;
    } else {
      apply_json_config(j, &c);
    }
  } else {
    std::cerr << "Config JSON not found/readable: " << json_path
              << " (using defaults + env secrets)" << std::endl;
  }

  // Only secrets come from env in normal adapter execution.
  c.spot_username = getenv_or("SPOT_USERNAME", c.spot_username);
  c.spot_password = getenv_or("SPOT_PASSWORD", c.spot_password);
  return c;
}

}  // namespace fsa
