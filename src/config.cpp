#include "formant_spot_adapter/config.hpp"

#include <cstdlib>
#include <cctype>
#include <cmath>
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

std::string trim_env_value(const char* v) {
  if (!v) return "";
  std::string value(v);
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) return "";
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

bool env_value_present(const char* k) {
  return !trim_env_value(std::getenv(k)).empty();
}

bool getenv_int_if_set(const char* k, int* out) {
  if (!out) return false;
  const std::string value = trim_env_value(std::getenv(k));
  if (value.empty()) return false;
  try {
    std::size_t idx = 0;
    const int parsed = std::stoi(value, &idx);
    if (idx != value.size()) return false;
    *out = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool getenv_bool_if_set(const char* k, bool* out) {
  if (!out) return false;
  std::string value = trim_env_value(std::getenv(k));
  if (value.empty()) return false;
  for (char& ch : value) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  if (value == "1" || value == "true" || value == "yes" || value == "on") {
    *out = true;
    return true;
  }
  if (value == "0" || value == "false" || value == "no" || value == "off") {
    *out = false;
    return true;
  }
  return false;
}

bool getenv_double_if_set(const char* k, double* out) {
  if (!out) return false;
  const std::string value = trim_env_value(std::getenv(k));
  if (value.empty()) return false;
  try {
    std::size_t idx = 0;
    const double parsed = std::stod(value, &idx);
    if (idx != value.size() || !std::isfinite(parsed)) return false;
    *out = parsed;
    return true;
  } catch (...) {
    return false;
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
  if (j.has_teleop_joy_stream()) c->teleop_joy_stream = j.teleop_joy_stream().value();
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
  if (j.has_joy_axis_forward()) c->joy_axis_forward = j.joy_axis_forward().value();
  if (j.has_joy_axis_strafe()) c->joy_axis_strafe = j.joy_axis_strafe().value();
  if (j.has_joy_axis_yaw()) c->joy_axis_yaw = j.joy_axis_yaw().value();
  if (j.has_joy_axis_body_pitch()) c->joy_axis_body_pitch = j.joy_axis_body_pitch().value();
  if (j.has_joy_axis_forward_inverted()) {
    c->joy_axis_forward_inverted = j.joy_axis_forward_inverted().value();
  }
  if (j.has_joy_axis_strafe_inverted()) {
    c->joy_axis_strafe_inverted = j.joy_axis_strafe_inverted().value();
  }
  if (j.has_joy_axis_yaw_inverted()) {
    c->joy_axis_yaw_inverted = j.joy_axis_yaw_inverted().value();
  }
  if (j.has_joy_axis_body_pitch_inverted()) {
    c->joy_axis_body_pitch_inverted = j.joy_axis_body_pitch_inverted().value();
  }
  if (j.has_joy_button_stand()) c->joy_button_stand = j.joy_button_stand().value();
  if (j.has_joy_button_sit()) c->joy_button_sit = j.joy_button_sit().value();
  if (j.has_joy_button_reset_arm()) c->joy_button_reset_arm = j.joy_button_reset_arm().value();
  if (j.has_joy_button_recover()) c->joy_button_recover = j.joy_button_recover().value();
  if (j.has_joy_button_walk()) c->joy_button_walk = j.joy_button_walk().value();
  if (j.has_joy_button_stairs()) c->joy_button_stairs = j.joy_button_stairs().value();
  if (j.has_joy_button_crawl()) c->joy_button_crawl = j.joy_button_crawl().value();
  if (j.has_joy_button_dock()) c->joy_button_dock = j.joy_button_dock().value();
  if (j.has_joy_button_estop()) c->joy_button_estop = j.joy_button_estop().value();
  if (j.has_camera_source()) c->camera_source = j.camera_source().value();
  if (j.has_camera_stream_name()) c->camera_stream_name = j.camera_stream_name().value();
  if (j.has_left_camera_source()) c->left_camera_source = j.left_camera_source().value();
  if (j.has_left_camera_stream_name()) c->left_camera_stream_name = j.left_camera_stream_name().value();
  if (j.has_right_camera_source()) c->right_camera_source = j.right_camera_source().value();
  if (j.has_right_camera_stream_name()) c->right_camera_stream_name = j.right_camera_stream_name().value();
  if (j.has_back_camera_source()) c->back_camera_source = j.back_camera_source().value();
  if (j.has_back_camera_stream_name()) c->back_camera_stream_name = j.back_camera_stream_name().value();
  if (j.has_front_left_camera_source()) {
    c->front_left_camera_source = j.front_left_camera_source().value();
  }
  if (j.has_front_right_camera_source()) {
    c->front_right_camera_source = j.front_right_camera_source().value();
  }
  if (j.has_front_image_stream_name()) {
    c->front_image_stream_name = j.front_image_stream_name().value();
  }
  if (j.has_localization_image_stream_name()) {
    c->localization_image_stream_name = j.localization_image_stream_name().value();
  }

  if (j.has_camera_fps()) c->camera_fps = j.camera_fps().value();
  if (j.has_surround_camera_fps()) c->surround_camera_fps = j.surround_camera_fps().value();
  if (j.has_surround_camera_poll_hz()) {
    c->surround_camera_poll_hz = j.surround_camera_poll_hz().value();
  }
  if (j.has_front_image_fps()) c->front_image_fps = j.front_image_fps().value();
  if (j.has_front_image_poll_hz()) {
    c->front_image_poll_hz = j.front_image_poll_hz().value();
  }
  if (j.has_front_image_roll_degrees()) {
    c->front_image_roll_degrees = j.front_image_roll_degrees().value();
    c->front_image_roll_degrees_configured = true;
  }
  if (j.has_right_camera_rotate_180()) {
    c->right_camera_rotate_180 = j.right_camera_rotate_180().value();
  }
  if (j.has_localization_image_fps()) c->localization_image_fps = j.localization_image_fps().value();
  if (j.has_localization_image_poll_hz()) {
    c->localization_image_poll_hz = j.localization_image_poll_hz().value();
  }
  if (j.has_graphnav_global_localization_stream()) {
    c->graphnav_global_localization_stream = j.graphnav_global_localization_stream().value();
  }
  if (j.has_graphnav_global_localization_hz()) {
    c->graphnav_global_localization_hz = j.graphnav_global_localization_hz().value();
  }
  if (j.has_graphnav_map_stream()) c->graphnav_map_stream = j.graphnav_map_stream().value();
  if (j.has_graphnav_metadata_stream()) {
    c->graphnav_metadata_stream = j.graphnav_metadata_stream().value();
  }
  if (j.has_graphnav_overlay_stream()) {
    c->graphnav_overlay_stream = j.graphnav_overlay_stream().value();
  }
  if (j.has_graphnav_nav_state_stream()) {
    c->graphnav_nav_state_stream = j.graphnav_nav_state_stream().value();
  }
  if (j.has_graphnav_map_image_stream_name()) {
    c->graphnav_map_image_stream_name = j.graphnav_map_image_stream_name().value();
  }
  if (j.has_graphnav_map_image_metadata_stream_name()) {
    c->graphnav_map_image_metadata_stream_name =
        j.graphnav_map_image_metadata_stream_name().value();
  }
  if (j.has_graphnav_map_image_fps()) c->graphnav_map_image_fps = j.graphnav_map_image_fps().value();
  if (j.has_graphnav_map_image_poll_hz()) {
    c->graphnav_map_image_poll_hz = j.graphnav_map_image_poll_hz().value();
  }
  if (j.has_arm_present()) c->arm_present_override = j.arm_present().value() ? 1 : 0;
  if (j.has_twist_deadband()) c->twist_deadband = j.twist_deadband().value();
  if (j.has_teleop_idle_timeout_ms()) c->teleop_idle_timeout_ms = j.teleop_idle_timeout_ms().value();
  if (j.has_max_vx_mps()) c->max_vx_mps = j.max_vx_mps().value();
  if (j.has_max_vy_mps()) c->max_vy_mps = j.max_vy_mps().value();
  if (j.has_max_wz_rps()) c->max_wz_rps = j.max_wz_rps().value();
  if (j.has_max_body_pitch_rad()) c->max_body_pitch_rad = j.max_body_pitch_rad().value();
  if (j.has_translation_response_curve()) {
    c->translation_response_curve = j.translation_response_curve().value();
  }
  if (j.has_rotation_response_curve()) c->rotation_response_curve = j.rotation_response_curve().value();
  if (j.has_linear_accel_limit_mps2()) c->linear_accel_limit_mps2 = j.linear_accel_limit_mps2().value();
  if (j.has_strafe_accel_limit_mps2()) c->strafe_accel_limit_mps2 = j.strafe_accel_limit_mps2().value();
  if (j.has_angular_accel_limit_rps2()) c->angular_accel_limit_rps2 = j.angular_accel_limit_rps2().value();
  if (j.has_body_pitch_rate_limit_radps()) {
    c->body_pitch_rate_limit_radps = j.body_pitch_rate_limit_radps().value();
  }
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
  if (j.has_graphnav_store_dir()) c->graphnav_store_dir = j.graphnav_store_dir().value();
  if (j.has_waypoint_text_stream()) c->waypoint_text_stream = j.waypoint_text_stream().value();
  if (j.has_maps_text_stream()) c->maps_text_stream = j.maps_text_stream().value();
  if (j.has_graphnav_command_timeout_sec()) {
    c->graphnav_command_timeout_sec = j.graphnav_command_timeout_sec().value();
  }
  for (const auto& stream_control : j.stream_controls()) {
    if (stream_control.stream().empty()) continue;
    c->stream_controls[stream_control.stream()] = StreamControl{stream_control.enabled()};
  }
}

void log_env_override(const char* key, bool secret, bool valid) {
  if (!valid) {
    std::cerr << "[config] ignoring invalid env override " << key << std::endl;
    return;
  }
  if (!secret) {
    std::cerr << "[config] env override applied " << key << std::endl;
  }
}

void apply_env_overrides(Config* c, bool log_overrides) {
  if (!c) return;

  const auto apply_string = [&](const char* key, std::string* field, bool secret = false) {
    const char* v = std::getenv(key);
    if (!v || !*v) return;
    *field = std::string(v);
    if (log_overrides) log_env_override(key, secret, true);
  };
  const auto apply_int = [&](const char* key, int* field) {
    if (!env_value_present(key)) return;
    int parsed = 0;
    if (!getenv_int_if_set(key, &parsed)) {
      if (log_overrides) log_env_override(key, false, false);
      return;
    }
    *field = parsed;
    if (log_overrides) log_env_override(key, false, true);
  };
  const auto apply_double = [&](const char* key, double* field) {
    if (!env_value_present(key)) return;
    double parsed = 0.0;
    if (!getenv_double_if_set(key, &parsed)) {
      if (log_overrides) log_env_override(key, false, false);
      return;
    }
    *field = parsed;
    if (log_overrides) log_env_override(key, false, true);
  };
  const auto apply_bool = [&](const char* key, bool* field) {
    if (!env_value_present(key)) return;
    bool parsed = false;
    if (!getenv_bool_if_set(key, &parsed)) {
      if (log_overrides) log_env_override(key, false, false);
      return;
    }
    *field = parsed;
    if (log_overrides) log_env_override(key, false, true);
  };

  apply_string("SPOT_HOST", &c->spot_host);
  apply_string("SPOT_USERNAME", &c->spot_username, true);
  apply_string("SPOT_PASSWORD", &c->spot_password, true);
  apply_string("FORMANT_AGENT_TARGET", &c->formant_agent_target);

  apply_string("TELEOP_TWIST_STREAM", &c->teleop_twist_stream);
  apply_string("TELEOP_JOY_STREAM", &c->teleop_joy_stream);
  apply_string("TELEOP_BUTTONS_STREAM", &c->teleop_buttons_stream);
  apply_string("STAND_BUTTON_STREAM", &c->stand_button_stream);
  apply_string("SIT_BUTTON_STREAM", &c->sit_button_stream);
  apply_string("ESTOP_BUTTON_STREAM", &c->estop_button_stream);
  apply_string("RECOVER_BUTTON_STREAM", &c->recover_button_stream);
  apply_string("WALK_BUTTON_STREAM", &c->walk_button_stream);
  apply_string("STAIRS_BUTTON_STREAM", &c->stairs_button_stream);
  apply_string("CRAWL_BUTTON_STREAM", &c->crawl_button_stream);
  apply_string("RESET_ARM_BUTTON_STREAM", &c->reset_arm_button_stream);
  apply_string("ARM_RAISE_BUTTON_STREAM", &c->arm_raise_button_stream);
  apply_string("DOCK_BUTTON_STREAM", &c->dock_button_stream);
  apply_string("CAN_DOCK_STREAM", &c->can_dock_stream);
  apply_string("STATEFUL_MODE_STREAM", &c->stateful_mode_stream);
  apply_int("JOY_AXIS_FORWARD", &c->joy_axis_forward);
  apply_int("JOY_AXIS_STRAFE", &c->joy_axis_strafe);
  apply_int("JOY_AXIS_YAW", &c->joy_axis_yaw);
  apply_int("JOY_AXIS_BODY_PITCH", &c->joy_axis_body_pitch);
  apply_bool("JOY_AXIS_FORWARD_INVERTED", &c->joy_axis_forward_inverted);
  apply_bool("JOY_AXIS_STRAFE_INVERTED", &c->joy_axis_strafe_inverted);
  apply_bool("JOY_AXIS_YAW_INVERTED", &c->joy_axis_yaw_inverted);
  apply_bool("JOY_AXIS_BODY_PITCH_INVERTED", &c->joy_axis_body_pitch_inverted);
  apply_int("JOY_BUTTON_STAND", &c->joy_button_stand);
  apply_int("JOY_BUTTON_SIT", &c->joy_button_sit);
  apply_int("JOY_BUTTON_RESET_ARM", &c->joy_button_reset_arm);
  apply_int("JOY_BUTTON_RECOVER", &c->joy_button_recover);
  apply_int("JOY_BUTTON_WALK", &c->joy_button_walk);
  apply_int("JOY_BUTTON_STAIRS", &c->joy_button_stairs);
  apply_int("JOY_BUTTON_CRAWL", &c->joy_button_crawl);
  apply_int("JOY_BUTTON_DOCK", &c->joy_button_dock);
  apply_int("JOY_BUTTON_ESTOP", &c->joy_button_estop);

  apply_string("CAMERA_SOURCE", &c->camera_source);
  apply_string("CAMERA_STREAM_NAME", &c->camera_stream_name);
  apply_string("LEFT_CAMERA_SOURCE", &c->left_camera_source);
  apply_string("LEFT_CAMERA_STREAM_NAME", &c->left_camera_stream_name);
  apply_string("RIGHT_CAMERA_SOURCE", &c->right_camera_source);
  apply_string("RIGHT_CAMERA_STREAM_NAME", &c->right_camera_stream_name);
  apply_string("BACK_CAMERA_SOURCE", &c->back_camera_source);
  apply_string("BACK_CAMERA_STREAM_NAME", &c->back_camera_stream_name);
  apply_string("FRONT_LEFT_CAMERA_SOURCE", &c->front_left_camera_source);
  apply_string("FRONT_RIGHT_CAMERA_SOURCE", &c->front_right_camera_source);
  apply_string("FRONT_IMAGE_STREAM_NAME", &c->front_image_stream_name);
  apply_int("CAMERA_FPS", &c->camera_fps);
  apply_int("SURROUND_CAMERA_FPS", &c->surround_camera_fps);
  apply_int("SURROUND_CAMERA_POLL_HZ", &c->surround_camera_poll_hz);
  apply_int("FRONT_IMAGE_FPS", &c->front_image_fps);
  apply_int("FRONT_IMAGE_POLL_HZ", &c->front_image_poll_hz);
  if (env_value_present("FRONT_IMAGE_ROLL_DEGREES")) {
    int parsed = 0;
    if (getenv_int_if_set("FRONT_IMAGE_ROLL_DEGREES", &parsed)) {
      c->front_image_roll_degrees = parsed;
      c->front_image_roll_degrees_configured = true;
      if (log_overrides) log_env_override("FRONT_IMAGE_ROLL_DEGREES", false, true);
    } else if (log_overrides) {
      log_env_override("FRONT_IMAGE_ROLL_DEGREES", false, false);
    }
  }
  apply_bool("RIGHT_CAMERA_ROTATE_180", &c->right_camera_rotate_180);
  apply_string("LOCALIZATION_IMAGE_STREAM_NAME", &c->localization_image_stream_name);
  apply_int("LOCALIZATION_IMAGE_FPS", &c->localization_image_fps);
  apply_int("LOCALIZATION_IMAGE_POLL_HZ", &c->localization_image_poll_hz);
  apply_string("GRAPHNAV_GLOBAL_LOCALIZATION_STREAM", &c->graphnav_global_localization_stream);
  apply_int("GRAPHNAV_GLOBAL_LOCALIZATION_HZ", &c->graphnav_global_localization_hz);
  apply_string("GRAPHNAV_MAP_STREAM", &c->graphnav_map_stream);
  apply_string("GRAPHNAV_METADATA_STREAM", &c->graphnav_metadata_stream);
  apply_string("GRAPHNAV_OVERLAY_STREAM", &c->graphnav_overlay_stream);
  apply_string("GRAPHNAV_NAV_STATE_STREAM", &c->graphnav_nav_state_stream);
  apply_string("GRAPHNAV_MAP_IMAGE_STREAM_NAME", &c->graphnav_map_image_stream_name);
  apply_string("GRAPHNAV_MAP_IMAGE_METADATA_STREAM_NAME", &c->graphnav_map_image_metadata_stream_name);
  apply_int("GRAPHNAV_MAP_IMAGE_FPS", &c->graphnav_map_image_fps);
  apply_int("GRAPHNAV_MAP_IMAGE_POLL_HZ", &c->graphnav_map_image_poll_hz);
  if (env_value_present("ARM_PRESENT")) {
    bool arm_present = false;
    if (getenv_bool_if_set("ARM_PRESENT", &arm_present)) {
      c->arm_present_override = arm_present ? 1 : 0;
      if (log_overrides) log_env_override("ARM_PRESENT", false, true);
    } else if (log_overrides) {
      log_env_override("ARM_PRESENT", false, false);
    }
  }
  apply_double("TWIST_DEADBAND", &c->twist_deadband);
  apply_int("TELEOP_IDLE_TIMEOUT_MS", &c->teleop_idle_timeout_ms);

  apply_double("MAX_VX_MPS", &c->max_vx_mps);
  apply_double("MAX_VY_MPS", &c->max_vy_mps);
  apply_double("MAX_WZ_RPS", &c->max_wz_rps);
  apply_double("MAX_BODY_PITCH_RAD", &c->max_body_pitch_rad);
  apply_double("TRANSLATION_RESPONSE_CURVE", &c->translation_response_curve);
  apply_double("ROTATION_RESPONSE_CURVE", &c->rotation_response_curve);
  apply_double("LINEAR_ACCEL_LIMIT_MPS2", &c->linear_accel_limit_mps2);
  apply_double("STRAFE_ACCEL_LIMIT_MPS2", &c->strafe_accel_limit_mps2);
  apply_double("ANGULAR_ACCEL_LIMIT_RPS2", &c->angular_accel_limit_rps2);
  apply_double("BODY_PITCH_RATE_LIMIT_RADPS", &c->body_pitch_rate_limit_radps);

  apply_int("LEASE_RETAIN_HZ", &c->lease_retain_hz);
  apply_int("HEARTBEAT_TIMEOUT_MS", &c->heartbeat_timeout_ms);
  apply_int("ZERO_VELOCITY_REPEATS", &c->zero_velocity_repeats);
  apply_int("DOCK_STATION_ID", &c->dock_station_id);
  apply_int("DOCK_ATTEMPTS", &c->dock_attempts);
  apply_int("DOCK_POLL_MS", &c->dock_poll_ms);
  apply_int("DOCK_COMMAND_TIMEOUT_SEC", &c->dock_command_timeout_sec);
  apply_double("ARM_RAISE_X", &c->arm_raise_x);
  apply_double("ARM_RAISE_Y", &c->arm_raise_y);
  apply_double("ARM_RAISE_Z", &c->arm_raise_z);
  apply_double("ARM_RAISE_QW", &c->arm_raise_qw);
  apply_double("ARM_RAISE_QX", &c->arm_raise_qx);
  apply_double("ARM_RAISE_QY", &c->arm_raise_qy);
  apply_double("ARM_RAISE_QZ", &c->arm_raise_qz);
  apply_double("ARM_RAISE_MOVE_SEC", &c->arm_raise_move_sec);
  apply_string("GRAPHNAV_STORE_DIR", &c->graphnav_store_dir);
  apply_string("WAYPOINT_TEXT_STREAM", &c->waypoint_text_stream);
  apply_string("MAPS_TEXT_STREAM", &c->maps_text_stream);
  apply_int("GRAPHNAV_COMMAND_TIMEOUT_SEC", &c->graphnav_command_timeout_sec);
}

}  // namespace

Config load_config_from_env() {
  Config c;
  apply_env_overrides(&c, false);
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
              << " (using defaults + env overrides)" << std::endl;
  }

  apply_env_overrides(&c, true);
  return c;
}

}  // namespace fsa
