#pragma once

#include <string>
#include <unordered_map>

namespace fsa {

struct StreamControl {
  bool enabled{true};
};

struct Config {
  std::string spot_host;
  std::string spot_username;
  std::string spot_password;
  std::string formant_agent_target{"localhost:5501"};

  std::string teleop_twist_stream{"Joystick"};
  std::string teleop_joy_stream{"Gamepad"};
  std::string teleop_buttons_stream{"Buttons"};
  std::string stand_button_stream{"Stand"};
  std::string sit_button_stream{"Sit"};
  std::string estop_button_stream{"E-Stop"};
  std::string recover_button_stream{"Recover"};
  std::string walk_button_stream{"Walk"};
  std::string stairs_button_stream{"Stairs"};
  std::string crawl_button_stream{"Crawl"};
  std::string reset_arm_button_stream{"Reset Arm"};
  std::string arm_raise_button_stream{"Arm Raise"};
  std::string dock_button_stream{"Dock"};
  std::string can_dock_stream{"spot.can_dock"};
  std::string stateful_mode_stream{"spot.mode_state"};

  int joy_axis_forward{1};
  int joy_axis_strafe{0};
  int joy_axis_yaw{2};
  int joy_axis_body_pitch{3};
  bool joy_axis_forward_inverted{true};
  bool joy_axis_strafe_inverted{true};
  bool joy_axis_yaw_inverted{true};
  bool joy_axis_body_pitch_inverted{true};
  int joy_button_stand{0};
  int joy_button_sit{1};
  int joy_button_reset_arm{-1};
  int joy_button_recover{-1};
  int joy_button_walk{2};
  int joy_button_stairs{-1};
  int joy_button_crawl{-1};
  int joy_button_dock{3};
  int joy_button_estop{-1};

  std::string camera_source{"hand_color_image"};
  std::string camera_stream_name{"spot.hand.image"};
  std::string left_camera_source{"left_fisheye_image"};
  std::string left_camera_stream_name{"spot.left.image"};
  std::string right_camera_source{"right_fisheye_image"};
  std::string right_camera_stream_name{"spot.right.image"};
  std::string back_camera_source{"back_fisheye_image"};
  std::string back_camera_stream_name{"spot.back.image"};
  std::string front_left_camera_source{"frontleft_fisheye_image"};
  std::string front_right_camera_source{"frontright_fisheye_image"};
  std::string front_image_stream_name{"spot.front.image"};
  int camera_fps{30};
  int surround_camera_fps{15};
  int surround_camera_poll_hz{15};
  int front_image_fps{15};
  int front_image_poll_hz{15};
  int front_image_roll_degrees{90};
  bool front_image_roll_degrees_configured{true};
  bool right_camera_rotate_180{true};
  std::string localization_image_stream_name{"spot.localization.graphnav.image"};
  int localization_image_fps{15};
  int localization_image_poll_hz{2};
  std::string graphnav_global_localization_stream{"spot.localization.graphnav.global"};
  int graphnav_global_localization_hz{2};
  std::string graphnav_map_stream{"spot.map.graphnav"};
  std::string graphnav_metadata_stream{"spot.graphnav.metadata"};
  std::string graphnav_overlay_stream{"spot.graphnav.overlay"};
  std::string graphnav_nav_state_stream{"spot.nav.state"};
  std::string graphnav_map_image_stream_name{"spot.localization.graphnav.global.image"};
  std::string graphnav_map_image_metadata_stream_name{"spot.localization.graphnav.global.image.meta"};
  int graphnav_map_image_fps{15};
  int graphnav_map_image_poll_hz{2};
  int arm_present_override{-1};  // -1 => auto-detect, 0 => no arm, 1 => arm present
  double twist_deadband{0.08};
  int teleop_idle_timeout_ms{300};

  double max_vx_mps{0.8};
  double max_vy_mps{0.5};
  double max_wz_rps{1.2};
  double max_body_pitch_rad{0.25};

  int lease_retain_hz{2};
  int heartbeat_timeout_ms{5000};
  int zero_velocity_repeats{3};

  int dock_station_id{-1};  // -1 => auto-discover if exactly one dock is configured.
  int dock_attempts{3};
  int dock_poll_ms{1000};
  int dock_command_timeout_sec{30};

  // Saved "raised" hand pose in body frame. Set in config JSON.
  double arm_raise_x{0.0};
  double arm_raise_y{0.0};
  double arm_raise_z{0.0};
  double arm_raise_qw{1.0};
  double arm_raise_qx{0.0};
  double arm_raise_qy{0.0};
  double arm_raise_qz{0.0};
  double arm_raise_move_sec{1.5};
  int arm_hold_interval_ms{2500};

  std::string graphnav_store_dir{"data/graphnav"};
  std::string waypoint_text_stream{"spot.waypoints"};
  std::string maps_text_stream{"spot.maps"};
  int graphnav_command_timeout_sec{60};
  std::unordered_map<std::string, StreamControl> stream_controls;

  bool IsStreamEnabled(const std::string& stream) const {
    if (stream.empty()) return false;
    const auto it = stream_controls.find(stream);
    return it == stream_controls.end() ? true : it->second.enabled;
  }
};

Config load_config_from_env();
Config load_config();

}  // namespace fsa
