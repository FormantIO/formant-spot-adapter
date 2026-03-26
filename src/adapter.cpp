#include "formant_spot_adapter/adapter.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "bosdyn/api/graph_nav/graph_nav.pb.h"

namespace fsa {

namespace {
enum MotionModeState {
  kMotionModeWalk = 0,
  kMotionModeStairs = 1,
  kMotionModeCrawl = 2,
};

long long now_ms() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

bool bitset_has_true(const v1::model::Bitset& bitset) {
  for (const auto& b : bitset.bits()) {
    if (b.value()) return true;
  }
  return false;
}

std::string normalize_button_token(const std::string& in) {
  std::string out;
  out.reserve(in.size());
  for (unsigned char ch : in) {
    if (std::isalnum(ch)) out.push_back(static_cast<char>(std::tolower(ch)));
  }
  return out;
}

std::string json_escape(const std::string& in) {
  std::string out;
  out.reserve(in.size() + 8);
  for (const char ch : in) {
    switch (ch) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out.push_back(ch);
        break;
    }
  }
  return out;
}

constexpr char kGraphNavLocalizationVizStream[] = "spot.localization.graphnav";

constexpr int kLocalizationImageWidth = 1280;
constexpr int kLocalizationImageHeight = 720;
constexpr int kLocalizationImageOuterPadding = 16;
constexpr int kLocalizationImagePanelGap = 16;
constexpr double kPi = 3.14159265358979323846;

struct Quaterniond {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct Vec3d {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

Quaterniond normalize_quaternion(Quaterniond q) {
  const double norm =
      std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm <= 1e-9) return Quaterniond{};
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;
  return q;
}

Quaterniond pose_quaternion(const SpotClient::Pose3D& pose) {
  return normalize_quaternion(Quaterniond{pose.qx, pose.qy, pose.qz, pose.qw});
}

Quaterniond quaternion_conjugate(const Quaterniond& q) {
  return Quaterniond{-q.x, -q.y, -q.z, q.w};
}

Quaterniond quaternion_multiply(const Quaterniond& a, const Quaterniond& b) {
  return Quaterniond{
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

Vec3d rotate_vector(const Quaterniond& q, const Vec3d& v) {
  const Quaterniond qn = normalize_quaternion(q);
  const Quaterniond p{v.x, v.y, v.z, 0.0};
  const Quaterniond rotated =
      quaternion_multiply(quaternion_multiply(qn, p), quaternion_conjugate(qn));
  return Vec3d{rotated.x, rotated.y, rotated.z};
}

SpotClient::Pose3D inverse_pose(const SpotClient::Pose3D& pose) {
  const Quaterniond q_inv = quaternion_conjugate(pose_quaternion(pose));
  const Vec3d t_inv = rotate_vector(q_inv, Vec3d{-pose.x, -pose.y, -pose.z});
  SpotClient::Pose3D out;
  out.x = t_inv.x;
  out.y = t_inv.y;
  out.z = t_inv.z;
  out.qx = q_inv.x;
  out.qy = q_inv.y;
  out.qz = q_inv.z;
  out.qw = q_inv.w;
  return out;
}

SpotClient::Pose3D compose_pose(const SpotClient::Pose3D& a, const SpotClient::Pose3D& b) {
  const Quaterniond qa = pose_quaternion(a);
  const Vec3d rotated_b = rotate_vector(qa, Vec3d{b.x, b.y, b.z});
  const Quaterniond q = normalize_quaternion(quaternion_multiply(qa, pose_quaternion(b)));
  SpotClient::Pose3D out;
  out.x = a.x + rotated_b.x;
  out.y = a.y + rotated_b.y;
  out.z = a.z + rotated_b.z;
  out.qx = q.x;
  out.qy = q.y;
  out.qz = q.z;
  out.qw = q.w;
  return out;
}

double quaternion_yaw_rad(const Quaterniond& q) {
  const Quaterniond qn = normalize_quaternion(q);
  const double siny_cosp = 2.0 * (qn.w * qn.z + qn.x * qn.y);
  const double cosy_cosp = 1.0 - 2.0 * (qn.y * qn.y + qn.z * qn.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::string shorten_identifier(const std::string& value, size_t limit = 20) {
  if (value.size() <= limit) return value;
  if (limit < 8) return value.substr(0, limit);
  const size_t head = limit / 2 - 1;
  const size_t tail = limit - head - 3;
  return value.substr(0, head) + "..." + value.substr(value.size() - tail);
}

std::string format_decimal(double value, int precision = 2) {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(precision);
  oss << value;
  return oss.str();
}

std::vector<std::string> wrap_text(const std::string& text, size_t max_chars) {
  std::vector<std::string> lines;
  if (text.empty() || max_chars == 0) return lines;
  std::istringstream iss(text);
  std::string word;
  std::string line;
  while (iss >> word) {
    if (line.empty()) {
      line = word;
    } else if (line.size() + 1 + word.size() <= max_chars) {
      line += " " + word;
    } else {
      lines.push_back(line);
      line = word;
    }
  }
  if (!line.empty()) lines.push_back(line);
  if (lines.empty()) lines.push_back(text.substr(0, max_chars));
  return lines;
}

void fill_rect_alpha(cv::Mat* image, const cv::Rect& rect, const cv::Scalar& color, double alpha) {
  if (!image || image->empty()) return;
  const cv::Rect bounded = rect & cv::Rect(0, 0, image->cols, image->rows);
  if (bounded.width <= 0 || bounded.height <= 0) return;
  cv::Mat roi = (*image)(bounded);
  cv::Mat overlay(roi.size(), roi.type(), color);
  cv::addWeighted(overlay, alpha, roi, 1.0 - alpha, 0.0, roi);
}

void draw_chip(cv::Mat* image,
               const cv::Point& origin,
               const std::string& text,
               const cv::Scalar& bg,
               const cv::Scalar& fg) {
  if (!image || text.empty()) return;
  int baseline = 0;
  const double font_scale = 0.55;
  const int thickness = 1;
  const cv::Size text_size =
      cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
  const cv::Rect rect(origin.x, origin.y, text_size.width + 18, text_size.height + 14);
  fill_rect_alpha(image, rect, bg, 0.96);
  cv::rectangle(*image, rect, bg, 1, cv::LINE_AA);
  cv::putText(*image, text, cv::Point(rect.x + 9, rect.y + rect.height - 6),
              cv::FONT_HERSHEY_SIMPLEX, font_scale, fg, thickness, cv::LINE_AA);
}

bool encode_jpeg(const cv::Mat& image, std::string* out_bytes) {
  if (!out_bytes || image.empty()) return false;
  std::vector<uchar> encoded;
  const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 88};
  if (!cv::imencode(".jpg", image, encoded, params)) return false;
  out_bytes->assign(reinterpret_cast<const char*>(encoded.data()), encoded.size());
  return true;
}

bool prepare_jpeg_frame(const std::string& input_bytes,
                        bool rotate_180,
                        bool normalize_jpeg,
                        std::string* out_bytes) {
  if (!out_bytes) return false;
  if (input_bytes.empty()) {
    out_bytes->clear();
    return false;
  }
  if (!rotate_180 && !normalize_jpeg) {
    *out_bytes = input_bytes;
    return true;
  }
  std::vector<uchar> compressed(input_bytes.begin(), input_bytes.end());
  cv::Mat decoded = cv::imdecode(compressed, cv::IMREAD_COLOR);
  if (decoded.empty()) return false;
  if (rotate_180) {
    cv::Mat rotated;
    cv::rotate(decoded, rotated, cv::ROTATE_180);
    return encode_jpeg(rotated, out_bytes);
  }
  return encode_jpeg(decoded, out_bytes);
}

void publish_cached_image_loop(FormantAgentClient* agent,
                               const std::atomic<bool>* running,
                               std::mutex* latest_mu,
                               std::shared_ptr<std::string>* latest_jpg,
                               const std::string& stream,
                               int output_period_ms,
                               int post_timeout_ms) {
  if (!agent || !running || !latest_mu || !latest_jpg) return;
  int post_backoff_ms = 0;
  long long next_post_allowed_ms = 0;
  const size_t phase_hash = std::hash<std::string>{}(stream);
  const int phase_offset_ms = output_period_ms > 1
                                  ? static_cast<int>(phase_hash % static_cast<size_t>(output_period_ms))
                                  : 0;
  auto next_tick = std::chrono::steady_clock::now() + std::chrono::milliseconds(phase_offset_ms);
  while (running->load()) {
    std::this_thread::sleep_until(next_tick);
    next_tick += std::chrono::milliseconds(output_period_ms);
    std::shared_ptr<std::string> frame;
    {
      std::lock_guard<std::mutex> lk(*latest_mu);
      frame = *latest_jpg;
    }
    if (frame && !frame->empty()) {
      const long long now = now_ms();
      if (now >= next_post_allowed_ms) {
        const bool ok = agent->PostImage(stream, "image/jpeg", *frame, post_timeout_ms);
        if (ok) {
          post_backoff_ms = 0;
          next_post_allowed_ms = 0;
        } else {
          post_backoff_ms =
              std::min(2000, std::max(100, post_backoff_ms == 0 ? 100 : post_backoff_ms * 2));
          next_post_allowed_ms = now + post_backoff_ms;
        }
      }
    }
  }
}

void set_identity_transform(v1::model::Transform* transform) {
  if (!transform) return;
  transform->mutable_translation()->set_x(0.0);
  transform->mutable_translation()->set_y(0.0);
  transform->mutable_translation()->set_z(0.0);
  transform->mutable_rotation()->set_x(0.0);
  transform->mutable_rotation()->set_y(0.0);
  transform->mutable_rotation()->set_z(0.0);
  transform->mutable_rotation()->set_w(1.0);
}

void set_transform_from_pose3d(const SpotClient::Pose3D& pose, v1::model::Transform* transform) {
  if (!transform) return;
  transform->mutable_translation()->set_x(pose.x);
  transform->mutable_translation()->set_y(pose.y);
  transform->mutable_translation()->set_z(pose.z);
  transform->mutable_rotation()->set_x(pose.qx);
  transform->mutable_rotation()->set_y(pose.qy);
  transform->mutable_rotation()->set_z(pose.qz);
  transform->mutable_rotation()->set_w(pose.qw);
}

v1::model::Localization build_formant_localization(
    const SpotClient::LocalizationMapSnapshot& snapshot) {
  v1::model::Localization localization;

  auto* odometry = localization.mutable_odometry();
  set_transform_from_pose3d(snapshot.seed_tform_body, odometry->mutable_pose());
  set_identity_transform(odometry->mutable_world_to_local());
  odometry->mutable_twist()->mutable_linear()->set_x(0.0);
  odometry->mutable_twist()->mutable_linear()->set_y(0.0);
  odometry->mutable_twist()->mutable_linear()->set_z(0.0);
  odometry->mutable_twist()->mutable_angular()->set_x(0.0);
  odometry->mutable_twist()->mutable_angular()->set_y(0.0);
  odometry->mutable_twist()->mutable_angular()->set_z(0.0);

  auto* map = localization.mutable_map();
  map->set_uuid(snapshot.waypoint_id + ":" + snapshot.map.map_type);
  map->set_resolution(snapshot.map.resolution_m);
  map->set_width(static_cast<uint32_t>(snapshot.map.width));
  map->set_height(static_cast<uint32_t>(snapshot.map.height));
  set_identity_transform(map->mutable_origin());
  set_transform_from_pose3d(snapshot.map.seed_tform_grid, map->mutable_world_to_local());
  for (int32_t cell : snapshot.map.occupancy) {
    map->mutable_occupancy_grid()->add_data(cell);
  }

  return localization;
}

std::string fault_list_to_json(const std::vector<SpotClient::FaultInfo>& faults) {
  std::ostringstream oss;
  oss << "{\"count\":" << faults.size() << ",\"faults\":[";
  for (size_t i = 0; i < faults.size(); ++i) {
    const auto& f = faults[i];
    if (i > 0) oss << ",";
    oss << "{\"id\":\"" << json_escape(f.id)
        << "\",\"severity\":\"" << json_escape(f.severity)
        << "\",\"message\":\"" << json_escape(f.message) << "\"}";
  }
  oss << "]}";
  return oss.str();
}

std::string camera_calibration_status_to_string(int status) {
  using Resp = ::bosdyn::api::spot::CameraCalibrationFeedbackResponse;
  const auto enum_value = static_cast<Resp::Status>(status);
  const std::string name = Resp::Status_Name(enum_value);
  return name.empty() ? std::string("STATUS_UNKNOWN") : name;
}

std::string to_lower_copy(const std::string& in) {
  std::string out;
  out.reserve(in.size());
  for (unsigned char ch : in) {
    out.push_back(static_cast<char>(std::tolower(ch)));
  }
  return out;
}

bool is_camera_server_init_fault(const SpotClient::FaultInfo& fault) {
  if (fault.id != "camera_server") return false;
  const std::string msg = to_lower_copy(fault.message);
  return msg.find("failed to initialize") != std::string::npos ||
         msg.find("enumerated usb2") != std::string::npos ||
         msg.find("not sending") != std::string::npos ||
         msg.find("stopped responding") != std::string::npos;
}

std::vector<std::string> camera_calibration_preflight_errors(const SpotClient::RobotStateSnapshot& snap) {
  std::vector<std::string> errors;
  if (snap.any_estopped) {
    errors.emplace_back("an E-Stop is asserted");
  }
  if (snap.motor_power_state != "on") {
    errors.emplace_back("motor power is not on");
  }
  if (snap.behavior_state != "standing") {
    errors.emplace_back("robot is not standing");
  }
  if (snap.shore_power_state == "on") {
    errors.emplace_back("robot is on shore power/docked");
  }
  for (const auto& fault : snap.system_faults) {
    if (!is_camera_server_init_fault(fault)) continue;
    errors.emplace_back("camera_server fault: " + fault.message);
  }
  return errors;
}

bool fault_requires_soft_recovery(const SpotClient::FaultInfo& fault) {
  return fault.severity == "critical" || fault.severity == "unclearable";
}

bool any_fault_requires_soft_recovery(const std::vector<SpotClient::FaultInfo>& faults) {
  for (const auto& fault : faults) {
    if (fault_requires_soft_recovery(fault)) return true;
  }
  return false;
}

std::string fault_event_key(const std::string& fault_type, const SpotClient::FaultInfo& fault) {
  return fault_type + ":" + fault.id;
}

std::string format_fault_event_prefix(const std::string& action,
                                      const std::string& fault_type,
                                      const SpotClient::FaultInfo& fault) {
  return "FAULT " + action + " [" + fault.severity + "] " + fault_type + ":" + fault.id;
}

void add_faults_for_events(const std::string& fault_type,
                           const std::vector<SpotClient::FaultInfo>& faults,
                           std::unordered_map<std::string, SpotClient::FaultInfo>* out_map) {
  if (!out_map) return;
  for (const auto& fault : faults) {
    (*out_map)[fault_event_key(fault_type, fault)] = fault;
  }
}

std::string fault_type_from_key(const std::string& key) {
  const size_t pos = key.find(':');
  if (pos == std::string::npos) return "fault";
  return key.substr(0, pos);
}

std::string fault_id_from_key(const std::string& key) {
  const size_t pos = key.find(':');
  if (pos == std::string::npos) return key;
  return key.substr(pos + 1);
}

bool fault_event_key_order_less(const std::string& a, const std::string& b) {
  return a < b;
}

std::string robot_state_to_json(const SpotClient::RobotStateSnapshot& s) {
  std::ostringstream oss;
  oss << "{\"motor_power_state\":\"" << json_escape(s.motor_power_state)
      << "\",\"shore_power_state\":\"" << json_escape(s.shore_power_state)
      << "\",\"behavior_state\":\"" << json_escape(s.behavior_state)
      << "\",\"any_estopped\":" << (s.any_estopped ? "true" : "false")
      << ",\"has_battery_pct\":" << (s.has_battery_pct ? "true" : "false")
      << ",\"has_body_pitch_rad\":" << (s.has_body_pitch_rad ? "true" : "false");
  if (s.has_battery_pct) {
    oss << ",\"battery_pct\":" << s.battery_pct;
  }
  if (s.has_body_pitch_rad) {
    oss << ",\"body_pitch_rad\":" << s.body_pitch_rad
        << ",\"body_pitch_parent_frame\":\"" << json_escape(s.body_pitch_parent_frame) << "\"";
  }
  oss << "}";
  return oss.str();
}

std::string mapping_status_to_json(const SpotClient::MappingStatus& s) {
  std::ostringstream oss;
  oss << "{\"is_recording\":" << (s.is_recording ? "true" : "false")
      << ",\"status\":" << s.status
      << ",\"map_state\":" << s.map_state
      << ",\"impaired\":" << (s.impaired ? "true" : "false")
      << ",\"waypoints\":" << s.waypoint_count
      << ",\"edges\":" << s.edge_count
      << ",\"waypoint_snapshots\":" << s.waypoint_snapshot_count
      << ",\"edge_snapshots\":" << s.edge_snapshot_count
      << ",\"total_path_length_m\":" << s.total_path_length_m
      << ",\"visible_fiducials\":[";
  for (size_t i = 0; i < s.visible_fiducial_ids.size(); ++i) {
    oss << s.visible_fiducial_ids[i];
    if (i + 1 < s.visible_fiducial_ids.size()) oss << ",";
  }
  oss << "]}";
  return oss.str();
}

std::string mapping_status_signature(const SpotClient::MappingStatus& s) {
  std::ostringstream oss;
  oss << (s.is_recording ? 1 : 0) << "|"
      << s.status << "|"
      << s.map_state << "|"
      << (s.impaired ? 1 : 0) << "|"
      << s.waypoint_count << "|"
      << s.edge_count << "|"
      << s.waypoint_snapshot_count << "|"
      << s.edge_snapshot_count << "|"
      << static_cast<int>(std::round(s.total_path_length_m * 10.0));
  for (int id : s.visible_fiducial_ids) {
    oss << "|" << id;
  }
  return oss.str();
}

std::string sanitize_id_token(const std::string& in) {
  std::string out;
  out.reserve(in.size());
  for (unsigned char ch : in) {
    if (std::isalnum(ch) || ch == '_' || ch == '-' || ch == '.') {
      out.push_back(static_cast<char>(ch));
    }
  }
  return out;
}

bool nav_status_in_progress(int status) {
  return status == ::bosdyn::api::graph_nav::NavigationFeedbackResponse::STATUS_FOLLOWING_ROUTE;
}

const char* nav_status_name(int status) {
  using Resp = ::bosdyn::api::graph_nav::NavigationFeedbackResponse;
  switch (status) {
    case Resp::STATUS_FOLLOWING_ROUTE:
      return "STATUS_FOLLOWING_ROUTE";
    case Resp::STATUS_REACHED_GOAL:
      return "STATUS_REACHED_GOAL";
    case Resp::STATUS_ROBOT_IMPAIRED:
      return "STATUS_ROBOT_IMPAIRED";
    default:
      return "STATUS_UNKNOWN";
  }
}

bool is_robot_impaired_error_text(const std::string& error_text) {
  return error_text.find("ROBOT_IMPAIRED") != std::string::npos ||
         error_text.find("status=5") != std::string::npos;
}

bool is_not_localized_error_text(const std::string& error_text) {
  std::string lower = error_text;
  for (char& ch : lower) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  return error_text.find("STATUS_NOT_LOCALIZED_TO_MAP") != std::string::npos ||
         lower.find("not_localized") != std::string::npos ||
         lower.find("not localized") != std::string::npos ||
         error_text.find("status=13") != std::string::npos;
}

std::string nav_feedback_signature(const SpotClient::NavigationFeedbackSnapshot& fb) {
  std::ostringstream oss;
  oss << fb.status << "|"
      << static_cast<int>(std::round(fb.remaining_route_length * 10.0)) << "|"
      << fb.body_movement_status << "|"
      << fb.goal_status << "|"
      << fb.route_following_status << "|"
      << fb.blockage_status << "|"
      << fb.stuck_reason << "|"
      << fb.waiting_region_count << "|"
      << fb.callback_in_control_region_count;
  return oss.str();
}

std::string nav_feedback_to_json(uint32_t command_id, const SpotClient::NavigationFeedbackSnapshot& fb) {
  std::ostringstream oss;
  oss << "{\"command_id\":" << command_id
      << ",\"status\":" << fb.status
      << ",\"status_name\":\"" << nav_status_name(fb.status) << "\""
      << ",\"remaining_route_length_m\":" << fb.remaining_route_length
      << ",\"body_movement_status\":" << fb.body_movement_status
      << ",\"goal_status\":" << fb.goal_status
      << ",\"route_following_status\":" << fb.route_following_status
      << ",\"blockage_status\":" << fb.blockage_status
      << ",\"stuck_reason\":" << fb.stuck_reason
      << ",\"waiting_region_count\":" << fb.waiting_region_count
      << ",\"callback_in_control_region_count\":" << fb.callback_in_control_region_count
      << "}";
  return oss.str();
}
}  // namespace

Adapter::Adapter(const Config& config)
    : cfg_(config), agent_(config.formant_agent_target) {}

bool Adapter::Init() {
  if (cfg_.spot_host.empty() || cfg_.spot_username.empty() || cfg_.spot_password.empty()) {
    std::cerr << "Missing Spot config. Require spot_host in config.json and SPOT_USERNAME/SPOT_PASSWORD in env."
              << std::endl;
    return false;
  }

  last_spot_connect_success_ms_ = 0;
  last_spot_connect_attempt_ms_ = 0;
  next_spot_connect_attempt_ms_ = 0;
  last_spot_healthcheck_ms_ = 0;
  spot_reconnect_attempt_ = 0;
  spot_health_failures_ = 0;
  spot_degraded_non_estop_ = false;
  last_soft_recovery_log_ms_ = 0;
  ResetFaultEventsState();

  if (!spot_.Connect(cfg_.spot_host, cfg_.spot_username, cfg_.spot_password)) {
    std::cerr << "Initial Spot connect failed; adapter will continue and retry in background: "
              << spot_.LastError() << std::endl;
    SetSpotDisconnected(spot_.LastError());
  } else {
    SetSpotConnected();
  }

  std::vector<SpotClient::ImageSourceInfo> sources;
  if (SpotConnected() && spot_.ListImageSources(&sources)) {
    std::unordered_map<std::string, SpotClient::ImageSourceInfo> source_map;
    for (const auto& src : sources) {
      source_map[src.name] = src;
    }
    std::cerr << "[camera] available image sources (" << sources.size() << "):" << std::endl;
    for (const auto& src : sources) {
      std::cerr << "  - " << src.name << " (" << src.cols << "x" << src.rows << ")" << std::endl;
    }

    const auto validate_configured_source = [&source_map, this](std::string* source, std::string* stream) {
      if (!source || !stream) return;
      if (source->empty() || stream->empty()) return;
      const std::string src = *source;
      const std::string out_stream = *stream;
      auto it = source_map.find(src);
      if (it == source_map.end()) {
        std::cerr << "[camera] configured source missing: source=" << src
                  << " stream=" << out_stream << " (stream disabled)" << std::endl;
        source->clear();
        stream->clear();
        return;
      }
      if (it->second.cols <= 0 || it->second.rows <= 0) {
        std::cerr << "[camera] configured source unavailable: source=" << src
                  << " stream=" << out_stream
                  << " size=" << it->second.cols << "x" << it->second.rows
                  << " (stream disabled)" << std::endl;
        source->clear();
        stream->clear();
        return;
      }
      std::cerr << "[camera] stream config: stream=" << out_stream << " source=" << src
                << " size=" << it->second.cols << "x" << it->second.rows << std::endl;
    };
    validate_configured_source(&cfg_.camera_source, &cfg_.camera_stream_name);
    validate_configured_source(&cfg_.left_camera_source, &cfg_.left_camera_stream_name);
    validate_configured_source(&cfg_.right_camera_source, &cfg_.right_camera_stream_name);
    validate_configured_source(&cfg_.back_camera_source, &cfg_.back_camera_stream_name);
  } else {
    std::cerr << "[camera] failed to list image sources: " << spot_.LastError() << std::endl;
  }
  if (!cfg_.localization_image_stream_name.empty()) {
    std::cerr << "[localization-image] stream config: stream=" << cfg_.localization_image_stream_name
              << " output_fps=" << std::max(1, cfg_.localization_image_fps)
              << " data_poll_hz=" << std::max(1, cfg_.localization_image_poll_hz)
              << " sdk_poll_cap_hz=5" << std::endl;
  }
  std::cerr << "[camera] surround stream config: output_fps=" << std::max(1, cfg_.surround_camera_fps)
            << " data_poll_hz=" << std::max(1, cfg_.surround_camera_poll_hz)
            << " right_rotate_180=" << (cfg_.right_camera_rotate_180 ? "true" : "false")
            << " normalize_jpeg=true"
            << std::endl;

  lease_owned_ = false;
  heartbeat_seen_ = false;
  control_seen_ = false;
  last_twist_ms_ = now_ms();
  last_twist_timeout_log_ms_ = 0;
  last_control_ms_ = now_ms();
  moving_ = false;
  desired_twist_valid_ = false;
  {
    std::lock_guard<std::mutex> lk(twist_cmd_mu_);
    desired_vx_ = 0.0;
    desired_vy_ = 0.0;
    desired_wz_ = 0.0;
    desired_body_pitch_ = 0.0;
  }
  desired_motion_mode_ = kMotionModeWalk;
  last_arm_hold_cmd_ms_ = 0;
  last_arm_error_log_ms_ = 0;
  last_lease_attempt_ms_ = 0;
  last_logged_control_stream_.clear();
  if (!LoadAdapterMapState()) {
    std::cerr << "[graphnav] failed to load persisted map state" << std::endl;
  }
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    if (active_map_id_.empty() && !default_map_id_.empty()) {
      active_map_id_ = default_map_id_;
    }
    if (!active_map_id_.empty()) {
      pending_restore_map_id_ = active_map_id_;
    }
    loaded_waypoint_id_to_label_.clear();
    loaded_waypoint_label_to_ids_.clear();
    loaded_waypoint_lower_label_to_ids_.clear();
  }
  force_waypoint_publish_ = true;
  force_maps_publish_ = true;
  return true;
}

void Adapter::Run() {
  running_ = true;
  std::cerr << "[adapter] starting teleop loops" << std::endl;

  std::vector<std::string> stream_filter{cfg_.teleop_twist_stream, cfg_.teleop_buttons_stream,
                                         cfg_.stand_button_stream, cfg_.sit_button_stream,
                                         cfg_.estop_button_stream, cfg_.recover_button_stream,
                                         cfg_.walk_button_stream, cfg_.stairs_button_stream,
                                         cfg_.crawl_button_stream, cfg_.reset_arm_button_stream,
                                         cfg_.dock_button_stream};
  stream_filter.erase(std::remove_if(stream_filter.begin(), stream_filter.end(),
                                     [](const std::string& s) { return s.empty(); }),
                      stream_filter.end());
  std::sort(stream_filter.begin(), stream_filter.end());
  stream_filter.erase(std::unique(stream_filter.begin(), stream_filter.end()), stream_filter.end());

  agent_.StartTeleopLoop(stream_filter,
                         [this](const v1::model::ControlDatapoint& dp) { HandleTeleop(dp); });
  agent_.StartCommandLoop({"spot.jetson.reboot", "spot.robot.reboot", "spot.camera.calibrate",
                           "spot.stand", "spot.sit", "spot.recover", "spot.dock",
                           "spot.return_and_dock", "spot.rotate_left", "spot.rotate_right",
                           "spot.reset_arm",
                           "spot.map.create", "spot.map.load", "spot.map.set_default", "spot.map.delete",
                           "spot.map.start_mapping", "spot.map.stop_mapping",
                           "spot.waypoint.save",
                           "spot.waypoint.delete", "spot.waypoint.goto",
                           "spot.waypoint.goto_straight"},
                          [this](const v1::model::CommandRequest& request) { HandleCommand(request); });
  agent_.StartHeartbeatLoop(
      [this](const v1::agent::GetTeleopHeartbeatStreamResponse& hb) { HandleHeartbeat(hb); });

  camera_threads_.clear();
  auto start_camera = [this](const std::string& source, const std::string& stream, int output_fps,
                             int poll_hz, bool rotate_180, bool normalize_jpeg,
                             int post_timeout_ms) {
    if (source.empty() || stream.empty()) return;
    camera_threads_.emplace_back(&Adapter::CameraLoop, this, source, stream, output_fps, poll_hz,
                                 rotate_180, normalize_jpeg, post_timeout_ms);
  };
  const int hand_fps = std::max(1, cfg_.camera_fps);
  const int surround_output_fps = std::max(1, cfg_.surround_camera_fps);
  const int surround_poll_hz = std::max(1, cfg_.surround_camera_poll_hz);
  start_camera(cfg_.camera_source, cfg_.camera_stream_name, hand_fps, hand_fps, false, false, 75);
  start_camera(cfg_.left_camera_source, cfg_.left_camera_stream_name, surround_output_fps,
               surround_poll_hz, false, true, 250);
  start_camera(cfg_.right_camera_source, cfg_.right_camera_stream_name, surround_output_fps,
               surround_poll_hz, cfg_.right_camera_rotate_180, true, 250);
  start_camera(cfg_.back_camera_source, cfg_.back_camera_stream_name, surround_output_fps,
               surround_poll_hz, false, true, 250);
  if (!cfg_.localization_image_stream_name.empty()) {
    localization_image_thread_ = std::thread(&Adapter::LocalizationImageLoop, this,
                                             cfg_.localization_image_stream_name,
                                             std::max(1, cfg_.localization_image_fps),
                                             std::max(1, cfg_.localization_image_poll_hz));
  }
  lease_thread_ = std::thread(&Adapter::LeaseRetainLoop, this);
  dock_thread_ = std::thread(&Adapter::DockLoop, this);
  connection_thread_ = std::thread(&Adapter::ConnectionLoop, this);
  command_exec_thread_ = std::thread(&Adapter::CommandExecutionLoop, this);
  EmitLog("[adapter] adapter log stream online");

  bool last_logged_teleop_active = false;
  while (running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const long long now = now_ms();
    FlushQueuedStatus(now);
    FlushAdapterLogStream(now);
    PublishWaypointsText(false);
    PublishMapsText(false);
    PublishCurrentMapText(false);
    PublishDefaultMapText(false);

    if (HeartbeatExpired() && lease_owned_.load() && moving_.load()) {
      spot_.ZeroVelocity(cfg_.zero_velocity_repeats);
      moving_ = false;
    }

    const bool teleop_active = TeleopSessionActive();
    if (teleop_active != last_logged_teleop_active) {
      if (teleop_active) {
        EmitLog("[teleop] session active");
      } else {
        EmitLog("[teleop] session inactive");
      }
      last_logged_teleop_active = teleop_active;
    }
    if (teleop_active && !lease_owned_ && SpotConnected()) {
      if ((now - last_lease_attempt_ms_.load()) >= 1000) {
        last_lease_attempt_ms_ = now;
        lease_owned_ = spot_.AcquireBodyLease();
        if (lease_owned_) {
          EmitLog("[adapter] body lease acquired");
          ApplyDesiredArmMode(true);
        } else {
          EmitLog(std::string("[adapter] failed to acquire body lease: ") + spot_.LastError());
        }
      }
    } else if (!teleop_active && lease_owned_) {
      const bool keep_lease_for_autonomy =
          IsGraphNavNavigationActive() || docking_in_progress_.load() ||
          map_recording_active_.load() || command_action_in_progress_.load();
      if (keep_lease_for_autonomy) {
        if ((now - last_nav_hold_log_ms_.load()) >= 5000) {
          last_nav_hold_log_ms_ = now;
          EmitLog("[adapter] teleop inactive but autonomy command is active; keeping lease");
        }
      } else {
        EmitLog("[adapter] teleop inactive, stopping motion and returning body lease");
        spot_.ZeroVelocity(cfg_.zero_velocity_repeats);
        moving_ = false;
        if (!WaitForArmStow(3000)) {
          EmitLog("[adapter] arm did not reach stow before lease return (timeout)");
        }
        if (!spot_.ReturnBodyLease()) {
          EmitLog(std::string("[adapter] failed returning body lease: ") + spot_.LastError());
        }
        lease_owned_ = false;
        desired_twist_valid_ = false;
        {
          std::lock_guard<std::mutex> lk(twist_cmd_mu_);
          desired_vx_ = 0.0;
          desired_vy_ = 0.0;
          desired_wz_ = 0.0;
          desired_body_pitch_ = 0.0;
        }
      }
    }

    if (lease_owned_ && teleop_active && !docking_in_progress_ &&
        SpotConnected() && !spot_degraded_non_estop_.load()) {
      double vx = 0.0;
      double vy = 0.0;
      double wz = 0.0;
      double body_pitch = 0.0;
      bool has_desired = desired_twist_valid_.load();
      const long long twist_age_ms = now - last_twist_ms_.load();
      const int stale_twist_timeout_ms = std::max(250, cfg_.teleop_idle_timeout_ms);
      if (has_desired && twist_age_ms > stale_twist_timeout_ms) {
        desired_twist_valid_ = false;
        has_desired = false;
        {
          std::lock_guard<std::mutex> lk(twist_cmd_mu_);
          desired_vx_ = 0.0;
          desired_vy_ = 0.0;
          desired_wz_ = 0.0;
          desired_body_pitch_ = 0.0;
        }
        if (moving_.load()) {
          spot_.ZeroVelocity(1);
          moving_ = false;
        }
        const long long last_timeout_log = last_twist_timeout_log_ms_.load();
        if ((now - last_timeout_log) >= 2000) {
          EmitLog("[teleop] twist stale timeout; stopping motion age_ms=" + std::to_string(twist_age_ms));
          last_twist_timeout_log_ms_ = now;
        }
      }
      {
        std::lock_guard<std::mutex> lk(twist_cmd_mu_);
        vx = desired_vx_;
        vy = desired_vy_;
        wz = desired_wz_;
        body_pitch = desired_body_pitch_;
      }

      if (!has_desired) {
        if (moving_.load()) {
          spot_.ZeroVelocity(1);
          moving_ = false;
        }
      } else if (vx == 0.0 && vy == 0.0 && wz == 0.0 && body_pitch == 0.0) {
        if (moving_.load()) {
          spot_.ZeroVelocity(1);
          moving_ = false;
        }
      } else if (!IsGraphNavNavigationActive()) {
        MotionMode mode = MotionMode::kWalk;
        switch (desired_motion_mode_.load()) {
          case kMotionModeStairs:
            mode = MotionMode::kStairs;
            break;
          case kMotionModeCrawl:
            mode = MotionMode::kCrawl;
            break;
          case kMotionModeWalk:
          default:
            mode = MotionMode::kWalk;
            break;
        }
        const int end_after_ms = std::max(400, cfg_.teleop_idle_timeout_ms + 200);
        spot_.Velocity(vx, vy, wz, end_after_ms, mode, body_pitch);
        moving_ = true;
      }
    }
  }

  spot_.RequestDockCancel();
  agent_.StopLoops();
  for (auto& t : camera_threads_) {
    if (t.joinable()) t.join();
  }
  camera_threads_.clear();
  if (localization_image_thread_.joinable()) localization_image_thread_.join();
  if (lease_thread_.joinable()) lease_thread_.join();
  if (dock_thread_.joinable()) dock_thread_.join();
  if (connection_thread_.joinable()) connection_thread_.join();
  if (command_exec_thread_.joinable()) command_exec_thread_.join();
  spot_.ZeroVelocity(cfg_.zero_velocity_repeats);
  if (lease_owned_) {
    (void)WaitForArmStow(3000);
    spot_.ReturnBodyLease();
    lease_owned_ = false;
  }
}

void Adapter::Stop() {
  running_ = false;
  spot_.RequestDockCancel();
  command_queue_cv_.notify_all();
  agent_.StopLoops();
}

void Adapter::CameraLoop(const std::string& source, const std::string& stream, int output_fps,
                         int poll_hz, bool rotate_180, bool normalize_jpeg,
                         int post_timeout_ms) {
  const int output_period_ms = std::max(1, 1000 / std::max(1, output_fps));
  const int poll_period_ms = std::max(1, 1000 / std::max(1, poll_hz));
  long long last_error_log_ms = 0;
  int failure_backoff_ms = poll_period_ms;
  std::mutex latest_mu;
  std::shared_ptr<std::string> latest_jpg;

  std::thread sender([this, &latest_mu, &latest_jpg, &stream, output_period_ms, post_timeout_ms]() {
    publish_cached_image_loop(&agent_, &running_, &latest_mu, &latest_jpg, stream,
                              output_period_ms, post_timeout_ms);
  });

  while (running_) {
    if (now_ms() < dock_cooldown_until_ms_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    if (!SpotConnected()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }
    std::string jpg;
    if (spot_.GetImageJpeg(source, &jpg)) {
      std::string prepared_jpg;
      const bool frame_ready =
          prepare_jpeg_frame(jpg, rotate_180, normalize_jpeg, &prepared_jpg);
      if (frame_ready) {
        failure_backoff_ms = poll_period_ms;
        auto frame = std::make_shared<std::string>();
        frame->swap(prepared_jpg);
        std::lock_guard<std::mutex> lk(latest_mu);
        latest_jpg = std::move(frame);
      } else {
        const long long now = now_ms();
        if ((now - last_error_log_ms) >= 2000) {
          std::cerr << "[camera] failed source=" << source
                    << " stream=" << stream
                    << " error=failed to prepare jpeg frame" << std::endl;
          last_error_log_ms = now;
        }
        failure_backoff_ms =
            std::min(2000, std::max(poll_period_ms, failure_backoff_ms * 2));
      }
    } else {
      const long long now = now_ms();
      if ((now - last_error_log_ms) >= 2000) {
        std::cerr << "[camera] failed source=" << source
                  << " stream=" << stream
                  << " error=" << spot_.LastError() << std::endl;
        last_error_log_ms = now;
      }
      failure_backoff_ms = std::min(2000, std::max(poll_period_ms, failure_backoff_ms * 2));
    }
    const int step_ms = 50;
    int slept_ms = 0;
    while (running_ && slept_ms < failure_backoff_ms) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(std::min(step_ms, failure_backoff_ms - slept_ms)));
      slept_ms += step_ms;
    }
  }
  if (sender.joinable()) sender.join();
}

void Adapter::LocalizationImageLoop(const std::string& stream, int fps, int poll_hz) {
  const int output_period_ms = std::max(1, 1000 / std::max(1, fps));
  const int poll_period_ms = std::max(200, 1000 / std::max(1, poll_hz));
  std::mutex latest_mu;
  std::shared_ptr<std::string> latest_jpg;

  auto build_frame = [fps, poll_hz](const SpotClient::LocalizationMapSnapshot* snapshot,
                                    const std::string& active_map_id,
                                    const std::string& waypoint_label,
                                    const std::string& status_title,
                                    const std::string& status_detail,
                                    std::string* out_jpg) -> bool {
    if (!out_jpg) return false;

    const cv::Scalar canvas_bg(20, 18, 16);
    const cv::Scalar panel_bg(31, 34, 40);
    const cv::Scalar panel_border(62, 70, 82);
    const cv::Scalar text_primary(236, 238, 242);
    const cv::Scalar text_secondary(168, 176, 188);
    const cv::Scalar accent_cyan(232, 196, 77);
    const cv::Scalar accent_green(109, 201, 77);
    const cv::Scalar accent_orange(77, 164, 255);
    const cv::Scalar map_unknown(45, 44, 42);
    const cv::Scalar map_free(92, 100, 110);
    const cv::Scalar map_occupied(58, 142, 244);
    const cv::Scalar robot_fill(202, 240, 255);
    const cv::Scalar robot_outline(255, 255, 255);

    cv::Mat canvas(kLocalizationImageHeight, kLocalizationImageWidth, CV_8UC3, canvas_bg);
    const int left_panel_width = 784;
    const int right_panel_width =
        kLocalizationImageWidth - left_panel_width - (2 * kLocalizationImageOuterPadding) -
        kLocalizationImagePanelGap;
    const cv::Rect left_panel(kLocalizationImageOuterPadding, kLocalizationImageOuterPadding,
                              left_panel_width,
                              kLocalizationImageHeight - (2 * kLocalizationImageOuterPadding));
    const cv::Rect right_panel(left_panel.x + left_panel.width + kLocalizationImagePanelGap,
                               kLocalizationImageOuterPadding, right_panel_width,
                               left_panel.height);
    fill_rect_alpha(&canvas, left_panel, panel_bg, 0.96);
    fill_rect_alpha(&canvas, right_panel, panel_bg, 0.96);
    cv::rectangle(canvas, left_panel, panel_border, 1, cv::LINE_AA);
    cv::rectangle(canvas, right_panel, panel_border, 1, cv::LINE_AA);

    const cv::Rect map_view(left_panel.x + 24, left_panel.y + 24, left_panel.width - 48,
                            left_panel.height - 48);
    fill_rect_alpha(&canvas, map_view, cv::Scalar(18, 22, 26), 1.0);
    cv::rectangle(canvas, map_view, cv::Scalar(50, 58, 68), 1, cv::LINE_AA);

    const bool has_live_map = snapshot && snapshot->localized && snapshot->has_seed_tform_body &&
                              snapshot->has_map && snapshot->map.width > 0 &&
                              snapshot->map.height > 0 && snapshot->map.resolution_m > 0.0;

    double span_x_m = 0.0;
    double span_y_m = 0.0;
    double yaw_deg = 0.0;
    std::string display_waypoint = waypoint_label;

    if (has_live_map) {
      const int grid_width = snapshot->map.width;
      const int grid_height = snapshot->map.height;
      const int scale_px =
          std::max(1, std::min(map_view.width / std::max(1, grid_width),
                               map_view.height / std::max(1, grid_height)));
      const int scaled_width = grid_width * scale_px;
      const int scaled_height = grid_height * scale_px;
      const cv::Rect draw_rect(map_view.x + (map_view.width - scaled_width) / 2,
                               map_view.y + (map_view.height - scaled_height) / 2, scaled_width,
                               scaled_height);

      cv::Mat grid_image(grid_height, grid_width, CV_8UC3, map_unknown);
      for (int y = 0; y < grid_height; ++y) {
        for (int x = 0; x < grid_width; ++x) {
          const size_t idx = static_cast<size_t>(x) +
                             static_cast<size_t>(grid_width) * static_cast<size_t>(y);
          const int draw_y = grid_height - 1 - y;
          const int32_t value = snapshot->map.occupancy[idx];
          cv::Vec3b* px = &grid_image.at<cv::Vec3b>(draw_y, x);
          if (value < 0) {
            *px = cv::Vec3b(static_cast<uchar>(map_unknown[0]),
                            static_cast<uchar>(map_unknown[1]),
                            static_cast<uchar>(map_unknown[2]));
          } else if (value >= 50) {
            *px = cv::Vec3b(static_cast<uchar>(map_occupied[0]),
                            static_cast<uchar>(map_occupied[1]),
                            static_cast<uchar>(map_occupied[2]));
          } else {
            *px = cv::Vec3b(static_cast<uchar>(map_free[0]),
                            static_cast<uchar>(map_free[1]),
                            static_cast<uchar>(map_free[2]));
          }
        }
      }

      cv::Mat scaled_grid;
      cv::resize(grid_image, scaled_grid, cv::Size(scaled_width, scaled_height), 0.0, 0.0,
                 cv::INTER_NEAREST);
      if (scale_px >= 3) {
        const cv::Scalar grid_line(74, 80, 88);
        for (int x = 0; x <= grid_width; x += 10) {
          const int line_x = std::min(scaled_width - 1, x * scale_px);
          cv::line(scaled_grid, cv::Point(line_x, 0), cv::Point(line_x, scaled_height - 1),
                   grid_line, 1, cv::LINE_8);
        }
        for (int y = 0; y <= grid_height; y += 10) {
          const int line_y = std::min(scaled_height - 1, y * scale_px);
          cv::line(scaled_grid, cv::Point(0, line_y), cv::Point(scaled_width - 1, line_y),
                   grid_line, 1, cv::LINE_8);
        }
      }
      scaled_grid.copyTo(canvas(draw_rect));

      const SpotClient::Pose3D grid_tform_body =
          compose_pose(inverse_pose(snapshot->map.seed_tform_grid), snapshot->seed_tform_body);
      const double yaw_rad = quaternion_yaw_rad(pose_quaternion(grid_tform_body));
      yaw_deg = yaw_rad * 180.0 / kPi;

      auto local_to_pixel = [&](double local_x_m, double local_y_m) {
        const double pixel_x = static_cast<double>(draw_rect.x) +
                               (local_x_m / snapshot->map.resolution_m) * scale_px;
        const double pixel_y = static_cast<double>(draw_rect.y + draw_rect.height) -
                               (local_y_m / snapshot->map.resolution_m) * scale_px;
        return cv::Point(cvRound(pixel_x), cvRound(pixel_y));
      };

      const double body_x = grid_tform_body.x;
      const double body_y = grid_tform_body.y;
      const double c = std::cos(yaw_rad);
      const double s = std::sin(yaw_rad);
      const std::array<cv::Point2d, 4> footprint_local = {
          cv::Point2d(0.45, 0.25), cv::Point2d(0.45, -0.25), cv::Point2d(-0.45, -0.25),
          cv::Point2d(-0.45, 0.25)};
      std::vector<cv::Point> footprint_pixels;
      footprint_pixels.reserve(footprint_local.size());
      for (const auto& corner : footprint_local) {
        const double px = body_x + c * corner.x - s * corner.y;
        const double py = body_y + s * corner.x + c * corner.y;
        footprint_pixels.push_back(local_to_pixel(px, py));
      }

      cv::fillConvexPoly(canvas, footprint_pixels, robot_fill, cv::LINE_AA);
      cv::polylines(canvas, footprint_pixels, true, robot_outline, 2, cv::LINE_AA);
      const cv::Point body_center = local_to_pixel(body_x, body_y);
      const cv::Point arrow_tip =
          local_to_pixel(body_x + c * 0.7, body_y + s * 0.7);
      cv::circle(canvas, body_center, 5, accent_cyan, cv::FILLED, cv::LINE_AA);
      cv::arrowedLine(canvas, body_center, arrow_tip, accent_green, 2, cv::LINE_AA, 0, 0.28);

      span_x_m = snapshot->map.width * snapshot->map.resolution_m;
      span_y_m = snapshot->map.height * snapshot->map.resolution_m;
      double scale_bar_m = 1.0;
      if (span_x_m >= 8.0) scale_bar_m = 2.0;
      if (span_x_m >= 16.0) scale_bar_m = 5.0;
      const int scale_bar_px =
          std::max(1, cvRound((scale_bar_m / snapshot->map.resolution_m) * scale_px));
      const cv::Point scale_start(draw_rect.x + 18, draw_rect.y + draw_rect.height - 20);
      const cv::Point scale_end(scale_start.x + scale_bar_px, scale_start.y);
      cv::line(canvas, scale_start, scale_end, text_primary, 3, cv::LINE_AA);
      cv::line(canvas, cv::Point(scale_start.x, scale_start.y - 5),
               cv::Point(scale_start.x, scale_start.y + 5), text_primary, 2, cv::LINE_AA);
      cv::line(canvas, cv::Point(scale_end.x, scale_end.y - 5),
               cv::Point(scale_end.x, scale_end.y + 5), text_primary, 2, cv::LINE_AA);
      cv::putText(canvas, format_decimal(scale_bar_m, scale_bar_m >= 5.0 ? 0 : 1) + " m",
                  cv::Point(scale_start.x, scale_start.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.48,
                  text_primary, 1, cv::LINE_AA);

      draw_chip(&canvas, cv::Point(draw_rect.x + 12, draw_rect.y + 12), "LOCAL PATCH",
                cv::Scalar(70, 52, 23), text_primary);
      draw_chip(&canvas, cv::Point(draw_rect.x + 156, draw_rect.y + 12), "LOCALIZED",
                cv::Scalar(44, 92, 40), text_primary);
      draw_chip(&canvas, cv::Point(draw_rect.x + draw_rect.width - 146, draw_rect.y + 12),
                shorten_identifier(snapshot->map.map_type, 16), cv::Scalar(58, 74, 98),
                text_primary);

      if (display_waypoint.empty()) {
        display_waypoint = shorten_identifier(snapshot->waypoint_id, 20);
      }
    } else {
      for (int offset = -map_view.height; offset < map_view.width; offset += 48) {
        cv::line(canvas, cv::Point(map_view.x + std::max(0, offset), map_view.y),
                 cv::Point(map_view.x + std::min(map_view.width, map_view.height + offset),
                           map_view.y + map_view.height),
                 cv::Scalar(42, 46, 54), 1, cv::LINE_AA);
      }
      const std::string title = status_title.empty() ? "Waiting for live localization" : status_title;
      int baseline = 0;
      const cv::Size title_size =
          cv::getTextSize(title, cv::FONT_HERSHEY_SIMPLEX, 0.95, 2, &baseline);
      const cv::Point title_pt(map_view.x + (map_view.width - title_size.width) / 2,
                               map_view.y + map_view.height / 2 - 20);
      cv::putText(canvas, title, title_pt, cv::FONT_HERSHEY_SIMPLEX, 0.95, text_primary, 2,
                  cv::LINE_AA);
      int detail_y = title_pt.y + 36;
      for (const auto& line : wrap_text(status_detail.empty() ? "Waiting for the first GraphNav occupancy patch."
                                                              : status_detail,
                                        42)) {
        cv::putText(canvas, line,
                    cv::Point(map_view.x + 52, detail_y), cv::FONT_HERSHEY_SIMPLEX, 0.58,
                    text_secondary, 1, cv::LINE_AA);
        detail_y += 26;
      }
      draw_chip(&canvas, cv::Point(map_view.x + 16, map_view.y + 16), "WAITING",
                cv::Scalar(46, 63, 92), text_primary);
    }

    int y = right_panel.y + 42;
    cv::putText(canvas, "GraphNav Local Grid", cv::Point(right_panel.x + 24, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.88, text_primary, 2, cv::LINE_AA);
    y += 34;
    cv::putText(canvas, "Rendered localization overlay for Formant", cv::Point(right_panel.x + 24, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.53, text_secondary, 1, cv::LINE_AA);
    y += 26;
    draw_chip(&canvas, cv::Point(right_panel.x + 24, y), has_live_map ? "LIVE" : "HOLD",
              has_live_map ? cv::Scalar(44, 92, 40) : cv::Scalar(46, 63, 92), text_primary);
    draw_chip(&canvas, cv::Point(right_panel.x + 104, y),
              std::to_string(std::max(1, fps)) + " FPS OUT", cv::Scalar(70, 52, 23),
              text_primary);
    draw_chip(&canvas, cv::Point(right_panel.x + 232, y),
              std::to_string(std::max(1, poll_hz)) + " HZ DATA", cv::Scalar(58, 74, 98),
              text_primary);
    y += 58;

    auto draw_section_title = [&](const std::string& title) {
      cv::putText(canvas, title, cv::Point(right_panel.x + 24, y), cv::FONT_HERSHEY_SIMPLEX,
                  0.56, accent_cyan, 1, cv::LINE_AA);
      y += 20;
    };
    auto draw_info_row = [&](const std::string& label, const std::string& value) {
      cv::putText(canvas, label, cv::Point(right_panel.x + 24, y), cv::FONT_HERSHEY_SIMPLEX, 0.48,
                  text_secondary, 1, cv::LINE_AA);
      y += 18;
      cv::putText(canvas, value.empty() ? "n/a" : value, cv::Point(right_panel.x + 24, y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.58, text_primary, 1, cv::LINE_AA);
      y += 28;
    };

    draw_section_title("Context");
    draw_info_row("Active map", active_map_id.empty() ? "none" : active_map_id);
    draw_info_row("Current waypoint", display_waypoint.empty() ? shorten_identifier(
                                                         has_live_map ? snapshot->waypoint_id : "", 28)
                                                               : display_waypoint);
    draw_info_row("Raw waypoint id",
                  has_live_map ? shorten_identifier(snapshot->waypoint_id, 28) : "n/a");

    draw_section_title("Patch");
    draw_info_row("Grid type",
                  has_live_map ? snapshot->map.map_type : shorten_identifier(status_title, 28));
    draw_info_row("Resolution",
                  has_live_map ? (format_decimal(snapshot->map.resolution_m, 3) + " m/cell")
                               : "waiting");
    draw_info_row("Extent",
                  has_live_map ? (format_decimal(span_x_m, 2) + " m x " + format_decimal(span_y_m, 2) +
                                  " m")
                               : "waiting");

    draw_section_title("Robot");
    draw_info_row("Seed pose",
                  has_live_map ? ("x=" + format_decimal(snapshot->seed_tform_body.x, 2) + ", y=" +
                                  format_decimal(snapshot->seed_tform_body.y, 2))
                               : "waiting");
    draw_info_row("Heading",
                  has_live_map ? (format_decimal(yaw_deg, 1) + " deg") : "waiting");

    draw_section_title("Legend");
    const int legend_x = right_panel.x + 24;
    auto draw_legend = [&](const cv::Scalar& color, const std::string& label) {
      cv::rectangle(canvas, cv::Rect(legend_x, y - 12, 16, 16), color, cv::FILLED, cv::LINE_AA);
      cv::rectangle(canvas, cv::Rect(legend_x, y - 12, 16, 16), panel_border, 1, cv::LINE_AA);
      cv::putText(canvas, label, cv::Point(legend_x + 28, y + 1), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  text_primary, 1, cv::LINE_AA);
      y += 26;
    };
    draw_legend(map_occupied, "Occupied / no-step");
    draw_legend(map_free, "Traversable");
    draw_legend(map_unknown, "Unknown");
    draw_legend(robot_fill, "Robot footprint");

    const std::string footer =
        has_live_map ? "This is a live local terrain patch around the robot, not a stitched global map."
                     : "The stream keeps a rendered placeholder live while waiting for GraphNav occupancy data.";
    y += 6;
    for (const auto& line : wrap_text(footer, 38)) {
      cv::putText(canvas, line, cv::Point(right_panel.x + 24, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  text_secondary, 1, cv::LINE_AA);
      y += 22;
    }

    return encode_jpeg(canvas, out_jpg);
  };

  {
    auto initial = std::make_shared<std::string>();
    build_frame(nullptr, "", "", "Waiting for live localization",
                "Waiting for the first GraphNav occupancy patch.", initial.get());
    std::lock_guard<std::mutex> lk(latest_mu);
    latest_jpg = std::move(initial);
  }

  std::thread sender([this, &latest_mu, &latest_jpg, &stream, output_period_ms]() {
    publish_cached_image_loop(&agent_, &running_, &latest_mu, &latest_jpg, stream,
                              output_period_ms, 250);
  });

  bool have_live_frame = false;
  int failure_count = 0;
  while (running_) {
    SpotClient::LocalizationMapSnapshot snapshot;
    bool use_snapshot = false;
    std::string status_title = "Waiting for live localization";
    std::string status_detail = "Waiting for the first GraphNav occupancy patch.";

    if (!SpotConnected()) {
      ++failure_count;
      status_title = "Spot disconnected";
      status_detail = "Waiting for robot connection before requesting GraphNav localization.";
    } else if (!spot_.GetLocalizationMapSnapshot(&snapshot)) {
      ++failure_count;
      status_title = "Localization fetch failed";
      status_detail = shorten_identifier(spot_.LastError(), 120);
    } else if (!snapshot.localized || !snapshot.has_seed_tform_body) {
      ++failure_count;
      status_title = "Robot not localized";
      status_detail = "GraphNav returned a waypoint without a seed-frame body pose.";
    } else if (!snapshot.has_map) {
      ++failure_count;
      status_title = "Live terrain map unavailable";
      status_detail =
          "GraphNav localization succeeded, but no supported local grid was returned yet.";
    } else {
      use_snapshot = true;
      have_live_frame = true;
      failure_count = 0;
    }

    const bool should_refresh =
        use_snapshot || !have_live_frame || failure_count >= std::max(2, std::max(1, poll_hz) * 2);
    if (should_refresh) {
      std::string active_map_id;
      std::string waypoint_label;
      {
        std::lock_guard<std::mutex> lk(map_mu_);
        active_map_id = active_map_id_;
        if (use_snapshot) {
          waypoint_label = ResolveWaypointNameForIdLocked(snapshot.waypoint_id);
        }
      }

      auto rendered = std::make_shared<std::string>();
      if (build_frame(use_snapshot ? &snapshot : nullptr, active_map_id, waypoint_label,
                      status_title, status_detail, rendered.get())) {
        std::lock_guard<std::mutex> lk(latest_mu);
        latest_jpg = std::move(rendered);
      }
    }

    const int step_ms = 50;
    int slept_ms = 0;
    while (running_ && slept_ms < poll_period_ms) {
      std::this_thread::sleep_for(std::chrono::milliseconds(
          std::min(step_ms, poll_period_ms - slept_ms)));
      slept_ms += step_ms;
    }
  }

  if (sender.joinable()) sender.join();
}

void Adapter::LeaseRetainLoop() {
  const int period_ms = 100;
  const int periodic_publish_ms = 5000;  // 0.2 Hz
  long long last_retain_ms = 0;
  long long last_can_dock_pub_ms = 0;
  long long last_status_pub_ms = 0;
  long long last_mode_pub_ms = 0;
  long long last_robot_diag_pub_ms = 0;
  long long last_map_progress_pub_ms = 0;
  long long last_localization_pub_ms = 0;
  long long last_localization_viz_pub_ms = 0;
  while (running_) {
    const long long now = now_ms();
    MaybeRestoreActiveMap(now);
    PollGraphNavNavigation(now);

    if (lease_owned_ && SpotConnected() &&
        (now - last_retain_ms) >= std::max(100, 1000 / std::max(1, cfg_.lease_retain_hz))) {
      spot_.RetainLease();
      last_retain_ms = now;
    }

    if ((now - last_can_dock_pub_ms) >= periodic_publish_ms) {
      int dock_id = resolved_dock_id_.load();
      if (dock_id <= 0) dock_id = ResolveDockId();
      bool can_dock = false;
      if (SpotConnected() && dock_id > 0) {
        bool ok = spot_.CanDock(dock_id, &can_dock);
        if (!ok) can_dock = false;
      }
      can_dock_ = can_dock;
      QueueStatusBitset(cfg_.can_dock_stream, {{"Can dock", can_dock_.load()}});
      last_can_dock_pub_ms = now;
    }

    if ((now - last_status_pub_ms) >= periodic_publish_ms) {
      QueueStatusBitset("spot.status", {
        {"Has lease", lease_owned_.load()},
        {"Robot available", SpotConnected()},
        {"Robot degraded", spot_degraded_non_estop_.load()},
        {"Teleop running", running_.load()},
        {"Teleop active", TeleopSessionActive()},
        {"Docking", docking_in_progress_.load()},
      });
      QueueStatusText("spot.connection", BuildSpotConnectionJson());
      last_status_pub_ms = now;
    }

    if ((now - last_mode_pub_ms) >= periodic_publish_ms) {
      QueueStatusBitset(cfg_.stateful_mode_stream, {
        {"Walk", desired_motion_mode_.load() == kMotionModeWalk},
        {"Stairs", desired_motion_mode_.load() == kMotionModeStairs},
        {"Crawl", desired_motion_mode_.load() == kMotionModeCrawl},
      });
      last_mode_pub_ms = now;
    }

    if ((now - last_robot_diag_pub_ms) >= periodic_publish_ms && SpotConnected()) {
      SpotClient::RobotStateSnapshot snap;
      if (spot_.GetRobotStateSnapshot(&snap)) {
        ApplySoftRecoveryForRobotState(snap);
        QueueStatusText("spot.robot_state.power", robot_state_to_json(snap));
        if (snap.has_battery_pct) {
          QueueStatusNumeric("spot.robot_state.battery", snap.battery_pct);
        }
        if (snap.has_body_pitch_rad) {
          QueueStatusNumeric("spot.robot_state.body_pitch_rad", snap.body_pitch_rad);
        }
        QueueStatusText("spot.faults.system", fault_list_to_json(snap.system_faults));
        QueueStatusText("spot.faults.behavior", fault_list_to_json(snap.behavior_faults));
        QueueStatusText("spot.faults.service", fault_list_to_json(snap.service_faults));
        PublishFaultEvents(snap);
      } else {
        std::cerr << "[state] failed to fetch robot state: " << spot_.LastError() << std::endl;
      }
      last_robot_diag_pub_ms = now;
    }

    if ((now - last_map_progress_pub_ms) >= periodic_publish_ms && SpotConnected()) {
      SpotClient::MappingStatus map_status;
      if (spot_.GetMappingStatus(&map_status)) {
        map_recording_active_ = map_status.is_recording;
        QueueStatusText("spot.map.progress", mapping_status_to_json(map_status));
        QueueStatusNumeric("spot.map.progress.waypoints",
                           static_cast<double>(map_status.waypoint_count));
        QueueStatusNumeric("spot.map.progress.path_length_m", map_status.total_path_length_m);
        QueueStatusNumeric("spot.map.progress.fiducials",
                           static_cast<double>(map_status.visible_fiducial_ids.size()));

        const std::string signature = mapping_status_signature(map_status);
        if (signature != last_map_progress_signature_) {
          last_map_progress_signature_ = signature;
          EmitLog("[graphnav] map progress: waypoints=" + std::to_string(map_status.waypoint_count) +
                  " edges=" + std::to_string(map_status.edge_count) +
                  " path_m=" + std::to_string(map_status.total_path_length_m) +
                  " fiducials=" + std::to_string(map_status.visible_fiducial_ids.size()) +
                  " recording=" + std::string(map_status.is_recording ? "true" : "false"));
        }
      } else if (map_recording_active_) {
        EmitLog(std::string("[graphnav] map progress poll failed: ") + spot_.LastError());
      }
      last_map_progress_pub_ms = now;
    }

    if ((now - last_localization_pub_ms) >= periodic_publish_ms) {
      bool localized = false;
      std::string waypoint_id;
      std::string error;
      if (SpotConnected()) {
        if (spot_.GetLocalizationWaypointId(&waypoint_id)) {
          localized = true;
        } else {
          error = spot_.LastError();
        }
      } else {
        error = "robot_disconnected";
      }
      std::ostringstream oss;
      oss << "{\"localized\":" << (localized ? "true" : "false")
          << ",\"waypoint_id\":\"" << json_escape(waypoint_id) << "\""
          << ",\"error\":\"" << json_escape(error) << "\"}";
      QueueStatusText("spot.localization", oss.str());
      last_localization_pub_ms = now;
    }

    if ((now - last_localization_viz_pub_ms) >= periodic_publish_ms && SpotConnected()) {
      SpotClient::LocalizationMapSnapshot localization_map;
      if (spot_.GetLocalizationMapSnapshot(&localization_map) &&
          localization_map.localized &&
          localization_map.has_seed_tform_body &&
          localization_map.has_map) {
        (void)agent_.PostLocalization(kGraphNavLocalizationVizStream,
                                      build_formant_localization(localization_map));
      }
      last_localization_viz_pub_ms = now;
    }

    PollCurrentWaypointAtStatus(now);

    if (lease_owned_ && TeleopSessionActive() && !docking_in_progress_) {
      ApplyDesiredArmMode(false);
    }

    PublishCurrentWaypointText(now);
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }
}

bool Adapter::IsGraphNavNavigationActive() const {
  return graphnav_navigation_active_.load();
}

bool Adapter::SpotConnected() const {
  return spot_connection_state_.load() == 2;
}

void Adapter::SetSpotConnected() {
  const bool was_connected = SpotConnected();
  spot_connection_state_ = 2;
  spot_health_failures_ = 0;
  spot_reconnect_attempt_ = 0;
  spot_degraded_non_estop_ = false;
  last_spot_connect_success_ms_ = now_ms();
  {
    std::lock_guard<std::mutex> lk(spot_connection_mu_);
    last_spot_error_.clear();
    spot_degraded_reason_.clear();
  }
  if (!was_connected) {
    EmitLog("[spot] connection established");
  }
}

void Adapter::SetSpotDisconnected(const std::string& reason) {
  const bool was_connected = SpotConnected();
  spot_connection_state_ = 0;
  lease_owned_ = false;
  moving_ = false;
  spot_degraded_non_estop_ = false;
  graphnav_navigation_active_ = false;
  last_graph_nav_command_id_ = 0;
  desired_twist_valid_ = false;
  {
    std::lock_guard<std::mutex> lk(twist_cmd_mu_);
    desired_vx_ = 0.0;
    desired_vy_ = 0.0;
    desired_wz_ = 0.0;
    desired_body_pitch_ = 0.0;
  }
  {
    std::lock_guard<std::mutex> lk(spot_connection_mu_);
    last_spot_error_ = reason;
    spot_degraded_reason_.clear();
  }
  ResetFaultEventsState();
  if (was_connected) {
    EmitLog(std::string("[spot] connection lost: ") + reason);
  }
}

std::string Adapter::BuildSpotConnectionJson() const {
  std::string state = "disconnected";
  const int raw_state = spot_connection_state_.load();
  if (raw_state == 1) {
    state = "connecting";
  } else if (raw_state == 2) {
    state = "connected";
  }
  std::string err;
  std::string degraded_reason;
  {
    std::lock_guard<std::mutex> lk(spot_connection_mu_);
    err = last_spot_error_;
    degraded_reason = spot_degraded_reason_;
  }
  std::ostringstream oss;
  oss << "{\"state\":\"" << state
      << "\",\"connected\":" << (SpotConnected() ? "true" : "false")
      << ",\"degraded_non_estop\":" << (spot_degraded_non_estop_.load() ? "true" : "false")
      << ",\"reconnect_attempt\":" << spot_reconnect_attempt_.load()
      << ",\"last_success_ms\":" << last_spot_connect_success_ms_.load()
      << ",\"last_attempt_ms\":" << last_spot_connect_attempt_ms_.load()
      << ",\"error\":\"" << json_escape(err) << "\""
      << ",\"degraded_reason\":\"" << json_escape(degraded_reason) << "\"}";
  return oss.str();
}

void Adapter::ApplySoftRecoveryForRobotState(const SpotClient::RobotStateSnapshot& snap) {
  std::string reason;
  if (!snap.any_estopped) {
    const bool has_critical_system_fault = any_fault_requires_soft_recovery(snap.system_faults);
    const bool has_critical_service_fault = any_fault_requires_soft_recovery(snap.service_faults);
    const bool has_unclearable_behavior_fault = any_fault_requires_soft_recovery(snap.behavior_faults);
    if (has_critical_system_fault) {
      reason = "critical_system_fault";
    } else if (has_critical_service_fault) {
      reason = "critical_service_fault";
    } else if (has_unclearable_behavior_fault) {
      reason = "unclearable_behavior_fault";
    } else if (snap.motor_power_state == "error") {
      reason = "motor_power_error";
    }
  }

  const bool degraded = !reason.empty();
  spot_degraded_non_estop_ = degraded;
  {
    std::lock_guard<std::mutex> lk(spot_connection_mu_);
    spot_degraded_reason_ = reason;
  }
  if (!degraded) return;

  desired_twist_valid_ = false;
  {
    std::lock_guard<std::mutex> lk(twist_cmd_mu_);
    desired_vx_ = 0.0;
    desired_vy_ = 0.0;
    desired_wz_ = 0.0;
    desired_body_pitch_ = 0.0;
  }
  if (moving_.load()) {
    spot_.ZeroVelocity(1);
    moving_ = false;
  }

  const long long now = now_ms();
  const long long last_log = last_soft_recovery_log_ms_.load();
  if ((now - last_log) >= 3000) {
    EmitLog("[safety] soft recovery active (non-estop): " + reason +
            " ; motion commands are gated");
    last_soft_recovery_log_ms_ = now;
  }
}

void Adapter::ResetFaultEventsState() {
  std::lock_guard<std::mutex> lk(fault_events_mu_);
  last_fault_events_.clear();
}

void Adapter::PublishFaultEvents(const SpotClient::RobotStateSnapshot& snap) {
  std::unordered_map<std::string, SpotClient::FaultInfo> current_faults;
  add_faults_for_events("system", snap.system_faults, &current_faults);
  add_faults_for_events("behavior", snap.behavior_faults, &current_faults);
  add_faults_for_events("service", snap.service_faults, &current_faults);

  std::unordered_map<std::string, SpotClient::FaultInfo> previous_faults;
  {
    std::lock_guard<std::mutex> lk(fault_events_mu_);
    previous_faults = last_fault_events_;
    last_fault_events_ = current_faults;
  }

  std::vector<std::string> opened_keys;
  std::vector<std::string> changed_keys;
  std::vector<std::string> cleared_keys;
  opened_keys.reserve(current_faults.size());
  changed_keys.reserve(current_faults.size());
  cleared_keys.reserve(previous_faults.size());

  for (const auto& kv : current_faults) {
    const auto it = previous_faults.find(kv.first);
    if (it == previous_faults.end()) {
      opened_keys.push_back(kv.first);
      continue;
    }
    const auto& prev = it->second;
    const auto& cur = kv.second;
    if (prev.severity != cur.severity || prev.message != cur.message) {
      changed_keys.push_back(kv.first);
    }
  }
  for (const auto& kv : previous_faults) {
    if (current_faults.find(kv.first) == current_faults.end()) {
      cleared_keys.push_back(kv.first);
    }
  }

  std::sort(opened_keys.begin(), opened_keys.end(), fault_event_key_order_less);
  std::sort(changed_keys.begin(), changed_keys.end(), fault_event_key_order_less);
  std::sort(cleared_keys.begin(), cleared_keys.end(), fault_event_key_order_less);

  std::vector<std::string> lines;
  lines.reserve(opened_keys.size() + changed_keys.size() + cleared_keys.size());

  for (const auto& key : opened_keys) {
    const auto& cur = current_faults[key];
    const std::string fault_type = fault_type_from_key(key);
    std::string line = format_fault_event_prefix("OPEN", fault_type, cur);
    if (!cur.message.empty()) line += " " + cur.message;
    lines.push_back(std::move(line));
  }
  for (const auto& key : changed_keys) {
    const auto& prev = previous_faults[key];
    const auto& cur = current_faults[key];
    const std::string fault_type = fault_type_from_key(key);
    std::string line = "FAULT CHANGED [" + prev.severity + "->" + cur.severity + "] " +
                       fault_type + ":" + cur.id;
    if (!cur.message.empty()) line += " " + cur.message;
    lines.push_back(std::move(line));
  }
  for (const auto& key : cleared_keys) {
    const std::string fault_type = fault_type_from_key(key);
    const std::string fault_id = fault_id_from_key(key);
    lines.push_back("FAULT CLEARED " + fault_type + ":" + fault_id);
  }

  if (lines.empty()) return;
  std::ostringstream payload;
  for (size_t i = 0; i < lines.size(); ++i) {
    payload << lines[i];
    if (i + 1 < lines.size()) payload << "\n";
  }
  QueueStatusText("spot.fault.events", payload.str());
}

void Adapter::ConnectionLoop() {
  while (running_) {
    const long long now = now_ms();
    if (!SpotConnected()) {
      if (now < next_spot_connect_attempt_ms_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      spot_connection_state_ = 1;
      const int attempt = spot_reconnect_attempt_.load() + 1;
      spot_reconnect_attempt_ = attempt;
      last_spot_connect_attempt_ms_ = now;
      if (spot_.Connect(cfg_.spot_host, cfg_.spot_username, cfg_.spot_password)) {
        SetSpotConnected();
        continue;
      }
      const std::string err = spot_.LastError();
      SetSpotDisconnected(err);
      const int backoff_ms = std::min(30000, 1000 * (1 << std::min(5, std::max(0, attempt - 1))));
      next_spot_connect_attempt_ms_ = now + backoff_ms;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    if ((now - last_spot_healthcheck_ms_.load()) >= 2000) {
      last_spot_healthcheck_ms_ = now;
      SpotClient::RobotStateSnapshot snap;
      if (spot_.GetRobotStateSnapshot(&snap)) {
        spot_health_failures_ = 0;
      } else {
        const int failures = spot_health_failures_.load() + 1;
        spot_health_failures_ = failures;
        if (failures >= 3) {
          SetSpotDisconnected(std::string("health check failed: ") + spot_.LastError());
          next_spot_connect_attempt_ms_ = now + 1000;
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void Adapter::MaybeRestoreActiveMap(long long now) {
  if (!SpotConnected()) return;
  if (now < next_map_restore_attempt_ms_.load()) return;

  std::string restore_map_id;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    restore_map_id = pending_restore_map_id_;
  }
  if (restore_map_id.empty()) return;

  bool temporary_lease = false;
  if (!lease_owned_.load()) {
    if (!spot_.TryAcquireBodyLeaseNoTakeover()) {
      EmitLog(std::string("[graphnav] deferred restore waiting for lease map_id=") +
              restore_map_id + " error=" + spot_.LastError());
      next_map_restore_attempt_ms_ = now + 5000;
      return;
    }
    temporary_lease = true;
  }

  SpotClient::StoredMap map_data;
  if (!LoadMapFromDisk(restore_map_id, &map_data)) {
    EmitLog(std::string("[graphnav] deferred restore failed reading map_id=") + restore_map_id);
    if (temporary_lease) (void)spot_.ReturnBodyLease();
    next_map_restore_attempt_ms_ = now + 5000;
    return;
  }
  if (!spot_.UploadGraphMap(map_data)) {
    EmitLog(std::string("[graphnav] deferred restore upload failed map_id=") + restore_map_id +
            " error=" + spot_.LastError());
    if (temporary_lease) (void)spot_.ReturnBodyLease();
    next_map_restore_attempt_ms_ = now + 5000;
    return;
  }
  if (!spot_.SetLocalizationFiducial()) {
    EmitLog(std::string("[graphnav] deferred restore localization warning map_id=") +
            restore_map_id + " error=" + spot_.LastError());
  }
  if (temporary_lease && !spot_.ReturnBodyLease()) {
    EmitLog(std::string("[graphnav] deferred restore warning returning temporary lease map_id=") +
            restore_map_id + " error=" + spot_.LastError());
  }

  {
    std::lock_guard<std::mutex> lk(map_mu_);
    pending_restore_map_id_.clear();
  }
  next_map_restore_attempt_ms_ = 0;
  RefreshGraphWaypointIndex();
  force_waypoint_publish_ = true;
  force_maps_publish_ = true;
  EmitLog(std::string("[graphnav] restored active map: ") + restore_map_id);
}

void Adapter::PollGraphNavNavigation(long long now) {
  if (!SpotConnected()) return;
  if ((now - last_nav_feedback_poll_ms_.load()) < 1000) return;
  last_nav_feedback_poll_ms_ = now;

  const uint32_t command_id = last_graph_nav_command_id_.load();
  if (command_id == 0) {
    graphnav_navigation_active_ = false;
    return;
  }

  SpotClient::NavigationFeedbackSnapshot fb;
  if (!spot_.GetNavigationFeedbackSnapshot(command_id, &fb)) {
    return;
  }
  const int status = fb.status;

  QueueStatusText("spot.nav.feedback", nav_feedback_to_json(command_id, fb));

  const std::string feedback_sig = nav_feedback_signature(fb);
  bool changed = false;
  {
    std::lock_guard<std::mutex> lk(nav_state_mu_);
    changed = (feedback_sig != last_nav_feedback_signature_);
    if (changed) last_nav_feedback_signature_ = feedback_sig;
  }
  if (changed) {
    EmitLog("[graphnav] nav feedback cmd_id=" + std::to_string(command_id) +
            " status=" + std::to_string(fb.status) + "(" + nav_status_name(fb.status) + ")" +
            " rem_m=" + std::to_string(fb.remaining_route_length) +
            " route_mode=" + std::to_string(fb.route_following_status) +
            " blocked=" + std::to_string(fb.blockage_status) +
            " goal=" + std::to_string(fb.goal_status) +
            " body_move=" + std::to_string(fb.body_movement_status));
  }

  if (nav_status_in_progress(status)) {
    graphnav_navigation_active_ = true;
    long long stale_ms = 0;
    bool should_log_pause = false;
    {
      std::lock_guard<std::mutex> lk(nav_state_mu_);
      if (last_nav_status_ != status) {
        last_nav_status_ = status;
        last_nav_progress_change_ms_ = now;
        last_nav_remaining_route_m_ = fb.remaining_route_length;
      } else {
        if ((last_nav_remaining_route_m_ - fb.remaining_route_length) > 0.15) {
          last_nav_remaining_route_m_ = fb.remaining_route_length;
          last_nav_progress_change_ms_ = now;
        }
        stale_ms = now - last_nav_progress_change_ms_;
        should_log_pause = (stale_ms >= 8000);
      }
    }
    if (should_log_pause && (now - last_nav_diag_log_ms_.load()) >= 4000) {
      last_nav_diag_log_ms_ = now;
      EmitLog("[graphnav] navigation pause detected cmd_id=" + std::to_string(command_id) +
              " stale_ms=" + std::to_string(stale_ms) +
              " rem_m=" + std::to_string(fb.remaining_route_length) +
              " route_mode=" + std::to_string(fb.route_following_status) +
              " blocked=" + std::to_string(fb.blockage_status) +
              " waiting_regions=" + std::to_string(fb.waiting_region_count) +
              " callback_in_control_regions=" +
              std::to_string(fb.callback_in_control_region_count));
      SpotClient::RobotStateSnapshot snap;
      if (spot_.GetRobotStateSnapshot(&snap)) {
        EmitLog("[graphnav] navigation pause robot_state=" + robot_state_to_json(snap));
        EmitLog("[graphnav] navigation pause system_faults=" + fault_list_to_json(snap.system_faults));
        EmitLog("[graphnav] navigation pause behavior_faults=" + fault_list_to_json(snap.behavior_faults));
        EmitLog("[graphnav] navigation pause service_faults=" + fault_list_to_json(snap.service_faults));
      } else {
        EmitLog(std::string("[graphnav] navigation pause robot state unavailable: ") + spot_.LastError());
      }
    }
    return;
  }

  const auto clear_nav_state = [this]() {
    std::lock_guard<std::mutex> lk(nav_state_mu_);
    last_nav_feedback_signature_.clear();
    last_nav_progress_change_ms_ = 0;
    last_nav_remaining_route_m_ = 0.0;
    last_nav_status_ = 0;
  };

  graphnav_navigation_active_ = false;
  using Resp = ::bosdyn::api::graph_nav::NavigationFeedbackResponse;
  if (status == Resp::STATUS_REACHED_GOAL) {
    EmitLog("[graphnav] navigation reached goal");
    last_graph_nav_command_id_ = 0;
    nav_auto_recovered_ = false;
    clear_nav_state();
    return;
  }

  if (status == Resp::STATUS_ROBOT_IMPAIRED) {
    const bool already_recovered = nav_auto_recovered_.exchange(true);
    std::string waypoint_id;
    std::string waypoint_name;
    {
      std::lock_guard<std::mutex> lk(map_mu_);
      waypoint_id = nav_target_waypoint_id_;
      waypoint_name = nav_target_waypoint_name_;
    }
    if (!already_recovered && !waypoint_id.empty()) {
      EmitLog("[graphnav] navigation impaired, auto-recovering and retrying");
      if (!spot_.RecoverSelfRight()) {
        EmitLog(std::string("[graphnav] auto-recover failed: ") + spot_.LastError());
        last_graph_nav_command_id_ = 0;
        clear_nav_state();
        return;
      }
      (void)spot_.Stand();

      uint32_t retry_command_id = 0;
      if (spot_.NavigateToWaypoint(waypoint_id, cfg_.graphnav_command_timeout_sec, &retry_command_id)) {
        last_graph_nav_command_id_ = retry_command_id;
        graphnav_navigation_active_ = true;
        EmitLog(std::string("[graphnav] retrying waypoint after recover name=") + waypoint_name +
                " id=" + waypoint_id);
        return;
      }
      EmitLog(std::string("[graphnav] waypoint retry after recover failed: ") + spot_.LastError());
    }
    last_graph_nav_command_id_ = 0;
    clear_nav_state();
    return;
  }

  EmitLog(std::string("[graphnav] navigation ended status=") + std::to_string(status) +
          " (" + nav_status_name(status) + ")");
  last_graph_nav_command_id_ = 0;
  nav_auto_recovered_ = false;
  clear_nav_state();
}

void Adapter::QueueStatusText(const std::string& stream, const std::string& value) {
  if (stream.empty()) return;
  std::lock_guard<std::mutex> lk(status_queue_mu_);
  auto& dp = status_queue_[stream];
  dp.kind = PendingStatusDatapoint::Kind::kText;
  dp.text_value = value;
  dp.pending = true;
}

void Adapter::QueueStatusNumeric(const std::string& stream, double value) {
  if (stream.empty()) return;
  std::lock_guard<std::mutex> lk(status_queue_mu_);
  auto& dp = status_queue_[stream];
  dp.kind = PendingStatusDatapoint::Kind::kNumeric;
  dp.numeric_value = value;
  dp.pending = true;
}

void Adapter::QueueStatusBitset(const std::string& stream,
                                const std::vector<std::pair<std::string, bool>>& bits) {
  if (stream.empty()) return;
  std::lock_guard<std::mutex> lk(status_queue_mu_);
  auto& dp = status_queue_[stream];
  dp.kind = PendingStatusDatapoint::Kind::kBitset;
  dp.bitset_value = bits;
  dp.pending = true;
}

void Adapter::FlushQueuedStatus(long long now) {
  if ((now - last_status_flush_ms_.load()) < 1000) return;

  struct Outgoing {
    std::string stream;
    PendingStatusDatapoint::Kind kind;
    std::string text_value;
    double numeric_value;
    std::vector<std::pair<std::string, bool>> bitset_value;
  };
  std::vector<Outgoing> out;
  out.reserve(16);

  {
    std::lock_guard<std::mutex> lk(status_queue_mu_);
    for (const auto& kv : status_queue_) {
      const auto& stream = kv.first;
      const auto& dp = kv.second;
      if (!dp.pending) continue;
      if (now < dp.retry_after_ms) continue;
      Outgoing o;
      o.stream = stream;
      o.kind = dp.kind;
      o.text_value = dp.text_value;
      o.numeric_value = dp.numeric_value;
      o.bitset_value = dp.bitset_value;
      out.push_back(std::move(o));
    }
  }

  for (const auto& o : out) {
    bool ok = false;
    if (o.kind == PendingStatusDatapoint::Kind::kText) {
      ok = agent_.PostText(o.stream, o.text_value);
    } else if (o.kind == PendingStatusDatapoint::Kind::kNumeric) {
      ok = agent_.PostNumeric(o.stream, o.numeric_value);
    } else {
      ok = agent_.PostBitset(o.stream, o.bitset_value);
    }

    std::lock_guard<std::mutex> lk(status_queue_mu_);
    auto it = status_queue_.find(o.stream);
    if (it == status_queue_.end()) continue;
    auto& dp = it->second;
    if (ok) {
      dp.pending = false;
      dp.backoff_ms = 1000;
      dp.retry_after_ms = 0;
    } else {
      const int cur = std::max(250, dp.backoff_ms);
      dp.retry_after_ms = now + cur;
      dp.backoff_ms = std::min(10000, cur * 2);
    }
  }

  last_status_flush_ms_ = now;
}

void Adapter::EmitLog(const std::string& msg) {
  std::cerr << msg << std::endl;
  const long long ts = now_ms();
  std::lock_guard<std::mutex> lk(adapter_log_mu_);
  adapter_log_buffer_.emplace_back(std::to_string(ts) + " " + msg);
  while (adapter_log_buffer_.size() > 200) {
    adapter_log_buffer_.pop_front();
  }
}

void Adapter::FlushAdapterLogStream(long long now) {
  static constexpr size_t kMaxPendingLogBytes = 64 * 1024;
  if (now < adapter_log_retry_after_ms_.load()) return;
  if ((now - last_adapter_log_pub_ms_.load()) < 1000) return;

  std::string payload;
  {
    std::lock_guard<std::mutex> lk(adapter_log_mu_);
    if (adapter_log_pending_payload_.empty() && adapter_log_buffer_.empty()) {
      last_adapter_log_pub_ms_ = now;
      return;
    }
    if (!adapter_log_buffer_.empty()) {
      if (!adapter_log_pending_payload_.empty() &&
          adapter_log_pending_payload_.back() != '\n') {
        adapter_log_pending_payload_ += "\n";
      }
      for (size_t i = 0; i < adapter_log_buffer_.size(); ++i) {
        adapter_log_pending_payload_ += adapter_log_buffer_[i];
        if (i + 1 < adapter_log_buffer_.size()) adapter_log_pending_payload_ += "\n";
      }
      adapter_log_buffer_.clear();
    }
    if (adapter_log_pending_payload_.size() > kMaxPendingLogBytes) {
      const size_t dropped = adapter_log_pending_payload_.size() - kMaxPendingLogBytes;
      const std::string marker =
          "[adapter] log buffer truncated dropped_bytes=" + std::to_string(dropped) + "\n";
      const size_t tail_keep = (marker.size() < kMaxPendingLogBytes) ?
          (kMaxPendingLogBytes - marker.size()) : 0;
      if (tail_keep > 0 && adapter_log_pending_payload_.size() > tail_keep) {
        adapter_log_pending_payload_ = marker +
            adapter_log_pending_payload_.substr(adapter_log_pending_payload_.size() - tail_keep);
      } else {
        adapter_log_pending_payload_ = marker.substr(0, kMaxPendingLogBytes);
      }
    }
    payload = adapter_log_pending_payload_;
  }

  const bool ok = agent_.PostText("spot.adapter.log", payload);
  if (ok) {
    std::lock_guard<std::mutex> lk(adapter_log_mu_);
    adapter_log_pending_payload_.clear();
    adapter_log_backoff_ms_ = 1000;
    adapter_log_retry_after_ms_ = 0;
    last_adapter_log_pub_ms_ = now;
    return;
  }

  const int cur_backoff = adapter_log_backoff_ms_.load();
  const int next_backoff = std::min(10000, cur_backoff * 2);
  adapter_log_backoff_ms_ = next_backoff;
  adapter_log_retry_after_ms_ = now + cur_backoff;
  last_adapter_log_pub_ms_ = now;
}

void Adapter::HandleTeleop(const v1::model::ControlDatapoint& dp) {
  static std::unordered_map<std::string, long long> last_non_bitset_press_ms;
  control_seen_ = true;
  const long long now = now_ms();
  last_control_ms_ = now;
  if (dp.stream() != last_logged_control_stream_) {
    EmitLog(std::string("[teleop] control stream changed to ") + dp.stream());
    last_logged_control_stream_ = dp.stream();
  }

  const bool dedicated_button_stream =
      (dp.stream() == cfg_.stand_button_stream || dp.stream() == cfg_.sit_button_stream ||
       dp.stream() == cfg_.estop_button_stream || dp.stream() == cfg_.recover_button_stream ||
       dp.stream() == cfg_.walk_button_stream || dp.stream() == cfg_.stairs_button_stream ||
       dp.stream() == cfg_.crawl_button_stream || dp.stream() == cfg_.dock_button_stream ||
       dp.stream() == cfg_.reset_arm_button_stream);
  if (dp.has_bitset()) {
    HandleButtons(dp.bitset(), dp.stream());
    return;
  }

  if (dedicated_button_stream) {
    // Some teleop UIs publish dedicated stream events without bitset payload.
    const long long last_ms = last_non_bitset_press_ms[dp.stream()];
    if ((now - last_ms) >= 300) {
      last_non_bitset_press_ms[dp.stream()] = now;
      HandleButtonPress(dp.stream(), dp.stream());
    }
    return;
  }

  if (dp.stream() == cfg_.teleop_twist_stream && dp.has_twist()) {
    HandleTwist(dp.twist());
    return;
  }
}

void Adapter::HandleHeartbeat(const v1::agent::GetTeleopHeartbeatStreamResponse& hb) {
  (void)hb;
  heartbeat_seen_ = true;
  MarkHeartbeat();
}

void Adapter::HandleCommand(const v1::model::CommandRequest& request) {
  bool reject = false;
  {
    std::lock_guard<std::mutex> lk(command_queue_mu_);
    if (command_queue_.size() >= kMaxCommandQueueDepth) {
      reject = true;
    } else {
      command_queue_.push_back(request);
    }
  }
  if (reject) {
    EmitLog("[command] queue full; auto-failing command=" + request.command() +
            " id=" + request.id());
    (void)agent_.SendCommandResponse(request.id(), false);
    return;
  }
  command_queue_cv_.notify_one();
}

void Adapter::CommandExecutionLoop() {
  while (running_.load()) {
    v1::model::CommandRequest request;
    {
      std::unique_lock<std::mutex> lk(command_queue_mu_);
      command_queue_cv_.wait(lk, [this]() { return !running_.load() || !command_queue_.empty(); });
      if (!running_.load() && command_queue_.empty()) break;
      request = command_queue_.front();
      command_queue_.pop_front();
    }

    command_action_in_progress_ = true;
    const bool ok = ExecuteQueuedCommand(request);
    command_action_in_progress_ = false;
    agent_.SendCommandResponse(request.id(), ok);
  }
}

bool Adapter::ExecuteCameraCalibrateCommand() {
  SpotClient::RobotStateSnapshot snap;
  if (!spot_.GetRobotStateSnapshot(&snap)) {
    EmitLog(std::string("[command] spot.camera.calibrate preflight failed: ") + spot_.LastError());
    return false;
  }
  const auto preflight_errors = camera_calibration_preflight_errors(snap);
  if (!preflight_errors.empty()) {
    std::ostringstream oss;
    for (size_t i = 0; i < preflight_errors.size(); ++i) {
      if (i > 0) oss << "; ";
      oss << preflight_errors[i];
    }
    EmitLog("[command] spot.camera.calibrate rejected: " + oss.str());
    return false;
  }

  if (!EnsureLeaseForCommand("spot.camera.calibrate")) return false;
  EmitLog("[command] spot.camera.calibrate requested");
  if (!spot_.StartCameraCalibration()) {
    EmitLog(std::string("[command] spot.camera.calibrate failed: ") + spot_.LastError());
    return false;
  }

  // Wait for terminal camera calibration feedback (1 Hz poll).
  const long long deadline_ms = now_ms() + 1200000;
  while (running_.load()) {
    if (now_ms() > deadline_ms) {
      EmitLog("[command] spot.camera.calibrate failed: timeout");
      return false;
    }
    int status = 0;
    float progress = 0.0f;
    if (!spot_.GetCameraCalibrationFeedback(&status, &progress)) {
      EmitLog(std::string("[command] spot.camera.calibrate feedback failed: ") + spot_.LastError());
      return false;
    }
    if (status == 1) {  // STATUS_PROCESSING
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }
    if (status == 2) {  // STATUS_SUCCESS
      EmitLog("[command] spot.camera.calibrate success");
      return true;
    }
    EmitLog("[command] spot.camera.calibrate failed status=" + std::to_string(status) +
            " (" + camera_calibration_status_to_string(status) + ")" +
            " progress=" + std::to_string(progress));
    return false;
  }
  return false;
}

bool Adapter::WaitForGraphNavCommandResult(uint32_t command_id, long long timeout_ms) {
  if (command_id == 0) return false;
  using Resp = ::bosdyn::api::graph_nav::NavigationFeedbackResponse;
  const long long deadline = now_ms() + timeout_ms;
  while (running_.load()) {
    if (now_ms() > deadline) {
      EmitLog("[command] graphnav command timeout");
      return false;
    }
    SpotClient::NavigationFeedbackSnapshot fb;
    if (!spot_.GetNavigationFeedbackSnapshot(command_id, &fb)) {
      EmitLog(std::string("[command] graphnav feedback failed: ") + spot_.LastError());
      return false;
    }
    QueueStatusText("spot.nav.feedback", nav_feedback_to_json(command_id, fb));
    if (fb.status == Resp::STATUS_REACHED_GOAL) return true;
    if (fb.status != Resp::STATUS_FOLLOWING_ROUTE) {
      EmitLog("[command] graphnav ended status=" + std::to_string(fb.status) +
              " (" + nav_status_name(fb.status) + ")");
      return false;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return false;
}

bool Adapter::EnsureLocalizedForNavigation(const std::string& action_name) {
  std::string waypoint_id;
  if (spot_.GetLocalizationWaypointId(&waypoint_id)) return true;

  EmitLog("[graphnav] " + action_name +
          " localization missing; attempting SetLocalization(fiducial)");
  if (!spot_.SetLocalizationFiducial()) {
    EmitLog("[graphnav] " + action_name + " relocalization failed: " + spot_.LastError());
    return false;
  }
  if (!spot_.GetLocalizationWaypointId(&waypoint_id)) {
    EmitLog("[graphnav] " + action_name + " relocalization did not produce waypoint: " +
            spot_.LastError());
    return false;
  }
  EmitLog("[graphnav] " + action_name + " relocalized to waypoint_id=" + waypoint_id);
  return true;
}

bool Adapter::StartNavigateWithRecovery(const std::string& action_name,
                                        const std::string& waypoint_id,
                                        uint32_t* out_command_id,
                                        bool straight) {
  if (!EnsureLocalizedForNavigation(action_name)) return false;

  bool retried_localization = false;
  bool retried_recover = false;
  for (;;) {
    uint32_t command_id = 0;
    const bool nav_ok =
        straight ? spot_.NavigateToWaypointStraight(waypoint_id, cfg_.graphnav_command_timeout_sec, &command_id)
                 : spot_.NavigateToWaypoint(waypoint_id, cfg_.graphnav_command_timeout_sec, &command_id);
    if (nav_ok) {
      if (out_command_id) *out_command_id = command_id;
      return true;
    }

    const std::string nav_error = spot_.LastError();
    if (is_not_localized_error_text(nav_error) && !retried_localization) {
      retried_localization = true;
      EmitLog("[graphnav] " + action_name + " navigate failed: not localized; relocalizing");
      if (!EnsureLocalizedForNavigation(action_name)) return false;
      continue;
    }
    if (is_robot_impaired_error_text(nav_error) && !retried_recover) {
      retried_recover = true;
      EmitLog("[graphnav] " + action_name + " navigate detected robot impairment, auto-recovering");
      if (!spot_.RecoverSelfRight()) {
        EmitLog("[graphnav] " + action_name + " auto-recover failed: " + spot_.LastError());
        return false;
      }
      (void)spot_.Stand();
      continue;
    }

    EmitLog("[graphnav] " + action_name + " navigate failed: " + nav_error);
    return false;
  }
}

bool Adapter::ExecuteWaypointGotoCommand(const v1::model::CommandRequest& request) {
  if (!EnsureLeaseForCommand("spot.waypoint.goto")) return false;
  std::string text;
  (void)ExtractCommandText(request, &text);
  std::string name = Trim(ExtractParam(text, {"name", "waypoint", "waypoint_name", "alias"}));
  if (name.empty()) {
    EmitLog("[graphnav] spot.waypoint.goto rejected: missing waypoint name");
    return false;
  }

  std::string waypoint_id;
    {
      std::lock_guard<std::mutex> lk(map_mu_);
      if (!ResolveWaypointNameLocked(name, &waypoint_id)) {
        EmitLog(std::string("[graphnav] spot.waypoint.goto failed: unresolved or ambiguous name=") + name);
        return false;
      }
    }

  uint32_t command_id = 0;
  if (!StartNavigateWithRecovery("spot.waypoint.goto", waypoint_id, &command_id)) return false;

  {
    std::lock_guard<std::mutex> lk(map_mu_);
    nav_target_waypoint_id_ = waypoint_id;
    nav_target_waypoint_name_ = name;
  }
  last_graph_nav_command_id_ = command_id;
  graphnav_navigation_active_ = true;
  nav_auto_recovered_ = false;
  const bool ok = WaitForGraphNavCommandResult(
      command_id, static_cast<long long>(std::max(30, cfg_.graphnav_command_timeout_sec)) * 1000LL + 30000LL);
  graphnav_navigation_active_ = false;
  last_graph_nav_command_id_ = 0;
  if (ok) EmitLog(std::string("[graphnav] spot.waypoint.goto success name=") + name);
  return ok;
}

bool Adapter::ExecuteWaypointGotoStraightCommand(const v1::model::CommandRequest& request) {
  if (!EnsureLeaseForCommand("spot.waypoint.goto_straight")) return false;
  std::string text;
  (void)ExtractCommandText(request, &text);
  std::string name = Trim(ExtractParam(text, {"name", "waypoint", "waypoint_name", "alias"}));
  if (name.empty()) {
    EmitLog("[graphnav] spot.waypoint.goto_straight rejected: missing waypoint name");
    return false;
  }

  std::string waypoint_id;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    if (!ResolveWaypointNameLocked(name, &waypoint_id)) {
      EmitLog(std::string("[graphnav] spot.waypoint.goto_straight failed: unresolved or ambiguous name=") + name);
      return false;
    }
  }

  uint32_t command_id = 0;
  if (!StartNavigateWithRecovery("spot.waypoint.goto_straight", waypoint_id, &command_id, true)) return false;

  {
    std::lock_guard<std::mutex> lk(map_mu_);
    nav_target_waypoint_id_ = waypoint_id;
    nav_target_waypoint_name_ = name;
  }
  last_graph_nav_command_id_ = command_id;
  graphnav_navigation_active_ = true;
  nav_auto_recovered_ = false;
  const bool ok = WaitForGraphNavCommandResult(
      command_id, static_cast<long long>(std::max(30, cfg_.graphnav_command_timeout_sec)) * 1000LL + 30000LL);
  graphnav_navigation_active_ = false;
  last_graph_nav_command_id_ = 0;
  if (ok) EmitLog(std::string("[graphnav] spot.waypoint.goto_straight success name=") + name);
  return ok;
}

bool Adapter::ExecuteDockSequence(bool return_and_dock) {
  if (docking_in_progress_.load()) {
    EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") +
            " request ignored: dock already in progress");
    return false;
  }
  if (!SpotConnected()) {
    EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") + " failed: robot unavailable");
    return false;
  }
  if (spot_degraded_non_estop_.load()) {
    EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") +
            " failed: robot degraded (non-estop)");
    return false;
  }
  if (!EnsureLeaseForCommand(return_and_dock ? "spot.return_and_dock" : "spot.dock")) return false;

  if (return_and_dock) {
    std::string dock_waypoint_id;
    std::string map_id;
    {
      std::lock_guard<std::mutex> lk(map_mu_);
      if (!ResolveSavedDockHomeLocked(&map_id, &dock_waypoint_id)) {
        EmitLog("[return_and_dock] failed: no saved docking waypoint for active map");
        return false;
      }
    }
    EmitLog("[return_and_dock] navigating to dock waypoint map_id=" + map_id +
            " waypoint_id=" + dock_waypoint_id);
    uint32_t nav_command_id = 0;
    if (!StartNavigateWithRecovery("spot.return_and_dock", dock_waypoint_id, &nav_command_id)) return false;
    last_graph_nav_command_id_ = nav_command_id;
    graphnav_navigation_active_ = true;
    {
      std::lock_guard<std::mutex> lk(map_mu_);
      nav_target_waypoint_id_ = dock_waypoint_id;
      nav_target_waypoint_name_ = "dock_home";
    }
    const bool nav_ok = WaitForGraphNavCommandResult(
        nav_command_id, static_cast<long long>(std::max(30, cfg_.graphnav_command_timeout_sec)) * 1000LL + 30000LL);
    graphnav_navigation_active_ = false;
    last_graph_nav_command_id_ = 0;
    if (!nav_ok) {
      EmitLog("[return_and_dock] failed: navigation did not reach dock waypoint");
      return false;
    }
    EmitLog("[return_and_dock] reached dock waypoint; starting dock");
  } else {
    CaptureDockWaypointCandidate();
  }

  int dock_id = ResolveDockId();
  if (dock_id <= 0) {
    EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") +
            " no dock id resolved: " + spot_.LastError());
    return false;
  }

  bool can_dock = false;
  if (!spot_.CanDock(dock_id, &can_dock) || !can_dock) {
    EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") +
            " dock unavailable id=" + std::to_string(dock_id));
    return false;
  }

  docking_in_progress_ = true;
  moving_ = false;
  spot_.ZeroVelocity(1);
  const int dock_poll_ms = std::max(1000, cfg_.dock_poll_ms);
  const bool ok = spot_.AutoDock(dock_id, cfg_.dock_attempts, dock_poll_ms, cfg_.dock_command_timeout_sec);
  docking_in_progress_ = false;
  if (!ok) {
    EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") + " failed: " + spot_.LastError());
    if (!return_and_dock) {
      std::lock_guard<std::mutex> lk(map_mu_);
      ClearDockWaypointCandidateLocked();
    }
    return false;
  }
  if (!return_and_dock) CommitDockWaypointCandidateOnSuccess();
  dock_cooldown_until_ms_ = now_ms() + 5000;
  EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") + " success");
  return true;
}

bool Adapter::ExecuteQueuedCommand(const v1::model::CommandRequest& request) {
  const std::string& command = request.command();
  if (command != "spot.jetson.reboot" && !SpotConnected()) {
    EmitLog(std::string("[command] rejected while robot unavailable: ") + command);
    return false;
  }

  if (command == "spot.jetson.reboot") {
    const bool accepted = !reboot_requested_.exchange(true);
    if (!accepted) return false;
    std::thread([]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      const int rc = std::system("/bin/systemctl reboot");
      (void)rc;
    }).detach();
    return true;
  }
  if (command == "spot.robot.reboot") {
    EmitLog("[command] spot.robot.reboot requested");
    const bool ok = spot_.RebootRobotBody();
    if (!ok) EmitLog(std::string("[command] spot.robot.reboot failed: ") + spot_.LastError());
    return ok;
  }
  if (command == "spot.camera.calibrate") return ExecuteCameraCalibrateCommand();
  if (command == "spot.stand") return ExecuteStandAction(false, true);
  if (command == "spot.sit") return ExecuteSitAction(false, true);
  if (command == "spot.recover") return ExecuteRecoverAction(false, true);
  if (command == "spot.dock") return ExecuteDockSequence(false);
  if (command == "spot.return_and_dock") return ExecuteDockSequence(true);
  if (command == "spot.rotate_left") return ExecuteRotateCommand(request, true);
  if (command == "spot.rotate_right") return ExecuteRotateCommand(request, false);
  if (command == "spot.reset_arm") {
    if (!ExecuteResetArmAction(false, true)) return false;
    return WaitForArmStow(15000);
  }
  if (command.rfind("spot.map.", 0) == 0) return HandleMapCommand(request);
  if (command == "spot.waypoint.goto") return ExecuteWaypointGotoCommand(request);
  if (command == "spot.waypoint.goto_straight") return ExecuteWaypointGotoStraightCommand(request);
  if (command.rfind("spot.waypoint.", 0) == 0) return HandleWaypointCommand(request);
  return false;
}

void Adapter::HandleButtons(const v1::model::Bitset& bitset, const std::string& source_stream) {
  for (const auto& b : bitset.bits()) {
    if (!b.value()) continue;
    const std::string key = b.key().empty() ? source_stream : b.key();
    HandleButtonPress(key, source_stream);
  }
}

void Adapter::HandleButtonPress(const std::string& button_key, const std::string& source_stream) {
  const std::string norm_key = normalize_button_token(button_key);
  const std::string norm_stream = normalize_button_token(source_stream);
  const auto matches = [&](const std::string& configured_name,
                           std::initializer_list<const char*> aliases) -> bool {
    const std::string norm_cfg = normalize_button_token(configured_name);
    if (!norm_cfg.empty() && (norm_key == norm_cfg || norm_stream == norm_cfg)) return true;
    for (const char* alias : aliases) {
      const std::string a = normalize_button_token(alias);
      if (norm_key == a || norm_stream == a) return true;
    }
    return false;
  };

  EmitLog(std::string("[teleop] button press key=") + button_key + " stream=" + source_stream);

  if (matches(cfg_.stand_button_stream, {"stand"})) {
    (void)ExecuteStandAction(true, false);
    return;
  }

  if (matches(cfg_.sit_button_stream, {"sit"})) {
    (void)ExecuteSitAction(true, false);
    return;
  }

  if (matches(cfg_.estop_button_stream, {"estop", "e-stop", "emergencystop"})) {
    if (!lease_owned_) {
      EmitLog("[teleop] action estop rejected: no body lease");
      return;
    }
    if (!spot_.EmergencyStop()) {
      EmitLog(std::string("[teleop] action estop failed: ") + spot_.LastError());
    } else {
      EmitLog("[teleop] action estop sent");
      moving_ = false;
    }
    return;
  }

  if (matches(cfg_.recover_button_stream, {"recover", "selfright", "self-right"})) {
    (void)ExecuteRecoverAction(true, false);
    return;
  }

  if (matches(cfg_.walk_button_stream, {"walk"})) {
    desired_motion_mode_ = kMotionModeWalk;
    EmitLog("[mode] selected walk");
    return;
  }

  if (matches(cfg_.stairs_button_stream, {"stairs"})) {
    desired_motion_mode_ = kMotionModeStairs;
    EmitLog("[mode] selected stairs");
    return;
  }

  if (matches(cfg_.crawl_button_stream, {"crawl"})) {
    desired_motion_mode_ = kMotionModeCrawl;
    EmitLog("[mode] selected crawl");
    return;
  }

  if (matches(cfg_.dock_button_stream, {"dock"})) {
    (void)ExecuteDockAction(true);
    return;
  }

  if (matches(cfg_.reset_arm_button_stream, {"resetarm", "armreset"})) {
    (void)ExecuteResetArmAction(true, false);
    return;
  }

  EmitLog(std::string("[teleop] action ignored: unknown button key=") + button_key +
          " stream=" + source_stream);
}

bool Adapter::EnsureLeaseForCommand(const std::string& action_name) {
  if (lease_owned_.load()) return true;
  lease_owned_ = spot_.AcquireBodyLease();
  if (!lease_owned_.load()) {
    EmitLog(std::string("[command] ") + action_name + " rejected: no body lease (" + spot_.LastError() + ")");
    return false;
  }
  EmitLog(std::string("[command] ") + action_name + " acquired body lease");
  return true;
}

bool Adapter::ExecuteStandAction(bool require_teleop, bool auto_acquire_lease) {
  if (require_teleop && !TeleopSessionActive()) {
    EmitLog("[teleop] action stand rejected: session inactive");
    return false;
  }
  if (!lease_owned_.load() && (!auto_acquire_lease || !EnsureLeaseForCommand("spot.stand"))) {
    if (require_teleop) EmitLog("[teleop] action stand rejected: no body lease");
    return false;
  }
  if (!spot_.Stand()) {
    EmitLog(std::string(require_teleop ? "[teleop] action stand failed: " : "[command] spot.stand failed: ") +
            spot_.LastError());
    return false;
  }
  EmitLog(require_teleop ? "[teleop] action stand sent" : "[command] spot.stand sent");
  return true;
}

bool Adapter::ExecuteSitAction(bool require_teleop, bool auto_acquire_lease) {
  if (require_teleop && !TeleopSessionActive()) {
    EmitLog("[teleop] action sit rejected: session inactive");
    return false;
  }
  if (!lease_owned_.load() && (!auto_acquire_lease || !EnsureLeaseForCommand("spot.sit"))) {
    if (require_teleop) EmitLog("[teleop] action sit rejected: no body lease");
    return false;
  }
  if (!spot_.Sit()) {
    EmitLog(std::string(require_teleop ? "[teleop] action sit failed: " : "[command] spot.sit failed: ") +
            spot_.LastError());
    return false;
  }
  EmitLog(require_teleop ? "[teleop] action sit sent" : "[command] spot.sit sent");
  return true;
}

bool Adapter::ExecuteRecoverAction(bool require_teleop, bool auto_acquire_lease) {
  if (require_teleop && !TeleopSessionActive()) {
    EmitLog("[teleop] action recover rejected: session inactive");
    return false;
  }
  if (!lease_owned_.load() && (!auto_acquire_lease || !EnsureLeaseForCommand("spot.recover"))) {
    if (require_teleop) EmitLog("[teleop] action recover rejected: no body lease");
    return false;
  }
  if (!spot_.RecoverSelfRight()) {
    EmitLog(std::string(require_teleop ? "[teleop] action recover failed: "
                                       : "[command] spot.recover failed: ") + spot_.LastError());
    return false;
  }
  EmitLog(require_teleop ? "[teleop] action recover sent" : "[command] spot.recover sent");
  moving_ = false;
  return true;
}

bool Adapter::ExecuteDockAction(bool require_teleop) {
  if (require_teleop && !TeleopSessionActive()) {
    EmitLog("[dock] request rejected: session inactive");
    return false;
  }
  CaptureDockWaypointCandidate();
  EmitLog("[dock] request accepted");
  RequestDock();
  return true;
}

bool Adapter::ExecuteResetArmAction(bool require_teleop, bool auto_acquire_lease) {
  if (require_teleop && !TeleopSessionActive()) {
    EmitLog("[arm] reset rejected: session inactive");
    return false;
  }
  if (!lease_owned_.load() && (!auto_acquire_lease || !EnsureLeaseForCommand("spot.reset_arm"))) {
    if (require_teleop) EmitLog("[arm] reset rejected: no body lease");
    return false;
  }
  EmitLog(require_teleop ? "[arm] reset requested" : "[command] spot.reset_arm requested");
  ApplyDesiredArmMode(true);
  return true;
}

bool Adapter::ExecuteRotateCommand(const v1::model::CommandRequest& request, bool left) {
  const std::string command_name = left ? "spot.rotate_left" : "spot.rotate_right";
  if (docking_in_progress_.load()) {
    EmitLog("[command] " + command_name + " rejected: docking in progress");
    return false;
  }
  if (IsGraphNavNavigationActive()) {
    EmitLog("[command] " + command_name + " rejected: graphnav navigation active");
    return false;
  }
  if (spot_degraded_non_estop_.load()) {
    EmitLog("[command] " + command_name + " rejected: robot degraded (non-estop)");
    return false;
  }
  if (!EnsureLeaseForCommand(command_name)) return false;

  std::string text;
  (void)ExtractCommandText(request, &text);
  const std::string degrees_text = Trim(ExtractParam(text, {"degrees", "deg"}));
  if (degrees_text.empty()) {
    EmitLog("[command] " + command_name + " rejected: missing required parameter degrees");
    return false;
  }

  double degrees = 0.0;
  try {
    size_t consumed = 0;
    degrees = std::stod(degrees_text, &consumed);
    if (consumed != degrees_text.size()) {
      EmitLog("[command] " + command_name + " rejected: invalid degrees value");
      return false;
    }
  } catch (...) {
    EmitLog("[command] " + command_name + " rejected: invalid degrees value");
    return false;
  }

  if (degrees <= 0.0 || degrees > 360.0) {
    EmitLog("[command] " + command_name + " rejected: degrees must be > 0 and <= 360");
    return false;
  }

  const double rotate_rad = degrees * 3.14159265358979323846 / 180.0;
  const double rate_mag = std::max(0.3, std::min(1.0, std::abs(cfg_.max_wz_rps)));
  const double wz = left ? rate_mag : -rate_mag;
  const int duration_ms = std::max(300, static_cast<int>(std::llround((rotate_rad / rate_mag) * 1000.0)));

  EmitLog("[command] " + command_name + " requested degrees=" + std::to_string(degrees) +
          " rate=" + std::to_string(wz) + " duration_ms=" + std::to_string(duration_ms));
  if (!spot_.Velocity(0.0, 0.0, wz, duration_ms + 300, MotionMode::kWalk, 0.0)) {
    EmitLog(std::string("[command] " + command_name + " failed: ") + spot_.LastError());
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms + 120));
  (void)spot_.ZeroVelocity(1);
  moving_ = false;
  EmitLog("[command] " + command_name + " complete");
  return true;
}

void Adapter::ApplyDesiredArmMode(bool force) {
  const long long now = now_ms();
  const int min_interval_ms = std::max(250, cfg_.arm_hold_interval_ms);
  if (!force && (now - last_arm_hold_cmd_ms_.load()) < min_interval_ms) return;
  if (!lease_owned_) return;

  const bool ok = spot_.ResetArmToStow();
  if (ok) {
    if (force && (now - last_arm_hold_cmd_ms_.load()) >= min_interval_ms) {
      EmitLog("[arm] stow command sent");
    }
  } else {
    const long long last_err = last_arm_error_log_ms_.load();
    if (force || (now - last_err) >= 2000) {
      EmitLog(std::string("[arm] stow command failed: ") + spot_.LastError());
      last_arm_error_log_ms_ = now;
    }
  }
  if (ok) last_arm_hold_cmd_ms_ = now;
}

bool Adapter::IsArmLikelyStowed() {
  double sh1 = 0.0;
  double el0 = 0.0;
  double el1 = 0.0;
  double wr1 = 0.0;
  if (!spot_.GetArmJointPosition("arm0.sh1", &sh1)) return false;
  if (!spot_.GetArmJointPosition("arm0.el0", &el0)) return false;
  if (!spot_.GetArmJointPosition("arm0.el1", &el1)) return false;
  if (!spot_.GetArmJointPosition("arm0.wr1", &wr1)) return false;

  // Heuristic stow envelope based on observed stow posture.
  if (sh1 > -2.6) return false;
  if (el0 < 2.6) return false;
  if (el1 < 1.0) return false;
  if (std::abs(wr1 - (-1.57)) > 0.7) return false;
  return true;
}

bool Adapter::WaitForArmStow(int timeout_ms) {
  const auto start = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::milliseconds(std::max(500, timeout_ms));
  while (lease_owned_) {
    if (IsArmLikelyStowed()) return true;
    ApplyDesiredArmMode(true);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if ((std::chrono::steady_clock::now() - start) >= timeout) break;
  }
  return IsArmLikelyStowed();
}

void Adapter::HandleTwist(const v1::model::Twist& twist) {
  static constexpr long long kPitchZeroDropoutGraceMs = 80;
  if (now_ms() < dock_cooldown_until_ms_.load()) return;
  if (!TeleopSessionActive()) return;
  if (docking_in_progress_) return;
  if (!SpotConnected()) return;
  if (spot_degraded_non_estop_.load()) return;
  if (!lease_owned_) return;
  const long long now = now_ms();
  last_twist_ms_ = now;

  const double raw_x = twist.linear().x();
  const double raw_y = twist.linear().y();
  const double raw_z = twist.angular().z();
  const double raw_pitch = twist.angular().y();

  double nx = raw_x;
  double ny = raw_y;
  double nz = raw_z;
  double np = raw_pitch;
  if (std::abs(nx) < cfg_.twist_deadband) nx = 0.0;
  if (std::abs(ny) < cfg_.twist_deadband) ny = 0.0;
  if (std::abs(nz) < cfg_.twist_deadband) nz = 0.0;
  if (std::abs(np) < cfg_.twist_deadband) np = 0.0;

  const double vx = std::max(-cfg_.max_vx_mps, std::min(cfg_.max_vx_mps, nx * cfg_.max_vx_mps));
  const double vy = std::max(-cfg_.max_vy_mps, std::min(cfg_.max_vy_mps, ny * cfg_.max_vy_mps));
  const double wz = std::max(-cfg_.max_wz_rps, std::min(cfg_.max_wz_rps, nz * cfg_.max_wz_rps));
  const double body_pitch =
      std::max(-cfg_.max_body_pitch_rad, std::min(cfg_.max_body_pitch_rad, np * cfg_.max_body_pitch_rad));

  const bool translational_cmd_nonzero = (vx != 0.0 || vy != 0.0 || wz != 0.0);
  const bool pitch_cmd_nonzero = (body_pitch != 0.0);
  if (pitch_cmd_nonzero) {
    last_nonzero_cmd_ms_ = now;
  } else if (!translational_cmd_nonzero) {
    double prev_pitch = 0.0;
    {
      std::lock_guard<std::mutex> lk(twist_cmd_mu_);
      prev_pitch = desired_body_pitch_;
    }
    if (std::abs(prev_pitch) > 0.0 &&
        (now - last_nonzero_cmd_ms_.load()) < kPitchZeroDropoutGraceMs) {
      // Ignore very short pitch-only zero pulses from jittery teleop transport.
      return;
    }
  }

  {
    std::lock_guard<std::mutex> lk(twist_cmd_mu_);
    desired_vx_ = vx;
    desired_vy_ = vy;
    desired_wz_ = wz;
    desired_body_pitch_ = body_pitch;
  }
  desired_twist_valid_ = true;
}

void Adapter::MarkHeartbeat() {
  std::lock_guard<std::mutex> lk(hb_mu_);
  last_heartbeat_ms_ = now_ms();
}

bool Adapter::HeartbeatExpired() const {
  if (!heartbeat_seen_) return true;
  std::lock_guard<std::mutex> lk(hb_mu_);
  return (now_ms() - last_heartbeat_ms_) > cfg_.heartbeat_timeout_ms;
}

bool Adapter::TeleopSessionActive() const {
  const bool heartbeat_active = heartbeat_seen_.load() && !HeartbeatExpired();
  const bool control_active = control_seen_.load() &&
                              ((now_ms() - last_control_ms_.load()) <= cfg_.heartbeat_timeout_ms);
  return heartbeat_active || control_active;
}

void Adapter::RequestDock() { dock_requested_ = true; }

int Adapter::ResolveDockId() {
  if (!SpotConnected()) return -1;
  int current = resolved_dock_id_.load();
  if (current > 0) return current;
  if (cfg_.dock_station_id > 0) {
    resolved_dock_id_ = cfg_.dock_station_id;
    return cfg_.dock_station_id;
  }
  int discovered = -1;
  if (spot_.DiscoverSingleDockId(&discovered) && discovered > 0) {
    resolved_dock_id_ = discovered;
    return discovered;
  }
  return -1;
}

void Adapter::DockLoop() {
  auto navigate_with_recovery = [this](const std::string& waypoint_id,
                                       uint32_t* out_command_id) -> bool {
    return StartNavigateWithRecovery("return_and_dock.teleop", waypoint_id, out_command_id);
  };

  auto wait_for_navigation = [this, &navigate_with_recovery](uint32_t command_id) -> bool {
    if (command_id == 0) return false;
    const long long timeout_ms =
        static_cast<long long>(std::max(30, cfg_.graphnav_command_timeout_sec)) * 1000LL + 30000LL;
    const long long start_ms = now_ms();
    using Resp = ::bosdyn::api::graph_nav::NavigationFeedbackResponse;
    bool recovered_once = false;
    while (running_.load()) {
      if (now_ms() - start_ms > timeout_ms) {
        EmitLog("[return_and_dock] navigation timeout");
        return false;
      }
      SpotClient::NavigationFeedbackSnapshot fb;
      if (!spot_.GetNavigationFeedbackSnapshot(command_id, &fb)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      QueueStatusText("spot.nav.feedback", nav_feedback_to_json(command_id, fb));
      if (nav_status_in_progress(fb.status)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      if (fb.status == Resp::STATUS_REACHED_GOAL) {
        return true;
      }
      if (fb.status == Resp::STATUS_ROBOT_IMPAIRED && !recovered_once) {
        recovered_once = true;
        EmitLog("[return_and_dock] navigation feedback impaired, retrying after recover");
        if (!spot_.RecoverSelfRight()) return false;
        (void)spot_.Stand();
        uint32_t retry_id = 0;
        std::string waypoint_id;
        {
          std::lock_guard<std::mutex> lk(map_mu_);
          waypoint_id = nav_target_waypoint_id_;
        }
        if (waypoint_id.empty()) return false;
        if (!navigate_with_recovery(waypoint_id, &retry_id)) return false;
        command_id = retry_id;
        last_graph_nav_command_id_ = command_id;
        continue;
      }
      EmitLog("[return_and_dock] navigation ended status=" + std::to_string(fb.status) +
              " (" + nav_status_name(fb.status) + ")");
      return false;
    }
    return false;
  };

  while (running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const bool return_and_dock = return_and_dock_requested_.exchange(false);
    const bool dock_requested = dock_requested_.exchange(false);
    if (!return_and_dock && !dock_requested) continue;
    if (docking_in_progress_) {
      EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") +
              " request ignored: dock already in progress");
      continue;
    }
    if (!SpotConnected()) {
      EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") +
              " request ignored: robot unavailable");
      continue;
    }
    if (spot_degraded_non_estop_.load()) {
      EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") +
              " request ignored: robot degraded (non-estop)");
      continue;
    }

    if (return_and_dock) {
      std::string dock_waypoint_id;
      std::string map_id;
      {
        std::lock_guard<std::mutex> lk(map_mu_);
        if (!ResolveSavedDockHomeLocked(&map_id, &dock_waypoint_id)) {
          EmitLog("[return_and_dock] failed: no saved docking waypoint for active map");
          continue;
        }
      }
      EmitLog("[return_and_dock] navigating to dock waypoint map_id=" + map_id +
              " waypoint_id=" + dock_waypoint_id);
      uint32_t nav_command_id = 0;
      if (!navigate_with_recovery(dock_waypoint_id, &nav_command_id)) {
        EmitLog(std::string("[return_and_dock] navigation start failed: ") + spot_.LastError());
        continue;
      }
      last_graph_nav_command_id_ = nav_command_id;
      graphnav_navigation_active_ = true;
      nav_auto_recovered_ = false;
      desired_twist_valid_ = false;
      {
        std::lock_guard<std::mutex> lk(twist_cmd_mu_);
        desired_vx_ = 0.0;
        desired_vy_ = 0.0;
        desired_wz_ = 0.0;
        desired_body_pitch_ = 0.0;
      }
      {
        std::lock_guard<std::mutex> lk(map_mu_);
        nav_target_waypoint_id_ = dock_waypoint_id;
        nav_target_waypoint_name_ = "dock_home";
      }
      if (!wait_for_navigation(nav_command_id)) {
        graphnav_navigation_active_ = false;
        last_graph_nav_command_id_ = 0;
        EmitLog("[return_and_dock] failed: navigation did not reach dock waypoint");
        continue;
      }
      graphnav_navigation_active_ = false;
      last_graph_nav_command_id_ = 0;
      EmitLog("[return_and_dock] reached dock waypoint; starting dock");
    }

    int dock_id = ResolveDockId();
    if (dock_id <= 0) {
      EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") +
              " no dock id resolved: " + spot_.LastError());
      continue;
    }

    bool can_dock = false;
    if (!spot_.CanDock(dock_id, &can_dock)) {
      EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") +
              " dock availability check failed id=" + std::to_string(dock_id) +
              " error=" + spot_.LastError());
      resolved_dock_id_ = -1;
      continue;
    }
    if (!can_dock) {
      EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") +
              " dock unavailable id=" + std::to_string(dock_id));
      continue;
    }

    if (!lease_owned_) {
      lease_owned_ = spot_.AcquireBodyLease();
      if (!lease_owned_) {
        EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") +
                " failed to acquire lease for dock: " + spot_.LastError());
        continue;
      }
    }

    docking_in_progress_ = true;
    moving_ = false;
    spot_.ZeroVelocity(1);
    EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") +
            " starting dock id=" + std::to_string(dock_id));
    const int dock_poll_ms = std::max(1000, cfg_.dock_poll_ms);
    bool ok = spot_.AutoDock(dock_id, cfg_.dock_attempts, dock_poll_ms, cfg_.dock_command_timeout_sec);
    if (!ok) {
      const std::string err = spot_.LastError();
      EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") + " failed: " + err);
      if (!return_and_dock) {
        std::lock_guard<std::mutex> lk(map_mu_);
        ClearDockWaypointCandidateLocked();
      }
      if (err.find("STATUS_ERROR_DOCK_NOT_FOUND") != std::string::npos) {
        resolved_dock_id_ = -1;
        EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") +
                " clearing cached dock id after not-found error");
      }
    } else {
      EmitLog(std::string(return_and_dock ? "[return_and_dock]" : "[dock]") + " success");
      if (!return_and_dock) CommitDockWaypointCandidateOnSuccess();
      dock_cooldown_until_ms_ = now_ms() + 5000;
    }
    docking_in_progress_ = false;
  }
}

std::string Adapter::Trim(const std::string& in) {
  size_t start = 0;
  while (start < in.size() && std::isspace(static_cast<unsigned char>(in[start]))) ++start;
  size_t end = in.size();
  while (end > start && std::isspace(static_cast<unsigned char>(in[end - 1]))) --end;
  return in.substr(start, end - start);
}

std::string Adapter::ToLower(const std::string& in) {
  std::string out = in;
  for (char& c : out) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return out;
}

bool Adapter::ExtractCommandText(const v1::model::CommandRequest& request, std::string* out) const {
  if (!out) return false;
  *out = "";
  if (request.parameter_case() == v1::model::CommandRequest::kText) {
    *out = Trim(request.text());
    return true;
  }
  return false;
}

std::string Adapter::ExtractParam(const std::string& input, const std::vector<std::string>& keys) const {
  const std::string trimmed = Trim(input);
  if (trimmed.empty()) return "";
  if (keys.empty()) return trimmed;

  for (const auto& key : keys) {
    const std::string quoted_key = "\"" + key + "\"";
    size_t pos = trimmed.find(quoted_key);
    if (pos != std::string::npos) {
      pos = trimmed.find(':', pos + quoted_key.size());
      if (pos != std::string::npos) {
        ++pos;
        while (pos < trimmed.size() && std::isspace(static_cast<unsigned char>(trimmed[pos]))) ++pos;
        if (pos < trimmed.size() && trimmed[pos] == '"') {
          ++pos;
          size_t end = trimmed.find('"', pos);
          if (end != std::string::npos) return Trim(trimmed.substr(pos, end - pos));
        } else {
          size_t end = pos;
          while (end < trimmed.size() && trimmed[end] != ',' && trimmed[end] != '}') ++end;
          return Trim(trimmed.substr(pos, end - pos));
        }
      }
    }

    const std::string eq = key + "=";
    pos = ToLower(trimmed).find(ToLower(eq));
    if (pos != std::string::npos) {
      pos += eq.size();
      size_t end = trimmed.find_first_of(",;\n\r", pos);
      if (end == std::string::npos) end = trimmed.size();
      return Trim(trimmed.substr(pos, end - pos));
    }
  }

  return trimmed;
}

std::string Adapter::MapDirectoryFor(const std::string& map_id) const {
  return cfg_.graphnav_store_dir + "/maps/" + sanitize_id_token(map_id);
}

bool Adapter::SaveCurrentMapToDisk(const std::string& map_id) {
  if (map_id.empty()) return false;
  SpotClient::StoredMap map_data;
  if (!spot_.DownloadCurrentMap(&map_data)) return false;

  std::error_code ec;
  const std::string map_dir = MapDirectoryFor(map_id);
  std::filesystem::create_directories(map_dir + "/waypoints", ec);
  std::filesystem::create_directories(map_dir + "/edges", ec);
  if (ec) {
    EmitLog(std::string("[graphnav] failed creating map directories: ") + ec.message());
    return false;
  }

  {
    std::ofstream graph_out(map_dir + "/graph.bin", std::ios::binary | std::ios::trunc);
    if (!graph_out || !map_data.graph.SerializeToOstream(&graph_out)) {
      EmitLog("[graphnav] failed writing graph file");
      return false;
    }
  }

  for (const auto& snapshot : map_data.waypoint_snapshots) {
    std::ofstream out(map_dir + "/waypoints/" + sanitize_id_token(snapshot.id()) + ".bin",
                      std::ios::binary | std::ios::trunc);
    if (!out || !snapshot.SerializeToOstream(&out)) {
      EmitLog("[graphnav] failed writing waypoint snapshot");
      return false;
    }
  }
  for (const auto& snapshot : map_data.edge_snapshots) {
    std::ofstream out(map_dir + "/edges/" + sanitize_id_token(snapshot.id()) + ".bin",
                      std::ios::binary | std::ios::trunc);
    if (!out || !snapshot.SerializeToOstream(&out)) {
      EmitLog("[graphnav] failed writing edge snapshot");
      return false;
    }
  }
  return true;
}

bool Adapter::LoadMapFromDisk(const std::string& map_id, SpotClient::StoredMap* out_map) {
  if (!out_map || map_id.empty()) return false;
  SpotClient::StoredMap map_data;
  const std::string map_dir = MapDirectoryFor(map_id);

  {
    std::ifstream graph_in(map_dir + "/graph.bin", std::ios::binary);
    if (!graph_in || !map_data.graph.ParseFromIstream(&graph_in)) return false;
  }

  std::unordered_set<std::string> expected_waypoint_snapshot_ids;
  expected_waypoint_snapshot_ids.reserve(static_cast<size_t>(map_data.graph.waypoints_size()));
  for (const auto& waypoint : map_data.graph.waypoints()) {
    if (!waypoint.snapshot_id().empty()) expected_waypoint_snapshot_ids.insert(waypoint.snapshot_id());
  }

  std::unordered_set<std::string> expected_edge_snapshot_ids;
  expected_edge_snapshot_ids.reserve(static_cast<size_t>(map_data.graph.edges_size()));
  for (const auto& edge : map_data.graph.edges()) {
    if (!edge.snapshot_id().empty()) expected_edge_snapshot_ids.insert(edge.snapshot_id());
  }

  std::unordered_set<std::string> loaded_waypoint_snapshot_ids;
  std::unordered_set<std::string> loaded_edge_snapshot_ids;
  std::error_code ec;

  for (const auto& entry : std::filesystem::directory_iterator(map_dir + "/waypoints", ec)) {
    if (ec) return false;
    if (!entry.is_regular_file()) continue;
    std::ifstream in(entry.path(), std::ios::binary);
    if (!in) return false;
    ::bosdyn::api::graph_nav::WaypointSnapshot snapshot;
    if (!snapshot.ParseFromIstream(&in)) return false;
    if (snapshot.id().empty()) return false;
    loaded_waypoint_snapshot_ids.insert(snapshot.id());
    map_data.waypoint_snapshots.push_back(std::move(snapshot));
  }
  if (ec) return false;

  ec.clear();
  for (const auto& entry : std::filesystem::directory_iterator(map_dir + "/edges", ec)) {
    if (ec) return false;
    if (!entry.is_regular_file()) continue;
    std::ifstream in(entry.path(), std::ios::binary);
    if (!in) return false;
    ::bosdyn::api::graph_nav::EdgeSnapshot snapshot;
    if (!snapshot.ParseFromIstream(&in)) return false;
    if (snapshot.id().empty()) return false;
    loaded_edge_snapshot_ids.insert(snapshot.id());
    map_data.edge_snapshots.push_back(std::move(snapshot));
  }
  if (ec) return false;

  for (const auto& expected_id : expected_waypoint_snapshot_ids) {
    if (loaded_waypoint_snapshot_ids.find(expected_id) == loaded_waypoint_snapshot_ids.end()) {
      return false;
    }
  }
  for (const auto& expected_id : expected_edge_snapshot_ids) {
    if (loaded_edge_snapshot_ids.find(expected_id) == loaded_edge_snapshot_ids.end()) {
      return false;
    }
  }

  *out_map = std::move(map_data);
  return true;
}

bool Adapter::DeleteMapFromDisk(const std::string& map_id) {
  if (map_id.empty()) return false;
  std::error_code ec;
  std::filesystem::remove_all(MapDirectoryFor(map_id), ec);
  return !ec;
}

bool Adapter::LoadAdapterMapState() {
  std::lock_guard<std::mutex> lk(map_mu_);
  waypoint_aliases_by_map_.clear();
  dock_waypoint_by_map_.clear();
  ClearDockWaypointCandidateLocked();
  active_map_id_.clear();
  default_map_id_.clear();
  std::ifstream in(cfg_.graphnav_store_dir + "/state.txt");
  if (!in) return true;
  std::string line;
  while (std::getline(in, line)) {
    const std::string t = Trim(line);
    if (t.empty()) continue;
    if (t.rfind("active_map=", 0) == 0) {
      active_map_id_ = t.substr(std::string("active_map=").size());
      continue;
    }
    if (t.rfind("default_map=", 0) == 0) {
      default_map_id_ = t.substr(std::string("default_map=").size());
      continue;
    }
    if (t.rfind("alias\t", 0) == 0) {
      std::vector<std::string> parts;
      std::stringstream ss(t);
      std::string part;
      while (std::getline(ss, part, '\t')) parts.push_back(part);
      if (parts.size() >= 4) {
        waypoint_aliases_by_map_[parts[1]][parts[2]] = parts[3];
      }
      continue;
    }
    if (t.rfind("dock_waypoint\t", 0) == 0) {
      std::vector<std::string> parts;
      std::stringstream ss(t);
      std::string part;
      while (std::getline(ss, part, '\t')) parts.push_back(part);
      if (parts.size() >= 3) {
        dock_waypoint_by_map_[parts[1]] = parts[2];
      }
    }
  }
  return true;
}

bool Adapter::SaveAdapterMapState() {
  std::lock_guard<std::mutex> lk(map_mu_);
  std::error_code ec;
  std::filesystem::create_directories(cfg_.graphnav_store_dir, ec);
  if (ec) return false;
  std::ofstream out(cfg_.graphnav_store_dir + "/state.txt", std::ios::trunc);
  if (!out) return false;
  out << "active_map=" << active_map_id_ << "\n";
  out << "default_map=" << default_map_id_ << "\n";
  for (const auto& map_entry : waypoint_aliases_by_map_) {
    for (const auto& alias : map_entry.second) {
      if (alias.first.find('\t') != std::string::npos || alias.first.find('\n') != std::string::npos) continue;
      out << "alias\t" << map_entry.first << "\t" << alias.first << "\t" << alias.second << "\n";
    }
  }
  for (const auto& entry : dock_waypoint_by_map_) {
    if (entry.first.find('\t') != std::string::npos || entry.first.find('\n') != std::string::npos) continue;
    out << "dock_waypoint\t" << entry.first << "\t" << entry.second << "\n";
  }
  return true;
}

void Adapter::RefreshGraphWaypointIndex() {
  ::bosdyn::api::graph_nav::Graph graph;
  if (!spot_.DownloadCurrentGraph(&graph)) {
    EmitLog(std::string("[graphnav] failed to download graph: ") + spot_.LastError());
    return;
  }
  std::unordered_map<std::string, std::string> id_to_label;
  std::unordered_map<std::string, std::vector<std::string>> label_to_ids;
  std::unordered_map<std::string, std::vector<std::string>> lower_label_to_ids;
  id_to_label.reserve(static_cast<size_t>(graph.waypoints_size()));
  for (const auto& waypoint : graph.waypoints()) {
    std::string label = waypoint.annotations().name();
    if (label.empty()) label = waypoint.id();
    id_to_label[waypoint.id()] = label;
    label_to_ids[label].push_back(waypoint.id());
    lower_label_to_ids[ToLower(label)].push_back(waypoint.id());
  }
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    loaded_waypoint_id_to_label_ = std::move(id_to_label);
    loaded_waypoint_label_to_ids_ = std::move(label_to_ids);
    loaded_waypoint_lower_label_to_ids_ = std::move(lower_label_to_ids);
    PruneAliasesForLoadedGraphLocked();
  }
  force_waypoint_publish_ = true;
}

void Adapter::PruneAliasesForLoadedGraphLocked() {
  if (active_map_id_.empty()) return;
  auto it = waypoint_aliases_by_map_.find(active_map_id_);
  if (it == waypoint_aliases_by_map_.end()) return;
  for (auto alias_it = it->second.begin(); alias_it != it->second.end();) {
    if (loaded_waypoint_id_to_label_.find(alias_it->second) == loaded_waypoint_id_to_label_.end()) {
      alias_it = it->second.erase(alias_it);
    } else {
      ++alias_it;
    }
  }
}

bool Adapter::ResolveWaypointNameLocked(const std::string& name, std::string* out_waypoint_id) const {
  if (!out_waypoint_id) return false;
  const std::string key = Trim(name);
  if (key.empty()) return false;

  if (!active_map_id_.empty()) {
    auto map_it = waypoint_aliases_by_map_.find(active_map_id_);
    if (map_it != waypoint_aliases_by_map_.end()) {
      auto it = map_it->second.find(key);
      if (it != map_it->second.end()) {
        *out_waypoint_id = it->second;
        return true;
      }
      const std::string lower_key = ToLower(key);
      for (const auto& alias : map_it->second) {
        if (ToLower(alias.first) == lower_key) {
          *out_waypoint_id = alias.second;
          return true;
        }
      }
    }
  }

  auto exact_it = loaded_waypoint_label_to_ids_.find(key);
  if (exact_it != loaded_waypoint_label_to_ids_.end()) {
    if (exact_it->second.size() == 1) {
      *out_waypoint_id = exact_it->second.front();
      return true;
    }
    return false;
  }

  const std::string lower_key = ToLower(key);
  auto lower_it = loaded_waypoint_lower_label_to_ids_.find(lower_key);
  if (lower_it != loaded_waypoint_lower_label_to_ids_.end()) {
    if (lower_it->second.size() == 1) {
      *out_waypoint_id = lower_it->second.front();
      return true;
    }
    return false;
  }
  return false;
}

bool Adapter::ResolveSavedDockHomeLocked(std::string* out_map_id, std::string* out_waypoint_id) const {
  if (!out_map_id || !out_waypoint_id) return false;
  std::string map_id = active_map_id_;
  if (map_id.empty()) map_id = default_map_id_;
  if (map_id.empty()) return false;
  const auto it = dock_waypoint_by_map_.find(map_id);
  if (it == dock_waypoint_by_map_.end() || it->second.empty()) return false;
  *out_map_id = map_id;
  *out_waypoint_id = it->second;
  return true;
}

void Adapter::ClearDockWaypointCandidateLocked() {
  pending_dock_candidate_map_id_.clear();
  pending_dock_candidate_waypoint_id_.clear();
}

void Adapter::CaptureDockWaypointCandidate() {
  std::string map_id;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    map_id = active_map_id_;
    if (map_id.empty()) map_id = default_map_id_;
    ClearDockWaypointCandidateLocked();
  }
  if (map_id.empty()) {
    EmitLog("[dock] not learning dock waypoint: no active/default map");
    return;
  }

  std::string waypoint_id;
  if (!spot_.GetLocalizationWaypointId(&waypoint_id)) {
    EmitLog(std::string("[dock] not learning dock waypoint: localization unavailable: ") + spot_.LastError());
    return;
  }

  {
    std::lock_guard<std::mutex> lk(map_mu_);
    pending_dock_candidate_map_id_ = map_id;
    pending_dock_candidate_waypoint_id_ = waypoint_id;
  }
  EmitLog("[dock] learned dock candidate map_id=" + map_id + " waypoint_id=" + waypoint_id);
}

void Adapter::CommitDockWaypointCandidateOnSuccess() {
  std::string map_id;
  std::string waypoint_id;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    map_id = pending_dock_candidate_map_id_;
    waypoint_id = pending_dock_candidate_waypoint_id_;
    ClearDockWaypointCandidateLocked();
    if (map_id.empty() || waypoint_id.empty()) return;
    dock_waypoint_by_map_[map_id] = waypoint_id;
  }
  if (!SaveAdapterMapState()) {
    EmitLog("[dock] warning: failed to persist dock waypoint state");
  }
  EmitLog("[dock] dock waypoint saved map_id=" + map_id + " waypoint_id=" + waypoint_id);
}

std::string Adapter::BuildWaypointTextLocked() const {
  std::vector<std::string> names;
  std::string map_id = active_map_id_;
  if (map_id.empty()) map_id = default_map_id_;
  if (!map_id.empty()) {
    auto it = waypoint_aliases_by_map_.find(map_id);
    if (it != waypoint_aliases_by_map_.end()) {
      for (const auto& alias : it->second) names.push_back(alias.first);
    }
  }
  std::sort(names.begin(), names.end());
  names.erase(std::unique(names.begin(), names.end()), names.end());
  std::ostringstream oss;
  for (size_t i = 0; i < names.size(); ++i) {
    oss << names[i];
    if (i + 1 < names.size()) oss << "\n";
  }
  return oss.str();
}

std::string Adapter::ResolveWaypointNameForIdLocked(const std::string& waypoint_id) const {
  if (waypoint_id.empty()) return "";
  std::string map_id = active_map_id_;
  if (map_id.empty()) map_id = default_map_id_;
  if (!map_id.empty()) {
    auto map_it = waypoint_aliases_by_map_.find(map_id);
    if (map_it != waypoint_aliases_by_map_.end()) {
      for (const auto& alias : map_it->second) {
        if (alias.second == waypoint_id) return alias.first;
      }
    }
  }
  auto it = loaded_waypoint_id_to_label_.find(waypoint_id);
  if (it != loaded_waypoint_id_to_label_.end()) return it->second;
  return "";
}

std::string Adapter::BuildMapsText(const std::string& active_map_id,
                                   const std::string& default_map_id) const {
  std::vector<std::string> map_ids;
  std::error_code ec;
  const std::string maps_dir = cfg_.graphnav_store_dir + "/maps";
  if (std::filesystem::exists(maps_dir, ec) && !ec) {
    for (const auto& entry : std::filesystem::directory_iterator(maps_dir, ec)) {
      if (ec) break;
      if (!entry.is_directory()) continue;
      map_ids.push_back(entry.path().filename().string());
    }
  }
  if (!active_map_id.empty()) map_ids.push_back(active_map_id);
  if (!default_map_id.empty()) map_ids.push_back(default_map_id);
  std::sort(map_ids.begin(), map_ids.end());
  map_ids.erase(std::unique(map_ids.begin(), map_ids.end()), map_ids.end());
  std::ostringstream oss;
  for (size_t i = 0; i < map_ids.size(); ++i) {
    oss << map_ids[i];
    if (i + 1 < map_ids.size()) oss << "\n";
  }
  return oss.str();
}

std::string Adapter::BuildCurrentMapTextLocked() const {
  return active_map_id_.empty() ? "none" : active_map_id_;
}

std::string Adapter::BuildDefaultMapTextLocked() const {
  return default_map_id_.empty() ? "none" : default_map_id_;
}

std::string Adapter::EnsureActiveMapIdLocked() {
  if (active_map_id_.empty()) {
    if (!default_map_id_.empty()) {
      active_map_id_ = default_map_id_;
    } else {
      active_map_id_ = "map-" + std::to_string(now_ms());
      default_map_id_ = active_map_id_;
    }
  } else if (default_map_id_.empty()) {
    default_map_id_ = active_map_id_;
  }
  return active_map_id_;
}

void Adapter::PublishWaypointsText(bool force) {
  const long long kWaypointPeriodicMs = 5000;
  const long long now = now_ms();
  const bool force_flag = force_waypoint_publish_.exchange(false);
  if (!force && !force_flag && (now - last_waypoint_pub_ms_.load()) < kWaypointPeriodicMs) return;
  std::string text;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    text = BuildWaypointTextLocked();
    last_waypoint_text_payload_ = text;
  }
  QueueStatusText(cfg_.waypoint_text_stream, text);
  last_waypoint_pub_ms_ = now;
}

void Adapter::PublishMapsText(bool force) {
  const long long kMapsPeriodicMs = 5000;
  const long long now = now_ms();
  const bool force_flag = force_maps_publish_.exchange(false);
  if (!force && !force_flag && (now - last_maps_pub_ms_.load()) < kMapsPeriodicMs) return;

  std::string active_map_id;
  std::string default_map_id;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    active_map_id = active_map_id_;
    default_map_id = default_map_id_;
  }

  const std::string text = BuildMapsText(active_map_id, default_map_id);
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    last_maps_text_payload_ = text;
  }
  QueueStatusText(cfg_.maps_text_stream, text);
  last_maps_pub_ms_ = now;
}

void Adapter::PublishCurrentMapText(bool force) {
  const long long kPeriodicMs = 10000;
  const long long now = now_ms();
  std::string text;
  bool changed = false;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    text = BuildCurrentMapTextLocked();
    changed = (text != last_current_map_text_payload_);
    if (changed) last_current_map_text_payload_ = text;
  }
  if (!force && !changed && (now - last_current_map_pub_ms_.load()) < kPeriodicMs) return;
  QueueStatusText("spot.map.current", text);
  last_current_map_pub_ms_ = now;
}

void Adapter::PublishDefaultMapText(bool force) {
  const long long kPeriodicMs = 10000;
  const long long now = now_ms();
  std::string text;
  bool changed = false;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    text = BuildDefaultMapTextLocked();
    changed = (text != last_default_map_text_payload_);
    if (changed) last_default_map_text_payload_ = text;
  }
  if (!force && !changed && (now - last_default_map_pub_ms_.load()) < kPeriodicMs) return;
  QueueStatusText("spot.map.default", text);
  last_default_map_pub_ms_ = now;
}

void Adapter::PollCurrentWaypointAtStatus(long long now) {
  if ((now - last_waypoint_at_poll_ms_.load()) < 1000) return;
  last_waypoint_at_poll_ms_ = now;

  std::string resolved_name;
  if (SpotConnected()) {
    SpotClient::LocalizationSnapshot localization;
    if (spot_.GetLocalizationSnapshot(&localization) && localization.has_waypoint_tform_body) {
      const double dx = localization.waypoint_tform_body_x;
      const double dy = localization.waypoint_tform_body_y;
      const double dz = localization.waypoint_tform_body_z;
      const double distance_m = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance_m <= 0.3048) {
        std::lock_guard<std::mutex> lk(map_mu_);
        resolved_name = ResolveWaypointNameForIdLocked(localization.waypoint_id);
      }
    }
  }

  std::lock_guard<std::mutex> lk(map_mu_);
  current_waypoint_at_text_ = resolved_name;
}

void Adapter::PublishCurrentWaypointText(long long now) {
  if ((now - last_waypoint_at_pub_ms_.load()) < 10000) return;
  std::string text;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    text = current_waypoint_at_text_;
  }
  QueueStatusText("spot.waypoint.current", text);
  last_waypoint_at_pub_ms_ = now;
}

bool Adapter::HandleMapCommand(const v1::model::CommandRequest& request) {
  std::string text;
  (void)ExtractCommandText(request, &text);

  if (request.command() == "spot.map.create") {
    if (!EnsureLeaseForCommand("spot.map.create")) return false;
    if (map_recording_active_) {
      EmitLog("[graphnav] map.create rejected: stop mapping first");
      return false;
    }

    std::string map_id = ExtractParam(text, {"name", "map_id", "map", "id"});
    map_id = sanitize_id_token(map_id);
    if (map_id.empty()) {
      EmitLog("[graphnav] map.create rejected: missing name");
      return false;
    }

    bool exists_in_state = false;
    {
      std::lock_guard<std::mutex> lk(map_mu_);
      exists_in_state = (waypoint_aliases_by_map_.find(map_id) != waypoint_aliases_by_map_.end()) ||
                        (dock_waypoint_by_map_.find(map_id) != dock_waypoint_by_map_.end());
    }
    std::error_code ec;
    const bool exists_on_disk = std::filesystem::exists(MapDirectoryFor(map_id), ec) && !ec;
    if (exists_in_state || exists_on_disk) {
      EmitLog(std::string("[graphnav] map.create rejected: map already exists map_id=") + map_id);
      return false;
    }

    if (!spot_.ClearGraph()) {
      EmitLog(std::string("[graphnav] map.create failed clearing graph: ") + spot_.LastError());
      return false;
    }

    {
      std::lock_guard<std::mutex> lk(map_mu_);
      active_map_id_ = map_id;
      if (default_map_id_.empty()) default_map_id_ = map_id;
      pending_restore_map_id_.clear();
      nav_target_waypoint_id_.clear();
      nav_target_waypoint_name_.clear();
      loaded_waypoint_id_to_label_.clear();
      loaded_waypoint_label_to_ids_.clear();
      loaded_waypoint_lower_label_to_ids_.clear();
      ClearDockWaypointCandidateLocked();
    }

    last_graph_nav_command_id_ = 0;
    graphnav_navigation_active_ = false;
    nav_auto_recovered_ = false;
    {
      std::lock_guard<std::mutex> lk(nav_state_mu_);
      last_nav_feedback_signature_.clear();
      last_nav_status_ = 0;
      last_nav_remaining_route_m_ = 0.0;
      last_nav_progress_change_ms_ = 0;
    }

    if (!SaveAdapterMapState()) {
      EmitLog("[graphnav] map.create warning: failed to persist adapter map state");
    }
    PublishWaypointsText(true);
    PublishMapsText(true);
    EmitLog(std::string("[graphnav] map created: ") + map_id);
    return true;
  }

  if (request.command() == "spot.map.load") {
    if (!EnsureLeaseForCommand("spot.map.load")) return false;
    std::string map_id = ExtractParam(text, {"map_id", "map", "id"});
    map_id = sanitize_id_token(map_id);
    if (map_id.empty()) {
      EmitLog("[graphnav] map.load rejected: missing map_id");
      return false;
    }
    SpotClient::StoredMap map_data;
    if (!LoadMapFromDisk(map_id, &map_data)) {
      EmitLog(std::string("[graphnav] map.load failed reading map_id=") + map_id);
      return false;
    }
    if (!spot_.UploadGraphMap(map_data)) {
      EmitLog(std::string("[graphnav] map.load upload failed: ") + spot_.LastError());
      return false;
    }
    {
      std::lock_guard<std::mutex> lk(map_mu_);
      active_map_id_ = map_id;
      pending_restore_map_id_.clear();
      nav_target_waypoint_id_.clear();
      nav_target_waypoint_name_.clear();
    }
    last_graph_nav_command_id_ = 0;
    graphnav_navigation_active_ = false;
    nav_auto_recovered_ = false;
    {
      std::lock_guard<std::mutex> lk(nav_state_mu_);
      last_nav_feedback_signature_.clear();
      last_nav_status_ = 0;
      last_nav_remaining_route_m_ = 0.0;
      last_nav_progress_change_ms_ = 0;
    }
    if (!spot_.SetLocalizationFiducial()) {
      EmitLog(std::string("[graphnav] map.load localization warning: ") + spot_.LastError());
    }
    RefreshGraphWaypointIndex();
    SaveAdapterMapState();
    PublishWaypointsText(true);
    PublishMapsText(true);
    EmitLog(std::string("[graphnav] map loaded: ") + map_id);
    return true;
  }

  if (request.command() == "spot.map.set_default") {
    std::string map_id = ExtractParam(text, {"map_id", "map", "id"});
    map_id = sanitize_id_token(map_id);
    if (map_id.empty()) {
      EmitLog("[graphnav] map.set_default rejected: missing map_id");
      return false;
    }
    bool known_in_state = false;
    {
      std::lock_guard<std::mutex> lk(map_mu_);
      known_in_state = (waypoint_aliases_by_map_.find(map_id) != waypoint_aliases_by_map_.end()) ||
                       (dock_waypoint_by_map_.find(map_id) != dock_waypoint_by_map_.end()) ||
                       active_map_id_ == map_id || default_map_id_ == map_id;
    }
    std::error_code ec;
    const bool exists_on_disk = std::filesystem::exists(MapDirectoryFor(map_id), ec) && !ec;
    if (!known_in_state && !exists_on_disk) {
      EmitLog(std::string("[graphnav] map.set_default rejected: unknown map_id=") + map_id);
      return false;
    }
    {
      std::lock_guard<std::mutex> lk(map_mu_);
      default_map_id_ = map_id;
    }
    SaveAdapterMapState();
    PublishMapsText(true);
    EmitLog(std::string("[graphnav] default map set: ") + map_id);
    return true;
  }

  if (request.command() == "spot.map.delete") {
    std::string map_id = ExtractParam(text, {"map_id", "map", "id"});
    map_id = sanitize_id_token(map_id);
    if (map_id.empty()) {
      EmitLog("[graphnav] map.delete rejected: missing map_id");
      return false;
    }
    if (map_recording_active_) {
      EmitLog("[graphnav] map.delete rejected: stop mapping first");
      return false;
    }
    bool clear_loaded_graph = false;
    {
      std::lock_guard<std::mutex> lk(map_mu_);
      clear_loaded_graph = (active_map_id_ == map_id);
    }
    if (clear_loaded_graph) {
      if (!spot_.ClearGraph()) {
        EmitLog(std::string("[graphnav] map.delete failed clearing active graph: ") + spot_.LastError());
        return false;
      }
      last_graph_nav_command_id_ = 0;
      graphnav_navigation_active_ = false;
      nav_auto_recovered_ = false;
      {
        std::lock_guard<std::mutex> lk(nav_state_mu_);
        last_nav_feedback_signature_.clear();
        last_nav_status_ = 0;
        last_nav_remaining_route_m_ = 0.0;
        last_nav_progress_change_ms_ = 0;
      }
    }

    {
      std::lock_guard<std::mutex> lk(map_mu_);
      waypoint_aliases_by_map_.erase(map_id);
      dock_waypoint_by_map_.erase(map_id);
      if (pending_dock_candidate_map_id_ == map_id) {
        ClearDockWaypointCandidateLocked();
      }
      if (default_map_id_ == map_id) default_map_id_.clear();
      if (active_map_id_ == map_id) {
        active_map_id_.clear();
        pending_restore_map_id_.clear();
        nav_target_waypoint_id_.clear();
        nav_target_waypoint_name_.clear();
        loaded_waypoint_id_to_label_.clear();
        loaded_waypoint_label_to_ids_.clear();
        loaded_waypoint_lower_label_to_ids_.clear();
      }
    }
    if (!DeleteMapFromDisk(map_id)) {
      EmitLog(std::string("[graphnav] map.delete failed removing map files map_id=") + map_id);
      return false;
    }
    SaveAdapterMapState();
    PublishWaypointsText(true);
    PublishMapsText(true);
    EmitLog(std::string("[graphnav] map deleted: ") + map_id);
    return true;
  }

  if (request.command() == "spot.map.start_mapping") {
    if (!EnsureLeaseForCommand("spot.map.start_mapping")) return false;
    SpotClient::MappingStatus status;
    if (spot_.GetMappingStatus(&status)) {
      map_recording_active_ = status.is_recording;
      if (status.is_recording) {
        EmitLog("[graphnav] mapping already active");
        return true;
      }
    }
    if (!spot_.StartGraphRecording()) {
      EmitLog(std::string("[graphnav] start_mapping failed: ") + spot_.LastError());
      return false;
    }
    map_recording_active_ = true;
    {
      std::lock_guard<std::mutex> lk(map_mu_);
      (void)EnsureActiveMapIdLocked();
    }
    SaveAdapterMapState();
    PublishMapsText(true);
    EmitLog("[graphnav] mapping started");
    return true;
  }

  if (request.command() == "spot.map.stop_mapping") {
    if (!EnsureLeaseForCommand("spot.map.stop_mapping")) return false;
    bool already_stopped = false;
    SpotClient::MappingStatus status;
    if (spot_.GetMappingStatus(&status)) {
      map_recording_active_ = status.is_recording;
      already_stopped = !status.is_recording;
    }
    if (!already_stopped) {
      if (!spot_.StopGraphRecording()) {
        EmitLog(std::string("[graphnav] stop_mapping failed: ") + spot_.LastError());
        return false;
      }
    } else {
      EmitLog("[graphnav] stop_mapping: recording already stopped");
    }
    map_recording_active_ = false;
    std::string active_map;
    {
      std::lock_guard<std::mutex> lk(map_mu_);
      active_map = EnsureActiveMapIdLocked();
    }
    if (!SaveCurrentMapToDisk(active_map)) {
      EmitLog(std::string("[graphnav] stop_mapping saved graph failed: ") + spot_.LastError());
      return false;
    }
    RefreshGraphWaypointIndex();
    SaveAdapterMapState();
    PublishWaypointsText(true);
    PublishMapsText(true);
    EmitLog(std::string("[graphnav] mapping stopped and saved map_id=") + active_map);
    return true;
  }

  return false;
}

bool Adapter::HandleWaypointCommand(const v1::model::CommandRequest& request) {
  std::string text;
  (void)ExtractCommandText(request, &text);
  std::string name = Trim(ExtractParam(text, {"name", "waypoint", "waypoint_name", "alias"}));

  if (request.command() == "spot.waypoint.delete") {
    if (name.empty()) {
      EmitLog("[graphnav] waypoint.delete rejected: missing waypoint name");
      return false;
    }
    bool had_active_map = true;
    bool removed = false;
    {
      std::lock_guard<std::mutex> lk(map_mu_);
      if (active_map_id_.empty()) {
        had_active_map = false;
      } else {
        auto map_it = waypoint_aliases_by_map_.find(active_map_id_);
        if (map_it != waypoint_aliases_by_map_.end()) {
          auto it = map_it->second.find(name);
          if (it != map_it->second.end()) {
            map_it->second.erase(it);
            removed = true;
          } else {
            const std::string lower_name = ToLower(name);
            for (auto alias_it = map_it->second.begin(); alias_it != map_it->second.end(); ++alias_it) {
              if (ToLower(alias_it->first) == lower_name) {
                map_it->second.erase(alias_it);
                removed = true;
                break;
              }
            }
          }
        }
      }
    }
    if (!had_active_map) {
      EmitLog("[graphnav] waypoint.delete rejected: no active map");
      return false;
    }
    if (!removed) {
      EmitLog(std::string("[graphnav] waypoint.delete rejected: alias not found name=") + name);
      return false;
    }
    SaveAdapterMapState();
    PublishWaypointsText(true);
    EmitLog(std::string("[graphnav] waypoint alias deleted: ") + name);
    return true;
  }

  if (request.command() == "spot.waypoint.save") {
    if (name.empty()) {
      name = "saved-" + std::to_string(now_ms());
      EmitLog(std::string("[graphnav] waypoint.save: auto-generated name=") + name);
    }

    if (!EnsureLeaseForCommand(request.command())) return false;
    bool robot_recording = map_recording_active_.load();
    SpotClient::MappingStatus status;
    if (spot_.GetMappingStatus(&status)) {
      robot_recording = status.is_recording;
      map_recording_active_ = status.is_recording;
    }
    bool started_temp_recording = false;
    std::string waypoint_id;
    if (!robot_recording) {
      if (!spot_.StartGraphRecording(&waypoint_id)) {
        EmitLog(std::string("[graphnav] waypoint create failed: unable to start temporary recording: ") +
                spot_.LastError());
        return false;
      }
      started_temp_recording = true;
    } else {
      if (!spot_.CreateGraphWaypoint("", &waypoint_id)) {
        EmitLog(std::string("[graphnav] waypoint create failed: ") + spot_.LastError());
        return false;
      }
    }

    if (started_temp_recording) {
      if (!spot_.StopGraphRecording()) {
        SpotClient::MappingStatus post_stop_status;
        if (spot_.GetMappingStatus(&post_stop_status)) {
          map_recording_active_ = post_stop_status.is_recording;
          if (!post_stop_status.is_recording) {
            EmitLog("[graphnav] waypoint save: stop recording returned error but recording is now false");
          } else {
            EmitLog(std::string("[graphnav] waypoint create failed: temp stop recording failed: ") +
                    spot_.LastError());
            return false;
          }
        } else {
          EmitLog(std::string("[graphnav] waypoint create failed: temp stop recording failed: ") +
                  spot_.LastError());
          return false;
        }
      }
    }
    if (waypoint_id.empty()) {
      EmitLog("[graphnav] waypoint create failed: empty waypoint id");
      return false;
    }

    std::string active_map;
    {
      std::lock_guard<std::mutex> lk(map_mu_);
      active_map = EnsureActiveMapIdLocked();
      waypoint_aliases_by_map_[active_map][name] = waypoint_id;
    }
    if (!SaveCurrentMapToDisk(active_map)) {
      EmitLog(std::string("[graphnav] waypoint create warning: failed to save map to disk: ") +
              spot_.LastError());
    }
    RefreshGraphWaypointIndex();
    SaveAdapterMapState();
    PublishWaypointsText(true);
    PublishMapsText(true);
    EmitLog(std::string("[graphnav] waypoint alias set name=") + name + " id=" + waypoint_id);
    return true;
  }

  return false;
}

}  // namespace fsa
