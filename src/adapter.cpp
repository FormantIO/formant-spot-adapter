#include "formant_spot_adapter/adapter.hpp"
#include "formant_spot_adapter/graphnav_map_render.hpp"

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
#include <limits>
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
constexpr char kStatusStream[] = "spot.status";
constexpr char kConnectionStream[] = "spot.connection";
constexpr char kConnectionStateStream[] = "spot.connection_state";
constexpr char kLocalizationStatusStream[] = "spot.localization";
constexpr char kBehaviorStateStream[] = "spot.behavior_state";
constexpr char kCommandedMotionModeStream[] = "spot.commanded_motion_mode";
constexpr char kDockingStateStream[] = "spot.docking_state";
constexpr char kMotorPowerStateStream[] = "spot.motor_power_state";
constexpr char kRobotStatePowerStream[] = "spot.robot_state.power";
constexpr char kRobotStateBatteryStream[] = "spot.robot_state.battery";
constexpr char kRobotStateBodyPitchStream[] = "spot.robot_state.body_pitch_rad";
constexpr char kShorePowerStateStream[] = "spot.shore_power_state";
constexpr char kFaultsSystemStream[] = "spot.faults.system";
constexpr char kFaultsBehaviorStream[] = "spot.faults.behavior";
constexpr char kFaultsServiceStream[] = "spot.faults.service";
constexpr char kFaultEventsStream[] = "spot.fault.events";
constexpr char kNavFeedbackStream[] = "spot.nav.feedback";
constexpr char kAdapterLogStream[] = "spot.adapter.log";
constexpr char kCurrentMapStream[] = "spot.map.current";
constexpr char kDefaultMapStream[] = "spot.map.default";
constexpr char kCurrentWaypointStream[] = "spot.waypoint.current";
constexpr char kMapProgressStream[] = "spot.map.progress";
constexpr char kMapProgressWaypointsStream[] = "spot.map.progress.waypoints";
constexpr char kMapProgressPathLengthStream[] = "spot.map.progress.path_length_m";
constexpr char kMapProgressFiducialsStream[] = "spot.map.progress.fiducials";

constexpr int kLocalizationImageWidth = 1280;
constexpr int kLocalizationImageHeight = 720;

const char* connection_state_name(int raw_state) {
  switch (raw_state) {
    case 1:
      return "connecting";
    case 2:
      return "connected";
    case 0:
    default:
      return "disconnected";
  }
}

const char* motion_mode_state_name(int raw_mode) {
  switch (raw_mode) {
    case kMotionModeStairs:
      return "stairs";
    case kMotionModeCrawl:
      return "crawl";
    case kMotionModeWalk:
    default:
      return "walk";
  }
}

const char* docking_state_name(int status) {
  switch (status) {
    case ::bosdyn::api::docking::DockState_DockedStatus_DOCK_STATUS_DOCKED:
      return "docked";
    case ::bosdyn::api::docking::DockState_DockedStatus_DOCK_STATUS_DOCKING:
      return "docking";
    case ::bosdyn::api::docking::DockState_DockedStatus_DOCK_STATUS_UNDOCKED:
      return "undocked";
    case ::bosdyn::api::docking::DockState_DockedStatus_DOCK_STATUS_UNDOCKING:
      return "undocking";
    case ::bosdyn::api::docking::DockState_DockedStatus_DOCK_STATUS_UNKNOWN:
    default:
      return "unknown";
  }
}

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

double normalize_angle_rad(double angle_rad) {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

const SpotClient::GraphNavWaypoint* find_graphnav_waypoint(
    const SpotClient::GraphNavMapSnapshot& snapshot,
    const std::string& waypoint_id) {
  for (const auto& waypoint : snapshot.waypoints) {
    if (waypoint.id == waypoint_id) return &waypoint;
  }
  return nullptr;
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

struct GraphNavImagePalette {
  cv::Scalar canvas_bg{16, 24, 34};
  cv::Scalar grid_minor{34, 47, 62};
  cv::Scalar grid_major{46, 64, 84};
  cv::Scalar text_primary{236, 240, 245};
  cv::Scalar text_secondary{154, 172, 190};
  cv::Scalar map_unknown{24, 33, 44};
  cv::Scalar map_free{73, 88, 105};
  cv::Scalar map_occupied{176, 193, 208};
  cv::Scalar map_occupied_edge{222, 232, 240};
  cv::Scalar waypoint_fill{224, 164, 72};
  cv::Scalar waypoint_dock{66, 186, 122};
  cv::Scalar edge_line{92, 128, 158};
  cv::Scalar robot_fill{48, 190, 245};
  cv::Scalar robot_outline{18, 25, 34};
};

const GraphNavImagePalette& graphnav_image_palette() {
  static const GraphNavImagePalette palette;
  return palette;
}

bool build_graphnav_status_frame(const std::string& default_title,
                                 const std::string& default_detail,
                                 const std::string& title,
                                 const std::string& detail,
                                 std::string* out_jpg) {
  if (!out_jpg) return false;
  const auto& palette = graphnav_image_palette();
  cv::Mat canvas(kLocalizationImageHeight, kLocalizationImageWidth, CV_8UC3, palette.canvas_bg);
  for (int x = 0; x <= canvas.cols; x += 80) {
    cv::line(canvas, cv::Point(x, 0), cv::Point(x, canvas.rows - 1),
             (x % 240) == 0 ? palette.grid_major : palette.grid_minor, 1, cv::LINE_AA);
  }
  for (int y = 0; y <= canvas.rows; y += 80) {
    cv::line(canvas, cv::Point(0, y), cv::Point(canvas.cols - 1, y),
             (y % 240) == 0 ? palette.grid_major : palette.grid_minor, 1, cv::LINE_AA);
  }
  const cv::Point center(canvas.cols / 2, canvas.rows / 2);
  cv::circle(canvas, center, 54, cv::Scalar(48, 68, 88), 1, cv::LINE_AA);
  cv::circle(canvas, center, 108, cv::Scalar(40, 58, 76), 1, cv::LINE_AA);
  cv::line(canvas, cv::Point(center.x - 18, center.y), cv::Point(center.x + 18, center.y),
           cv::Scalar(78, 104, 132), 1, cv::LINE_AA);
  cv::line(canvas, cv::Point(center.x, center.y - 18), cv::Point(center.x, center.y + 18),
           cv::Scalar(78, 104, 132), 1, cv::LINE_AA);

  const std::string display_title = title.empty() ? default_title : title;
  const std::string display_detail = detail.empty() ? default_detail : detail;
  int baseline = 0;
  const cv::Size title_size =
      cv::getTextSize(display_title, cv::FONT_HERSHEY_SIMPLEX, 0.9, 2, &baseline);
  const cv::Point title_pt((canvas.cols - title_size.width) / 2, (canvas.rows / 2) + 18);
  cv::putText(canvas, display_title, title_pt, cv::FONT_HERSHEY_SIMPLEX, 0.9,
              palette.text_primary, 2, cv::LINE_AA);
  int detail_y = title_pt.y + 34;
  for (const auto& line : wrap_text(display_detail, 46)) {
    cv::putText(canvas, line, cv::Point(std::max(32, (canvas.cols - 960) / 2), detail_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.56, palette.text_secondary, 1, cv::LINE_AA);
    detail_y += 26;
  }
  return encode_jpeg(canvas, out_jpg);
}

bool rasterize_occupancy_grid(const SpotClient::OccupancyGridMapSnapshot& map,
                              const GraphNavImagePalette& palette,
                              cv::Mat* out_grid_image,
                              cv::Mat* out_occupied_mask) {
  if (!out_grid_image || !out_occupied_mask) return false;
  if (map.width <= 0 || map.height <= 0) return false;
  *out_grid_image = cv::Mat(map.height, map.width, CV_8UC3, palette.map_unknown);
  *out_occupied_mask = cv::Mat(map.height, map.width, CV_8UC1, cv::Scalar(0));
  for (int y = 0; y < map.height; ++y) {
    for (int x = 0; x < map.width; ++x) {
      const size_t idx = static_cast<size_t>(x) +
                         static_cast<size_t>(map.width) * static_cast<size_t>(y);
      const int32_t value = idx < map.occupancy.size() ? map.occupancy[idx] : -1;
      const int draw_y = map.height - 1 - y;
      cv::Vec3b* px = &out_grid_image->at<cv::Vec3b>(draw_y, x);
      if (value < 0) {
        *px = cv::Vec3b(static_cast<uchar>(palette.map_unknown[0]),
                        static_cast<uchar>(palette.map_unknown[1]),
                        static_cast<uchar>(palette.map_unknown[2]));
      } else if (value >= 50) {
        *px = cv::Vec3b(static_cast<uchar>(palette.map_occupied[0]),
                        static_cast<uchar>(palette.map_occupied[1]),
                        static_cast<uchar>(palette.map_occupied[2]));
        out_occupied_mask->at<uchar>(draw_y, x) = 255;
      } else {
        *px = cv::Vec3b(static_cast<uchar>(palette.map_free[0]),
                        static_cast<uchar>(palette.map_free[1]),
                        static_cast<uchar>(palette.map_free[2]));
      }
    }
  }
  return true;
}

void draw_occupied_contours(cv::Mat* canvas,
                            const cv::Mat& occupied_mask,
                            const cv::Rect& draw_rect,
                            const cv::Scalar& color) {
  if (!canvas || canvas->empty() || occupied_mask.empty() || draw_rect.width <= 0 ||
      draw_rect.height <= 0) {
    return;
  }
  cv::Mat scaled_mask;
  cv::resize(occupied_mask, scaled_mask, draw_rect.size(), 0.0, 0.0, cv::INTER_NEAREST);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(scaled_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  for (const auto& contour : contours) {
    if (contour.size() < 3) continue;
    if (draw_rect.x == 0 && draw_rect.y == 0) {
      cv::polylines(*canvas, contour, true, color, 1, cv::LINE_AA);
      continue;
    }
    std::vector<cv::Point> shifted;
    shifted.reserve(contour.size());
    for (const auto& point : contour) {
      shifted.emplace_back(point.x + draw_rect.x, point.y + draw_rect.y);
    }
    cv::polylines(*canvas, shifted, true, color, 1, cv::LINE_AA);
  }
}

template <typename ToPixelFn>
void draw_robot_pose_overlay(cv::Mat* canvas,
                             ToPixelFn&& to_pixel,
                             double body_x,
                             double body_y,
                             double yaw_rad,
                             double aura_radius_px,
                             double aura_alpha,
                             const cv::Scalar& robot_fill,
                             const cv::Scalar& robot_outline) {
  if (!canvas || canvas->empty()) return;
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  const std::array<cv::Point2d, 4> footprint_local = {
      cv::Point2d(0.45, 0.25),
      cv::Point2d(0.45, -0.25),
      cv::Point2d(-0.45, -0.25),
      cv::Point2d(-0.45, 0.25),
  };
  std::vector<cv::Point> footprint_pixels;
  footprint_pixels.reserve(footprint_local.size());
  for (const auto& corner : footprint_local) {
    const double px = body_x + c * corner.x - s * corner.y;
    const double py = body_y + s * corner.x + c * corner.y;
    footprint_pixels.push_back(to_pixel(px, py));
  }
  cv::Mat overlay = canvas->clone();
  cv::circle(overlay, to_pixel(body_x, body_y), std::max(10, cvRound(aura_radius_px)), robot_fill,
             cv::FILLED, cv::LINE_AA);
  cv::addWeighted(overlay, aura_alpha, *canvas, 1.0 - aura_alpha, 0.0, *canvas);
  cv::fillConvexPoly(*canvas, footprint_pixels, robot_fill, cv::LINE_AA);
  cv::polylines(*canvas, footprint_pixels, true, robot_outline, 2, cv::LINE_AA);
  const cv::Point body_center = to_pixel(body_x, body_y);
  const cv::Point arrow_tip = to_pixel(body_x + c * 0.8, body_y + s * 0.8);
  cv::circle(*canvas, body_center, 4, robot_outline, cv::FILLED, cv::LINE_AA);
  cv::arrowedLine(*canvas, body_center, arrow_tip, robot_outline, 3, cv::LINE_AA, 0, 0.28);
}

void draw_scale_bar(cv::Mat* canvas,
                    const cv::Point& scale_start,
                    int scale_bar_px,
                    double scale_bar_m,
                    const cv::Scalar& text_primary,
                    const cv::Scalar& robot_outline,
                    bool draw_shadowed_text) {
  if (!canvas || canvas->empty() || scale_bar_px <= 0) return;
  const cv::Point scale_end(scale_start.x + scale_bar_px, scale_start.y);
  cv::line(*canvas, cv::Point(scale_start.x, scale_start.y + 1),
           cv::Point(scale_end.x, scale_end.y + 1), robot_outline, 5, cv::LINE_AA);
  cv::line(*canvas, scale_start, scale_end, text_primary, 3, cv::LINE_AA);
  cv::line(*canvas, cv::Point(scale_start.x, scale_start.y - 5),
           cv::Point(scale_start.x, scale_start.y + 5), text_primary, 2, cv::LINE_AA);
  cv::line(*canvas, cv::Point(scale_end.x, scale_end.y - 5),
           cv::Point(scale_end.x, scale_end.y + 5), text_primary, 2, cv::LINE_AA);
  const std::string label = format_decimal(scale_bar_m, scale_bar_m >= 5.0 ? 0 : 1) + " m";
  if (draw_shadowed_text) {
    cv::putText(*canvas, label, cv::Point(scale_end.x + 13, scale_start.y + 6),
                cv::FONT_HERSHEY_SIMPLEX, 0.48, robot_outline, 2, cv::LINE_AA);
  }
  cv::putText(*canvas, label, cv::Point(scale_end.x + 12, scale_start.y + 5),
              cv::FONT_HERSHEY_SIMPLEX, 0.48, text_primary, 1, cv::LINE_AA);
}

double dot_vec(const Vec3d& a, const Vec3d& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3d cross_vec(const Vec3d& a, const Vec3d& b) {
  return Vec3d{
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x,
  };
}

double norm_vec(const Vec3d& v) {
  return std::sqrt(dot_vec(v, v));
}

Vec3d scale_vec(const Vec3d& v, double scale) {
  return Vec3d{v.x * scale, v.y * scale, v.z * scale};
}

Vec3d add_vec(const Vec3d& a, const Vec3d& b) {
  return Vec3d{a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3d normalize_vec(const Vec3d& v, const Vec3d& fallback) {
  const double norm = norm_vec(v);
  if (norm <= 1e-9) return fallback;
  return scale_vec(v, 1.0 / norm);
}

cv::Matx33d rotation_matrix_from_quaternion(const Quaterniond& q) {
  const Quaterniond qn = normalize_quaternion(q);
  const double xx = qn.x * qn.x;
  const double yy = qn.y * qn.y;
  const double zz = qn.z * qn.z;
  const double xy = qn.x * qn.y;
  const double xz = qn.x * qn.z;
  const double yz = qn.y * qn.z;
  const double wx = qn.w * qn.x;
  const double wy = qn.w * qn.y;
  const double wz = qn.w * qn.z;
  return cv::Matx33d(
      1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
      2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
      2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy));
}

cv::Matx33d rotation_matrix_from_pose(const SpotClient::Pose3D& pose) {
  return rotation_matrix_from_quaternion(pose_quaternion(pose));
}

Vec3d mat_mul(const cv::Matx33d& matrix, const Vec3d& v) {
  const cv::Vec3d vec(v.x, v.y, v.z);
  const cv::Vec3d out = matrix * vec;
  return Vec3d{out[0], out[1], out[2]};
}

std::string image_status_to_string(int status) {
  const auto enum_value = static_cast<::bosdyn::api::ImageResponse::Status>(status);
  const std::string name = ::bosdyn::api::ImageResponse::Status_Name(enum_value);
  return name.empty() ? std::string("STATUS_UNKNOWN") : name;
}

bool frame_has_stitch_calibration(const SpotClient::ImageFrame& frame) {
  return frame.has_body_tform_sensor &&
         frame.cols > 0 &&
         frame.rows > 0 &&
         frame.intrinsics.fx > 0.0 &&
         frame.intrinsics.fy > 0.0 &&
         frame.intrinsics.model_type != SpotClient::CameraIntrinsics::ModelType::kUnknown;
}

std::string build_front_stitch_calibration_signature(const SpotClient::ImageFrame& left,
                                                     const SpotClient::ImageFrame& right) {
  auto append_frame = [](std::ostringstream* oss, const SpotClient::ImageFrame& frame) {
    if (!oss) return;
    oss->setf(std::ios::fixed);
    oss->precision(12);
    *oss << frame.source_name << "|" << frame.cols << "x" << frame.rows
         << "|" << static_cast<int>(frame.intrinsics.model_type)
         << "|" << frame.intrinsics.fx << "|" << frame.intrinsics.fy
         << "|" << frame.intrinsics.cx << "|" << frame.intrinsics.cy
         << "|" << frame.intrinsics.skew_x << "|" << frame.intrinsics.skew_y
         << "|" << frame.intrinsics.k1 << "|" << frame.intrinsics.k2
         << "|" << frame.intrinsics.k3 << "|" << frame.intrinsics.k4
         << "|" << frame.body_tform_sensor.x << "|" << frame.body_tform_sensor.y
         << "|" << frame.body_tform_sensor.z << "|" << frame.body_tform_sensor.qx
         << "|" << frame.body_tform_sensor.qy << "|" << frame.body_tform_sensor.qz
         << "|" << frame.body_tform_sensor.qw;
  };
  std::ostringstream oss;
  append_frame(&oss, left);
  oss << "#";
  append_frame(&oss, right);
  return oss.str();
}

int normalize_front_roll_quarter_turns(int roll_degrees) {
  const int normalized = ((roll_degrees % 360) + 360) % 360;
  return ((normalized + 45) / 90) % 4;
}

std::string front_roll_label(int roll_quarter_turns) {
  switch ((roll_quarter_turns % 4 + 4) % 4) {
    case 1:
      return "cw90";
    case 2:
      return "180";
    case 3:
      return "ccw90";
    case 0:
    default:
      return "0";
  }
}

int front_roll_preference_rank(int roll_quarter_turns) {
  switch ((roll_quarter_turns % 4 + 4) % 4) {
    case 0:
      return 0;
    case 1:
      return 1;
    case 3:
      return 2;
    case 2:
    default:
      return 3;
  }
}

bool write_binary_file(const std::filesystem::path& path, const std::string& bytes) {
  std::ofstream ofs(path, std::ios::binary);
  if (!ofs) return false;
  ofs.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  return static_cast<bool>(ofs);
}

bool decode_jpeg_mat(const std::string& bytes, cv::Mat* out_image) {
  if (!out_image || bytes.empty()) return false;
  std::vector<uchar> compressed(bytes.begin(), bytes.end());
  cv::Mat decoded = cv::imdecode(compressed, cv::IMREAD_COLOR);
  if (decoded.empty()) return false;
  *out_image = std::move(decoded);
  return true;
}

bool build_front_camera_status_frame(const std::string& title,
                                     const std::string& detail,
                                     std::string* out_jpg) {
  if (!out_jpg) return false;
  const cv::Scalar canvas_bg(15, 22, 30);
  const cv::Scalar grid_minor(32, 41, 54);
  const cv::Scalar grid_major(46, 58, 74);
  const cv::Scalar text_primary(234, 239, 244);
  const cv::Scalar text_secondary(156, 171, 186);
  cv::Mat canvas(kLocalizationImageHeight, kLocalizationImageWidth, CV_8UC3, canvas_bg);
  for (int x = 0; x <= canvas.cols; x += 80) {
    cv::line(canvas, cv::Point(x, 0), cv::Point(x, canvas.rows - 1),
             (x % 240) == 0 ? grid_major : grid_minor, 1, cv::LINE_AA);
  }
  for (int y = 0; y <= canvas.rows; y += 80) {
    cv::line(canvas, cv::Point(0, y), cv::Point(canvas.cols - 1, y),
             (y % 240) == 0 ? grid_major : grid_minor, 1, cv::LINE_AA);
  }
  const std::string display_title =
      title.empty() ? std::string("Waiting for front camera stitch") : title;
  int baseline = 0;
  const cv::Size title_size =
      cv::getTextSize(display_title, cv::FONT_HERSHEY_SIMPLEX, 0.95, 2, &baseline);
  const cv::Point title_pt((canvas.cols - title_size.width) / 2, (canvas.rows / 2) - 18);
  cv::putText(canvas, display_title, title_pt, cv::FONT_HERSHEY_SIMPLEX, 0.95, text_primary, 2,
              cv::LINE_AA);
  int detail_y = title_pt.y + 42;
  const std::string display_detail =
      detail.empty() ? std::string("Waiting for both front fisheye images and calibration data.")
                     : detail;
  for (const auto& line : wrap_text(display_detail, 56)) {
    int detail_baseline = 0;
    const cv::Size line_size =
        cv::getTextSize(line, cv::FONT_HERSHEY_SIMPLEX, 0.58, 1, &detail_baseline);
    cv::putText(canvas, line, cv::Point((canvas.cols - line_size.width) / 2, detail_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.58, text_secondary, 1, cv::LINE_AA);
    detail_y += 28;
  }
  return encode_jpeg(canvas, out_jpg);
}

struct FrontStitchArtifacts {
  std::string signature;
  cv::Mat left_map_x;
  cv::Mat left_map_y;
  cv::Mat right_map_x;
  cv::Mat right_map_y;
  cv::Mat left_weight;
  cv::Mat right_weight;
};

struct FrontRollAutoSelection {
  bool valid{false};
  int roll_quarter_turns{0};
  double score{-std::numeric_limits<double>::infinity()};
  std::array<double, 4> candidate_scores{
      -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
  };
  FrontStitchArtifacts artifacts;
};

bool project_camera_ray_to_pixel(const SpotClient::ImageFrame& frame,
                                 const Vec3d& ray_camera,
                                 double* out_u,
                                 double* out_v,
                                 double* out_weight) {
  if (!out_u || !out_v) return false;
  if (ray_camera.z <= 1e-6) return false;

  const auto& intrinsics = frame.intrinsics;
  const double a = ray_camera.x / ray_camera.z;
  const double b = ray_camera.y / ray_camera.z;
  double distorted_x = a;
  double distorted_y = b;

  if (intrinsics.model_type == SpotClient::CameraIntrinsics::ModelType::kKannalaBrandt) {
    const double radius = std::sqrt(a * a + b * b);
    if (radius > 1e-9) {
      const double theta = std::atan(radius);
      const double theta2 = theta * theta;
      const double theta4 = theta2 * theta2;
      const double theta6 = theta4 * theta2;
      const double theta8 = theta4 * theta4;
      const double theta_distorted =
          theta * (1.0 + intrinsics.k1 * theta2 + intrinsics.k2 * theta4 +
                   intrinsics.k3 * theta6 + intrinsics.k4 * theta8);
      const double scale = theta_distorted / radius;
      distorted_x = a * scale;
      distorted_y = b * scale;
    }
  } else if (intrinsics.model_type != SpotClient::CameraIntrinsics::ModelType::kPinhole) {
    return false;
  }

  const double u = intrinsics.fx * distorted_x + intrinsics.skew_x * distorted_y + intrinsics.cx;
  const double v = intrinsics.skew_y * distorted_x + intrinsics.fy * distorted_y + intrinsics.cy;
  if (u < 0.0 || v < 0.0 ||
      u > static_cast<double>(frame.cols - 1) ||
      v > static_cast<double>(frame.rows - 1)) {
    return false;
  }

  *out_u = u;
  *out_v = v;
  if (out_weight) {
    const double cosine = std::max(0.0, ray_camera.z / norm_vec(ray_camera));
    *out_weight = cosine * cosine;
  }
  return true;
}

bool build_front_stitch_artifacts(const SpotClient::ImageFrame& left,
                                  const SpotClient::ImageFrame& right,
                                  FrontStitchArtifacts* out_artifacts,
                                  std::string* out_error) {
  if (!out_artifacts) return false;
  if (!frame_has_stitch_calibration(left) || !frame_has_stitch_calibration(right)) {
    if (out_error) *out_error = "front camera calibration is incomplete";
    return false;
  }

  const cv::Matx33d body_R_left = rotation_matrix_from_pose(left.body_tform_sensor);
  const cv::Matx33d body_R_right = rotation_matrix_from_pose(right.body_tform_sensor);
  const Vec3d body_forward_hint = normalize_vec(
      add_vec(mat_mul(body_R_left, Vec3d{0.0, 0.0, 1.0}),
              mat_mul(body_R_right, Vec3d{0.0, 0.0, 1.0})),
      Vec3d{1.0, 0.0, 0.0});
  Vec3d body_up_hint = normalize_vec(
      add_vec(scale_vec(mat_mul(body_R_left, Vec3d{0.0, 1.0, 0.0}), -1.0),
              scale_vec(mat_mul(body_R_right, Vec3d{0.0, 1.0, 0.0}), -1.0)),
      Vec3d{0.0, 0.0, 1.0});
  if (std::fabs(dot_vec(body_forward_hint, body_up_hint)) > 0.95) {
    body_up_hint = Vec3d{0.0, 0.0, 1.0};
  }
  Vec3d body_right = normalize_vec(cross_vec(body_up_hint, body_forward_hint),
                                   Vec3d{0.0, -1.0, 0.0});
  Vec3d body_down = normalize_vec(cross_vec(body_forward_hint, body_right),
                                  Vec3d{0.0, 0.0, 1.0});
  const Vec3d source_right_hint = normalize_vec(
      add_vec(mat_mul(body_R_left, Vec3d{1.0, 0.0, 0.0}),
              mat_mul(body_R_right, Vec3d{1.0, 0.0, 0.0})),
      body_right);
  if (dot_vec(body_right, source_right_hint) < 0.0) {
    body_right = scale_vec(body_right, -1.0);
    body_down = scale_vec(body_down, -1.0);
  }
  const cv::Matx33d body_R_virtual(
      body_right.x, body_down.x, body_forward_hint.x,
      body_right.y, body_down.y, body_forward_hint.y,
      body_right.z, body_down.z, body_forward_hint.z);
  const cv::Matx33d left_R_virtual = body_R_left.t() * body_R_virtual;
  const cv::Matx33d right_R_virtual = body_R_right.t() * body_R_virtual;

  constexpr int kOutputWidth = 1280;
  constexpr int kOutputHeight = 720;
  constexpr double kHorizontalFovDeg = 130.0;
  const double horizontal_fov_rad = kHorizontalFovDeg * 3.14159265358979323846 / 180.0;
  const double fx = (static_cast<double>(kOutputWidth) * 0.5) / std::tan(horizontal_fov_rad * 0.5);
  const double fy = fx;
  const double cx = (static_cast<double>(kOutputWidth) - 1.0) * 0.5;
  const double cy = (static_cast<double>(kOutputHeight) - 1.0) * 0.5;

  FrontStitchArtifacts artifacts;
  artifacts.signature = build_front_stitch_calibration_signature(left, right);
  artifacts.left_map_x = cv::Mat(kOutputHeight, kOutputWidth, CV_32FC1, cv::Scalar(0));
  artifacts.left_map_y = cv::Mat(kOutputHeight, kOutputWidth, CV_32FC1, cv::Scalar(0));
  artifacts.right_map_x = cv::Mat(kOutputHeight, kOutputWidth, CV_32FC1, cv::Scalar(0));
  artifacts.right_map_y = cv::Mat(kOutputHeight, kOutputWidth, CV_32FC1, cv::Scalar(0));
  artifacts.left_weight = cv::Mat(kOutputHeight, kOutputWidth, CV_32FC1, cv::Scalar(0));
  artifacts.right_weight = cv::Mat(kOutputHeight, kOutputWidth, CV_32FC1, cv::Scalar(0));

  for (int y = 0; y < kOutputHeight; ++y) {
    float* left_map_x_row = artifacts.left_map_x.ptr<float>(y);
    float* left_map_y_row = artifacts.left_map_y.ptr<float>(y);
    float* right_map_x_row = artifacts.right_map_x.ptr<float>(y);
    float* right_map_y_row = artifacts.right_map_y.ptr<float>(y);
    float* left_weight_row = artifacts.left_weight.ptr<float>(y);
    float* right_weight_row = artifacts.right_weight.ptr<float>(y);
    for (int x = 0; x < kOutputWidth; ++x) {
      const Vec3d ray_virtual = normalize_vec(
          Vec3d{(static_cast<double>(x) - cx) / fx, (static_cast<double>(y) - cy) / fy, 1.0},
          Vec3d{0.0, 0.0, 1.0});

      const Vec3d ray_left = mat_mul(left_R_virtual, ray_virtual);
      double pixel_u = 0.0;
      double pixel_v = 0.0;
      double weight = 0.0;
      if (project_camera_ray_to_pixel(left, ray_left, &pixel_u, &pixel_v, &weight)) {
        left_map_x_row[x] = static_cast<float>(pixel_u);
        left_map_y_row[x] = static_cast<float>(pixel_v);
        left_weight_row[x] = static_cast<float>(weight);
      }

      const Vec3d ray_right = mat_mul(right_R_virtual, ray_virtual);
      if (project_camera_ray_to_pixel(right, ray_right, &pixel_u, &pixel_v, &weight)) {
        right_map_x_row[x] = static_cast<float>(pixel_u);
        right_map_y_row[x] = static_cast<float>(pixel_v);
        right_weight_row[x] = static_cast<float>(weight);
      }
    }
  }

  *out_artifacts = std::move(artifacts);
  return true;
}

bool rotate_image_quarter_turns(const cv::Mat& input,
                                int roll_quarter_turns,
                                cv::Mat* out_image) {
  if (!out_image || input.empty()) return false;
  const int normalized = (roll_quarter_turns % 4 + 4) % 4;
  switch (normalized) {
    case 1:
      cv::rotate(input, *out_image, cv::ROTATE_90_CLOCKWISE);
      return true;
    case 2:
      cv::rotate(input, *out_image, cv::ROTATE_180);
      return true;
    case 3:
      cv::rotate(input, *out_image, cv::ROTATE_90_COUNTERCLOCKWISE);
      return true;
    case 0:
    default:
      *out_image = input.clone();
      return true;
  }
}

bool render_front_stitched_image(const FrontStitchArtifacts& artifacts,
                                 const cv::Mat* left_image,
                                 const cv::Mat* right_image,
                                 int roll_quarter_turns,
                                 cv::Mat* out_image,
                                 cv::Mat* out_valid_mask) {
  if (!out_image) return false;
  const bool have_left = left_image && !left_image->empty();
  const bool have_right = right_image && !right_image->empty();
  if (!have_left && !have_right) return false;

  cv::Mat left_rectified;
  cv::Mat right_rectified;
  if (have_left) {
    cv::remap(*left_image, left_rectified, artifacts.left_map_x, artifacts.left_map_y,
              cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  }
  if (have_right) {
    cv::remap(*right_image, right_rectified, artifacts.right_map_x, artifacts.right_map_y,
              cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  }

  cv::Mat composite(artifacts.left_map_x.rows, artifacts.left_map_x.cols, CV_8UC3,
                    cv::Scalar(10, 14, 18));
  cv::Mat valid_mask(artifacts.left_map_x.rows, artifacts.left_map_x.cols, CV_8UC1,
                     cv::Scalar(0));
  for (int y = 0; y < composite.rows; ++y) {
    const cv::Vec3b* left_row = have_left ? left_rectified.ptr<cv::Vec3b>(y) : nullptr;
    const cv::Vec3b* right_row = have_right ? right_rectified.ptr<cv::Vec3b>(y) : nullptr;
    const float* left_weight_row = have_left ? artifacts.left_weight.ptr<float>(y) : nullptr;
    const float* right_weight_row = have_right ? artifacts.right_weight.ptr<float>(y) : nullptr;
    cv::Vec3b* out_row = composite.ptr<cv::Vec3b>(y);
    uchar* mask_row = valid_mask.ptr<uchar>(y);
    for (int x = 0; x < composite.cols; ++x) {
      const float wl = left_weight_row ? left_weight_row[x] : 0.0f;
      const float wr = right_weight_row ? right_weight_row[x] : 0.0f;
      if (wl <= 0.0f && wr <= 0.0f) continue;
      mask_row[x] = 255;
      if (wr <= 0.0f) {
        out_row[x] = left_row[x];
        continue;
      }
      if (wl <= 0.0f) {
        out_row[x] = right_row[x];
        continue;
      }
      const float denom = wl + wr;
      cv::Vec3b pixel;
      for (int c = 0; c < 3; ++c) {
        pixel[c] = cv::saturate_cast<uchar>(
            (left_row[x][c] * wl + right_row[x][c] * wr) / denom);
      }
      out_row[x] = pixel;
    }
  }

  if (!rotate_image_quarter_turns(composite, roll_quarter_turns, out_image)) return false;
  if (out_valid_mask) {
    if (!rotate_image_quarter_turns(valid_mask, roll_quarter_turns, out_valid_mask)) return false;
  }
  return true;
}

double score_front_stitched_image(const FrontStitchArtifacts& artifacts,
                                  const cv::Mat& left_image,
                                  const cv::Mat& right_image,
                                  int roll_quarter_turns) {
  cv::Mat composite;
  cv::Mat valid_mask;
  if (!render_front_stitched_image(artifacts, &left_image, &right_image,
                                   roll_quarter_turns, &composite, &valid_mask)) {
    return -std::numeric_limits<double>::infinity();
  }

  const int inset_x = std::max(8, composite.cols / 10);
  const int inset_y = std::max(8, composite.rows / 10);
  const cv::Rect roi(inset_x, inset_y,
                     composite.cols - (2 * inset_x),
                     composite.rows - (2 * inset_y));
  if (roi.width <= 0 || roi.height <= 0) {
    return -std::numeric_limits<double>::infinity();
  }

  const cv::Mat roi_image = composite(roi);
  const cv::Mat roi_mask = valid_mask(roi);
  const int valid_pixels = cv::countNonZero(roi_mask);
  if (valid_pixels <= 0) return -std::numeric_limits<double>::infinity();

  cv::Mat gray;
  cv::Mat grad_x;
  cv::Mat grad_y;
  cv::Mat gradient_magnitude;
  cv::cvtColor(roi_image, gray, cv::COLOR_BGR2GRAY);
  cv::Sobel(gray, grad_x, CV_32F, 1, 0, 3);
  cv::Sobel(gray, grad_y, CV_32F, 0, 1, 3);
  cv::magnitude(grad_x, grad_y, gradient_magnitude);

  const double coverage =
      static_cast<double>(valid_pixels) / static_cast<double>(roi.width * roi.height);
  const double gradient_mean = cv::mean(gradient_magnitude, roi_mask)[0];
  return gradient_mean + (coverage * 0.01);
}

bool front_roll_score_is_better(double candidate_score,
                                int candidate_roll_quarter_turns,
                                double best_score,
                                int best_roll_quarter_turns) {
  if (!std::isfinite(candidate_score)) return false;
  if (!std::isfinite(best_score)) return true;
  constexpr double kScoreTieThreshold = 0.02;
  if (candidate_score > (best_score + kScoreTieThreshold)) return true;
  if (candidate_score + kScoreTieThreshold < best_score) return false;
  return front_roll_preference_rank(candidate_roll_quarter_turns) <
         front_roll_preference_rank(best_roll_quarter_turns);
}

FrontRollAutoSelection auto_select_front_roll(const SpotClient::ImageFrame& left_frame,
                                              const SpotClient::ImageFrame& right_frame,
                                              const cv::Mat& left_image,
                                              const cv::Mat& right_image) {
  FrontRollAutoSelection selection;
  FrontStitchArtifacts base_artifacts;
  std::string error;
  if (!build_front_stitch_artifacts(left_frame, right_frame, &base_artifacts, &error)) {
    return selection;
  }
  for (int roll_quarter_turns = 0; roll_quarter_turns < 4; ++roll_quarter_turns) {
    const double score =
        score_front_stitched_image(base_artifacts, left_image, right_image, roll_quarter_turns);
    selection.candidate_scores[roll_quarter_turns] = score;
    if (front_roll_score_is_better(score, roll_quarter_turns,
                                   selection.score, selection.roll_quarter_turns)) {
      selection.valid = true;
      selection.roll_quarter_turns = roll_quarter_turns;
      selection.score = score;
      selection.artifacts = base_artifacts;
    }
  }
  return selection;
}

std::string describe_front_roll_scores(const FrontRollAutoSelection& selection) {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(3);
  for (int roll_quarter_turns = 0; roll_quarter_turns < 4; ++roll_quarter_turns) {
    if (roll_quarter_turns > 0) oss << ", ";
    oss << front_roll_label(roll_quarter_turns) << "=";
    const double score = selection.candidate_scores[roll_quarter_turns];
    if (std::isfinite(score)) {
      oss << score;
    } else {
      oss << "n/a";
    }
  }
  return oss.str();
}

bool render_front_stitched_jpeg(const FrontStitchArtifacts& artifacts,
                                const cv::Mat* left_image,
                                const cv::Mat* right_image,
                                int roll_quarter_turns,
                                const std::string& status_title,
                                const std::string& status_detail,
                                std::string* out_jpg) {
  if (!out_jpg) return false;
  const bool have_left = left_image && !left_image->empty();
  const bool have_right = right_image && !right_image->empty();
  if (!have_left && !have_right) {
    return build_front_camera_status_frame(status_title, status_detail, out_jpg);
  }

  cv::Mat composite;
  if (!render_front_stitched_image(artifacts, left_image, right_image,
                                   roll_quarter_turns, &composite, nullptr)) {
    return false;
  }

  if (!status_title.empty() || !status_detail.empty()) {
    fill_rect_alpha(&composite, cv::Rect(20, 20, 270, status_detail.empty() ? 52 : 82),
                    cv::Scalar(12, 18, 24), 0.55);
    cv::putText(composite, status_title, cv::Point(34, 54), cv::FONT_HERSHEY_SIMPLEX, 0.72,
                cv::Scalar(236, 240, 245), 2, cv::LINE_AA);
    if (!status_detail.empty()) {
      cv::putText(composite, status_detail, cv::Point(34, 82), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(164, 177, 191), 1, cv::LINE_AA);
    }
  }
  return encode_jpeg(composite, out_jpg);
}

bool write_front_stitch_debug_bundle(const SpotClient::ImageFrame& left_frame,
                                     const SpotClient::ImageFrame& right_frame,
                                     const cv::Mat& left_image,
                                     const cv::Mat& right_image,
                                     int selected_roll_quarter_turns,
                                     std::string* out_dir) {
  const std::filesystem::path debug_dir =
      std::filesystem::path("logs") / "front-stitch-debug" / std::to_string(now_ms());
  std::error_code ec;
  std::filesystem::create_directories(debug_dir, ec);
  if (ec) return false;

  if (!write_binary_file(debug_dir / "left_raw.jpg", left_frame.encoded_image)) return false;
  if (!write_binary_file(debug_dir / "right_raw.jpg", right_frame.encoded_image)) return false;

  FrontStitchArtifacts debug_artifacts;
  std::string base_error;
  if (!build_front_stitch_artifacts(left_frame, right_frame, &debug_artifacts, &base_error)) {
    return false;
  }

  for (int roll_quarter_turns = 0; roll_quarter_turns < 4; ++roll_quarter_turns) {
    std::string output_jpg;
    if (!render_front_stitched_jpeg(debug_artifacts, &left_image, &right_image,
                                    roll_quarter_turns,
                                    std::string(), std::string(), &output_jpg)) {
      continue;
    }
    const std::string filename = "stitched_roll_" + front_roll_label(roll_quarter_turns) + ".jpg";
    if (!write_binary_file(debug_dir / filename, output_jpg)) return false;
  }

  std::ostringstream metadata;
  metadata << "selected_roll_quarter_turns=" << selected_roll_quarter_turns << "\n";
  metadata << "selected_roll_label=" << front_roll_label(selected_roll_quarter_turns) << "\n";
  metadata << "left_source=" << left_frame.source_name << "\n";
  metadata << "right_source=" << right_frame.source_name << "\n";
  metadata << "left_frame_name=" << left_frame.frame_name_image_sensor << "\n";
  metadata << "right_frame_name=" << right_frame.frame_name_image_sensor << "\n";
  metadata << "left_dims=" << left_frame.cols << "x" << left_frame.rows << "\n";
  metadata << "right_dims=" << right_frame.cols << "x" << right_frame.rows << "\n";
  metadata << "left_signature_status=" << left_frame.status << "\n";
  metadata << "right_signature_status=" << right_frame.status << "\n";
  if (!write_binary_file(debug_dir / "metadata.txt", metadata.str())) return false;

  if (out_dir) *out_dir = debug_dir.string();
  return true;
}

struct CachedImageState {
  std::mutex mu;
  std::condition_variable cv;
  std::shared_ptr<std::string> latest_jpg;
  uint64_t version = 0;
};

void update_cached_image(CachedImageState* state, std::shared_ptr<std::string> frame) {
  if (!state) return;
  {
    std::lock_guard<std::mutex> lk(state->mu);
    state->latest_jpg = std::move(frame);
    ++state->version;
  }
  state->cv.notify_one();
}

void publish_cached_image_loop(FormantAgentClient* agent,
                               const std::atomic<bool>* running,
                               CachedImageState* state,
                               const std::string& stream,
                               int output_period_ms,
                               int post_timeout_ms) {
  if (!agent || !running || !state) return;
  int post_backoff_ms = 0;
  long long next_post_allowed_ms = 0;
  long long next_keepalive_ms = now_ms();
  uint64_t last_published_version = 0;
  uint64_t last_attempted_version = 0;
  while (running->load()) {
    std::shared_ptr<std::string> frame;
    uint64_t version = 0;
    {
      std::unique_lock<std::mutex> lk(state->mu);
      for (;;) {
        frame = state->latest_jpg;
        version = state->version;
        const long long now = now_ms();
        const bool has_frame = frame && !frame->empty();
        const bool has_unpublished_frame = has_frame && version != last_published_version;
        const bool frame_changed_since_attempt =
            has_unpublished_frame && version != last_attempted_version;
        const bool can_retry = now >= next_post_allowed_ms;
        const bool should_post_fresh_frame =
            has_unpublished_frame && (frame_changed_since_attempt || can_retry);
        const bool keepalive_due =
            has_frame && version == last_published_version && now >= next_keepalive_ms && can_retry;
        if (should_post_fresh_frame || keepalive_due) break;
        if (!running->load()) return;

        long long wait_until_ms = 0;
        if (has_unpublished_frame && !frame_changed_since_attempt) {
          wait_until_ms = next_post_allowed_ms;
        } else if (has_frame) {
          wait_until_ms = next_keepalive_ms;
          if (!can_retry) wait_until_ms = std::min(wait_until_ms, next_post_allowed_ms);
        }

        if (wait_until_ms > 0) {
          const long long wait_ms = std::max<long long>(1, wait_until_ms - now);
          state->cv.wait_for(lk, std::chrono::milliseconds(std::min<long long>(100, wait_ms)));
        } else {
          state->cv.wait_for(lk, std::chrono::milliseconds(100));
        }
      }
    }

    if (!frame || frame->empty()) continue;

    const long long now = now_ms();
    last_attempted_version = version;
    const auto result = agent->PostImage(stream, "image/jpeg", *frame, post_timeout_ms);
    if (result.ok) {
      post_backoff_ms = 0;
      next_post_allowed_ms = 0;
      // Repeat the most recent JPEG at the configured stream cadence so downstream
      // video/clip generation sees a steady frame rate even when the producer
      // refreshes more slowly than the publish rate.
      next_keepalive_ms = now + output_period_ms;
      last_published_version = version;
    } else {
      if (result.retry_after_ms > now) {
        post_backoff_ms = 0;
        next_post_allowed_ms = result.retry_after_ms;
      } else {
        post_backoff_ms =
            std::min(500, std::max(100, post_backoff_ms == 0 ? 100 : post_backoff_ms * 2));
        next_post_allowed_ms = now + post_backoff_ms;
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

SpotClient::Pose3D pose3d_from_localization_snapshot(
    const SpotClient::LocalizationSnapshot& snapshot) {
  SpotClient::Pose3D pose;
  pose.x = snapshot.seed_tform_body_x;
  pose.y = snapshot.seed_tform_body_y;
  pose.z = snapshot.seed_tform_body_z;
  pose.qx = snapshot.seed_tform_body_qx;
  pose.qy = snapshot.seed_tform_body_qy;
  pose.qz = snapshot.seed_tform_body_qz;
  pose.qw = snapshot.seed_tform_body_qw;
  return pose;
}

v1::model::Map build_formant_map(const SpotClient::OccupancyGridMapSnapshot& map_snapshot,
                                 const std::string& uuid) {
  v1::model::Map map;
  map.set_uuid(uuid);
  map.set_resolution(map_snapshot.resolution_m);
  map.set_width(static_cast<uint32_t>(std::max(0, map_snapshot.width)));
  map.set_height(static_cast<uint32_t>(std::max(0, map_snapshot.height)));
  set_identity_transform(map.mutable_origin());
  set_transform_from_pose3d(map_snapshot.seed_tform_grid, map.mutable_world_to_local());
  for (int32_t cell : map_snapshot.occupancy) {
    map.mutable_occupancy_grid()->add_data(cell);
  }
  return map;
}

v1::model::Localization build_formant_localization(
    const SpotClient::Pose3D& seed_tform_body,
    const SpotClient::OccupancyGridMapSnapshot& map_snapshot,
    const std::string& uuid) {
  v1::model::Localization localization;

  auto* odometry = localization.mutable_odometry();
  set_transform_from_pose3d(seed_tform_body, odometry->mutable_pose());
  set_identity_transform(odometry->mutable_world_to_local());
  odometry->mutable_twist()->mutable_linear()->set_x(0.0);
  odometry->mutable_twist()->mutable_linear()->set_y(0.0);
  odometry->mutable_twist()->mutable_linear()->set_z(0.0);
  odometry->mutable_twist()->mutable_angular()->set_x(0.0);
  odometry->mutable_twist()->mutable_angular()->set_y(0.0);
  odometry->mutable_twist()->mutable_angular()->set_z(0.0);

  *localization.mutable_map() = build_formant_map(map_snapshot, uuid);
  return localization;
}

v1::model::Localization build_formant_localization(
    const SpotClient::LocalizationMapSnapshot& snapshot) {
  return build_formant_localization(snapshot.seed_tform_body, snapshot.map,
                                    snapshot.waypoint_id + ":" + snapshot.map.map_type);
}

uint64_t fnv1a64(const std::string& data) {
  uint64_t hash = 1469598103934665603ULL;
  for (unsigned char ch : data) {
    hash ^= static_cast<uint64_t>(ch);
    hash *= 1099511628211ULL;
  }
  return hash;
}

uint64_t fnv1a64_append(uint64_t hash, const std::string& data) {
  for (unsigned char ch : data) {
    hash ^= static_cast<uint64_t>(ch);
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::string hex_u64(uint64_t value) {
  static constexpr char kHex[] = "0123456789abcdef";
  std::string out(16, '0');
  for (int i = 15; i >= 0; --i) {
    out[static_cast<size_t>(i)] = kHex[value & 0x0fULL];
    value >>= 4U;
  }
  return out;
}

std::string build_graphnav_map_uuid(const std::string& map_id,
                                    const SpotClient::StoredMap& map_data) {
  std::string graph_bytes;
  (void)map_data.graph.SerializeToString(&graph_bytes);
  uint64_t hash = fnv1a64(graph_bytes);

  std::vector<const ::bosdyn::api::graph_nav::WaypointSnapshot*> waypoint_snapshots;
  waypoint_snapshots.reserve(map_data.waypoint_snapshots.size());
  for (const auto& snapshot : map_data.waypoint_snapshots) {
    waypoint_snapshots.push_back(&snapshot);
  }
  std::sort(waypoint_snapshots.begin(), waypoint_snapshots.end(),
            [](const auto* lhs, const auto* rhs) {
              if (!lhs || !rhs) return lhs < rhs;
              return lhs->id() < rhs->id();
            });
  for (const auto* snapshot : waypoint_snapshots) {
    if (!snapshot) continue;
    std::string snapshot_bytes;
    (void)snapshot->SerializeToString(&snapshot_bytes);
    hash = fnv1a64_append(hash, snapshot_bytes);
  }

  std::vector<const ::bosdyn::api::graph_nav::EdgeSnapshot*> edge_snapshots;
  edge_snapshots.reserve(map_data.edge_snapshots.size());
  for (const auto& snapshot : map_data.edge_snapshots) {
    edge_snapshots.push_back(&snapshot);
  }
  std::sort(edge_snapshots.begin(), edge_snapshots.end(),
            [](const auto* lhs, const auto* rhs) {
              if (!lhs || !rhs) return lhs < rhs;
              return lhs->id() < rhs->id();
            });
  for (const auto* snapshot : edge_snapshots) {
    if (!snapshot) continue;
    std::string snapshot_bytes;
    (void)snapshot->SerializeToString(&snapshot_bytes);
    hash = fnv1a64_append(hash, snapshot_bytes);
  }

  return (map_id.empty() ? std::string("graphnav") : map_id) + ":" + hex_u64(hash);
}

std::string pose3d_to_json(const SpotClient::Pose3D& pose);
SpotClient::Pose3D pose3d_from_localization_snapshot(
    const SpotClient::LocalizationSnapshot& snapshot);

GraphNavMapGeometry graphnav_map_geometry_from_snapshot(
    const SpotClient::OccupancyGridMapSnapshot& map_snapshot) {
  GraphNavMapGeometry geometry;
  geometry.width = map_snapshot.width;
  geometry.height = map_snapshot.height;
  geometry.resolution_m = map_snapshot.resolution_m;
  geometry.seed_tform_grid.x = map_snapshot.seed_tform_grid.x;
  geometry.seed_tform_grid.y = map_snapshot.seed_tform_grid.y;
  geometry.seed_tform_grid.yaw_rad =
      quaternion_yaw_rad(pose_quaternion(map_snapshot.seed_tform_grid));
  return geometry;
}

std::string build_graphnav_metadata_json(
    const std::string& map_id,
    const std::string& map_uuid,
    const SpotClient::GraphNavMapSnapshot& graphnav_snapshot,
    const std::unordered_map<std::string, std::vector<std::string>>& aliases_by_waypoint,
    const std::string& dock_waypoint_id) {
  std::ostringstream metadata;
  metadata << "{\"map_id\":\"" << json_escape(map_id)
           << "\",\"map_uuid\":\"" << json_escape(map_uuid)
           << "\",\"has_anchoring\":" << (graphnav_snapshot.has_anchoring ? "true" : "false")
           << ",\"has_map\":" << (graphnav_snapshot.has_map ? "true" : "false");
  if (graphnav_snapshot.has_map) {
    metadata << ",\"map\":{\"type\":\"" << json_escape(graphnav_snapshot.map.map_type)
             << "\",\"resolution_m\":" << graphnav_snapshot.map.resolution_m
             << ",\"width\":" << graphnav_snapshot.map.width
             << ",\"height\":" << graphnav_snapshot.map.height
             << ",\"seed_tform_grid\":" << pose3d_to_json(graphnav_snapshot.map.seed_tform_grid)
             << "}";
  }
  metadata << ",\"waypoints\":[";
  for (size_t i = 0; i < graphnav_snapshot.waypoints.size(); ++i) {
    const auto& waypoint = graphnav_snapshot.waypoints[i];
    const auto alias_it = aliases_by_waypoint.find(waypoint.id);
    const std::vector<std::string>* aliases =
        (alias_it == aliases_by_waypoint.end()) ? nullptr : &alias_it->second;
    const std::string display_name =
        (aliases && !aliases->empty()) ? aliases->front()
                                       : (waypoint.label.empty() ? waypoint.id : waypoint.label);
    if (i > 0) metadata << ",";
    metadata << "{\"id\":\"" << json_escape(waypoint.id)
             << "\",\"snapshot_id\":\"" << json_escape(waypoint.snapshot_id)
             << "\",\"label\":\"" << json_escape(waypoint.label)
             << "\",\"display_name\":\"" << json_escape(display_name)
             << "\",\"is_dock\":" << (waypoint.id == dock_waypoint_id ? "true" : "false")
             << ",\"aliases\":[";
    if (aliases) {
      for (size_t alias_index = 0; alias_index < aliases->size(); ++alias_index) {
        if (alias_index > 0) metadata << ",";
        metadata << "\"" << json_escape((*aliases)[alias_index]) << "\"";
      }
    }
    metadata << "],\"seed_tform_waypoint\":" << pose3d_to_json(waypoint.seed_tform_waypoint)
             << "}";
  }
  metadata << "],\"edges\":[";
  for (size_t i = 0; i < graphnav_snapshot.edges.size(); ++i) {
    const auto& edge = graphnav_snapshot.edges[i];
    if (i > 0) metadata << ",";
    metadata << "{\"from_waypoint_id\":\"" << json_escape(edge.from_waypoint_id)
             << "\",\"to_waypoint_id\":\"" << json_escape(edge.to_waypoint_id)
             << "\",\"snapshot_id\":\"" << json_escape(edge.snapshot_id)
             << "\",\"from_tform_to\":" << pose3d_to_json(edge.from_tform_to)
             << "}";
  }
  metadata << "],\"objects\":[";
  for (size_t i = 0; i < graphnav_snapshot.objects.size(); ++i) {
    const auto& object = graphnav_snapshot.objects[i];
    if (i > 0) metadata << ",";
    metadata << "{\"id\":\"" << json_escape(object.id)
             << "\",\"seed_tform_object\":" << pose3d_to_json(object.seed_tform_object)
             << "}";
  }
  metadata << "]}";
  return metadata.str();
}

std::string build_graphnav_map_image_metadata_json(
    const std::string& image_stream,
    const std::string& metadata_stream,
    const std::string& map_id,
    const std::string& map_uuid,
    const SpotClient::OccupancyGridMapSnapshot* map_snapshot,
    const SpotClient::LocalizationSnapshot* localization,
    const GraphNavMapImageLayout* layout,
    const std::string& status_title,
    const std::string& status_detail) {
  std::ostringstream metadata;
  metadata << "{\"image_stream\":\"" << json_escape(image_stream)
           << "\",\"metadata_stream\":\"" << json_escape(metadata_stream)
           << "\",\"has_map\":"
           << ((map_snapshot && layout) ? "true" : "false")
           << ",\"status_title\":\"" << json_escape(status_title)
           << "\",\"status_detail\":\"" << json_escape(status_detail) << "\"";

  metadata << ",\"map_id\":\"" << json_escape(map_id)
           << "\",\"map_uuid\":\"" << json_escape(map_uuid) << "\"";

  const bool localized = localization && !localization->waypoint_id.empty();
  const bool has_seed_pose = localized && localization->has_seed_tform_body;
  metadata << ",\"localized\":" << (localized ? "true" : "false")
           << ",\"current_waypoint_id\":\""
           << json_escape(localized ? localization->waypoint_id : std::string())
           << "\",\"has_live_pose_overlay\":" << (has_seed_pose ? "true" : "false");
  if (has_seed_pose) {
    const SpotClient::Pose3D seed_tform_body = pose3d_from_localization_snapshot(*localization);
    metadata << ",\"current_seed_x\":" << seed_tform_body.x
             << ",\"current_seed_y\":" << seed_tform_body.y
             << ",\"current_seed_z\":" << seed_tform_body.z
             << ",\"current_seed_yaw_rad\":"
             << quaternion_yaw_rad(pose_quaternion(seed_tform_body));
  }

  if (layout && map_snapshot) {
    metadata << ",\"canvas\":{\"width\":" << layout->canvas_width
             << ",\"height\":" << layout->canvas_height << "}"
             << ",\"draw_rect\":{\"x\":" << layout->draw_x
             << ",\"y\":" << layout->draw_y
             << ",\"width\":" << layout->draw_width
             << ",\"height\":" << layout->draw_height << "}"
             << ",\"render_scale\":" << layout->scale
             << ",\"map\":{\"width\":" << map_snapshot->width
             << ",\"height\":" << map_snapshot->height
             << ",\"resolution_m\":" << map_snapshot->resolution_m
             << ",\"seed_tform_grid\":" << pose3d_to_json(map_snapshot->seed_tform_grid)
             << "}";
  }

  metadata << "}";
  return metadata.str();
}

std::string map_staging_directory(const std::string& map_dir) {
  return map_dir + ".tmp";
}

std::string map_backup_directory(const std::string& map_dir) {
  return map_dir + ".bak";
}

std::string map_staging_ready_marker(const std::string& temp_dir) {
  return temp_dir + "/.complete";
}

bool recover_map_directory_state(const std::string& map_dir, std::string* out_error) {
  std::error_code ec;
  const std::string temp_dir = map_staging_directory(map_dir);
  const std::string backup_dir = map_backup_directory(map_dir);

  const bool has_map_dir = std::filesystem::exists(map_dir, ec);
  if (ec) {
    if (out_error) *out_error = "failed checking map directory: " + ec.message();
    return false;
  }
  const bool has_backup_dir = std::filesystem::exists(backup_dir, ec);
  if (ec) {
    if (out_error) *out_error = "failed checking map backup directory: " + ec.message();
    return false;
  }
  const bool has_temp_dir = std::filesystem::exists(temp_dir, ec);
  if (ec) {
    if (out_error) *out_error = "failed checking map staging directory: " + ec.message();
    return false;
  }

  if (has_map_dir) {
    if (has_backup_dir) {
      std::filesystem::remove_all(backup_dir, ec);
      if (ec) {
        if (out_error) *out_error = "failed removing stale map backup: " + ec.message();
        return false;
      }
    }
    if (has_temp_dir) {
      std::filesystem::remove_all(temp_dir, ec);
      if (ec) {
        if (out_error) *out_error = "failed removing stale map staging dir: " + ec.message();
        return false;
      }
    }
    return true;
  }

  if (has_backup_dir) {
    if (has_temp_dir) {
      std::filesystem::remove_all(temp_dir, ec);
      if (ec) {
        if (out_error) *out_error = "failed removing stale map staging dir: " + ec.message();
        return false;
      }
    }
    std::filesystem::rename(backup_dir, map_dir, ec);
    if (ec) {
      if (out_error) *out_error = "failed restoring map backup: " + ec.message();
      return false;
    }
    return true;
  }

  if (has_temp_dir) {
    const bool staging_ready = std::filesystem::exists(map_staging_ready_marker(temp_dir), ec);
    if (ec) {
      if (out_error) *out_error = "failed checking map staging marker: " + ec.message();
      return false;
    }
    if (!staging_ready) {
      std::filesystem::remove_all(temp_dir, ec);
      if (ec) {
        if (out_error) *out_error = "failed removing incomplete map staging dir: " + ec.message();
        return false;
      }
    } else {
      std::filesystem::rename(temp_dir, map_dir, ec);
      if (ec) {
        if (out_error) *out_error = "failed promoting staged map dir: " + ec.message();
        return false;
      }
    }
  }
  return true;
}

std::string pose3d_to_json(const SpotClient::Pose3D& pose) {
  std::ostringstream oss;
  oss << "{\"x\":" << pose.x
      << ",\"y\":" << pose.y
      << ",\"z\":" << pose.z
      << ",\"qx\":" << pose.qx
      << ",\"qy\":" << pose.qy
      << ",\"qz\":" << pose.qz
      << ",\"qw\":" << pose.qw
      << ",\"yaw_rad\":" << quaternion_yaw_rad(pose_quaternion(pose)) << "}";
  return oss.str();
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
  spot_.SetArmPresentOverride(cfg_.arm_present_override);

  if (!spot_.Connect(cfg_.spot_host, cfg_.spot_username, cfg_.spot_password)) {
    std::cerr << "Initial Spot connect failed; adapter will continue and retry in background: "
              << spot_.LastError() << std::endl;
    SetSpotDisconnected(spot_.LastError());
  } else {
    SetSpotConnected();
  }

  DisableConfiguredStreamIfDisabled(&cfg_.can_dock_stream, "can_dock_stream");
  DisableConfiguredStreamIfDisabled(&cfg_.stateful_mode_stream, "stateful_mode_stream");
  DisableConfiguredStreamIfDisabled(&cfg_.camera_stream_name, "camera_stream_name");
  DisableConfiguredStreamIfDisabled(&cfg_.left_camera_stream_name, "left_camera_stream_name");
  DisableConfiguredStreamIfDisabled(&cfg_.right_camera_stream_name, "right_camera_stream_name");
  DisableConfiguredStreamIfDisabled(&cfg_.back_camera_stream_name, "back_camera_stream_name");
  DisableConfiguredStreamIfDisabled(&cfg_.front_image_stream_name, "front_image_stream_name");
  DisableConfiguredStreamIfDisabled(&cfg_.localization_image_stream_name,
                                    "localization_image_stream_name");
  DisableConfiguredStreamIfDisabled(&cfg_.graphnav_global_localization_stream,
                                    "graphnav_global_localization_stream");
  DisableConfiguredStreamIfDisabled(&cfg_.graphnav_map_stream, "graphnav_map_stream");
  DisableConfiguredStreamIfDisabled(&cfg_.graphnav_metadata_stream, "graphnav_metadata_stream");
  DisableConfiguredStreamIfDisabled(&cfg_.graphnav_nav_state_stream, "graphnav_nav_state_stream");
  DisableConfiguredStreamIfDisabled(&cfg_.graphnav_map_image_stream_name,
                                    "graphnav_map_image_stream_name");
  DisableConfiguredStreamIfDisabled(&cfg_.graphnav_map_image_metadata_stream_name,
                                    "graphnav_map_image_metadata_stream_name");
  DisableConfiguredStreamIfDisabled(&cfg_.waypoint_text_stream, "waypoint_text_stream");
  DisableConfiguredStreamIfDisabled(&cfg_.maps_text_stream, "maps_text_stream");

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

    if (!cfg_.front_image_stream_name.empty()) {
      const auto left_it = source_map.find(cfg_.front_left_camera_source);
      const auto right_it = source_map.find(cfg_.front_right_camera_source);
      const bool left_ok = left_it != source_map.end() &&
                           left_it->second.cols > 0 &&
                           left_it->second.rows > 0;
      const bool right_ok = right_it != source_map.end() &&
                            right_it->second.cols > 0 &&
                            right_it->second.rows > 0;
      if (!left_ok || !right_ok) {
        std::cerr << "[camera] stitched front stream disabled:"
                  << " left_source=" << cfg_.front_left_camera_source
                  << " right_source=" << cfg_.front_right_camera_source
                  << " reason=" << (!left_ok ? "left unavailable " : "")
                  << (!right_ok ? "right unavailable" : "")
                  << std::endl;
        cfg_.front_image_stream_name.clear();
      } else {
        std::cerr << "[camera] stitched front stream config: stream=" << cfg_.front_image_stream_name
                  << " left_source=" << cfg_.front_left_camera_source
                  << " right_source=" << cfg_.front_right_camera_source
                  << " output_fps=" << std::max(1, cfg_.front_image_fps)
                  << " data_poll_hz=" << std::max(1, cfg_.front_image_poll_hz)
                  << " roll_degrees="
                  << (cfg_.front_image_roll_degrees_configured
                          ? std::to_string(cfg_.front_image_roll_degrees)
                          : std::string("auto"))
                  << " left_size=" << left_it->second.cols << "x" << left_it->second.rows
                  << " right_size=" << right_it->second.cols << "x" << right_it->second.rows
                  << std::endl;
      }
    }
  } else {
    std::cerr << "[camera] failed to list image sources: " << spot_.LastError() << std::endl;
  }
  if (!cfg_.localization_image_stream_name.empty()) {
    std::cerr << "[localization-image] stream config: stream=" << cfg_.localization_image_stream_name
              << " output_fps=" << std::max(1, cfg_.localization_image_fps)
              << " data_poll_hz=" << std::max(1, cfg_.localization_image_poll_hz)
              << " sdk_poll_cap_hz=5" << std::endl;
  }
  if (!cfg_.graphnav_global_localization_stream.empty()) {
    std::cerr << "[graphnav] global localization stream config: stream="
              << cfg_.graphnav_global_localization_stream
              << " publish_hz=" << std::max(1, cfg_.graphnav_global_localization_hz)
              << std::endl;
  }
  if (!cfg_.graphnav_nav_state_stream.empty()) {
    std::cerr << "[graphnav] nav state stream config: stream="
              << cfg_.graphnav_nav_state_stream
              << " publish_hz=1"
              << std::endl;
  }
  if (!cfg_.graphnav_map_image_stream_name.empty() ||
      !cfg_.graphnav_map_image_metadata_stream_name.empty()) {
    std::cerr << "[graphnav] global map image config: image_stream="
              << (cfg_.graphnav_map_image_stream_name.empty()
                      ? std::string("(disabled)")
                      : cfg_.graphnav_map_image_stream_name)
              << " metadata_stream="
              << (cfg_.graphnav_map_image_metadata_stream_name.empty()
                      ? std::string("(disabled)")
                      : cfg_.graphnav_map_image_metadata_stream_name)
              << " output_fps=" << std::max(1, cfg_.graphnav_map_image_fps)
              << " data_poll_hz=" << std::max(1, cfg_.graphnav_map_image_poll_hz)
              << std::endl;
  }
  std::cerr << "[camera] surround stream config: output_fps=" << std::max(1, cfg_.surround_camera_fps)
            << " data_poll_hz=" << std::max(1, cfg_.surround_camera_poll_hz)
            << " batch_sources=left,right,back"
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
  force_graphnav_metadata_publish_ = true;
  next_graphnav_map_post_ms_ = 0;
  graphnav_map_post_backoff_ms_ = 1000;
  last_graphnav_map_posted_version_ = 0;

  std::string initial_map_id;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    initial_map_id = active_map_id_;
    if (initial_map_id.empty()) initial_map_id = default_map_id_;
  }
  if (!initial_map_id.empty()) {
    (void)RefreshGraphNavMapArtifactsFromDisk(initial_map_id);
  } else {
    ClearGraphNavMapArtifacts();
  }
  return true;
}

bool Adapter::IsStreamEnabled(const std::string& stream) const {
  return cfg_.IsStreamEnabled(stream);
}

bool Adapter::AnyStreamEnabled(std::initializer_list<const char*> streams) const {
  for (const char* stream : streams) {
    if (stream && *stream && IsStreamEnabled(stream)) return true;
  }
  return false;
}

void Adapter::DisableConfiguredStreamIfDisabled(std::string* stream, const char* label) {
  if (!stream || stream->empty()) return;
  if (IsStreamEnabled(*stream)) return;
  std::cerr << "[stream] disabled " << (label ? label : "stream")
            << ": " << *stream << std::endl;
  stream->clear();
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
                           "spot.undock",
                           "spot.return_and_dock", "spot.rotate_left", "spot.rotate_right",
                           "spot.reset_arm",
                           "spot.map.create", "spot.map.load", "spot.map.set_default", "spot.map.delete",
                           "spot.map.start_mapping", "spot.map.stop_mapping",
                           "spot.waypoint.save",
                           "spot.waypoint.delete", "spot.waypoint.goto",
                           "spot.waypoint.goto_straight",
                           "spot.graphnav.goto_pose", "spot.graphnav.goto_pose_straight"},
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
  const bool have_surround_stream =
      (!cfg_.left_camera_source.empty() && !cfg_.left_camera_stream_name.empty()) ||
      (!cfg_.right_camera_source.empty() && !cfg_.right_camera_stream_name.empty()) ||
      (!cfg_.back_camera_source.empty() && !cfg_.back_camera_stream_name.empty());
  if (have_surround_stream) {
    surround_image_thread_ = std::thread(&Adapter::SurroundImageLoop, this,
                                         surround_output_fps, surround_poll_hz);
  }
  if (!cfg_.front_image_stream_name.empty()) {
    front_image_thread_ = std::thread(&Adapter::FrontImageLoop, this,
                                      cfg_.front_image_stream_name,
                                      std::max(1, cfg_.front_image_fps),
                                      std::max(1, cfg_.front_image_poll_hz));
  }
  if (!cfg_.localization_image_stream_name.empty()) {
    localization_image_thread_ = std::thread(&Adapter::LocalizationImageLoop, this,
                                             cfg_.localization_image_stream_name,
                                             std::max(1, cfg_.localization_image_fps),
                                             std::max(1, cfg_.localization_image_poll_hz));
  }
  if (!cfg_.graphnav_map_image_stream_name.empty() ||
      !cfg_.graphnav_map_image_metadata_stream_name.empty()) {
    graphnav_map_image_thread_ = std::thread(&Adapter::GraphNavMapImageLoop, this,
                                             cfg_.graphnav_map_image_stream_name,
                                             std::max(1, cfg_.graphnav_map_image_fps),
                                             std::max(1, cfg_.graphnav_map_image_poll_hz));
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
    PublishGraphNavMetadataText(false);
    PublishCurrentMapText(false);
    PublishDefaultMapText(false);
    MaybePublishGraphNavMap(now);

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
  if (surround_image_thread_.joinable()) surround_image_thread_.join();
  if (front_image_thread_.joinable()) front_image_thread_.join();
  if (localization_image_thread_.joinable()) localization_image_thread_.join();
  if (graphnav_map_image_thread_.joinable()) graphnav_map_image_thread_.join();
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
  constexpr int kMaxImageFetchBackoffMs = 1000;
  const int output_period_ms = std::max(1, 1000 / std::max(1, output_fps));
  const int poll_period_ms = std::max(1, 1000 / std::max(1, poll_hz));
  long long last_error_log_ms = 0;
  int failure_backoff_ms = poll_period_ms;
  CachedImageState cached_image;

  std::thread sender([this, &cached_image, &stream, output_period_ms, post_timeout_ms]() {
    publish_cached_image_loop(&agent_, &running_, &cached_image, stream,
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
        update_cached_image(&cached_image, std::move(frame));
      } else {
        const long long now = now_ms();
        if ((now - last_error_log_ms) >= 2000) {
          std::cerr << "[camera] failed source=" << source
                    << " stream=" << stream
                    << " error=failed to prepare jpeg frame" << std::endl;
          last_error_log_ms = now;
        }
        failure_backoff_ms =
            std::min(kMaxImageFetchBackoffMs, std::max(poll_period_ms, failure_backoff_ms * 2));
      }
    } else {
      const long long now = now_ms();
      if ((now - last_error_log_ms) >= 2000) {
        std::cerr << "[camera] failed source=" << source
                  << " stream=" << stream
                  << " error=" << spot_.LastError() << std::endl;
        last_error_log_ms = now;
      }
      failure_backoff_ms =
          std::min(kMaxImageFetchBackoffMs, std::max(poll_period_ms, failure_backoff_ms * 2));
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

void Adapter::SurroundImageLoop(int output_fps, int poll_hz) {
  constexpr int kMaxImageFetchBackoffMs = 1000;
  const int output_period_ms = std::max(1, 1000 / std::max(1, output_fps));
  const int poll_period_ms = std::max(1, 1000 / std::max(1, poll_hz));
  int failure_backoff_ms = poll_period_ms;
  long long last_batch_error_log_ms = 0;
  std::unordered_map<std::string, long long> last_stream_error_log_ms;

  struct SurroundStreamState {
    std::string source;
    std::string stream;
    bool rotate_180{false};
    bool normalize_jpeg{false};
    int post_timeout_ms{250};
    CachedImageState cached_image;
  };

  auto make_stream_state = [](const std::string& source,
                              const std::string& stream,
                              bool rotate_180,
                              bool normalize_jpeg,
                              int post_timeout_ms) {
    auto state = std::make_unique<SurroundStreamState>();
    state->source = source;
    state->stream = stream;
    state->rotate_180 = rotate_180;
    state->normalize_jpeg = normalize_jpeg;
    state->post_timeout_ms = post_timeout_ms;
    return state;
  };

  std::vector<std::unique_ptr<SurroundStreamState>> stream_states;
  if (!cfg_.left_camera_source.empty() && !cfg_.left_camera_stream_name.empty()) {
    stream_states.push_back(make_stream_state(
        cfg_.left_camera_source, cfg_.left_camera_stream_name, false, true, 250));
  }
  if (!cfg_.right_camera_source.empty() && !cfg_.right_camera_stream_name.empty()) {
    stream_states.push_back(make_stream_state(
        cfg_.right_camera_source, cfg_.right_camera_stream_name, cfg_.right_camera_rotate_180,
        true, 250));
  }
  if (!cfg_.back_camera_source.empty() && !cfg_.back_camera_stream_name.empty()) {
    stream_states.push_back(make_stream_state(
        cfg_.back_camera_source, cfg_.back_camera_stream_name, false, true, 250));
  }
  if (stream_states.empty()) return;

  std::vector<std::thread> sender_threads;
  std::vector<std::string> sources;
  sources.reserve(stream_states.size());
  sender_threads.reserve(stream_states.size());
  for (const auto& stream_state : stream_states) {
    sources.push_back(stream_state->source);
    auto* raw_state = stream_state.get();
    sender_threads.emplace_back([this, raw_state, output_period_ms]() {
      publish_cached_image_loop(&agent_, &running_, &raw_state->cached_image, raw_state->stream,
                                output_period_ms, raw_state->post_timeout_ms);
    });
  }

  auto find_frame = [](const std::vector<SpotClient::ImageFrame>& frames,
                       const std::string& source_name) -> const SpotClient::ImageFrame* {
    for (const auto& frame : frames) {
      if (frame.source_name == source_name) return &frame;
    }
    return nullptr;
  };

  auto log_batch_error = [&last_batch_error_log_ms](const std::string& message) {
    const long long now = now_ms();
    if ((now - last_batch_error_log_ms) < 2000) return;
    std::cerr << message << std::endl;
    last_batch_error_log_ms = now;
  };

  auto log_stream_error = [&last_stream_error_log_ms](const std::string& stream_key,
                                                      const std::string& message) {
    const long long now = now_ms();
    long long& last_log_ms = last_stream_error_log_ms[stream_key];
    if ((now - last_log_ms) < 2000) return;
    std::cerr << message << std::endl;
    last_log_ms = now;
  };

  while (running_) {
    if (now_ms() < dock_cooldown_until_ms_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    if (!SpotConnected()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }

    std::vector<SpotClient::ImageFrame> frames;
    bool updated_any_frame = false;
    if (spot_.GetImageFrames(sources, &frames)) {
      for (const auto& stream_state : stream_states) {
        const auto* frame = find_frame(frames, stream_state->source);
        if (!frame) {
          log_stream_error(stream_state->stream,
                           "[camera] surround batch missing source=" + stream_state->source +
                               " stream=" + stream_state->stream);
          continue;
        }
        if (frame->status != static_cast<int>(::bosdyn::api::ImageResponse::STATUS_OK)) {
          log_stream_error(stream_state->stream,
                           "[camera] surround batch bad status source=" + stream_state->source +
                               " stream=" + stream_state->stream +
                               " status=" + image_status_to_string(frame->status));
          continue;
        }
        std::string prepared_jpg;
        if (!prepare_jpeg_frame(frame->encoded_image, stream_state->rotate_180,
                                stream_state->normalize_jpeg, &prepared_jpg)) {
          log_stream_error(stream_state->stream,
                           "[camera] surround batch failed source=" + stream_state->source +
                               " stream=" + stream_state->stream +
                               " error=failed to prepare jpeg frame");
          continue;
        }
        auto output_frame = std::make_shared<std::string>();
        output_frame->swap(prepared_jpg);
        update_cached_image(&stream_state->cached_image, std::move(output_frame));
        updated_any_frame = true;
      }
      if (updated_any_frame) {
        failure_backoff_ms = poll_period_ms;
      } else {
        failure_backoff_ms =
            std::min(kMaxImageFetchBackoffMs, std::max(poll_period_ms, failure_backoff_ms * 2));
      }
    } else {
      log_batch_error("[camera] failed fetching surround image batch: " + spot_.LastError());
      failure_backoff_ms =
          std::min(kMaxImageFetchBackoffMs, std::max(poll_period_ms, failure_backoff_ms * 2));
    }

    const int step_ms = 25;
    int slept_ms = 0;
    while (running_ && slept_ms < failure_backoff_ms) {
      const int sleep_ms = std::min(step_ms, failure_backoff_ms - slept_ms);
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
      slept_ms += sleep_ms;
    }
  }

  for (auto& sender : sender_threads) {
    if (sender.joinable()) sender.join();
  }
}

void Adapter::FrontImageLoop(const std::string& stream, int fps, int poll_hz) {
  constexpr int kMaxImageFetchBackoffMs = 1000;
  const int output_period_ms = std::max(1, 1000 / std::max(1, fps));
  const int poll_period_ms = std::max(1, 1000 / std::max(1, poll_hz));
  const bool manual_roll_override = cfg_.front_image_roll_degrees_configured;
  const int configured_roll_quarter_turns =
      normalize_front_roll_quarter_turns(cfg_.front_image_roll_degrees);
  long long last_error_log_ms = 0;
  long long last_good_frame_ms = 0;
  int failure_backoff_ms = poll_period_ms;
  bool have_rendered_frame = false;
  bool have_artifacts = false;
  bool debug_capture_attempted = false;
  int selected_roll_quarter_turns = configured_roll_quarter_turns;
  std::string selected_calibration_signature;
  FrontStitchArtifacts artifacts;
  CachedImageState cached_image;
  const std::vector<std::string> sources{
      cfg_.front_left_camera_source,
      cfg_.front_right_camera_source,
  };

  auto maybe_log_error = [&last_error_log_ms](const std::string& message) {
    const long long now = now_ms();
    if ((now - last_error_log_ms) < 2000) return;
    std::cerr << message << std::endl;
    last_error_log_ms = now;
  };

  auto find_frame = [](const std::vector<SpotClient::ImageFrame>& frames,
                       const std::string& source_name) -> const SpotClient::ImageFrame* {
    for (const auto& frame : frames) {
      if (frame.source_name == source_name) return &frame;
    }
    return nullptr;
  };

  auto build_status_detail = [&](const SpotClient::ImageFrame* left_frame,
                                 const SpotClient::ImageFrame* right_frame,
                                 const std::string& extra_detail) {
    std::vector<std::string> parts;
    if (left_frame) {
      parts.push_back("left=" + image_status_to_string(left_frame->status));
    } else {
      parts.push_back("left=missing");
    }
    if (right_frame) {
      parts.push_back("right=" + image_status_to_string(right_frame->status));
    } else {
      parts.push_back("right=missing");
    }
    if (!extra_detail.empty()) parts.push_back(extra_detail);
    std::ostringstream oss;
    for (size_t i = 0; i < parts.size(); ++i) {
      if (i > 0) oss << " | ";
      oss << parts[i];
    }
    return oss.str();
  };

  std::thread sender([this, &cached_image, &stream, output_period_ms]() {
    publish_cached_image_loop(&agent_, &running_, &cached_image, stream,
                              output_period_ms, 150);
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

    std::vector<SpotClient::ImageFrame> frames;
    if (spot_.GetImageFrames(sources, &frames)) {
      const SpotClient::ImageFrame* left_frame = find_frame(frames, cfg_.front_left_camera_source);
      const SpotClient::ImageFrame* right_frame = find_frame(frames, cfg_.front_right_camera_source);
      const bool can_build_artifacts =
          left_frame && right_frame &&
          frame_has_stitch_calibration(*left_frame) &&
          frame_has_stitch_calibration(*right_frame);

      cv::Mat left_image;
      cv::Mat right_image;
      const bool have_left_image =
          left_frame &&
          left_frame->status == static_cast<int>(::bosdyn::api::ImageResponse::STATUS_OK) &&
          decode_jpeg_mat(left_frame->encoded_image, &left_image);
      const bool have_right_image =
          right_frame &&
          right_frame->status == static_cast<int>(::bosdyn::api::ImageResponse::STATUS_OK) &&
          decode_jpeg_mat(right_frame->encoded_image, &right_image);
      const std::string calibration_signature =
          can_build_artifacts
              ? build_front_stitch_calibration_signature(*left_frame, *right_frame)
              : std::string();

      if (manual_roll_override && can_build_artifacts) {
        selected_roll_quarter_turns = configured_roll_quarter_turns;
        if (!have_artifacts || selected_calibration_signature != calibration_signature) {
          FrontStitchArtifacts new_artifacts;
          std::string build_error;
          if (build_front_stitch_artifacts(*left_frame, *right_frame,
                                           &new_artifacts, &build_error)) {
            artifacts = std::move(new_artifacts);
            have_artifacts = true;
            selected_calibration_signature = calibration_signature;
            debug_capture_attempted = false;
          } else {
            have_artifacts = false;
            maybe_log_error("[front-camera] failed building stitch maps: " + build_error);
          }
        }
      } else if (!manual_roll_override &&
                 can_build_artifacts &&
                 have_left_image &&
                 have_right_image &&
                 (!have_artifacts || selected_calibration_signature != calibration_signature)) {
        const FrontRollAutoSelection selection =
            auto_select_front_roll(*left_frame, *right_frame, left_image, right_image);
        if (selection.valid) {
          const bool roll_changed =
              !have_artifacts || selected_roll_quarter_turns != selection.roll_quarter_turns;
          artifacts = selection.artifacts;
          have_artifacts = true;
          selected_roll_quarter_turns = selection.roll_quarter_turns;
          selected_calibration_signature = calibration_signature;
          debug_capture_attempted = false;
          std::cerr << "[front-camera] auto-selected stitch roll: label="
                    << front_roll_label(selected_roll_quarter_turns)
                    << " degrees=" << (selected_roll_quarter_turns * 90)
                    << " score=" << selection.score
                    << " scores={" << describe_front_roll_scores(selection) << "}"
                    << (roll_changed ? "" : " (unchanged)")
                    << std::endl;
        } else {
          have_artifacts = false;
          maybe_log_error("[front-camera] failed auto-selecting stitch roll");
        }
      }

      bool updated_frame = false;
      if (have_artifacts && (have_left_image || have_right_image)) {
        if (!debug_capture_attempted && have_left_image && have_right_image &&
            left_frame && right_frame) {
          debug_capture_attempted = true;
          std::string debug_dir;
          if (write_front_stitch_debug_bundle(*left_frame, *right_frame, left_image, right_image,
                                              selected_roll_quarter_turns, &debug_dir)) {
            std::cerr << "[front-camera] wrote stitch debug bundle: " << debug_dir << std::endl;
          } else {
            maybe_log_error("[front-camera] failed writing stitch debug bundle");
          }
        }
        std::string output_jpg;
        if (render_front_stitched_jpeg(artifacts,
                                       have_left_image ? &left_image : nullptr,
                                       have_right_image ? &right_image : nullptr,
                                       selected_roll_quarter_turns,
                                       std::string(),
                                       std::string(),
                                       &output_jpg)) {
          auto frame = std::make_shared<std::string>();
          frame->swap(output_jpg);
          update_cached_image(&cached_image, std::move(frame));
          updated_frame = true;
          have_rendered_frame = true;
          last_good_frame_ms = now_ms();
          failure_backoff_ms = poll_period_ms;
        } else {
          maybe_log_error("[front-camera] failed rendering stitched image");
        }
      }

      if (!updated_frame) {
        const long long now = now_ms();
        if (!have_rendered_frame || (now - last_good_frame_ms) >= 2000) {
          std::string output_jpg;
          const std::string detail = build_status_detail(
              left_frame, right_frame,
              have_artifacts ? std::string() : std::string("waiting for stitch calibration"));
          if (build_front_camera_status_frame("Front stitch unavailable", detail, &output_jpg)) {
            auto frame = std::make_shared<std::string>();
            frame->swap(output_jpg);
            update_cached_image(&cached_image, std::move(frame));
            updated_frame = true;
          }
        }
        if (!updated_frame) {
          maybe_log_error("[front-camera] no usable stitched front frame in current batch");
        }
        failure_backoff_ms =
            std::min(kMaxImageFetchBackoffMs, std::max(poll_period_ms, failure_backoff_ms * 2));
      }
    } else {
      maybe_log_error("[front-camera] failed fetching front image batch: " + spot_.LastError());
      const long long now = now_ms();
      if (!have_rendered_frame || (now - last_good_frame_ms) >= 2000) {
        std::string output_jpg;
        if (build_front_camera_status_frame(
                "Front stitch unavailable",
                "Spot image service batch request failed. Waiting for a new front camera batch.",
                &output_jpg)) {
          auto frame = std::make_shared<std::string>();
          frame->swap(output_jpg);
          update_cached_image(&cached_image, std::move(frame));
        }
      }
      failure_backoff_ms =
          std::min(kMaxImageFetchBackoffMs, std::max(poll_period_ms, failure_backoff_ms * 2));
    }

    const int step_ms = 25;
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
  CachedImageState cached_image;

  auto build_frame = [](const SpotClient::LocalizationMapSnapshot* snapshot,
                        const std::string& status_title,
                        const std::string& status_detail,
                        std::string* out_jpg) -> bool {
    if (!out_jpg) return false;
    const auto& palette = graphnav_image_palette();

    const bool has_live_map = snapshot && snapshot->localized && snapshot->has_seed_tform_body &&
                              snapshot->has_map && snapshot->map.width > 0 &&
                              snapshot->map.height > 0 && snapshot->map.resolution_m > 0.0;

    if (!has_live_map) {
      return build_graphnav_status_frame("Waiting for live localization",
                                         "Waiting for the first GraphNav occupancy patch.",
                                         status_title, status_detail, out_jpg);
    }

    const int grid_width = snapshot->map.width;
    const int grid_height = snapshot->map.height;
    cv::Mat canvas(kLocalizationImageHeight, kLocalizationImageWidth, CV_8UC3, palette.map_unknown);
    cv::Mat grid_image;
    cv::Mat occupied_mask;
    if (!rasterize_occupancy_grid(snapshot->map, palette, &grid_image, &occupied_mask)) {
      return build_graphnav_status_frame("Waiting for live localization",
                                         "Waiting for the first GraphNav occupancy patch.",
                                         "Live terrain map unavailable",
                                         "GraphNav returned an invalid occupancy grid payload.",
                                         out_jpg);
    }

    const SpotClient::Pose3D grid_tform_body =
        compose_pose(inverse_pose(snapshot->map.seed_tform_grid), snapshot->seed_tform_body);
    const double yaw_rad = quaternion_yaw_rad(pose_quaternion(grid_tform_body));
    const double body_grid_x = grid_tform_body.x / snapshot->map.resolution_m;
    const double body_grid_y_img =
        static_cast<double>(grid_height) - (grid_tform_body.y / snapshot->map.resolution_m);

    const double output_aspect =
        static_cast<double>(kLocalizationImageWidth) / static_cast<double>(kLocalizationImageHeight);
    double crop_width = static_cast<double>(grid_width);
    double crop_height = static_cast<double>(grid_height);
    if ((crop_width / crop_height) > output_aspect) {
      crop_width = crop_height * output_aspect;
    } else {
      crop_height = crop_width / output_aspect;
    }
    crop_width = std::max(1.0, std::min(crop_width, static_cast<double>(grid_width)));
    crop_height = std::max(1.0, std::min(crop_height, static_cast<double>(grid_height)));

    double crop_x = body_grid_x - (crop_width / 2.0);
    double crop_y = body_grid_y_img - (crop_height / 2.0);
    crop_x = std::max(0.0, std::min(crop_x, static_cast<double>(grid_width) - crop_width));
    crop_y = std::max(0.0, std::min(crop_y, static_cast<double>(grid_height) - crop_height));

    const int crop_x0 = std::max(0, std::min(grid_width - 1, static_cast<int>(std::floor(crop_x))));
    const int crop_y0 = std::max(0, std::min(grid_height - 1, static_cast<int>(std::floor(crop_y))));
    const int crop_x1 = std::max(crop_x0 + 1,
                                 std::min(grid_width, static_cast<int>(std::ceil(crop_x + crop_width))));
    const int crop_y1 = std::max(crop_y0 + 1,
                                 std::min(grid_height, static_cast<int>(std::ceil(crop_y + crop_height))));
    const cv::Rect crop_rect(crop_x0, crop_y0, crop_x1 - crop_x0, crop_y1 - crop_y0);

    const double x_scale = static_cast<double>(canvas.cols) / std::max(1, crop_rect.width);
    const double y_scale = static_cast<double>(canvas.rows) / std::max(1, crop_rect.height);

    cv::Mat cropped_grid = grid_image(crop_rect);
    cv::Mat scaled_grid;
    cv::resize(cropped_grid, scaled_grid, canvas.size(), 0.0, 0.0, cv::INTER_NEAREST);
    if (std::min(x_scale, y_scale) >= 3.0) {
      const cv::Scalar grid_line(74, 80, 88);
      const int start_grid_x = std::max(0, static_cast<int>(std::floor(crop_x / 10.0)) * 10);
      const int end_grid_x =
          std::min(grid_width, static_cast<int>(std::ceil((crop_x + crop_width) / 10.0)) * 10);
      for (int x = start_grid_x; x <= end_grid_x; x += 10) {
        const int line_x = std::min(canvas.cols - 1, std::max(0, cvRound((x - crop_x) * x_scale)));
        cv::line(scaled_grid, cv::Point(line_x, 0), cv::Point(line_x, canvas.rows - 1), grid_line,
                 1, cv::LINE_8);
      }
      const int start_grid_y = std::max(0, static_cast<int>(std::floor(crop_y / 10.0)) * 10);
      const int end_grid_y =
          std::min(grid_height, static_cast<int>(std::ceil((crop_y + crop_height) / 10.0)) * 10);
      for (int y = start_grid_y; y <= end_grid_y; y += 10) {
        const int line_y = std::min(canvas.rows - 1, std::max(0, cvRound((y - crop_y) * y_scale)));
        cv::line(scaled_grid, cv::Point(0, line_y), cv::Point(canvas.cols - 1, line_y), grid_line,
                 1, cv::LINE_8);
      }
    }

    scaled_grid.copyTo(canvas);
    for (int x = 0; x <= canvas.cols; x += 96) {
      cv::line(canvas, cv::Point(x, 0), cv::Point(x, canvas.rows - 1), palette.grid_major, 1,
               cv::LINE_AA);
    }
    for (int y = 0; y <= canvas.rows; y += 96) {
      cv::line(canvas, cv::Point(0, y), cv::Point(canvas.cols - 1, y), palette.grid_major, 1,
               cv::LINE_AA);
    }

    draw_occupied_contours(&canvas, occupied_mask(crop_rect),
                           cv::Rect(0, 0, canvas.cols, canvas.rows),
                           palette.map_occupied_edge);

    auto local_to_pixel = [&](double local_x_m, double local_y_m) {
      const double pixel_x = ((local_x_m / snapshot->map.resolution_m) - crop_x) * x_scale;
      const double pixel_y =
          ((static_cast<double>(grid_height) - (local_y_m / snapshot->map.resolution_m)) - crop_y) *
          y_scale;
      return cv::Point(cvRound(pixel_x), cvRound(pixel_y));
    };

    const double body_x = grid_tform_body.x;
    const double body_y = grid_tform_body.y;
    draw_robot_pose_overlay(&canvas, local_to_pixel, body_x, body_y, yaw_rad,
                            std::max(x_scale, y_scale) * 4.0, 0.18, palette.robot_fill,
                            palette.robot_outline);

    if (snapshot->has_waypoint_tform_body) {
      const SpotClient::Pose3D seed_tform_waypoint =
          compose_pose(snapshot->seed_tform_body, inverse_pose(snapshot->waypoint_tform_body));
      const SpotClient::Pose3D grid_tform_waypoint =
          compose_pose(inverse_pose(snapshot->map.seed_tform_grid), seed_tform_waypoint);
      const cv::Point waypoint_pt = local_to_pixel(grid_tform_waypoint.x, grid_tform_waypoint.y);
      cv::Mat waypoint_overlay = canvas.clone();
      cv::circle(waypoint_overlay, waypoint_pt,
                 std::max(10, cvRound(std::max(x_scale, y_scale) * 3.0)), palette.waypoint_fill,
                 cv::FILLED, cv::LINE_AA);
      cv::addWeighted(waypoint_overlay, 0.12, canvas, 0.88, 0.0, canvas);
      cv::circle(canvas, waypoint_pt, std::max(6, cvRound(std::max(x_scale, y_scale) * 2.0)),
                 cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
      cv::circle(canvas, waypoint_pt, 3, palette.waypoint_fill, cv::FILLED, cv::LINE_AA);
      cv::line(canvas, cv::Point(waypoint_pt.x - 10, waypoint_pt.y),
               cv::Point(waypoint_pt.x + 10, waypoint_pt.y), palette.waypoint_fill, 1,
               cv::LINE_AA);
      cv::line(canvas, cv::Point(waypoint_pt.x, waypoint_pt.y - 10),
               cv::Point(waypoint_pt.x, waypoint_pt.y + 10), palette.waypoint_fill, 1,
               cv::LINE_AA);
    }

    const double span_x_m = snapshot->map.width * snapshot->map.resolution_m;
    double scale_bar_m = 1.0;
    if (span_x_m >= 8.0) scale_bar_m = 2.0;
    if (span_x_m >= 16.0) scale_bar_m = 5.0;
    const int scale_bar_px =
        std::max(1, cvRound((scale_bar_m / snapshot->map.resolution_m) * x_scale));
    draw_scale_bar(&canvas, cv::Point(16, std::max(24, canvas.rows - 20)), scale_bar_px,
                   scale_bar_m, palette.text_primary, palette.robot_outline, true);

    return encode_jpeg(canvas, out_jpg);
  };

  {
    auto initial = std::make_shared<std::string>();
    build_frame(nullptr, "Waiting for live localization",
                "Waiting for the first GraphNav occupancy patch.", initial.get());
    update_cached_image(&cached_image, std::move(initial));
  }

  std::thread sender([this, &cached_image, &stream, output_period_ms]() {
    publish_cached_image_loop(&agent_, &running_, &cached_image, stream, output_period_ms, 250);
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
      auto rendered = std::make_shared<std::string>();
      if (build_frame(use_snapshot ? &snapshot : nullptr, status_title, status_detail,
                      rendered.get())) {
        update_cached_image(&cached_image, std::move(rendered));
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

void Adapter::GraphNavMapImageLoop(const std::string& stream, int fps, int poll_hz) {
  const int output_period_ms = std::max(1, 1000 / std::max(1, fps));
  const int poll_period_ms = std::max(200, 1000 / std::max(1, poll_hz));
  const bool publish_image = !stream.empty();
  const bool publish_metadata = !cfg_.graphnav_map_image_metadata_stream_name.empty();
  CachedImageState cached_image;

  auto build_frame = [](const std::shared_ptr<GraphNavMapArtifacts>& artifacts,
                        const SpotClient::LocalizationSnapshot* localization,
                        const std::unordered_set<std::string>& dock_waypoints,
                        const std::string& status_title,
                        const std::string& status_detail,
                        GraphNavMapImageLayout* out_layout,
                        std::string* out_jpg) -> bool {
    if (!out_jpg) return false;
    const auto& palette = graphnav_image_palette();

    const bool has_map = artifacts && artifacts->snapshot.has_map &&
                         artifacts->snapshot.map.width > 0 &&
                         artifacts->snapshot.map.height > 0 &&
                         artifacts->snapshot.map.resolution_m > 0.0;

    if (!has_map) {
      return build_graphnav_status_frame("Waiting for saved GraphNav map",
                                         "Waiting for a stitched GraphNav map to be available.",
                                         status_title, status_detail, out_jpg);
    }

    cv::Mat canvas(kLocalizationImageHeight, kLocalizationImageWidth, CV_8UC3, palette.map_unknown);
    const auto& map = artifacts->snapshot.map;
    const GraphNavMapGeometry geometry = graphnav_map_geometry_from_snapshot(map);
    GraphNavMapImageLayout layout;
    if (!BuildGraphNavMapImageLayout(geometry, kLocalizationImageWidth, kLocalizationImageHeight,
                                     12, 12, 12, &layout)) {
      return build_graphnav_status_frame("Waiting for saved GraphNav map",
                                         "Waiting for a stitched GraphNav map to be available.",
                                         "Saved GraphNav map invalid",
                                         "The stitched map payload is missing render geometry.",
                                         out_jpg);
    }
    cv::Mat grid_image;
    cv::Mat occupied_mask;
    if (!rasterize_occupancy_grid(map, palette, &grid_image, &occupied_mask)) {
      return build_graphnav_status_frame("Waiting for saved GraphNav map",
                                         "Waiting for a stitched GraphNav map to be available.",
                                         "Saved GraphNav map invalid",
                                         "The stitched map payload is missing a usable occupancy grid.",
                                         out_jpg);
    }

    const cv::Rect draw_rect(layout.draw_x, layout.draw_y, layout.draw_width, layout.draw_height);

    cv::Mat scaled_grid;
    cv::resize(grid_image, scaled_grid, draw_rect.size(), 0.0, 0.0, cv::INTER_NEAREST);
    scaled_grid.copyTo(canvas(draw_rect));

    draw_occupied_contours(&canvas, occupied_mask, draw_rect, palette.map_occupied_edge);

    auto local_to_pixel = [&](double local_x_m, double local_y_m) {
      double pixel_x = 0.0;
      double pixel_y = 0.0;
      (void)GraphNavMapImageLocalToPixel(layout, local_x_m, local_y_m, &pixel_x, &pixel_y);
      return cv::Point(cvRound(pixel_x), cvRound(pixel_y));
    };

    std::unordered_map<std::string, SpotClient::Pose3D> local_waypoints;
    local_waypoints.reserve(artifacts->snapshot.waypoints.size());
    const SpotClient::Pose3D map_tform_seed = inverse_pose(map.seed_tform_grid);
    for (const auto& waypoint : artifacts->snapshot.waypoints) {
      local_waypoints[waypoint.id] =
          compose_pose(map_tform_seed, waypoint.seed_tform_waypoint);
    }

    for (const auto& edge : artifacts->snapshot.edges) {
      const auto from_it = local_waypoints.find(edge.from_waypoint_id);
      const auto to_it = local_waypoints.find(edge.to_waypoint_id);
      if (from_it == local_waypoints.end() || to_it == local_waypoints.end()) continue;
      cv::line(canvas, local_to_pixel(from_it->second.x, from_it->second.y),
               local_to_pixel(to_it->second.x, to_it->second.y), palette.edge_line, 1,
               cv::LINE_AA);
    }

    for (const auto& waypoint : artifacts->snapshot.waypoints) {
      const auto pose_it = local_waypoints.find(waypoint.id);
      if (pose_it == local_waypoints.end()) continue;
      const bool is_dock = dock_waypoints.find(waypoint.id) != dock_waypoints.end();
      const bool is_current =
          localization && !localization->waypoint_id.empty() &&
          localization->waypoint_id == waypoint.id;
      const cv::Point point = local_to_pixel(pose_it->second.x, pose_it->second.y);
      const cv::Scalar fill_color = is_dock ? palette.waypoint_dock : palette.waypoint_fill;
      const int radius = is_current ? 7 : 5;
      cv::circle(canvas, point, radius + 2, palette.robot_outline, cv::FILLED, cv::LINE_AA);
      cv::circle(canvas, point, radius, fill_color, cv::FILLED, cv::LINE_AA);
      cv::circle(canvas, point, radius, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    if (localization && localization->has_seed_tform_body) {
      const SpotClient::Pose3D seed_tform_body = pose3d_from_localization_snapshot(*localization);
      const SpotClient::Pose3D map_tform_body = compose_pose(map_tform_seed, seed_tform_body);
      const double yaw_rad = quaternion_yaw_rad(pose_quaternion(map_tform_body));
      draw_robot_pose_overlay(&canvas, local_to_pixel, map_tform_body.x, map_tform_body.y,
                              yaw_rad, layout.scale * 3.5, 0.14, palette.robot_fill,
                              palette.robot_outline);
    }

    const double span_x_m = map.width * map.resolution_m;
    double scale_bar_m = 1.0;
    if (span_x_m >= 12.0) scale_bar_m = 2.0;
    if (span_x_m >= 24.0) scale_bar_m = 5.0;
    if (span_x_m >= 60.0) scale_bar_m = 10.0;
    const int scale_bar_px =
        std::max(1, cvRound((scale_bar_m / map.resolution_m) * layout.scale));
    draw_scale_bar(&canvas, cv::Point(draw_rect.x + 18, draw_rect.y + draw_rect.height - 18),
                   scale_bar_px, scale_bar_m, palette.text_primary, palette.robot_outline, false);

    if (out_layout) *out_layout = layout;
    return encode_jpeg(canvas, out_jpg);
  };

  if (publish_image) {
    auto initial = std::make_shared<std::string>();
    build_frame(nullptr, nullptr, {}, "Waiting for saved GraphNav map",
                "Waiting for a stitched GraphNav map to be available.",
                nullptr, initial.get());
    update_cached_image(&cached_image, std::move(initial));
  }

  std::thread sender;
  if (publish_image) {
    sender = std::thread([this, &cached_image, &stream, output_period_ms]() {
      publish_cached_image_loop(&agent_, &running_, &cached_image, stream, output_period_ms, 250);
    });
  }

  std::string last_metadata_payload;
  long long last_metadata_pub_ms = 0;

  while (running_) {
    std::shared_ptr<GraphNavMapArtifacts> artifacts;
    std::unordered_set<std::string> dock_waypoints;
    {
      std::lock_guard<std::mutex> lk(map_mu_);
      artifacts = graphnav_map_artifacts_;
      if (artifacts) {
        const auto dock_it = dock_waypoint_by_map_.find(artifacts->map_id);
        if (dock_it != dock_waypoint_by_map_.end() && !dock_it->second.empty()) {
          dock_waypoints.insert(dock_it->second);
        }
      }
    }

    SpotClient::LocalizationSnapshot localization;
    SpotClient::LocalizationSnapshot* localization_ptr = nullptr;
    std::string status_title = "GraphNav global map";
    std::string status_detail;

    if (!artifacts || !artifacts->snapshot.has_map) {
      status_title = "Waiting for saved GraphNav map";
      status_detail = "Waiting for a stitched GraphNav map to be available.";
    } else if (!SpotConnected()) {
      status_title = "Saved GraphNav map";
      status_detail = "Robot disconnected. Showing the stitched map without a live pose.";
    } else if (spot_.GetLocalizationSnapshot(&localization) && localization.has_seed_tform_body) {
      localization_ptr = &localization;
      status_detail = "map=" + artifacts->map_id + " live pose available";
    } else {
      status_title = "Saved GraphNav map";
      status_detail = "Stitched map loaded, but live GraphNav pose is unavailable.";
    }

    GraphNavMapImageLayout render_layout;
    GraphNavMapImageLayout* render_layout_ptr = nullptr;
    if (publish_image) {
      auto rendered = std::make_shared<std::string>();
      if (build_frame(artifacts, localization_ptr, dock_waypoints,
                      status_title, status_detail, &render_layout, rendered.get())) {
        update_cached_image(&cached_image, std::move(rendered));
        if (artifacts && artifacts->snapshot.has_map) render_layout_ptr = &render_layout;
      }
    } else if (artifacts && artifacts->snapshot.has_map) {
      const GraphNavMapGeometry geometry =
          graphnav_map_geometry_from_snapshot(artifacts->snapshot.map);
      if (BuildGraphNavMapImageLayout(geometry, kLocalizationImageWidth, kLocalizationImageHeight,
                                      12, 12, 12, &render_layout)) {
        render_layout_ptr = &render_layout;
      }
    }

    if (publish_metadata) {
      const long long now = now_ms();
      const SpotClient::OccupancyGridMapSnapshot* map_snapshot =
          (artifacts && artifacts->snapshot.has_map) ? &artifacts->snapshot.map : nullptr;
      const std::string payload = build_graphnav_map_image_metadata_json(
          stream, cfg_.graphnav_map_image_metadata_stream_name,
          artifacts ? artifacts->map_id : std::string(),
          artifacts ? artifacts->map_uuid : std::string(),
          map_snapshot, localization_ptr, render_layout_ptr, status_title, status_detail);
      const bool changed = (payload != last_metadata_payload);
      if (changed || (now - last_metadata_pub_ms) >= 5000) {
        QueueStatusText(cfg_.graphnav_map_image_metadata_stream_name, payload);
        last_metadata_payload = payload;
        last_metadata_pub_ms = now;
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
  const int graphnav_global_localization_period_ms =
      std::max(250, 1000 / std::max(1, cfg_.graphnav_global_localization_hz));
  long long last_retain_ms = 0;
  long long last_can_dock_pub_ms = 0;
  long long last_status_pub_ms = 0;
  long long last_mode_pub_ms = 0;
  long long last_robot_diag_pub_ms = 0;
  long long last_map_progress_pub_ms = 0;
  long long last_localization_pub_ms = 0;
  long long last_localization_viz_pub_ms = 0;
  long long last_graphnav_global_localization_pub_ms = 0;
  while (running_) {
    const long long now = now_ms();
    MaybeRestoreActiveMap(now);
    PollGraphNavNavigation(now);

    if (lease_owned_ && SpotConnected() &&
        (now - last_retain_ms) >= std::max(100, 1000 / std::max(1, cfg_.lease_retain_hz))) {
      spot_.RetainLease();
      last_retain_ms = now;
    }

    if ((!cfg_.can_dock_stream.empty() || IsStreamEnabled(kDockingStateStream)) &&
        (now - last_can_dock_pub_ms) >= periodic_publish_ms) {
      int dock_id = resolved_dock_id_.load();
      if (dock_id <= 0) dock_id = ResolveDockId();
      bool can_dock = false;
      std::string docking_state = "unknown";
      if (SpotConnected() && dock_id > 0) {
        if (!cfg_.can_dock_stream.empty()) {
          bool ok = spot_.CanDock(dock_id, &can_dock);
          if (!ok) can_dock = false;
        }
      }
      if (SpotConnected()) {
        int dock_status = 0;
        if (spot_.GetDockingStatus(&dock_status)) {
          docking_state = docking_state_name(dock_status);
        }
      }
      can_dock_ = can_dock;
      if (!cfg_.can_dock_stream.empty()) {
        QueueStatusBitset(cfg_.can_dock_stream, {{"Can dock", can_dock_.load()}});
      }
      if (IsStreamEnabled(kDockingStateStream)) {
        QueueStatusText(kDockingStateStream, docking_state);
      }
      last_can_dock_pub_ms = now;
    }

    if ((now - last_status_pub_ms) >= periodic_publish_ms) {
      if (IsStreamEnabled(kStatusStream)) {
        QueueStatusBitset(kStatusStream, {
          {"Has lease", lease_owned_.load()},
          {"Robot available", SpotConnected()},
          {"Robot degraded", spot_degraded_non_estop_.load()},
          {"Teleop running", running_.load()},
          {"Teleop active", TeleopSessionActive()},
          {"Docking", docking_in_progress_.load()},
        });
      }
      if (IsStreamEnabled(kConnectionStateStream)) {
        QueueStatusText(kConnectionStateStream, connection_state_name(spot_connection_state_.load()));
      }
      if (IsStreamEnabled(kCommandedMotionModeStream)) {
        QueueStatusText(kCommandedMotionModeStream,
                        motion_mode_state_name(desired_motion_mode_.load()));
      }
      if (IsStreamEnabled(kConnectionStream)) {
        QueueStatusText(kConnectionStream, BuildSpotConnectionJson());
      }
      last_status_pub_ms = now;
    }

    if (!cfg_.stateful_mode_stream.empty() &&
        (now - last_mode_pub_ms) >= periodic_publish_ms) {
      QueueStatusBitset(cfg_.stateful_mode_stream, {
        {"Walk", desired_motion_mode_.load() == kMotionModeWalk},
        {"Stairs", desired_motion_mode_.load() == kMotionModeStairs},
        {"Crawl", desired_motion_mode_.load() == kMotionModeCrawl},
      });
      last_mode_pub_ms = now;
    }

    const bool need_robot_diag = AnyStreamEnabled({
        kBehaviorStateStream,
        kMotorPowerStateStream,
        kRobotStatePowerStream,
        kRobotStateBatteryStream,
        kRobotStateBodyPitchStream,
        kShorePowerStateStream,
        kFaultsSystemStream,
        kFaultsBehaviorStream,
        kFaultsServiceStream,
        kFaultEventsStream,
    });
    if (need_robot_diag && (now - last_robot_diag_pub_ms) >= periodic_publish_ms) {
      if (SpotConnected()) {
        SpotClient::RobotStateSnapshot snap;
        if (spot_.GetRobotStateSnapshot(&snap)) {
          ApplySoftRecoveryForRobotState(snap);
          if (IsStreamEnabled(kBehaviorStateStream)) {
            QueueStatusText(kBehaviorStateStream, snap.behavior_state);
          }
          if (IsStreamEnabled(kMotorPowerStateStream)) {
            QueueStatusText(kMotorPowerStateStream, snap.motor_power_state);
          }
          if (IsStreamEnabled(kRobotStatePowerStream)) {
            QueueStatusText(kRobotStatePowerStream, robot_state_to_json(snap));
          }
          if (snap.has_battery_pct && IsStreamEnabled(kRobotStateBatteryStream)) {
            QueueStatusNumeric(kRobotStateBatteryStream, snap.battery_pct);
          }
          if (snap.has_body_pitch_rad && IsStreamEnabled(kRobotStateBodyPitchStream)) {
            QueueStatusNumeric(kRobotStateBodyPitchStream, snap.body_pitch_rad);
          }
          if (IsStreamEnabled(kShorePowerStateStream)) {
            QueueStatusText(kShorePowerStateStream, snap.shore_power_state);
          }
          if (IsStreamEnabled(kFaultsSystemStream)) {
            QueueStatusText(kFaultsSystemStream, fault_list_to_json(snap.system_faults));
          }
          if (IsStreamEnabled(kFaultsBehaviorStream)) {
            QueueStatusText(kFaultsBehaviorStream, fault_list_to_json(snap.behavior_faults));
          }
          if (IsStreamEnabled(kFaultsServiceStream)) {
            QueueStatusText(kFaultsServiceStream, fault_list_to_json(snap.service_faults));
          }
          if (IsStreamEnabled(kFaultEventsStream)) PublishFaultEvents(snap);
        } else {
          std::cerr << "[state] failed to fetch robot state: " << spot_.LastError() << std::endl;
        }
      } else {
        if (IsStreamEnabled(kBehaviorStateStream)) {
          QueueStatusText(kBehaviorStateStream, "unknown");
        }
        if (IsStreamEnabled(kMotorPowerStateStream)) {
          QueueStatusText(kMotorPowerStateStream, "unknown");
        }
        if (IsStreamEnabled(kShorePowerStateStream)) {
          QueueStatusText(kShorePowerStateStream, "unknown");
        }
      }
      last_robot_diag_pub_ms = now;
    }

    const bool need_map_progress = AnyStreamEnabled({
        kMapProgressStream,
        kMapProgressWaypointsStream,
        kMapProgressPathLengthStream,
        kMapProgressFiducialsStream,
    });
    if (need_map_progress &&
        (now - last_map_progress_pub_ms) >= periodic_publish_ms && SpotConnected()) {
      SpotClient::MappingStatus map_status;
      if (spot_.GetMappingStatus(&map_status)) {
        map_recording_active_ = map_status.is_recording;
        QueueStatusText(kMapProgressStream, mapping_status_to_json(map_status));
        QueueStatusNumeric(kMapProgressWaypointsStream,
                           static_cast<double>(map_status.waypoint_count));
        QueueStatusNumeric(kMapProgressPathLengthStream, map_status.total_path_length_m);
        QueueStatusNumeric(kMapProgressFiducialsStream,
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

    if (IsStreamEnabled(kLocalizationStatusStream) &&
        (now - last_localization_pub_ms) >= periodic_publish_ms) {
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
      QueueStatusText(kLocalizationStatusStream, oss.str());
      last_localization_pub_ms = now;
    }

    if (IsStreamEnabled(kGraphNavLocalizationVizStream) &&
        (now - last_localization_viz_pub_ms) >= periodic_publish_ms && SpotConnected()) {
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

    if ((now - last_graphnav_global_localization_pub_ms) >= graphnav_global_localization_period_ms &&
        SpotConnected()) {
      std::shared_ptr<GraphNavMapArtifacts> artifacts;
      {
        std::lock_guard<std::mutex> lk(map_mu_);
        artifacts = graphnav_map_artifacts_;
      }
      if (artifacts && artifacts->snapshot.has_map &&
          !cfg_.graphnav_global_localization_stream.empty()) {
        SpotClient::LocalizationSnapshot localization;
        if (spot_.GetLocalizationSnapshot(&localization) && localization.has_seed_tform_body) {
          (void)agent_.PostLocalization(
              cfg_.graphnav_global_localization_stream,
              build_formant_localization(pose3d_from_localization_snapshot(localization),
                                         artifacts->snapshot.map,
                                         artifacts->map_uuid));
        }
      }
      last_graphnav_global_localization_pub_ms = now;
    }

    PollCurrentWaypointAtStatus(now);
    PublishGraphNavNavState(now);

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
  const bool has_arm = spot_.HasArm();
  if (spot_.ArmPresenceKnown()) {
    EmitLog(has_arm ? "[spot] manipulator detected"
                    : "[spot] no manipulator detected; arm actions disabled");
  } else {
    EmitLog("[spot] manipulator presence unavailable; arm actions disabled until detected");
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
  {
    std::lock_guard<std::mutex> lk(nav_state_mu_);
    ClearGraphNavNavTargetLocked();
    last_nav_feedback_signature_.clear();
    last_nav_status_ = 0;
    last_nav_remaining_route_m_ = 0.0;
    last_nav_progress_change_ms_ = 0;
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
  if (!IsStreamEnabled(kFaultEventsStream)) return;
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
  QueueStatusText(kFaultEventsStream, payload.str());
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
  (void)RefreshGraphNavMapArtifacts(restore_map_id, map_data);
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

        if (IsStreamEnabled(kNavFeedbackStream)) {
          QueueStatusText(kNavFeedbackStream, nav_feedback_to_json(command_id, fb));
        }

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
  };

  graphnav_navigation_active_ = false;
  using Resp = ::bosdyn::api::graph_nav::NavigationFeedbackResponse;
  if (status == Resp::STATUS_REACHED_GOAL) {
    EmitLog("[graphnav] navigation reached goal");
    {
      std::lock_guard<std::mutex> lk(nav_state_mu_);
      last_nav_status_ = status;
      last_nav_remaining_route_m_ = fb.remaining_route_length;
    }
    last_graph_nav_command_id_ = 0;
    nav_auto_recovered_ = false;
    clear_nav_state();
    return;
  }

  if (status == Resp::STATUS_ROBOT_IMPAIRED) {
    const bool already_recovered = nav_auto_recovered_.exchange(true);
    std::string waypoint_name;
    {
      std::lock_guard<std::mutex> lk(nav_state_mu_);
      waypoint_name = nav_target_waypoint_name_;
      last_nav_status_ = status;
      last_nav_remaining_route_m_ = fb.remaining_route_length;
    }
    if (!already_recovered) {
      EmitLog("[graphnav] navigation impaired, auto-recovering and retrying");
      if (!spot_.RecoverSelfRight()) {
        EmitLog(std::string("[graphnav] auto-recover failed: ") + spot_.LastError());
        last_graph_nav_command_id_ = 0;
        clear_nav_state();
        return;
      }
      (void)spot_.Stand();

      uint32_t retry_command_id = 0;
      if (RetryActiveGraphNavCommandWithRecovery("graphnav.navigation.retry",
                                                 &retry_command_id)) {
        last_graph_nav_command_id_ = retry_command_id;
        graphnav_navigation_active_ = true;
        EmitLog(std::string("[graphnav] retrying navigation after recover name=") + waypoint_name);
        return;
      }
      EmitLog(std::string("[graphnav] navigation retry after recover failed: ") + spot_.LastError());
    }
    last_graph_nav_command_id_ = 0;
    clear_nav_state();
    return;
  }

  {
    std::lock_guard<std::mutex> lk(nav_state_mu_);
    last_nav_status_ = status;
    last_nav_remaining_route_m_ = fb.remaining_route_length;
  }
  EmitLog(std::string("[graphnav] navigation ended status=") + std::to_string(status) +
          " (" + nav_status_name(status) + ")");
  last_graph_nav_command_id_ = 0;
  nav_auto_recovered_ = false;
  clear_nav_state();
}

void Adapter::QueueStatusText(const std::string& stream, const std::string& value) {
  if (stream.empty() || !IsStreamEnabled(stream)) return;
  std::lock_guard<std::mutex> lk(status_queue_mu_);
  auto& dp = status_queue_[stream];
  dp.kind = PendingStatusDatapoint::Kind::kText;
  dp.text_value = value;
  dp.pending = true;
}

void Adapter::QueueStatusNumeric(const std::string& stream, double value) {
  if (stream.empty() || !IsStreamEnabled(stream)) return;
  std::lock_guard<std::mutex> lk(status_queue_mu_);
  auto& dp = status_queue_[stream];
  dp.kind = PendingStatusDatapoint::Kind::kNumeric;
  dp.numeric_value = value;
  dp.pending = true;
}

void Adapter::QueueStatusBitset(const std::string& stream,
                                const std::vector<std::pair<std::string, bool>>& bits) {
  if (stream.empty() || !IsStreamEnabled(stream)) return;
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
    FormantAgentClient::PostResult result;
    if (o.kind == PendingStatusDatapoint::Kind::kText) {
      result = agent_.PostText(o.stream, o.text_value);
    } else if (o.kind == PendingStatusDatapoint::Kind::kNumeric) {
      result = agent_.PostNumeric(o.stream, o.numeric_value);
    } else {
      result = agent_.PostBitset(o.stream, o.bitset_value);
    }

    std::lock_guard<std::mutex> lk(status_queue_mu_);
    auto it = status_queue_.find(o.stream);
    if (it == status_queue_.end()) continue;
    auto& dp = it->second;
    if (result.ok) {
      dp.pending = false;
      dp.backoff_ms = 1000;
      dp.retry_after_ms = 0;
    } else {
      if (result.retry_after_ms > now) {
        const long long retry_delay_ms = result.retry_after_ms - now;
        dp.retry_after_ms = result.retry_after_ms;
        dp.backoff_ms = std::min(30000, std::max(1000, static_cast<int>(retry_delay_ms)));
      } else {
        const int cur = std::max(250, dp.backoff_ms);
        dp.retry_after_ms = now + cur;
        dp.backoff_ms = std::min(10000, cur * 2);
      }
    }
  }

  last_status_flush_ms_ = now;
}

void Adapter::EmitLog(const std::string& msg) {
  std::cerr << msg << std::endl;
  if (!IsStreamEnabled(kAdapterLogStream)) return;
  const long long ts = now_ms();
  std::lock_guard<std::mutex> lk(adapter_log_mu_);
  adapter_log_buffer_.emplace_back(std::to_string(ts) + " " + msg);
  while (adapter_log_buffer_.size() > 200) {
    adapter_log_buffer_.pop_front();
  }
}

void Adapter::FlushAdapterLogStream(long long now) {
  static constexpr size_t kMaxPendingLogBytes = 64 * 1024;
  if (!IsStreamEnabled(kAdapterLogStream)) return;
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

  const auto result = agent_.PostText(kAdapterLogStream, payload);
  if (result.ok) {
    std::lock_guard<std::mutex> lk(adapter_log_mu_);
    adapter_log_pending_payload_.clear();
    adapter_log_backoff_ms_ = 1000;
    adapter_log_retry_after_ms_ = 0;
    last_adapter_log_pub_ms_ = now;
    return;
  }

  if (result.retry_after_ms > now) {
    const long long retry_delay_ms = result.retry_after_ms - now;
    adapter_log_backoff_ms_ = std::min(30000, std::max(1000, static_cast<int>(retry_delay_ms)));
    adapter_log_retry_after_ms_ = result.retry_after_ms;
  } else {
    const int cur_backoff = adapter_log_backoff_ms_.load();
    const int next_backoff = std::min(10000, cur_backoff * 2);
    adapter_log_backoff_ms_ = next_backoff;
    adapter_log_retry_after_ms_ = now + cur_backoff;
  }
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
    if (IsStreamEnabled(kNavFeedbackStream)) {
      QueueStatusText(kNavFeedbackStream, nav_feedback_to_json(command_id, fb));
    }
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
                                        bool straight,
                                        bool has_waypoint_goal,
                                        double waypoint_goal_x,
                                        double waypoint_goal_y,
                                        double waypoint_goal_yaw_rad) {
  if (!EnsureLocalizedForNavigation(action_name)) return false;

  bool retried_localization = false;
  bool retried_recover = false;
  for (;;) {
    uint32_t command_id = 0;
    const bool nav_ok = has_waypoint_goal
                            ? (straight ? spot_.NavigateToWaypointPoseStraight(
                                              waypoint_id, waypoint_goal_x, waypoint_goal_y,
                                              waypoint_goal_yaw_rad,
                                              cfg_.graphnav_command_timeout_sec, &command_id)
                                        : spot_.NavigateToWaypointPose(
                                              waypoint_id, waypoint_goal_x, waypoint_goal_y,
                                              waypoint_goal_yaw_rad,
                                              cfg_.graphnav_command_timeout_sec, &command_id))
                            : (straight ? spot_.NavigateToWaypointStraight(
                                              waypoint_id, cfg_.graphnav_command_timeout_sec,
                                              &command_id)
                                        : spot_.NavigateToWaypoint(
                                              waypoint_id, cfg_.graphnav_command_timeout_sec,
                                              &command_id));
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

bool Adapter::RetryActiveGraphNavCommandWithRecovery(const std::string& action_name,
                                                     uint32_t* out_command_id) {
  std::string waypoint_id;
  std::string mode;
  bool has_waypoint_goal = false;
  double waypoint_goal_x = 0.0;
  double waypoint_goal_y = 0.0;
  double waypoint_goal_yaw_rad = 0.0;
  {
    std::lock_guard<std::mutex> lk(nav_state_mu_);
    waypoint_id = nav_target_waypoint_id_;
    mode = nav_target_mode_;
    has_waypoint_goal = nav_target_has_waypoint_goal_;
    waypoint_goal_x = nav_target_waypoint_goal_x_;
    waypoint_goal_y = nav_target_waypoint_goal_y_;
    waypoint_goal_yaw_rad = nav_target_waypoint_goal_yaw_rad_;
  }
  if (waypoint_id.empty()) return false;
  const bool straight =
      (mode == "waypoint_straight" || mode == "pose_straight");
  return StartNavigateWithRecovery(action_name, waypoint_id, out_command_id, straight,
                                   has_waypoint_goal, waypoint_goal_x, waypoint_goal_y,
                                   waypoint_goal_yaw_rad);
}

bool Adapter::ExecuteWaypointGotoCommand(const v1::model::CommandRequest& request) {
  if (!EnsureLeaseForCommand("spot.waypoint.goto")) return false;
  std::string text;
  (void)ExtractCommandText(request, &text);
  const std::string requested_map_uuid = Trim(ExtractParam(text, {"map_uuid"}));
  std::string map_id;
  std::string map_uuid;
  if (!ValidateGraphNavCommandMapUuid("spot.waypoint.goto", requested_map_uuid, false,
                                      &map_id, &map_uuid)) {
    return false;
  }
  std::string waypoint_id = Trim(ExtractParam(text, {"waypoint_id"}));
  std::string name = Trim(ExtractParam(text, {"name", "waypoint", "waypoint_name", "alias"}));
  if (waypoint_id.empty() && name.empty()) {
    EmitLog("[graphnav] spot.waypoint.goto rejected: missing waypoint_id or waypoint name");
    return false;
  }

  {
    std::lock_guard<std::mutex> lk(map_mu_);
    if (waypoint_id.empty()) {
      if (!ResolveWaypointNameLocked(name, &waypoint_id)) {
        EmitLog(std::string("[graphnav] spot.waypoint.goto failed: unresolved or ambiguous name=") + name);
        return false;
      }
    } else if (name.empty()) {
      name = ResolveWaypointNameForIdLocked(waypoint_id);
      if (name.empty()) name = waypoint_id;
    }
  }

  uint32_t command_id = 0;
  if (!StartNavigateWithRecovery("spot.waypoint.goto", waypoint_id, &command_id)) return false;

  bool has_seed_goal = false;
  double seed_x = 0.0;
  double seed_y = 0.0;
  double seed_yaw_rad = 0.0;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    SpotClient::Pose3D waypoint_pose;
    std::string resolved_map_uuid;
    if (LookupWaypointSeedPoseLocked(waypoint_id, &waypoint_pose, &resolved_map_uuid)) {
      has_seed_goal = true;
      seed_x = waypoint_pose.x;
      seed_y = waypoint_pose.y;
      seed_yaw_rad = quaternion_yaw_rad(pose_quaternion(waypoint_pose));
      if (map_uuid.empty()) map_uuid = resolved_map_uuid;
    }
  }
  SetGraphNavNavTarget("waypoint", waypoint_id, name, map_id, map_uuid,
                       has_seed_goal, seed_x, seed_y, seed_yaw_rad);
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
  const std::string requested_map_uuid = Trim(ExtractParam(text, {"map_uuid"}));
  std::string map_id;
  std::string map_uuid;
  if (!ValidateGraphNavCommandMapUuid("spot.waypoint.goto_straight", requested_map_uuid, false,
                                      &map_id, &map_uuid)) {
    return false;
  }
  std::string waypoint_id = Trim(ExtractParam(text, {"waypoint_id"}));
  std::string name = Trim(ExtractParam(text, {"name", "waypoint", "waypoint_name", "alias"}));
  if (waypoint_id.empty() && name.empty()) {
    EmitLog("[graphnav] spot.waypoint.goto_straight rejected: missing waypoint_id or waypoint name");
    return false;
  }

  {
    std::lock_guard<std::mutex> lk(map_mu_);
    if (waypoint_id.empty()) {
      if (!ResolveWaypointNameLocked(name, &waypoint_id)) {
        EmitLog(std::string("[graphnav] spot.waypoint.goto_straight failed: unresolved or ambiguous name=") + name);
        return false;
      }
    } else if (name.empty()) {
      name = ResolveWaypointNameForIdLocked(waypoint_id);
      if (name.empty()) name = waypoint_id;
    }
  }

  uint32_t command_id = 0;
  if (!StartNavigateWithRecovery("spot.waypoint.goto_straight", waypoint_id, &command_id, true)) return false;

  bool has_seed_goal = false;
  double seed_x = 0.0;
  double seed_y = 0.0;
  double seed_yaw_rad = 0.0;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    SpotClient::Pose3D waypoint_pose;
    std::string resolved_map_uuid;
    if (LookupWaypointSeedPoseLocked(waypoint_id, &waypoint_pose, &resolved_map_uuid)) {
      has_seed_goal = true;
      seed_x = waypoint_pose.x;
      seed_y = waypoint_pose.y;
      seed_yaw_rad = quaternion_yaw_rad(pose_quaternion(waypoint_pose));
      if (map_uuid.empty()) map_uuid = resolved_map_uuid;
    }
  }
  SetGraphNavNavTarget("waypoint_straight", waypoint_id, name, map_id, map_uuid,
                       has_seed_goal, seed_x, seed_y, seed_yaw_rad);
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

bool Adapter::ExecuteGraphNavGotoPoseCommand(const v1::model::CommandRequest& request,
                                             bool straight) {
  const std::string action_name =
      straight ? "spot.graphnav.goto_pose_straight" : "spot.graphnav.goto_pose";
  if (!EnsureLeaseForCommand(action_name)) return false;

  std::string text;
  (void)ExtractCommandText(request, &text);
  std::string map_id;
  std::string map_uuid;
  if (!ValidateGraphNavCommandMapUuid(action_name, Trim(ExtractParam(text, {"map_uuid"})), true,
                                      &map_id, &map_uuid)) {
    return false;
  }

  const auto parse_double_value = [](const std::string& raw, double* out_value) -> bool {
    if (!out_value) return false;
    const std::string trimmed = Adapter::Trim(raw);
    if (trimmed.empty()) return false;
    try {
      size_t consumed = 0;
      const double value = std::stod(trimmed, &consumed);
      if (consumed != trimmed.size()) return false;
      *out_value = value;
      return true;
    } catch (...) {
      return false;
    }
  };

  double seed_x = 0.0;
  double seed_y = 0.0;
  if (!parse_double_value(ExtractParam(text, {"x", "seed_x"}), &seed_x) ||
      !parse_double_value(ExtractParam(text, {"y", "seed_y"}), &seed_y)) {
    EmitLog("[graphnav] " + action_name + " rejected: missing x/y seed-frame goal");
    return false;
  }

  bool has_explicit_yaw = false;
  double seed_yaw_rad = 0.0;
  if (parse_double_value(ExtractParam(text, {"yaw_rad", "yaw", "theta", "heading_rad"}),
                         &seed_yaw_rad)) {
    has_explicit_yaw = true;
  } else {
    double seed_yaw_deg = 0.0;
    if (parse_double_value(ExtractParam(text, {"yaw_deg", "heading_deg", "degrees"}),
                           &seed_yaw_deg)) {
      has_explicit_yaw = true;
      seed_yaw_rad = seed_yaw_deg * 3.14159265358979323846 / 180.0;
    }
  }

  std::string waypoint_id;
  std::string waypoint_name;
  double waypoint_seed_x = 0.0;
  double waypoint_seed_y = 0.0;
  double waypoint_yaw_rad = 0.0;
  double waypoint_distance_m = 0.0;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    if (!graphnav_map_artifacts_ || graphnav_map_artifacts_->map_id != map_id ||
        graphnav_map_artifacts_->map_uuid != map_uuid) {
      EmitLog("[graphnav] " + action_name +
              " rejected: active stitched map artifacts do not match requested map_uuid");
      return false;
    }
    const auto& waypoints = graphnav_map_artifacts_->snapshot.waypoints;
    if (waypoints.empty()) {
      EmitLog("[graphnav] " + action_name + " rejected: active map has no anchored waypoints");
      return false;
    }

    const SpotClient::GraphNavWaypoint* nearest_waypoint = nullptr;
    double best_distance_sq = std::numeric_limits<double>::infinity();
    for (const auto& waypoint : waypoints) {
      const double dx = seed_x - waypoint.seed_tform_waypoint.x;
      const double dy = seed_y - waypoint.seed_tform_waypoint.y;
      const double distance_sq = dx * dx + dy * dy;
      if (!nearest_waypoint || distance_sq < best_distance_sq) {
        nearest_waypoint = &waypoint;
        best_distance_sq = distance_sq;
      }
    }
    if (!nearest_waypoint) {
      EmitLog("[graphnav] " + action_name + " rejected: failed to resolve nearest waypoint");
      return false;
    }
    waypoint_id = nearest_waypoint->id;
    waypoint_name = ResolveWaypointNameForIdLocked(waypoint_id);
    if (waypoint_name.empty()) {
      waypoint_name = nearest_waypoint->label.empty() ? waypoint_id : nearest_waypoint->label;
    }
    waypoint_seed_x = nearest_waypoint->seed_tform_waypoint.x;
    waypoint_seed_y = nearest_waypoint->seed_tform_waypoint.y;
    waypoint_yaw_rad = quaternion_yaw_rad(pose_quaternion(nearest_waypoint->seed_tform_waypoint));
    waypoint_distance_m = std::sqrt(best_distance_sq);
  }

  if (!has_explicit_yaw) seed_yaw_rad = waypoint_yaw_rad;
  seed_yaw_rad = normalize_angle_rad(seed_yaw_rad);

  const double dx = seed_x - waypoint_seed_x;
  const double dy = seed_y - waypoint_seed_y;
  const double c = std::cos(waypoint_yaw_rad);
  const double s = std::sin(waypoint_yaw_rad);
  const double waypoint_goal_x = c * dx + s * dy;
  const double waypoint_goal_y = -s * dx + c * dy;
  const double waypoint_goal_yaw_rad = normalize_angle_rad(seed_yaw_rad - waypoint_yaw_rad);

  EmitLog("[graphnav] " + action_name + " target_seed=(" + format_decimal(seed_x, 3) + "," +
          format_decimal(seed_y, 3) + "," + format_decimal(seed_yaw_rad, 3) +
          ") via_waypoint=" + waypoint_name + " id=" + waypoint_id +
          " waypoint_distance_m=" + format_decimal(waypoint_distance_m, 2) +
          " waypoint_goal=(" + format_decimal(waypoint_goal_x, 3) + "," +
          format_decimal(waypoint_goal_y, 3) + "," +
          format_decimal(waypoint_goal_yaw_rad, 3) + ")");

  uint32_t command_id = 0;
  if (!StartNavigateWithRecovery(action_name, waypoint_id, &command_id, straight, true,
                                 waypoint_goal_x, waypoint_goal_y,
                                 waypoint_goal_yaw_rad)) {
    return false;
  }

  SetGraphNavNavTarget(straight ? "pose_straight" : "pose",
                       waypoint_id, waypoint_name, map_id, map_uuid,
                       true, seed_x, seed_y, seed_yaw_rad,
                       true, waypoint_goal_x, waypoint_goal_y, waypoint_goal_yaw_rad);
  last_graph_nav_command_id_ = command_id;
  graphnav_navigation_active_ = true;
  nav_auto_recovered_ = false;
  const bool ok = WaitForGraphNavCommandResult(
      command_id, static_cast<long long>(std::max(30, cfg_.graphnav_command_timeout_sec)) * 1000LL + 30000LL);
  graphnav_navigation_active_ = false;
  last_graph_nav_command_id_ = 0;
  if (ok) {
    EmitLog("[graphnav] " + action_name + " success via_waypoint=" + waypoint_name);
  }
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
    std::string map_uuid;
    {
      std::lock_guard<std::mutex> lk(map_mu_);
      if (!ResolveSavedDockHomeLocked(&map_id, &dock_waypoint_id)) {
        EmitLog("[return_and_dock] failed: no saved docking waypoint for active map");
        return false;
      }
      if (graphnav_map_artifacts_ && graphnav_map_artifacts_->map_id == map_id) {
        map_uuid = graphnav_map_artifacts_->map_uuid;
      }
    }
    EmitLog("[return_and_dock] navigating to dock waypoint map_id=" + map_id +
            " waypoint_id=" + dock_waypoint_id);
    uint32_t nav_command_id = 0;
    if (!StartNavigateWithRecovery("spot.return_and_dock", dock_waypoint_id, &nav_command_id)) return false;
    last_graph_nav_command_id_ = nav_command_id;
    graphnav_navigation_active_ = true;
    SetGraphNavNavTarget("return_and_dock", dock_waypoint_id, "dock_home", map_id, map_uuid);
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

bool Adapter::ExecuteUndockCommand() {
  if (docking_in_progress_.load()) {
    EmitLog("[undock] request ignored: dock operation already in progress");
    return false;
  }
  if (!SpotConnected()) {
    EmitLog("[undock] failed: robot unavailable");
    return false;
  }
  if (spot_degraded_non_estop_.load()) {
    EmitLog("[undock] failed: robot degraded (non-estop)");
    return false;
  }
  if (!EnsureLeaseForCommand("spot.undock")) return false;

  int dock_status = 0;
  if (!spot_.GetDockingStatus(&dock_status)) {
    EmitLog(std::string("[undock] failed: unable to query dock state: ") + spot_.LastError());
    return false;
  }

  if (dock_status == ::bosdyn::api::docking::DockState_DockedStatus_DOCK_STATUS_UNDOCKED) {
    EmitLog("[undock] rejected: robot is already undocked");
    return false;
  }
  if (dock_status == ::bosdyn::api::docking::DockState_DockedStatus_DOCK_STATUS_UNDOCKING) {
    EmitLog("[undock] rejected: robot is already undocking");
    return false;
  }
  if (dock_status != ::bosdyn::api::docking::DockState_DockedStatus_DOCK_STATUS_DOCKED) {
    EmitLog("[undock] rejected: unexpected dock state=" + std::to_string(dock_status));
    return false;
  }

  docking_in_progress_ = true;
  moving_ = false;
  spot_.ZeroVelocity(1);
  const int dock_poll_ms = std::max(250, cfg_.dock_poll_ms);
  EmitLog("[undock] starting");
  const bool ok = spot_.Undock(dock_poll_ms, cfg_.dock_command_timeout_sec);
  docking_in_progress_ = false;
  if (!ok) {
    EmitLog(std::string("[undock] failed: ") + spot_.LastError());
    return false;
  }
  dock_cooldown_until_ms_ = now_ms() + 5000;
  EmitLog("[undock] success");
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
  if (command == "spot.undock") return ExecuteUndockCommand();
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
  if (command == "spot.graphnav.goto_pose") return ExecuteGraphNavGotoPoseCommand(request, false);
  if (command == "spot.graphnav.goto_pose_straight") {
    return ExecuteGraphNavGotoPoseCommand(request, true);
  }
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
  if (!spot_.HasArm()) {
    EmitLog(std::string(require_teleop ? "[arm] reset rejected: no manipulator detected"
                                       : "[command] spot.reset_arm rejected: no manipulator detected"));
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
  if (!spot_.HasArm()) return;

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
  if (!spot_.HasArm()) return true;
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
  if (!spot_.HasArm()) return true;
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
      if (IsStreamEnabled(kNavFeedbackStream)) {
        QueueStatusText(kNavFeedbackStream, nav_feedback_to_json(command_id, fb));
      }
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
        if (!RetryActiveGraphNavCommandWithRecovery("return_and_dock.teleop.retry", &retry_id)) {
          return false;
        }
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
        std::string map_uuid;
        {
          std::lock_guard<std::mutex> lk(map_mu_);
          if (graphnav_map_artifacts_ && graphnav_map_artifacts_->map_id == map_id) {
            map_uuid = graphnav_map_artifacts_->map_uuid;
          }
        }
        SetGraphNavNavTarget("return_and_dock", dock_waypoint_id, "dock_home", map_id, map_uuid);
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
  return SaveMapToDisk(map_id, map_data);
}

bool Adapter::SaveMapToDisk(const std::string& map_id, const SpotClient::StoredMap& map_data) {
  if (map_id.empty()) return false;
  const std::string map_dir = MapDirectoryFor(map_id);
  std::string storage_error;
  if (!recover_map_directory_state(map_dir, &storage_error)) {
    EmitLog(std::string("[graphnav] failed preparing map directory: ") + storage_error);
    return false;
  }

  const std::string temp_dir = map_staging_directory(map_dir);
  const std::string backup_dir = map_backup_directory(map_dir);
  const auto cleanup_temp_dir = [&temp_dir]() {
    std::error_code cleanup_ec;
    std::filesystem::remove_all(temp_dir, cleanup_ec);
  };
  std::error_code ec;
  std::filesystem::remove_all(temp_dir, ec);
  if (ec) {
    EmitLog(std::string("[graphnav] failed clearing map staging dir: ") + ec.message());
    return false;
  }
  std::filesystem::create_directories(temp_dir + "/waypoints", ec);
  std::filesystem::create_directories(temp_dir + "/edges", ec);
  if (ec) {
    EmitLog(std::string("[graphnav] failed creating map directories: ") + ec.message());
    cleanup_temp_dir();
    return false;
  }

  {
    std::ofstream graph_out(temp_dir + "/graph.bin", std::ios::binary | std::ios::trunc);
    if (!graph_out || !map_data.graph.SerializeToOstream(&graph_out)) {
      EmitLog("[graphnav] failed writing graph file");
      cleanup_temp_dir();
      return false;
    }
  }

  for (const auto& snapshot : map_data.waypoint_snapshots) {
    std::ofstream out(temp_dir + "/waypoints/" + sanitize_id_token(snapshot.id()) + ".bin",
                      std::ios::binary | std::ios::trunc);
    if (!out || !snapshot.SerializeToOstream(&out)) {
      EmitLog("[graphnav] failed writing waypoint snapshot");
      cleanup_temp_dir();
      return false;
    }
  }
  for (const auto& snapshot : map_data.edge_snapshots) {
    std::ofstream out(temp_dir + "/edges/" + sanitize_id_token(snapshot.id()) + ".bin",
                      std::ios::binary | std::ios::trunc);
    if (!out || !snapshot.SerializeToOstream(&out)) {
      EmitLog("[graphnav] failed writing edge snapshot");
      cleanup_temp_dir();
      return false;
    }
  }
  {
    std::ofstream ready_out(map_staging_ready_marker(temp_dir), std::ios::binary | std::ios::trunc);
    if (!ready_out) {
      EmitLog("[graphnav] failed writing map staging marker");
      cleanup_temp_dir();
      return false;
    }
  }

  bool moved_existing_map = false;
  const bool has_existing_map = std::filesystem::exists(map_dir, ec);
  if (ec) {
    EmitLog(std::string("[graphnav] failed checking existing map dir: ") + ec.message());
    cleanup_temp_dir();
    return false;
  }
  if (has_existing_map) {
    std::filesystem::remove_all(backup_dir, ec);
    if (ec) {
      EmitLog(std::string("[graphnav] failed clearing map backup dir: ") + ec.message());
      cleanup_temp_dir();
      return false;
    }
    std::filesystem::rename(map_dir, backup_dir, ec);
    if (ec) {
      EmitLog(std::string("[graphnav] failed moving current map to backup: ") + ec.message());
      cleanup_temp_dir();
      return false;
    }
    moved_existing_map = true;
  }

  std::filesystem::rename(temp_dir, map_dir, ec);
  if (ec) {
    if (moved_existing_map) {
      std::error_code restore_ec;
      std::filesystem::rename(backup_dir, map_dir, restore_ec);
      if (restore_ec) {
        EmitLog(std::string("[graphnav] failed restoring map backup after save error: ") +
                restore_ec.message());
      }
    }
    EmitLog(std::string("[graphnav] failed promoting staged map dir: ") + ec.message());
    return false;
  }

  if (moved_existing_map) {
    std::filesystem::remove_all(backup_dir, ec);
    if (ec) {
      EmitLog(std::string("[graphnav] warning: failed removing map backup dir: ") + ec.message());
    }
  }
  return true;
}

bool Adapter::LoadMapFromDisk(const std::string& map_id, SpotClient::StoredMap* out_map) {
  if (!out_map || map_id.empty()) return false;
  SpotClient::StoredMap map_data;
  const std::string map_dir = MapDirectoryFor(map_id);
  std::string storage_error;
  if (!recover_map_directory_state(map_dir, &storage_error)) {
    EmitLog(std::string("[graphnav] failed recovering saved map dir: ") + storage_error);
    return false;
  }

  {
    std::ifstream graph_in(map_dir + "/graph.bin", std::ios::binary);
    if (!graph_in || !map_data.graph.ParseFromIstream(&graph_in)) return false;
  }

  std::vector<std::string> expected_waypoint_snapshot_ids;
  expected_waypoint_snapshot_ids.reserve(static_cast<size_t>(map_data.graph.waypoints_size()));
  std::unordered_set<std::string> seen_waypoint_snapshot_ids;
  seen_waypoint_snapshot_ids.reserve(static_cast<size_t>(map_data.graph.waypoints_size()));
  for (const auto& waypoint : map_data.graph.waypoints()) {
    if (!waypoint.snapshot_id().empty() &&
        seen_waypoint_snapshot_ids.insert(waypoint.snapshot_id()).second) {
      expected_waypoint_snapshot_ids.push_back(waypoint.snapshot_id());
    }
  }

  std::vector<std::string> expected_edge_snapshot_ids;
  expected_edge_snapshot_ids.reserve(static_cast<size_t>(map_data.graph.edges_size()));
  std::unordered_set<std::string> seen_edge_snapshot_ids;
  seen_edge_snapshot_ids.reserve(static_cast<size_t>(map_data.graph.edges_size()));
  for (const auto& edge : map_data.graph.edges()) {
    if (!edge.snapshot_id().empty() &&
        seen_edge_snapshot_ids.insert(edge.snapshot_id()).second) {
      expected_edge_snapshot_ids.push_back(edge.snapshot_id());
    }
  }

  map_data.waypoint_snapshots.reserve(expected_waypoint_snapshot_ids.size());
  for (const auto& snapshot_id : expected_waypoint_snapshot_ids) {
    std::ifstream in(map_dir + "/waypoints/" + sanitize_id_token(snapshot_id) + ".bin",
                     std::ios::binary);
    if (!in) return false;
    ::bosdyn::api::graph_nav::WaypointSnapshot snapshot;
    if (!snapshot.ParseFromIstream(&in)) return false;
    if (snapshot.id() != snapshot_id) return false;
    map_data.waypoint_snapshots.push_back(std::move(snapshot));
  }

  map_data.edge_snapshots.reserve(expected_edge_snapshot_ids.size());
  for (const auto& snapshot_id : expected_edge_snapshot_ids) {
    std::ifstream in(map_dir + "/edges/" + sanitize_id_token(snapshot_id) + ".bin",
                     std::ios::binary);
    if (!in) return false;
    ::bosdyn::api::graph_nav::EdgeSnapshot snapshot;
    if (!snapshot.ParseFromIstream(&in)) return false;
    if (snapshot.id() != snapshot_id) return false;
    map_data.edge_snapshots.push_back(std::move(snapshot));
  }

  *out_map = std::move(map_data);
  return true;
}

bool Adapter::DeleteMapFromDisk(const std::string& map_id) {
  if (map_id.empty()) return false;
  std::error_code ec;
  const std::string map_dir = MapDirectoryFor(map_id);
  std::filesystem::remove_all(map_dir, ec);
  if (ec) return false;
  std::filesystem::remove_all(map_staging_directory(map_dir), ec);
  if (ec) return false;
  std::filesystem::remove_all(map_backup_directory(map_dir), ec);
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

void Adapter::ClearGraphNavMapArtifacts() {
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    graphnav_map_artifacts_.reset();
    ++graphnav_map_artifacts_version_;
  }
  next_graphnav_map_post_ms_ = 0;
  graphnav_map_post_backoff_ms_ = 1000;
  last_graphnav_map_posted_version_ = 0;
  force_graphnav_metadata_publish_ = true;
}

bool Adapter::RefreshGraphNavMapArtifacts(const std::string& map_id,
                                          const SpotClient::StoredMap& map_data) {
  SpotClient::GraphNavMapSnapshot graphnav_snapshot;
  if (!spot_.BuildGraphNavMapSnapshot(map_data, &graphnav_snapshot)) {
    EmitLog(std::string("[graphnav] failed to build stitched map snapshot map_id=") + map_id);
    ClearGraphNavMapArtifacts();
    return false;
  }

  std::unordered_map<std::string, std::vector<std::string>> aliases_by_waypoint;
  std::string dock_waypoint_id;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    const auto alias_it = waypoint_aliases_by_map_.find(map_id);
    if (alias_it != waypoint_aliases_by_map_.end()) {
      for (const auto& alias : alias_it->second) {
        aliases_by_waypoint[alias.second].push_back(alias.first);
      }
    }
    const auto dock_it = dock_waypoint_by_map_.find(map_id);
    if (dock_it != dock_waypoint_by_map_.end()) {
      dock_waypoint_id = dock_it->second;
    }
  }
  for (auto& kv : aliases_by_waypoint) {
    std::sort(kv.second.begin(), kv.second.end());
    kv.second.erase(std::unique(kv.second.begin(), kv.second.end()), kv.second.end());
  }

  const std::string map_uuid = build_graphnav_map_uuid(map_id, map_data);

  auto artifacts = std::make_shared<GraphNavMapArtifacts>();
  artifacts->map_id = map_id;
  artifacts->map_uuid = map_uuid;
  artifacts->snapshot = std::move(graphnav_snapshot);
  artifacts->metadata_json = build_graphnav_metadata_json(
      map_id, map_uuid, artifacts->snapshot, aliases_by_waypoint, dock_waypoint_id);

  {
    std::lock_guard<std::mutex> lk(map_mu_);
    graphnav_map_artifacts_ = std::move(artifacts);
    ++graphnav_map_artifacts_version_;
  }
  next_graphnav_map_post_ms_ = 0;
  graphnav_map_post_backoff_ms_ = 1000;
  force_graphnav_metadata_publish_ = true;
  return true;
}

bool Adapter::RefreshGraphNavMapArtifactsFromDisk(const std::string& map_id) {
  if (map_id.empty()) {
    ClearGraphNavMapArtifacts();
    return false;
  }

  SpotClient::StoredMap map_data;
  if (!LoadMapFromDisk(map_id, &map_data)) {
    EmitLog(std::string("[graphnav] failed loading stitched map artifacts map_id=") + map_id);
    ClearGraphNavMapArtifacts();
    return false;
  }
  return RefreshGraphNavMapArtifacts(map_id, map_data);
}

void Adapter::RefreshGraphNavMetadataForMap(const std::string& map_id) {
  if (map_id.empty()) return;

  std::unordered_map<std::string, std::vector<std::string>> aliases_by_waypoint;
  std::string dock_waypoint_id;
  std::shared_ptr<GraphNavMapArtifacts> current_artifacts;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    current_artifacts = graphnav_map_artifacts_;
    if (!current_artifacts || current_artifacts->map_id != map_id) return;

    const auto alias_it = waypoint_aliases_by_map_.find(map_id);
    if (alias_it != waypoint_aliases_by_map_.end()) {
      for (const auto& alias : alias_it->second) {
        aliases_by_waypoint[alias.second].push_back(alias.first);
      }
    }
    const auto dock_it = dock_waypoint_by_map_.find(map_id);
    if (dock_it != dock_waypoint_by_map_.end()) {
      dock_waypoint_id = dock_it->second;
    }
  }

  for (auto& kv : aliases_by_waypoint) {
    std::sort(kv.second.begin(), kv.second.end());
    kv.second.erase(std::unique(kv.second.begin(), kv.second.end()), kv.second.end());
  }

  auto updated_artifacts = std::make_shared<GraphNavMapArtifacts>(*current_artifacts);
  updated_artifacts->metadata_json = build_graphnav_metadata_json(
      updated_artifacts->map_id, updated_artifacts->map_uuid, updated_artifacts->snapshot,
      aliases_by_waypoint, dock_waypoint_id);

  {
    std::lock_guard<std::mutex> lk(map_mu_);
    if (!graphnav_map_artifacts_ || graphnav_map_artifacts_->map_id != map_id ||
        graphnav_map_artifacts_->map_uuid != current_artifacts->map_uuid) {
      return;
    }
    graphnav_map_artifacts_ = std::move(updated_artifacts);
  }
  force_graphnav_metadata_publish_ = true;
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
  RefreshGraphNavMetadataForMap(map_id);
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
  if (cfg_.waypoint_text_stream.empty()) return;
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
  if (cfg_.maps_text_stream.empty()) return;
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

void Adapter::PublishGraphNavMetadataText(bool force) {
  if (cfg_.graphnav_metadata_stream.empty()) return;
  const long long kPeriodicMs = 5000;
  const long long now = now_ms();
  const bool force_flag = force_graphnav_metadata_publish_.exchange(false);
  std::string payload;
  bool changed = false;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    payload = graphnav_map_artifacts_ ? graphnav_map_artifacts_->metadata_json : "";
    changed = (payload != last_graphnav_metadata_payload_);
    if (changed) last_graphnav_metadata_payload_ = payload;
  }
  if (!force && !force_flag && !changed &&
      (now - last_graphnav_metadata_pub_ms_.load()) < kPeriodicMs) {
    return;
  }
  QueueStatusText(cfg_.graphnav_metadata_stream, payload);
  last_graphnav_metadata_pub_ms_ = now;
}

void Adapter::MaybePublishGraphNavMap(long long now) {
  if (cfg_.graphnav_map_stream.empty()) return;
  if (now < next_graphnav_map_post_ms_.load()) return;

  std::shared_ptr<GraphNavMapArtifacts> artifacts;
  uint64_t version = 0;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    artifacts = graphnav_map_artifacts_;
    version = graphnav_map_artifacts_version_;
  }
  if (!artifacts || !artifacts->snapshot.has_map) return;
  if (version == last_graphnav_map_posted_version_) return;

  const SpotClient::Pose3D identity_pose;
  // The current vendored datapoint proto has no top-level Datapoint.map field.
  // Keep the dedicated map stream isolated here so switching to agent_.PostMap(...)
  // is a transport-only change when the upstream proto surface catches up.
  const auto result = agent_.PostLocalization(
      cfg_.graphnav_map_stream,
      build_formant_localization(identity_pose, artifacts->snapshot.map,
                                 artifacts->map_uuid + ":map_only"));
  if (result.ok) {
    last_graphnav_map_posted_version_ = version;
    graphnav_map_post_backoff_ms_ = 1000;
    next_graphnav_map_post_ms_ = 0;
    return;
  }

  if (result.retry_after_ms > now) {
    const long long retry_delay_ms = result.retry_after_ms - now;
    next_graphnav_map_post_ms_ = result.retry_after_ms;
    graphnav_map_post_backoff_ms_ =
        std::min(30000, std::max(1000, static_cast<int>(retry_delay_ms)));
  } else {
    const int backoff_ms = std::max(250, graphnav_map_post_backoff_ms_);
    next_graphnav_map_post_ms_ = now + backoff_ms;
    graphnav_map_post_backoff_ms_ = std::min(10000, backoff_ms * 2);
  }
}

void Adapter::PublishCurrentMapText(bool force) {
  if (!IsStreamEnabled(kCurrentMapStream)) return;
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
  QueueStatusText(kCurrentMapStream, text);
  last_current_map_pub_ms_ = now;
}

void Adapter::PublishDefaultMapText(bool force) {
  if (!IsStreamEnabled(kDefaultMapStream)) return;
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
  QueueStatusText(kDefaultMapStream, text);
  last_default_map_pub_ms_ = now;
}

void Adapter::PollCurrentWaypointAtStatus(long long now) {
  if (!IsStreamEnabled(kCurrentWaypointStream)) return;
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
  if (!IsStreamEnabled(kCurrentWaypointStream)) return;
  if ((now - last_waypoint_at_pub_ms_.load()) < 10000) return;
  std::string text;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    text = current_waypoint_at_text_;
  }
  QueueStatusText(kCurrentWaypointStream, text);
  last_waypoint_at_pub_ms_ = now;
}

bool Adapter::LookupWaypointSeedPoseLocked(const std::string& waypoint_id,
                                           SpotClient::Pose3D* out_pose,
                                           std::string* out_map_uuid) const {
  if (waypoint_id.empty() || !out_pose || !graphnav_map_artifacts_) return false;
  const auto* waypoint = find_graphnav_waypoint(graphnav_map_artifacts_->snapshot, waypoint_id);
  if (!waypoint) return false;
  *out_pose = waypoint->seed_tform_waypoint;
  if (out_map_uuid) *out_map_uuid = graphnav_map_artifacts_->map_uuid;
  return true;
}

void Adapter::ClearGraphNavNavTargetLocked() {
  nav_target_mode_.clear();
  nav_target_waypoint_id_.clear();
  nav_target_waypoint_name_.clear();
  nav_target_map_id_.clear();
  nav_target_map_uuid_.clear();
  nav_target_has_seed_goal_ = false;
  nav_target_seed_x_ = 0.0;
  nav_target_seed_y_ = 0.0;
  nav_target_seed_yaw_rad_ = 0.0;
  nav_target_has_waypoint_goal_ = false;
  nav_target_waypoint_goal_x_ = 0.0;
  nav_target_waypoint_goal_y_ = 0.0;
  nav_target_waypoint_goal_yaw_rad_ = 0.0;
}

void Adapter::SetGraphNavNavTarget(const std::string& mode,
                                   const std::string& waypoint_id,
                                   const std::string& waypoint_name,
                                   const std::string& map_id,
                                   const std::string& map_uuid,
                                   bool has_seed_goal,
                                   double seed_x,
                                   double seed_y,
                                   double seed_yaw_rad,
                                   bool has_waypoint_goal,
                                   double waypoint_goal_x,
                                   double waypoint_goal_y,
                                   double waypoint_goal_yaw_rad) {
  std::lock_guard<std::mutex> lk(nav_state_mu_);
  nav_target_mode_ = mode;
  nav_target_waypoint_id_ = waypoint_id;
  nav_target_waypoint_name_ = waypoint_name;
  nav_target_map_id_ = map_id;
  nav_target_map_uuid_ = map_uuid;
  nav_target_has_seed_goal_ = has_seed_goal;
  nav_target_seed_x_ = seed_x;
  nav_target_seed_y_ = seed_y;
  nav_target_seed_yaw_rad_ = seed_yaw_rad;
  nav_target_has_waypoint_goal_ = has_waypoint_goal;
  nav_target_waypoint_goal_x_ = waypoint_goal_x;
  nav_target_waypoint_goal_y_ = waypoint_goal_y;
  nav_target_waypoint_goal_yaw_rad_ = waypoint_goal_yaw_rad;
  last_nav_feedback_signature_.clear();
  last_nav_status_ = 0;
  last_nav_remaining_route_m_ = 0.0;
  last_nav_progress_change_ms_ = 0;
}

bool Adapter::ValidateGraphNavCommandMapUuid(const std::string& command_name,
                                             const std::string& requested_map_uuid,
                                             bool require_map_uuid,
                                             std::string* out_map_id,
                                             std::string* out_map_uuid) {
  const std::string requested = Trim(requested_map_uuid);
  std::string active_map_id;
  std::string active_map_uuid;
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    active_map_id = active_map_id_.empty() ? default_map_id_ : active_map_id_;
    if (graphnav_map_artifacts_ &&
        (active_map_id.empty() || graphnav_map_artifacts_->map_id == active_map_id)) {
      active_map_uuid = graphnav_map_artifacts_->map_uuid;
      if (active_map_id.empty()) active_map_id = graphnav_map_artifacts_->map_id;
    }
  }

  if (active_map_id.empty()) {
    EmitLog("[graphnav] " + command_name + " rejected: no active GraphNav map");
    return false;
  }
  if (require_map_uuid && requested.empty()) {
    EmitLog("[graphnav] " + command_name + " rejected: missing map_uuid");
    return false;
  }
  if (!requested.empty()) {
    if (active_map_uuid.empty()) {
      EmitLog("[graphnav] " + command_name +
              " rejected: active map has no published map_uuid");
      return false;
    }
    if (requested != active_map_uuid) {
      EmitLog("[graphnav] " + command_name + " rejected: stale map_uuid requested=" +
              requested + " active=" + active_map_uuid);
      return false;
    }
  }
  if (out_map_id) *out_map_id = active_map_id;
  if (out_map_uuid) *out_map_uuid = active_map_uuid;
  return true;
}

std::string Adapter::BuildGraphNavNavStateJson(
    const SpotClient::LocalizationSnapshot* localization) const {
  uint32_t command_id = last_graph_nav_command_id_.load();
  const bool active = graphnav_navigation_active_.load();
  const bool connected = SpotConnected();
  const bool auto_recovered = nav_auto_recovered_.load();

  std::string mode;
  std::string waypoint_id;
  std::string waypoint_name;
  std::string map_id;
  std::string map_uuid;
  bool has_seed_goal = false;
  double seed_x = 0.0;
  double seed_y = 0.0;
  double seed_yaw_rad = 0.0;
  bool has_waypoint_goal = false;
  double waypoint_goal_x = 0.0;
  double waypoint_goal_y = 0.0;
  double waypoint_goal_yaw_rad = 0.0;
  int status = 0;
  double remaining_route_m = 0.0;
  const bool localized = localization && !localization->waypoint_id.empty();
  const bool has_current_seed_pose = localized && localization->has_seed_tform_body;
  double current_seed_x = 0.0;
  double current_seed_y = 0.0;
  double current_seed_z = 0.0;
  double current_seed_yaw_rad = 0.0;
  if (has_current_seed_pose) {
    current_seed_x = localization->seed_tform_body_x;
    current_seed_y = localization->seed_tform_body_y;
    current_seed_z = localization->seed_tform_body_z;
    const SpotClient::Pose3D seed_tform_body =
        pose3d_from_localization_snapshot(*localization);
    current_seed_yaw_rad = quaternion_yaw_rad(pose_quaternion(seed_tform_body));
  }

  {
    std::lock_guard<std::mutex> lk(nav_state_mu_);
    mode = nav_target_mode_;
    waypoint_id = nav_target_waypoint_id_;
    waypoint_name = nav_target_waypoint_name_;
    map_id = nav_target_map_id_;
    map_uuid = nav_target_map_uuid_;
    has_seed_goal = nav_target_has_seed_goal_;
    seed_x = nav_target_seed_x_;
    seed_y = nav_target_seed_y_;
    seed_yaw_rad = nav_target_seed_yaw_rad_;
    has_waypoint_goal = nav_target_has_waypoint_goal_;
    waypoint_goal_x = nav_target_waypoint_goal_x_;
    waypoint_goal_y = nav_target_waypoint_goal_y_;
    waypoint_goal_yaw_rad = nav_target_waypoint_goal_yaw_rad_;
    status = last_nav_status_;
    remaining_route_m = last_nav_remaining_route_m_;
  }

  std::ostringstream oss;
  oss << "{\"connected\":" << (connected ? "true" : "false")
      << ",\"active\":" << (active ? "true" : "false")
      << ",\"command_id\":" << command_id
      << ",\"status\":" << status
      << ",\"status_name\":\"" << nav_status_name(status) << "\""
      << ",\"remaining_route_length_m\":" << remaining_route_m
      << ",\"auto_recovered\":" << (auto_recovered ? "true" : "false")
      << ",\"mode\":\"" << json_escape(mode) << "\""
      << ",\"target_waypoint_id\":\"" << json_escape(waypoint_id) << "\""
      << ",\"target_name\":\"" << json_escape(waypoint_name) << "\""
      << ",\"map_id\":\"" << json_escape(map_id) << "\""
      << ",\"map_uuid\":\"" << json_escape(map_uuid) << "\""
      << ",\"localized\":" << (localized ? "true" : "false")
      << ",\"current_waypoint_id\":\""
      << json_escape(localized ? localization->waypoint_id : std::string()) << "\""
      << ",\"has_current_seed_pose\":" << (has_current_seed_pose ? "true" : "false")
      << ",\"has_seed_goal\":" << (has_seed_goal ? "true" : "false")
      << ",\"has_waypoint_goal\":" << (has_waypoint_goal ? "true" : "false");
  if (has_current_seed_pose) {
    oss << ",\"current_seed_x\":" << current_seed_x
        << ",\"current_seed_y\":" << current_seed_y
        << ",\"current_seed_z\":" << current_seed_z
        << ",\"current_seed_yaw_rad\":" << current_seed_yaw_rad;
  }
  if (has_seed_goal) {
    oss << ",\"target_seed_x\":" << seed_x
        << ",\"target_seed_y\":" << seed_y
        << ",\"target_seed_yaw_rad\":" << seed_yaw_rad;
  }
  if (has_waypoint_goal) {
    oss << ",\"target_waypoint_goal_x\":" << waypoint_goal_x
        << ",\"target_waypoint_goal_y\":" << waypoint_goal_y
        << ",\"target_waypoint_goal_yaw_rad\":" << waypoint_goal_yaw_rad;
  }
  oss << "}";
  return oss.str();
}

void Adapter::PublishGraphNavNavState(long long now) {
  if (cfg_.graphnav_nav_state_stream.empty()) return;
  const long long kPeriodicMs = 1000;
  SpotClient::LocalizationSnapshot localization;
  SpotClient::LocalizationSnapshot* localization_ptr = nullptr;
  if (SpotConnected() && spot_.GetLocalizationSnapshot(&localization)) {
    localization_ptr = &localization;
  }
  const std::string payload = BuildGraphNavNavStateJson(localization_ptr);
  bool changed = false;
  {
    std::lock_guard<std::mutex> lk(nav_state_mu_);
    changed = (payload != last_nav_state_payload_);
    if (changed) last_nav_state_payload_ = payload;
  }
  if (!changed && (now - last_nav_state_pub_ms_.load()) < kPeriodicMs) return;
  QueueStatusText(cfg_.graphnav_nav_state_stream, payload);
  last_nav_state_pub_ms_ = now;
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
      ClearGraphNavNavTargetLocked();
      last_nav_feedback_signature_.clear();
      last_nav_status_ = 0;
      last_nav_remaining_route_m_ = 0.0;
      last_nav_progress_change_ms_ = 0;
    }

    if (!SaveAdapterMapState()) {
      EmitLog("[graphnav] map.create warning: failed to persist adapter map state");
    }
    ClearGraphNavMapArtifacts();
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
    }
    last_graph_nav_command_id_ = 0;
    graphnav_navigation_active_ = false;
    nav_auto_recovered_ = false;
    {
      std::lock_guard<std::mutex> lk(nav_state_mu_);
      ClearGraphNavNavTargetLocked();
      last_nav_feedback_signature_.clear();
      last_nav_status_ = 0;
      last_nav_remaining_route_m_ = 0.0;
      last_nav_progress_change_ms_ = 0;
    }
    if (!spot_.SetLocalizationFiducial()) {
      EmitLog(std::string("[graphnav] map.load localization warning: ") + spot_.LastError());
    }
    RefreshGraphWaypointIndex();
    (void)RefreshGraphNavMapArtifacts(map_id, map_data);
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
        ClearGraphNavNavTargetLocked();
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
        loaded_waypoint_id_to_label_.clear();
        loaded_waypoint_label_to_ids_.clear();
        loaded_waypoint_lower_label_to_ids_.clear();
      }
    }
    if (clear_loaded_graph) {
      std::lock_guard<std::mutex> lk(nav_state_mu_);
      ClearGraphNavNavTargetLocked();
    }
    if (!DeleteMapFromDisk(map_id)) {
      EmitLog(std::string("[graphnav] map.delete failed removing map files map_id=") + map_id);
      return false;
    }
    SaveAdapterMapState();
    if (clear_loaded_graph) ClearGraphNavMapArtifacts();
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
      EmitLog("[graphnav] stop_mapping failed saving map to disk");
      return false;
    }
    RefreshGraphWaypointIndex();
    (void)RefreshGraphNavMapArtifactsFromDisk(active_map);
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
    std::string active_map;
    {
      std::lock_guard<std::mutex> lk(map_mu_);
      active_map = active_map_id_;
    }
    RefreshGraphNavMetadataForMap(active_map);
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
    SpotClient::StoredMap live_map_data;
    const bool have_live_map_data = spot_.DownloadCurrentMap(&live_map_data);
    if (have_live_map_data) {
      if (!SaveMapToDisk(active_map, live_map_data)) {
        EmitLog("[graphnav] waypoint create warning: failed to save current map to disk");
      }
    } else if (!SaveCurrentMapToDisk(active_map)) {
      EmitLog("[graphnav] waypoint create warning: failed to save current map to disk");
    }
    RefreshGraphWaypointIndex();
    if (have_live_map_data) {
      (void)RefreshGraphNavMapArtifacts(active_map, live_map_data);
    } else {
      (void)RefreshGraphNavMapArtifactsFromDisk(active_map);
    }
    SaveAdapterMapState();
    PublishWaypointsText(true);
    PublishMapsText(true);
    EmitLog(std::string("[graphnav] waypoint alias set name=") + name + " id=" + waypoint_id);
    return true;
  }

  return false;
}

}  // namespace fsa
