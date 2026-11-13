#include "formant_spot_adapter/localize_recovery.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#endif

#include "formant_spot_adapter/command_params.hpp"

namespace fsa {

namespace {

constexpr double kPi = 3.14159265358979323846;
// About the difference between Spot sitting and standing.
constexpr double kMaxBodyHeightCorrectionM = 0.4;

std::string trim(const std::string& in) {
  size_t begin = 0;
  while (begin < in.size() && std::isspace(static_cast<unsigned char>(in[begin]))) ++begin;
  size_t end = in.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(in[end - 1]))) --end;
  return in.substr(begin, end - begin);
}

std::string to_upper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return s;
}

bool parse_double(const std::string& text, double* out) {
  const std::string t = trim(text);
  if (t.empty()) return false;
  errno = 0;
  char* end = nullptr;
  const double v = std::strtod(t.c_str(), &end);
  if (errno != 0 || end == t.c_str() || *end != '\0' || !std::isfinite(v)) return false;
  *out = v;
  return true;
}

bool parse_long_long(const std::string& text, long long* out) {
  const std::string t = trim(text);
  if (t.empty()) return false;
  errno = 0;
  char* end = nullptr;
  const long long v = std::strtoll(t.c_str(), &end, 10);
  if (errno != 0 || end == t.c_str() || *end != '\0') return false;
  *out = v;
  return true;
}

bool safe_token(const std::string& s) {
  return s.find('\n') == std::string::npos && s.find('\r') == std::string::npos;
}

double yaw_of(const LastLocalization& r) {
  return std::atan2(2.0 * (r.qw * r.qz + r.qx * r.qy), 1.0 - 2.0 * (r.qy * r.qy + r.qz * r.qz));
}

double angle_diff(double a, double b) {
  double d = std::fmod(a - b, 2.0 * kPi);
  if (d > kPi) d -= 2.0 * kPi;
  if (d < -kPi) d += 2.0 * kPi;
  return std::fabs(d);
}

}  // namespace

long long SecondsToMsClamped(double seconds) {
  if (!std::isfinite(seconds) || seconds <= 0.0) return 0;
  constexpr double kMaxMs = 100.0 * 24.0 * 60.0 * 60.0 * 1000.0;  // 100 days
  const double ms = seconds * 1000.0;
  return static_cast<long long>(ms > kMaxMs ? kMaxMs : ms);
}

LocalizeCommandParams ParseLocalizeCommandParams(const std::string& text) {
  LocalizeCommandParams p;
  p.waypoint_id = trim(ExtractCommandParam(text, {"waypoint_id"}));
  if (p.waypoint_id.empty()) {
    p.waypoint_name =
        trim(ExtractCommandParamOrBareValue(text, {"name", "waypoint", "waypoint_name", "alias"}));
  }
  p.map_uuid = trim(ExtractCommandParam(text, {"map_uuid"}));

  const std::string distance_text = ExtractCommandParam(text, {"max_distance", "max_distance_m"});
  if (!distance_text.empty()) {
    double v = 0.0;
    if (!parse_double(distance_text, &v) || v <= 0.0 || v > kMaxLocalizeDistanceM) {
      std::ostringstream oss;
      oss << "max_distance must be a number of meters in (0, " << kMaxLocalizeDistanceM
          << "], got '" << distance_text << "'";
      p.error = oss.str();
      return p;
    }
    p.limits.max_distance_m = v;
  }
  const std::string yaw_text = ExtractCommandParam(text, {"max_yaw_deg"});
  if (!yaw_text.empty()) {
    double v = 0.0;
    if (!parse_double(yaw_text, &v) || v <= 0.0 || v > kMaxLocalizeYawDeg) {
      std::ostringstream oss;
      oss << "max_yaw_deg must be a number of degrees in (0, " << kMaxLocalizeYawDeg << "], got '"
          << yaw_text << "'";
      p.error = oss.str();
      return p;
    }
    p.limits.max_yaw_rad = v * kPi / 180.0;
  }

  const std::string force_text = trim(ExtractCommandParam(text, {"force"}));
  if (!force_text.empty()) {
    if (force_text == "true" || force_text == "1") {
      p.force = true;
    } else if (force_text != "false" && force_text != "0") {
      p.error = "force must be true or false, got '" + force_text + "'";
      return p;
    }
  }

  if (p.waypoint_id.empty() && p.waypoint_name.empty()) {
    p.error = "missing waypoint: pass a waypoint name or waypoint_id=<id>";
  }
  return p;
}

std::string SerializeLastLocalization(const LastLocalization& rec) {
  std::ostringstream oss;
  oss.precision(9);
  oss << "map_id=" << rec.map_id << "\n";
  oss << "waypoint_id=" << rec.waypoint_id << "\n";
  if (rec.has_waypoint_tform_body) {
    oss << "waypoint_tform_body=" << rec.x << "," << rec.y << "," << rec.z << "," << rec.qx << ","
        << rec.qy << "," << rec.qz << "," << rec.qw << "\n";
  }
  if (rec.has_body_height) oss << "body_height_m=" << rec.body_height_m << "\n";
  oss << "updated_ms=" << rec.updated_ms << "\n";
  if (rec.moved_ms > 0) oss << "moved_ms=" << rec.moved_ms << "\n";
  return oss.str();
}

bool ParseLastLocalization(const std::string& text, LastLocalization* out) {
  if (!out) return false;
  LastLocalization rec;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    const std::string t = trim(line);
    const size_t eq = t.find('=');
    if (t.empty() || eq == std::string::npos) continue;
    const std::string key = trim(t.substr(0, eq));
    const std::string value = trim(t.substr(eq + 1));
    if (key == "map_id") {
      rec.map_id = value;
    } else if (key == "waypoint_id") {
      rec.waypoint_id = value;
    } else if (key == "updated_ms") {
      long long v = 0;
      if (parse_long_long(value, &v)) rec.updated_ms = v;
    } else if (key == "moved_ms") {
      long long v = 0;
      if (parse_long_long(value, &v)) rec.moved_ms = v;
    } else if (key == "body_height_m") {
      double v = 0.0;
      if (parse_double(value, &v)) {
        rec.has_body_height = true;
        rec.body_height_m = v;
      }
    } else if (key == "waypoint_tform_body") {
      double v[7];
      std::stringstream ss(value);
      std::string part;
      int n = 0;
      bool ok = true;
      while (std::getline(ss, part, ',')) {
        if (n >= 7 || !parse_double(part, &v[n])) {
          ok = false;
          break;
        }
        ++n;
      }
      const double qnorm = ok && n == 7
                               ? std::sqrt(v[3] * v[3] + v[4] * v[4] + v[5] * v[5] + v[6] * v[6])
                               : 0.0;
      if (ok && n == 7 && qnorm > 1e-6) {
        rec.has_waypoint_tform_body = true;
        rec.x = v[0];
        rec.y = v[1];
        rec.z = v[2];
        rec.qx = v[3] / qnorm;
        rec.qy = v[4] / qnorm;
        rec.qz = v[5] / qnorm;
        rec.qw = v[6] / qnorm;
      }
    }
  }
  if (rec.map_id.empty() || rec.waypoint_id.empty()) return false;
  if (rec.moved_ms <= 0) rec.moved_ms = rec.updated_ms;
  *out = rec;
  return true;
}

namespace {

/** fsync a file or directory by path. False when it could not be done. */
bool FsyncPath(const std::string& path) {
#ifdef _WIN32
  (void)path;
  return false;
#else
  const int fd = ::open(path.c_str(), O_RDONLY);
  if (fd < 0) return false;
  const bool ok = ::fsync(fd) == 0;
  ::close(fd);
  return ok;
#endif
}

}  // namespace

bool SaveLastLocalization(const std::string& path, const LastLocalization& rec,
                          std::string* error) {
  auto fail = [&](const std::string& msg) {
    if (error) *error = msg;
    return false;
  };
  if (rec.map_id.empty() || rec.waypoint_id.empty()) return fail("map_id and waypoint_id required");
  if (!safe_token(rec.map_id) || !safe_token(rec.waypoint_id)) return fail("id contains a newline");
  std::error_code ec;
  const std::filesystem::path target(path);
  if (target.has_parent_path()) {
    std::filesystem::create_directories(target.parent_path(), ec);
    if (ec) return fail("create_directories: " + ec.message());
  }
  const std::string tmp = path + ".tmp";
  {
    std::ofstream out(tmp, std::ios::trunc);
    if (!out) return fail("cannot open " + tmp);
    out << SerializeLastLocalization(rec);
    out.flush();
    if (!out) return fail("write failed: " + tmp);
  }
  // The whole point of this record is to survive a reboot or a power cut, and
  // flush() only reaches the OS. Without fsync the file can come back empty or
  // stale after an unclean shutdown: it then parses as "no saved
  // localization", which fails safe but silently loses the recovery this
  // feature exists for. Best effort: a platform without fsync still gets the
  // atomic rename.
  if (!FsyncPath(tmp)) {
    // Not fatal: the data is written, just not known to be durable.
  }
  std::filesystem::rename(tmp, target, ec);
  if (ec) {
    std::filesystem::remove(tmp, ec);
    return fail("rename to " + path + " failed");
  }
  // And the directory entry, so the rename itself survives too.
  if (target.has_parent_path()) {
    (void)FsyncPath(target.parent_path().string());
  }
  return true;
}

bool LoadLastLocalization(const std::string& path, LastLocalization* out) {
  std::ifstream in(path);
  if (!in) return false;
  std::stringstream buf;
  buf << in.rdbuf();
  return ParseLastLocalization(buf.str(), out);
}

bool BodyHeightUsable(const std::string& behavior_state) {
  return behavior_state == "not_ready" || behavior_state == "standing";
}

namespace {
std::string moved_text(const char* frame, const LocalizePose& before, const LocalizePose& now,
                       double max_m, double max_yaw_rad);
}  // namespace

std::string BodyMovedReason(const LocalizePose& before, const LocalizePose& now, double max_m,
                            double max_yaw_rad) {
  return moved_text("odometry", before, now, max_m, max_yaw_rad);
}

std::string BodyMovedInEitherFrame(const LocalizePose& odom_before, const LocalizePose& odom_now,
                                   bool has_vision_before, const LocalizePose& vision_before,
                                   bool has_vision_now, const LocalizePose& vision_now,
                                   bool* vision_compared) {
  const bool compare_vision = has_vision_before && has_vision_now;
  if (vision_compared) *vision_compared = compare_vision;
  const std::string odom = BodyMovedReason(odom_before, odom_now);
  if (!odom.empty()) return odom;
  if (!compare_vision) return "";
  return moved_text("the vision frame", vision_before, vision_now, 0.1, 0.08726646259971647);
}

namespace {
std::string moved_text(const char* frame, const LocalizePose& before, const LocalizePose& now,
                       double max_m, double max_yaw_rad) {
  double xy = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  PoseCorrection(before, now, &xy, &z, &yaw);
  if (xy <= max_m && yaw <= max_yaw_rad) return "";
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(2);
  oss << frame << " shows the body moved " << xy << " m";
  oss.precision(1);
  oss << " / " << yaw * 180.0 / kPi << " deg";
  return oss.str();
}
}  // namespace

LastLocalizationThrottle::LastLocalizationThrottle(long long min_interval_ms, double min_move_m,
                                                   double min_turn_rad,
                                                   long long refresh_interval_ms)
    : min_interval_ms_(min_interval_ms),
      min_move_m_(min_move_m),
      min_turn_rad_(min_turn_rad),
      refresh_interval_ms_(refresh_interval_ms) {}

void LastLocalizationThrottle::ObserveOdom(long long now_ms, bool has_odom,
                                           const LocalizePose& odom_tform_body, bool has_vision,
                                           const LocalizePose& vision_tform_body) {
  if (!has_odom) {
    has_odom_ = false;
    has_vision_ = false;
    still_since_ms_ = 0;
    return;
  }
  if (!has_odom_ || still_since_ms_ == 0 ||
      !BodyMovedInEitherFrame(still_anchor_, odom_tform_body, still_anchor_has_vision_,
                              still_anchor_vision_, has_vision, vision_tform_body)
           .empty()) {
    still_anchor_ = odom_tform_body;
    still_anchor_has_vision_ = has_vision;
    still_anchor_vision_ = vision_tform_body;
    still_since_ms_ = now_ms;
  } else if (has_vision && !still_anchor_has_vision_) {
    // Vision just became available: start comparing from here.
    still_anchor_has_vision_ = true;
    still_anchor_vision_ = vision_tform_body;
  }
  has_odom_ = true;
  odom_ = odom_tform_body;
  has_vision_ = has_vision;
  vision_ = vision_tform_body;
}

LastLocalizationThrottle::Reason LastLocalizationThrottle::Decide(const LastLocalization& current,
                                                                  long long now_ms) const {
  if (current.map_id.empty() || current.waypoint_id.empty()) return Reason::kNone;
  if (!has_last_) return Reason::kChanged;
  if (last_write_ms_ != 0 && now_ms >= last_write_ms_ &&
      (now_ms - last_write_ms_) < min_interval_ms_) {
    return Reason::kNone;
  }
  // A saved stamp ahead of the clock (the clock was set back) would be
  // rejected as "from the future" after the next reboot: rewrite it.
  if (now_ms < last_stamp_ms_) return Reason::kChanged;
  if (current.map_id != last_.map_id || current.waypoint_id != last_.waypoint_id) {
    return Reason::kChanged;
  }
  if (current.has_waypoint_tform_body != last_.has_waypoint_tform_body) return Reason::kChanged;
  if (current.has_waypoint_tform_body) {
    const double dx = current.x - last_.x;
    const double dy = current.y - last_.y;
    const double dz = current.z - last_.z;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) >= min_move_m_) return Reason::kChanged;
    if (angle_diff(yaw_of(current), yaw_of(last_)) >= min_turn_rad_) return Reason::kChanged;
  }
  // Odometry moved since the last write, even if GraphNav's pose barely did.
  if (has_odom_ && has_odom_at_write_ &&
      !BodyMovedInEitherFrame(odom_at_write_, odom_, has_vision_at_write_, vision_at_write_,
                              has_vision_, vision_)
           .empty()) {
    return Reason::kChanged;
  }
  // Still here: refresh the timestamp, but only on odometry's word that the
  // body has not moved since the last write (or for a whole refresh interval).
  if (refresh_interval_ms_ > 0 && (now_ms - last_stamp_ms_) >= refresh_interval_ms_ &&
      has_odom_ && still_since_ms_ > 0) {
    const bool still_since_write = last_write_ms_ != 0 && still_since_ms_ <= last_write_ms_;
    const bool still_long_enough = (now_ms - still_since_ms_) >= refresh_interval_ms_;
    if (still_since_write || still_long_enough) return Reason::kRefresh;
  }
  return Reason::kNone;
}

void LastLocalizationThrottle::MarkPersisted(const LastLocalization& rec, long long now_ms) {
  has_last_ = true;
  last_ = rec;
  last_write_ms_ = now_ms;
  last_stamp_ms_ = now_ms;
  has_odom_at_write_ = has_odom_;
  odom_at_write_ = odom_;
  has_vision_at_write_ = has_vision_;
  vision_at_write_ = vision_;
}

void LastLocalizationThrottle::Seed(const LastLocalization& rec) {
  has_last_ = true;
  last_ = rec;
  if (last_.moved_ms <= 0) last_.moved_ms = last_.updated_ms;
  last_write_ms_ = 0;
  last_stamp_ms_ = rec.updated_ms;
  has_odom_at_write_ = false;
  has_vision_at_write_ = false;
}

std::string FiducialFailureBlocksWaypointRecovery(const std::string& error_text) {
  const std::string upper = to_upper(error_text);
  if (upper.find("STATUS_NO_MAP_LOADED") != std::string::npos) {
    return "GraphNav has no map loaded";
  }
  if (upper.find("STATUS_INCOMPATIBLE_SENSORS") != std::string::npos) {
    return "the map was recorded with a different sensor setup than the robot has now";
  }
  return "";
}

WaypointRecoveryPlan DecideWaypointRecovery(const WaypointRecoveryInputs& in) {
  WaypointRecoveryPlan plan;
  if (in.fiducial_localization_ok) {
    plan.reason = "fiducial localization succeeded";
    return plan;
  }
  const std::string blocked = FiducialFailureBlocksWaypointRecovery(in.fiducial_error);
  if (!blocked.empty()) {
    plan.reason = "fiducial localization failed and a waypoint guess cannot help (" + blocked +
                  "): " + in.fiducial_error;
    return plan;
  }
  if (in.already_attempted) {
    plan.reason = "waypoint recovery already attempted for this restore";
    return plan;
  }
  if (!in.has_last_localization) {
    plan.reason = "no saved localization to start from";
    return plan;
  }
  if (in.last.map_id != in.restored_map_id) {
    plan.reason = "saved localization is for map " + in.last.map_id + ", restored map is " +
                  in.restored_map_id;
    return plan;
  }
  if (!in.last_waypoint_in_map) {
    plan.reason = "saved waypoint " + in.last.waypoint_id + " is not in the restored map";
    return plan;
  }
  if (in.max_record_age_ms <= 0) {
    plan.reason = "restart recovery is turned off (max record age is 0)";
    return plan;
  }
  if (in.last.updated_ms <= 0) {
    plan.reason = "saved localization has no timestamp";
    return plan;
  }
  // Allow a little clock skew, but a record from the future means the clock moved.
  if (in.last.updated_ms > in.now_ms + 60000LL) {
    plan.reason = "saved localization is timestamped in the future; clock changed?";
    return plan;
  }
  const long long age_ms = std::max(0LL, in.now_ms - in.last.updated_ms);
  if (age_ms > in.max_record_age_ms) {
    plan.reason = "saved localization is " + std::to_string(age_ms / 1000) +
                  " s old, over the " + std::to_string(in.max_record_age_ms / 1000) + " s limit";
    return plan;
  }
  if (in.max_restore_delay_ms > 0 && in.restore_delay_ms > in.max_restore_delay_ms) {
    plan.reason = "map restore ran " + std::to_string(in.restore_delay_ms / 1000) +
                  " s after it was scheduled, over the " +
                  std::to_string(in.max_restore_delay_ms / 1000) +
                  " s limit; the robot may have been moved";
    return plan;
  }
  plan.attempt = true;
  plan.reason = "fiducial localization failed (" +
                (in.fiducial_error.empty() ? std::string("no error text") : in.fiducial_error) +
                "); trying saved waypoint " + in.last.waypoint_id;
  return plan;
}

LocalizeAcceptance EvaluateWaypointLocalization(const LocalizeResponseFacts& facts,
                                                const LocalizeAcceptancePolicy& policy) {
  LocalizeAcceptance a;
  if (facts.status != "STATUS_OK") {
    a.reason = "GraphNav status " + (facts.status.empty() ? std::string("unknown") : facts.status);
    return a;
  }
  a.applied = true;
  if (facts.suspected_ambiguity) {
    a.reason = "GraphNav suspects an ambiguous localization (a similar-looking pose nearby)";
    return a;
  }
  if (facts.quality_check == "QUALITY_CHECK_SUCCESS") {
    a.ok = true;
    a.basis = "quality_check";
    return a;
  }
  if (facts.quality_check == "QUALITY_CHECK_SKIPPED") {
    if (facts.visual_verification_requested) {
      // The ONLY evidence available: the request asked for visual refinement
      // AND for its quality to be verified, and GraphNav answered STATUS_OK
      // rather than STATUS_VISUAL_ALIGNMENT_FAILED (checked above). The SDK's
      // SetLocalizationResponse carries no positive "the refinement ran"
      // field, only that failure status, so this is inference from a request
      // flag plus the absence of the failure, not a reported fact. Hence the
      // name `*_requested`, and hence the hardware checklist in the PR: a map
      // with no usable visual data must come back STATUS_VISUAL_ALIGNMENT_FAILED
      // rather than STATUS_OK with the refinement silently skipped. If a later
      // SDK adds such a field, read it here instead.
      a.ok = true;
      a.basis = "visual_verification";
      return a;
    }
    if (!policy.allow_skipped_with_small_correction) {
      a.reason = "GraphNav skipped its quality check and visual verification was not used";
      return a;
    }
    if (!facts.ambiguity_check_requested) {
      a.reason = "GraphNav skipped its quality check and no ambiguity check was requested";
      return a;
    }
    if (!facts.has_correction) {
      a.reason = "GraphNav skipped its quality check and localized to a different waypoint than "
                 "the guess";
      return a;
    }
    if (facts.correction_m > policy.max_correction_m ||
        facts.correction_yaw_rad > policy.max_correction_yaw_rad) {
      std::ostringstream oss;
      oss.precision(3);
      oss << "GraphNav skipped its quality check and moved the pose " << facts.correction_m
          << " m horizontally / " << facts.correction_yaw_rad * 180.0 / kPi << " deg from the guess (limit "
          << policy.max_correction_m << " m / " << policy.max_correction_yaw_rad * 180.0 / kPi
          << " deg)";
      a.reason = oss.str();
      return a;
    }
    a.ok = true;
    a.basis = policy.small_correction_basis;
    return a;
  }
  a.reason = "GraphNav quality check " +
             (facts.quality_check.empty() ? std::string("unknown") : facts.quality_check);
  return a;
}

void PoseCorrection(const LocalizePose& guess, const LocalizePose& result, double* out_xy_m,
                    double* out_z_m, double* out_yaw_rad) {
  const double dx = result.x - guess.x;
  const double dy = result.y - guess.y;
  if (out_xy_m) *out_xy_m = std::sqrt(dx * dx + dy * dy);
  if (out_z_m) *out_z_m = result.z - guess.z;
  LastLocalization g;
  g.qx = guess.qx;
  g.qy = guess.qy;
  g.qz = guess.qz;
  g.qw = guess.qw;
  LastLocalization r;
  r.qx = result.qx;
  r.qy = result.qy;
  r.qz = result.qz;
  r.qw = result.qw;
  if (out_yaw_rad) *out_yaw_rad = angle_diff(yaw_of(r), yaw_of(g));
}

LocalizePose ComposePoses(const LocalizePose& ab, const LocalizePose& bc) {
  // Rotate bc's position by ab's rotation: v' = v + 2w(q x v) + 2 q x (q x v).
  const double qx = ab.qx, qy = ab.qy, qz = ab.qz, qw = ab.qw;
  const double vx = bc.x, vy = bc.y, vz = bc.z;
  const double tx = 2.0 * (qy * vz - qz * vy);
  const double ty = 2.0 * (qz * vx - qx * vz);
  const double tz = 2.0 * (qx * vy - qy * vx);
  LocalizePose out;
  out.x = ab.x + vx + qw * tx + (qy * tz - qz * ty);
  out.y = ab.y + vy + qw * ty + (qz * tx - qx * tz);
  out.z = ab.z + vz + qw * tz + (qx * ty - qy * tx);
  // Hamilton product ab.q * bc.q.
  out.qw = qw * bc.qw - qx * bc.qx - qy * bc.qy - qz * bc.qz;
  out.qx = qw * bc.qx + qx * bc.qw + qy * bc.qz - qz * bc.qy;
  out.qy = qw * bc.qy - qx * bc.qz + qy * bc.qw + qz * bc.qx;
  out.qz = qw * bc.qz + qx * bc.qy - qy * bc.qx + qz * bc.qw;
  return out;
}

LocalizePose InvertPose(const LocalizePose& ab) {
  LocalizePose rot_only;
  rot_only.qx = -ab.qx;
  rot_only.qy = -ab.qy;
  rot_only.qz = -ab.qz;
  rot_only.qw = ab.qw;
  LocalizePose neg;
  neg.x = -ab.x;
  neg.y = -ab.y;
  neg.z = -ab.z;
  // b_tform_a = (R^-1, -R^-1 p): the conjugate rotation applied to -p.
  return ComposePoses(rot_only, neg);
}

std::vector<NeighborWaypointTform> NeighborTforms(const std::string& waypoint_id,
                                                  const std::vector<WaypointEdgeTform>& edges) {
  std::vector<NeighborWaypointTform> out;
  auto add = [&](const std::string& id, const LocalizePose& tform) {
    if (id.empty() || id == waypoint_id) return;
    for (const auto& n : out) {
      if (n.waypoint_id == id) return;  // parallel edges: keep the first
    }
    out.push_back({id, tform});
  };
  for (const auto& e : edges) {
    if (e.from_waypoint_id == waypoint_id) {
      add(e.to_waypoint_id, e.from_tform_to);
    } else if (e.to_waypoint_id == waypoint_id) {
      add(e.from_waypoint_id, InvertPose(e.from_tform_to));
    }
  }
  return out;
}

bool ResultInGuessFrame(const WaypointLocalizeSpec& spec, const std::string& result_waypoint_id,
                        const LocalizePose& result_waypoint_tform_body, LocalizePose* out) {
  if (!out || result_waypoint_id.empty()) return false;
  if (result_waypoint_id == spec.waypoint_id) {
    *out = result_waypoint_tform_body;
    return true;
  }
  for (const auto& n : spec.neighbors) {
    if (n.waypoint_id == result_waypoint_id) {
      *out = ComposePoses(n.guess_tform_waypoint, result_waypoint_tform_body);
      return true;
    }
  }
  return false;
}

std::string FormatLocalizeDetail(const LocalizeDetailFields& f) {
  std::ostringstream oss;
  oss << "status=" << (f.status.empty() ? "unknown" : f.status)
      << " quality=" << (f.quality_check.empty() ? "unknown" : f.quality_check)
      << " accepted_by=" << (f.accepted_by.empty() ? "none" : f.accepted_by)
      << " visual_verification=" << (f.visual_verification ? "true" : "false");
  if (f.has_correction) {
    oss.setf(std::ios::fixed);
    oss.precision(3);
    oss << " correction_m=" << f.correction_xy_m << " correction_z_m=" << f.correction_z_m;
    oss.precision(1);
    oss << " correction_deg=" << f.correction_yaw_rad * 180.0 / kPi;
  } else {
    oss << " correction_m=unknown correction_z_m=unknown correction_deg=unknown";
  }
  return oss.str();
}

WaypointLocalizeSpec OperatorLocalizeSpec(const std::string& waypoint_id,
                                          const WaypointLocalizeLimits& limits,
                                          bool verify_visual_refinement) {
  WaypointLocalizeSpec spec;
  spec.waypoint_id = waypoint_id;  // identity waypoint_tform_body: "at the waypoint"
  spec.max_distance_m = limits.max_distance_m;
  spec.max_yaw_rad = limits.max_yaw_rad;
  spec.refine_with_visual_features = true;
  spec.verify_visual_refinement = verify_visual_refinement;
  spec.do_ambiguity_check = true;
  spec.send_current_ko_tform_body = true;
  return spec;
}

bool OperatorAcceptNeedsConfirmation(const std::string& accepted_by, bool visual_verification) {
  return accepted_by == "operator_small_correction" && !visual_verification;
}

LocalizeAcceptancePolicy OperatorAcceptancePolicy() {
  LocalizeAcceptancePolicy policy;
  policy.allow_skipped_with_small_correction = true;
  return policy;
}

LocalizePose RecoverySeedPose(const LastLocalization& last, bool has_current_body_height,
                              double current_body_height_m) {
  LocalizePose pose;  // identity: "at the waypoint"
  if (!last.has_waypoint_tform_body) return pose;
  pose = {last.x, last.y, last.z, last.qx, last.qy, last.qz, last.qw};
  if (last.has_body_height && has_current_body_height && std::isfinite(current_body_height_m) &&
      std::isfinite(last.body_height_m)) {
    const double delta = current_body_height_m - last.body_height_m;
    pose.z += std::max(-kMaxBodyHeightCorrectionM, std::min(kMaxBodyHeightCorrectionM, delta));
  }
  return pose;
}

WaypointLocalizeSpec RecoveryLocalizeSpec(const LastLocalization& last,
                                          bool verify_visual_refinement,
                                          bool has_current_body_height,
                                          double current_body_height_m) {
  WaypointLocalizeSpec spec;
  spec.waypoint_id = last.waypoint_id;
  spec.waypoint_tform_body =
      RecoverySeedPose(last, has_current_body_height, current_body_height_m);
  const WaypointLocalizeLimits limits;
  spec.max_distance_m = limits.max_distance_m;
  spec.max_yaw_rad = limits.max_yaw_rad;
  spec.refine_with_visual_features = true;
  spec.verify_visual_refinement = verify_visual_refinement;
  spec.do_ambiguity_check = true;
  // The saved pose was taken before the restart; that odom frame is gone.
  spec.send_current_ko_tform_body = false;
  return spec;
}

LocalizeAcceptancePolicy RecoveryAcceptancePolicy(bool unverified_pass) {
  LocalizeAcceptancePolicy policy;
  policy.allow_skipped_with_small_correction = unverified_pass;
  policy.small_correction_basis = "saved_pose_unverified";
  return policy;
}

bool RecoveryNeedsUnverifiedPass(bool accepted, bool applied, const std::string& status) {
  return !accepted && !applied && status == "STATUS_VISUAL_ALIGNMENT_FAILED";
}

RecoveryRetryDecision DecideRecoveryRetry(bool applied, const std::string& status,
                                          int attempts_made, const RecoveryRetryPolicy& policy) {
  RecoveryRetryDecision d;
  if (applied) {
    d.reason = "GraphNav applied a result that was rejected; not retrying";
    return d;
  }
  const bool transient = status.empty() || status == "STATUS_UNKNOWN" ||
                         status == "STATUS_ROBOT_IMPAIRED" || status == "STATUS_ABORTED";
  if (!transient) {
    d.reason = "GraphNav answered " + status + "; the saved guess does not match, not retrying";
    return d;
  }
  const std::string what = status.empty() ? std::string("no answer from GraphNav") : status;
  if (attempts_made >= policy.max_attempts || policy.retry_interval_ms <= 0) {
    d.reason = what + " after " + std::to_string(attempts_made) + " attempts; giving up";
    return d;
  }
  d.retry = true;
  d.delay_ms = policy.retry_interval_ms;
  d.reason = what + "; retrying in " + std::to_string(policy.retry_interval_ms / 1000) +
             " s (attempt " + std::to_string(attempts_made + 1) + " of " +
             std::to_string(policy.max_attempts) + ")";
  return d;
}

bool IsMotionCommand(const std::string& command) {
  static const char* const kMotion[] = {
      "spot.stand",          "spot.sit",
      "spot.recover",        "spot.dock",
      "spot.undock",         "spot.return_and_dock",
      "spot.rotate_left",    "spot.rotate_right",
      "spot.waypoint.goto",  "spot.waypoint.goto_straight",
      "spot.graphnav.goto_pose", "spot.graphnav.goto_pose_straight",
  };
  for (const char* m : kMotion) {
    if (command == m) return true;
  }
  return false;
}

RecoveryRetryGateDecision CheckRecoveryRetry(const RecoveryRetryCheck& in) {
  RecoveryRetryGateDecision d;
  if (in.max_window_ms > 0 && in.now_ms - in.first_attempt_ms > in.max_window_ms) {
    d.reason = "retry window of " + std::to_string(in.max_window_ms / 1000) +
               " s since the first attempt has passed";
    return d;
  }
  if (in.motion_epoch_now != in.motion_epoch_at_first) {
    d.reason = "a walk, dock or teleop motion happened since the first attempt";
    return d;
  }
  if (!in.has_odom_at_first) {
    d.reason = "no odometry from the first attempt to check the robot has not moved";
    return d;
  }
  if (!in.has_odom_now) {
    d.gate = RecoveryRetryGate::kWait;
    d.reason = "odometry not available yet";
    return d;
  }
  const std::string moved =
      BodyMovedInEitherFrame(in.odom_at_first, in.odom_now, in.has_vision_at_first,
                             in.vision_at_first, in.has_vision_now, in.vision_now,
                             &d.vision_compared);
  if (!moved.empty()) {
    d.reason = moved + " since the first attempt";
    return d;
  }
  if (in.map_activity) {
    d.gate = RecoveryRetryGate::kWait;
    d.reason = "a map command, recording or map processing is active";
    return d;
  }
  d.gate = RecoveryRetryGate::kRun;
  return d;
}

std::string LocalizationConfirmation::pending_reason() const {
  if (recovery_in_progress_) return "a localization attempt (restart recovery or spot.localize) is in progress";
  switch (state_) {
    case State::kConfirmed:
      return "";
    case State::kRecovering:
      return "a localization attempt (restart recovery or spot.localize) is in progress";
    case State::kAutomatic:
      if (!unverified_note_.empty()) {
        return "localization at waypoint " + waypoint_id_ + " (" + unverified_note_ +
               ") is not confirmed";
      }
      return "localization at waypoint " + waypoint_id_ +
             " was set automatically after a restart and is not confirmed";
    case State::kRejected:
      return "the robot holds a localization" +
             (waypoint_id_.empty() ? std::string() : " at waypoint " + waypoint_id_) +
             " that was rejected (" + rejection_ + ")";
  }
  return "";
}

void LocalizationConfirmation::OnRecoveryStarted() {
  recovery_in_progress_ = true;
  state_ = State::kRecovering;
  waypoint_id_.clear();
  rejection_.clear();
}

bool LocalizationConfirmation::TryBeginOperatorAttempt(std::string* refusal) {
  const std::string blocked = LocalizeCommandBlockReason();
  if (!blocked.empty()) {
    if (refusal) *refusal = blocked;
    return false;
  }
  recovery_in_progress_ = true;
  if (refusal) refusal->clear();
  return true;
}

void LocalizationConfirmation::OnRecoveryFinished() {
  recovery_in_progress_ = false;
  if (state_ == State::kRecovering) state_ = State::kConfirmed;
}

void LocalizationConfirmation::OnAutomaticLocalization(const std::string& waypoint_id,
                                                       const std::string& note) {
  state_ = State::kAutomatic;
  waypoint_id_ = waypoint_id;
  rejection_.clear();
  unverified_note_ = note;
}

void LocalizationConfirmation::OnUnverifiedOperatorLocalization(const std::string& waypoint_id,
                                                                const std::string& why) {
  state_ = State::kAutomatic;
  waypoint_id_ = waypoint_id;
  rejection_.clear();
  unverified_note_ = why;
}

void LocalizationConfirmation::OnRejectedLocalization(const std::string& waypoint_id,
                                                      const std::string& reason) {
  state_ = State::kRejected;
  waypoint_id_ = waypoint_id;
  rejection_ = reason;
}

bool LocalizationConfirmation::TryOperatorConfirm(bool force, std::string* refusal) {
  auto refuse = [&](const std::string& why) {
    if (refusal) *refusal = why;
    return false;
  };
  if (recovery_in_progress_ || state_ == State::kRecovering) {
    return refuse(
        "a localization attempt (restart recovery or spot.localize) is in progress; wait for it "
        "to finish");
  }
  if (state_ == State::kRejected && !force) {
    return refuse(pending_reason() +
                  "; GraphNav's own checks failed, so confirming needs force=true (or run "
                  "spot.localize at the right waypoint)");
  }
  state_ = State::kConfirmed;
  waypoint_id_.clear();
  rejection_.clear();
  if (refusal) refusal->clear();
  return true;
}

void LocalizationConfirmation::OnOperatorLocalize() {
  state_ = State::kConfirmed;
  waypoint_id_.clear();
  rejection_.clear();
}

void LocalizationConfirmation::OnFiducialLocalization() {
  state_ = State::kConfirmed;
  waypoint_id_.clear();
  rejection_.clear();
}

void LocalizationConfirmation::OnLocalizationCleared() {
  // While recovery runs it stays "recovering" so OnRecoveryFinished can tell
  // that nothing is left to confirm.
  state_ = recovery_in_progress_ ? State::kRecovering : State::kConfirmed;
  waypoint_id_.clear();
  rejection_.clear();
}

std::string LocalizationConfirmation::NavigationBlockReason(const std::string& action) const {
  if (confirmed()) return "";
  if (recovery_in_progress_ || state_ == State::kRecovering) {
    return action + " refused: a localization attempt (restart recovery or spot.localize) is in "
                    "progress; try again shortly.";
  }
  return action + " refused: " + pending_reason() +
         ". Check the robot's position on the map, then run spot.localize.confirm, or run "
         "spot.localize at the right waypoint, or let it see the dock fiducial.";
}

std::string LocalizationConfirmation::LocalizeCommandBlockReason() const {
  if (recovery_in_progress_ || state_ == State::kRecovering) {
    return "a localization attempt (restart recovery or spot.localize) is in progress; try "
           "again shortly";
  }
  return "";
}

}  // namespace fsa
