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

#include "formant_spot_adapter/command_params.hpp"

namespace fsa {

namespace {

constexpr double kPi = 3.14159265358979323846;

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
  oss << "updated_ms=" << rec.updated_ms << "\n";
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
  *out = rec;
  return true;
}

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
  std::filesystem::rename(tmp, target, ec);
  if (ec) {
    std::filesystem::remove(tmp, ec);
    return fail("rename to " + path + " failed");
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

LastLocalizationThrottle::LastLocalizationThrottle(long long min_interval_ms, double min_move_m,
                                                   double min_turn_rad)
    : min_interval_ms_(min_interval_ms), min_move_m_(min_move_m), min_turn_rad_(min_turn_rad) {}

bool LastLocalizationThrottle::ShouldPersist(const LastLocalization& current,
                                             long long now_ms) const {
  if (current.map_id.empty() || current.waypoint_id.empty()) return false;
  if (!has_last_) return true;
  if (last_write_ms_ != 0 && (now_ms - last_write_ms_) < min_interval_ms_) return false;
  if (current.map_id != last_.map_id || current.waypoint_id != last_.waypoint_id) return true;
  if (current.has_waypoint_tform_body != last_.has_waypoint_tform_body) return true;
  if (!current.has_waypoint_tform_body) return false;
  const double dx = current.x - last_.x;
  const double dy = current.y - last_.y;
  const double dz = current.z - last_.z;
  if (std::sqrt(dx * dx + dy * dy + dz * dz) >= min_move_m_) return true;
  return angle_diff(yaw_of(current), yaw_of(last_)) >= min_turn_rad_;
}

void LastLocalizationThrottle::MarkPersisted(const LastLocalization& rec, long long now_ms) {
  has_last_ = true;
  last_ = rec;
  last_write_ms_ = now_ms;
}

void LastLocalizationThrottle::Seed(const LastLocalization& rec) {
  has_last_ = true;
  last_ = rec;
  last_write_ms_ = 0;
}

bool IsNoUsableFiducialError(const std::string& error_text) {
  const std::string upper = to_upper(error_text);
  return upper.find("NO_MATCHING_FIDUCIAL") != std::string::npos ||
         upper.find("FIDUCIAL_TOO_FAR_AWAY") != std::string::npos ||
         upper.find("FIDUCIAL_TOO_OLD") != std::string::npos ||
         upper.find("FIDUCIAL_POSE_UNCERTAIN") != std::string::npos;
}

WaypointRecoveryPlan DecideWaypointRecovery(const WaypointRecoveryInputs& in) {
  WaypointRecoveryPlan plan;
  if (in.fiducial_localization_ok) {
    plan.reason = "fiducial localization succeeded";
    return plan;
  }
  if (!IsNoUsableFiducialError(in.fiducial_error)) {
    plan.reason = "fiducial localization failed for a reason a waypoint guess does not fix: " +
                  in.fiducial_error;
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
  plan.reason = "no usable fiducial; trying saved waypoint " + in.last.waypoint_id;
  return plan;
}

LocalizeAcceptance EvaluateWaypointLocalization(const std::string& status,
                                                const std::string& quality_check,
                                                bool suspected_ambiguity) {
  LocalizeAcceptance a;
  if (status != "STATUS_OK") {
    a.reason = "GraphNav status " + (status.empty() ? std::string("unknown") : status);
    return a;
  }
  a.applied = true;
  if (quality_check != "QUALITY_CHECK_SUCCESS") {
    a.reason = "GraphNav quality check " +
               (quality_check.empty() ? std::string("unknown") : quality_check);
    return a;
  }
  if (suspected_ambiguity) {
    a.reason = "GraphNav suspects an ambiguous localization (a similar-looking pose nearby)";
    return a;
  }
  a.ok = true;
  return a;
}

std::string LocalizationConfirmation::pending_reason() const {
  if (recovery_in_progress_) return "restart localization recovery is in progress";
  switch (state_) {
    case State::kConfirmed:
      return "";
    case State::kRecovering:
      return "restart localization recovery is in progress";
    case State::kAutomatic:
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

void LocalizationConfirmation::OnRecoveryFinished() {
  recovery_in_progress_ = false;
  if (state_ == State::kRecovering) state_ = State::kConfirmed;
}

void LocalizationConfirmation::OnAutomaticLocalization(const std::string& waypoint_id) {
  state_ = State::kAutomatic;
  waypoint_id_ = waypoint_id;
  rejection_.clear();
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
    return refuse("restart localization recovery is in progress; wait for it to finish");
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
    return action + " refused: restart localization recovery is in progress; try again shortly.";
  }
  return action + " refused: " + pending_reason() +
         ". Check the robot's position on the map, then run spot.localize.confirm, or run "
         "spot.localize at the right waypoint, or let it see the dock fiducial.";
}

std::string LocalizationConfirmation::LocalizeCommandBlockReason() const {
  if (recovery_in_progress_ || state_ == State::kRecovering) {
    return "restart localization recovery is in progress; try again shortly";
  }
  return "";
}

}  // namespace fsa
