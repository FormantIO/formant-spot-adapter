#include "formant_spot_adapter/dock_preflight.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>

namespace fsa {
namespace {

std::string format_fixed(double value, int decimals) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", decimals, value);
  return buf;
}

}  // namespace

DockPreflightResult EvaluateDockPreflight(int dock_id, const DockFiducialSighting& sighting,
                                          const DockPreflightLimits& limits) {
  DockPreflightResult result;
  const std::string tag = "fiducial " + std::to_string(dock_id);

  if (sighting.found && sighting.has_age) {
    result.detail = "age_s=" + format_fixed(sighting.age_sec, 1);
  }
  double distance_m = -1.0;
  if (sighting.found && sighting.has_position) {
    distance_m = std::sqrt(sighting.body_x * sighting.body_x + sighting.body_y * sighting.body_y +
                           sighting.body_z * sighting.body_z);
    // 0 means straight ahead, 180 straight behind (a back-on approach), positive
    // to the robot's left.
    const double bearing_deg = std::atan2(sighting.body_y, sighting.body_x) * 180.0 / M_PI;
    if (!result.detail.empty()) result.detail += " ";
    result.detail += "distance_m=" + format_fixed(distance_m, 2) +
                     " bearing_deg=" + format_fixed(bearing_deg, 0) +
                     " body_xyz=" + format_fixed(sighting.body_x, 2) + "," +
                     format_fixed(sighting.body_y, 2) + "," + format_fixed(sighting.body_z, 2);
  } else if (sighting.found) {
    if (!result.detail.empty()) result.detail += " ";
    result.detail += "distance=unknown";
  }

  if (limits.max_fiducial_age_sec <= 0.0) {
    result.ok = true;
    if (!result.detail.empty()) result.detail += " ";
    result.detail += "preflight=disabled";
    return result;
  }

  if (!sighting.found) {
    result.reason = tag + " is not in view; move the robot so a camera can see the dock tag";
    return result;
  }
  if (!sighting.has_age) {
    result.reason = tag + " has no detection time, so it is not known to be in view";
    return result;
  }
  if (sighting.age_sec > limits.max_fiducial_age_sec) {
    result.reason = tag + " was last seen " + format_fixed(sighting.age_sec, 1) +
                    " s ago (limit " + format_fixed(limits.max_fiducial_age_sec, 1) +
                    " s); move the robot so a camera can see the dock tag";
    return result;
  }
  if (limits.max_distance_m > 0.0 && distance_m > limits.max_distance_m) {
    result.reason = tag + " is " + format_fixed(distance_m, 2) + " m away (limit " +
                    format_fixed(limits.max_distance_m, 2) + " m); move closer to the dock";
    return result;
  }

  result.ok = true;
  return result;
}

DockAliasSync DecideDockAliasSync(const std::unordered_map<std::string, std::string>& aliases,
                                  const std::string& alias, const std::string& previous_dock_waypoint,
                                  const std::string& new_dock_waypoint) {
  DockAliasSync sync;
  if (alias.empty() || new_dock_waypoint.empty()) return sync;

  auto lower = [](std::string s) {
    for (auto& ch : s) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    return s;
  };
  const std::string wanted = lower(alias);
  const std::string* existing_key = nullptr;
  const std::string* existing_target = nullptr;
  for (const auto& entry : aliases) {
    if (lower(entry.first) != wanted) continue;
    // Prefer an exact-case match when several keys differ only in case.
    if (!existing_key || entry.first == alias) {
      existing_key = &entry.first;
      existing_target = &entry.second;
    }
  }

  if (!existing_key) {
    sync.write = true;
    sync.key = alias;
    return sync;
  }
  sync.key = *existing_key;
  sync.previous = *existing_target;
  if (*existing_target == new_dock_waypoint) return sync;
  if (!previous_dock_waypoint.empty() && *existing_target == previous_dock_waypoint) {
    sync.write = true;
    return sync;
  }
  sync.conflict = "alias \"" + *existing_key + "\" points at " + *existing_target +
                  ", not the old dock waypoint; leaving it";
  return sync;
}

}  // namespace fsa
