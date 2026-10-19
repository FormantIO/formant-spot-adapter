#include <iostream>
#include <string>
#include <unordered_map>

#include "formant_spot_adapter/dock_preflight.hpp"

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
  if (condition) return;
  ++failures;
  std::cerr << "dock_preflight_test failed: " << message << std::endl;
}

bool contains(const std::string& haystack, const std::string& needle) {
  return haystack.find(needle) != std::string::npos;
}

fsa::DockFiducialSighting seen(double age_sec, double x, double y, double z) {
  fsa::DockFiducialSighting s;
  s.found = true;
  s.has_age = true;
  s.age_sec = age_sec;
  s.has_position = true;
  s.body_x = x;
  s.body_y = y;
  s.body_z = z;
  return s;
}

}  // namespace

int main() {
  const fsa::DockPreflightLimits defaults;
  check(defaults.max_fiducial_age_sec == 5.0, "default max age is 5 s");
  check(defaults.max_distance_m == 5.0, "default max distance is 5 m");

  {
    // Walt's validated dock-home: about 1.5 m in front of the dock, rear toward it.
    const auto r = fsa::EvaluateDockPreflight(522, seen(0.4, -1.5, 0.05, -0.3), defaults);
    check(r.ok, "fresh tag 1.5 m behind the robot passes");
    check(r.reason.empty(), "no reason when ok");
    check(contains(r.detail, "distance_m=1.53"), "detail has distance: " + r.detail);
    check(contains(r.detail, "bearing_deg=178"), "detail has bearing behind the robot: " + r.detail);
    check(contains(r.detail, "age_s=0.4"), "detail has age: " + r.detail);
  }
  {
    fsa::DockFiducialSighting not_found;
    const auto r = fsa::EvaluateDockPreflight(522, not_found, defaults);
    check(!r.ok, "tag not listed is refused");
    check(contains(r.reason, "fiducial 522 is not in view"), "reason names the tag: " + r.reason);
  }
  {
    auto s = seen(0.0, 1.0, 0.0, 0.0);
    s.has_age = false;
    const auto r = fsa::EvaluateDockPreflight(522, s, defaults);
    check(!r.ok, "tag without a detection time is refused");
    check(contains(r.reason, "no detection time"), "reason explains missing time: " + r.reason);
  }
  {
    const auto r = fsa::EvaluateDockPreflight(522, seen(12.3, -1.5, 0.0, 0.0), defaults);
    check(!r.ok, "stale tag is refused");
    check(contains(r.reason, "last seen 12.3 s ago (limit 5.0 s)"), "reason has age and limit: " + r.reason);
  }
  {
    const auto r = fsa::EvaluateDockPreflight(522, seen(5.0, -1.5, 0.0, 0.0), defaults);
    check(r.ok, "tag exactly at the age limit passes");
  }
  {
    const auto r = fsa::EvaluateDockPreflight(522, seen(0.2, -6.0, 0.0, 0.0), defaults);
    check(!r.ok, "tag 6 m away is refused");
    check(contains(r.reason, "6.00 m away (limit 5.00 m)"), "reason has distance and limit: " + r.reason);
  }
  {
    fsa::DockPreflightLimits no_distance = defaults;
    no_distance.max_distance_m = 0.0;
    const auto r = fsa::EvaluateDockPreflight(522, seen(0.2, -9.0, 0.0, 0.0), no_distance);
    check(r.ok, "distance check off when the limit is 0");
  }
  {
    auto s = seen(0.2, 0.0, 0.0, 0.0);
    s.has_position = false;
    const auto r = fsa::EvaluateDockPreflight(522, s, defaults);
    check(r.ok, "fresh tag without a body pose still passes");
    check(!contains(r.detail, "distance_m"), "no distance in detail without a pose: " + r.detail);
  }
  {
    fsa::DockPreflightLimits off = defaults;
    off.max_fiducial_age_sec = 0.0;
    fsa::DockFiducialSighting not_found;
    const auto r = fsa::EvaluateDockPreflight(522, not_found, off);
    check(r.ok, "preflight off when the age limit is 0");
    check(contains(r.detail, "preflight=disabled"), "detail says disabled: " + r.detail);
  }
  {
    const auto r = fsa::EvaluateDockPreflight(522, seen(0.1, 2.0, 2.0, 0.0), defaults);
    check(contains(r.detail, "bearing_deg=45"), "positive bearing is to the left: " + r.detail);
  }
  {
    const auto r = fsa::EvaluateDockPreflight(7, fsa::DockFiducialSighting{}, defaults);
    check(contains(r.reason, "fiducial 7 "), "reason uses the configured dock id: " + r.reason);
  }

  // Alias sync.
  {
    std::unordered_map<std::string, std::string> aliases;
    const auto s = fsa::DecideDockAliasSync(aliases, "charging dock", "", "wp-new");
    check(s.write && s.key == "charging dock" && s.previous.empty(), "missing alias is created");
  }
  {
    std::unordered_map<std::string, std::string> aliases{{"charging dock", "wp-old"}, {"desk", "wp-d"}};
    const auto s = fsa::DecideDockAliasSync(aliases, "charging dock", "wp-old", "wp-new");
    check(s.write && s.key == "charging dock" && s.previous == "wp-old", "alias on old dock-home moves");
  }
  {
    std::unordered_map<std::string, std::string> aliases{{"Charging Dock", "wp-old"}};
    const auto s = fsa::DecideDockAliasSync(aliases, "charging dock", "wp-old", "wp-new");
    check(s.write && s.key == "Charging Dock", "existing capitalisation is reused, not duplicated");
  }
  {
    std::unordered_map<std::string, std::string> aliases{{"charging dock", "wp-new"}};
    const auto s = fsa::DecideDockAliasSync(aliases, "charging dock", "wp-old", "wp-new");
    check(!s.write && s.conflict.empty(), "alias already on the new dock-home is a no-op");
  }
  {
    std::unordered_map<std::string, std::string> aliases{{"charging dock", "wp-chosen"}};
    const auto s = fsa::DecideDockAliasSync(aliases, "charging dock", "wp-old", "wp-new");
    check(!s.write, "alias pointed elsewhere on purpose is not overwritten");
    check(contains(s.conflict, "points at wp-chosen"), "conflict is explained: " + s.conflict);
  }
  {
    std::unordered_map<std::string, std::string> aliases{{"charging dock", "wp-chosen"}};
    const auto s = fsa::DecideDockAliasSync(aliases, "charging dock", "", "wp-new");
    check(!s.write, "with no previous dock-home an existing alias is left alone");
  }
  {
    std::unordered_map<std::string, std::string> aliases;
    const auto s = fsa::DecideDockAliasSync(aliases, "", "wp-old", "wp-new");
    check(!s.write && s.conflict.empty(), "empty alias name turns the sync off");
  }
  {
    auto sight = seen(0.2, 0.0, 0.0, 0.0);
    sight.has_position = false;
    const auto r = fsa::EvaluateDockPreflight(522, sight, defaults);
    check(contains(r.detail, "distance=unknown"), "unknown distance is visible in the log: " + r.detail);
  }
  check(defaults.wait_sec == 4.0, "default wait is 4 s");

  if (failures == 0) std::cout << "dock_preflight_test: all checks passed" << std::endl;
  return failures == 0 ? 0 : 1;
}
