#pragma once

#include <string>
#include <unordered_map>

namespace fsa {

// What the robot currently knows about the AprilTag on a dock. The tag id is the
// dock id (522 for Walt's dock).
struct DockFiducialSighting {
  // The tag is in the robot's world-object list at all.
  bool found{false};
  // Seconds between the tag's last detection and the world-object query, both on
  // the robot's clock.
  bool has_age{false};
  double age_sec{0.0};
  // Where the tag is relative to the robot, in the body frame (x forward, y left,
  // z up). Distance and bearing do not depend on the tag frame's own axes.
  bool has_position{false};
  double body_x{0.0};
  double body_y{0.0};
  double body_z{0.0};
};

struct DockPreflightLimits {
  // The tag must have been detected this recently. 0 or less turns the whole
  // preflight off.
  double max_fiducial_age_sec{5.0};
  // The tag must be at most this far away. 0 or less turns the distance check off.
  double max_distance_m{5.0};
  // How long to keep looking for a fresh sighting before refusing. After a turn
  // the tag can be briefly out of view; this gives the cameras another look.
  double wait_sec{4.0};
};

struct DockPreflightResult {
  bool ok{false};
  // Why the dock was refused, written for an operator. Empty when ok.
  std::string reason;
  // Measurements for the log, present whether or not the dock was refused.
  std::string detail;
};

// Decides whether to start an AutoDock. AutoDock needs the dock's tag in view;
// starting it without one is what produced DOCK_NOT_FOUND, STUCK and a fall on
// Walt. Refusing here fails fast with a reason instead.
DockPreflightResult EvaluateDockPreflight(int dock_id, const DockFiducialSighting& sighting,
                                          const DockPreflightLimits& limits);

// What to do with the dock-home alias (for example "charging dock") after a
// successful dock moves the dock-home from `previous_dock_waypoint` to
// `new_dock_waypoint`. The alias is only moved when it is missing or was already
// on the old dock-home, so a name someone pointed elsewhere on purpose is never
// overwritten. Matching is case-insensitive, like waypoint name lookup.
struct DockAliasSync {
  // Write `key` -> new_dock_waypoint.
  bool write{false};
  // The existing alias key (keeps the user's capitalisation) or the configured name.
  std::string key;
  // The waypoint the alias pointed at before, empty if it did not exist.
  std::string previous;
  // Set when the alias points somewhere else and is left alone.
  std::string conflict;
};

DockAliasSync DecideDockAliasSync(const std::unordered_map<std::string, std::string>& aliases,
                                  const std::string& alias, const std::string& previous_dock_waypoint,
                                  const std::string& new_dock_waypoint);

}  // namespace fsa
