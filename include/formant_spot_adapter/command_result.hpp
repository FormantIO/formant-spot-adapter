#pragma once

#include <string>

namespace fsa {

// What the navigation state says about the last nav command when it ended.
struct NavOutcome {
  std::string result;  // "reached", "held_position", "failed", "interrupted", or ""
  std::string reason;  // prose, e.g. "Navigation ended with STATUS_STUCK."
  std::string target;  // the waypoint name, when the command named one
  // True only when this command's own navigation reached a terminal state.
  // A command that failed before navigating (a lease, a stale map, an
  // unknown name, a dock step) must not report the previous command's reason.
  bool fresh = false;
};

inline bool is_nav_command(const std::string& command) {
  return command == "spot.waypoint.goto" || command == "spot.waypoint.goto_straight" ||
         command == "spot.graphnav.goto_pose" || command == "spot.graphnav.goto_pose_straight" ||
         command == "spot.dock" || command == "spot.return_and_dock";
}

// A reason inside the grammar's parentheses: no nested parentheses, one line, at most 120 chars.
inline std::string grammar_reason(const std::string& reason) {
  std::string out;
  out.reserve(reason.size());
  for (char c : reason) {
    if (c == '(' || c == ')') continue;
    out.push_back(c == '\n' || c == '\r' ? ' ' : c);
  }
  while (!out.empty() && (out.back() == ' ' || out.back() == '.')) out.pop_back();
  if (out.size() > 120) out.resize(120);
  return out;
}

// How a command ended, in the mission response grammar cortex reads
// (apps/api/.../response-grammar.ts): ARRIVED: <target>, DONE: <command>,
// STOPPED(<reason>) when interrupted, ABORTED(<reason>) otherwise.
inline std::string command_result_text(const std::string& command, bool ok, const NavOutcome& nav) {
  if (!is_nav_command(command)) {
    return ok ? "DONE: " + command : "ABORTED(" + grammar_reason(command + " failed") + ")";
  }
  // A dock does not set a waypoint target; a name left there is the last goto's.
  const bool dock = command == "spot.dock" || command == "spot.return_and_dock";
  const std::string target =
      dock ? "dock" : (nav.fresh && !nav.target.empty() ? nav.target : command);
  if (ok) return "ARRIVED: " + target;
  // Navigation did not run, or reached its goal and a later step failed
  // (return_and_dock's dock after the nav leg): the command failed, not the nav.
  if (!nav.fresh || nav.result == "reached" || nav.result == "held_position") {
    return "ABORTED(" + grammar_reason(command + " failed") + ")";
  }
  const std::string why = grammar_reason(nav.reason.empty() ? "navigation failed" : nav.reason);
  return (nav.result == "interrupted" ? "STOPPED(" : "ABORTED(") + why + ")";
}

}  // namespace fsa
