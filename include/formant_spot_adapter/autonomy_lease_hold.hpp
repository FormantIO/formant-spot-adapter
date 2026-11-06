#pragma once

namespace fsa {

// Inputs for deciding whether to give the body lease back once teleop is
// inactive and no autonomy command (GraphNav, docking, mapping, queued
// command) is running.
struct AutonomyLeaseHoldInput {
  long long now_ms{0};
  // When autonomy was last seen running. 0 or less means no autonomy has run
  // since the lease was last taken or since teleop last used it.
  long long last_autonomy_activity_ms{0};
  // How long to keep the lease after the last autonomy activity. 0 or less
  // returns the lease immediately (the behavior before the hold existed).
  int hold_s{0};
  // Set when something finished that means the robot should be let go now:
  // a dock that ended with the robot docked, or an explicit sit.
  bool release_requested{false};
};

// True when the lease should be returned now, false to keep holding it.
bool ShouldReturnBodyLeaseAfterAutonomy(const AutonomyLeaseHoldInput& in);

}  // namespace fsa
