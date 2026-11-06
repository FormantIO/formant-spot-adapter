#include <iostream>
#include <string>

#include "formant_spot_adapter/autonomy_lease_hold.hpp"

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
  if (condition) return;
  ++failures;
  std::cerr << "autonomy_lease_hold_test failed: " << message << std::endl;
}

fsa::AutonomyLeaseHoldInput input(long long now, long long last, int hold_s, bool release) {
  fsa::AutonomyLeaseHoldInput in;
  in.now_ms = now;
  in.last_autonomy_activity_ms = last;
  in.hold_s = hold_s;
  in.release_requested = release;
  return in;
}

}  // namespace

int main() {
  using fsa::ShouldReturnBodyLeaseAfterAutonomy;

  // Hold disabled: old behavior, return right away.
  check(ShouldReturnBodyLeaseAfterAutonomy(input(100000, 99000, 0, false)),
        "hold_s=0 returns immediately");
  check(ShouldReturnBodyLeaseAfterAutonomy(input(100000, 99000, -5, false)),
        "negative hold_s returns immediately");

  // Inside the hold window: keep the lease.
  check(!ShouldReturnBodyLeaseAfterAutonomy(input(100000, 99000, 180, false)),
        "1 s after autonomy with 180 s hold keeps lease");
  check(!ShouldReturnBodyLeaseAfterAutonomy(input(100000 + 179999, 100000, 180, false)),
        "just under the hold keeps lease");

  // Hold expired: return.
  check(ShouldReturnBodyLeaseAfterAutonomy(input(100000 + 180000, 100000, 180, false)),
        "exactly at the hold returns");
  check(ShouldReturnBodyLeaseAfterAutonomy(input(100000 + 500000, 100000, 180, false)),
        "well past the hold returns");

  // Explicit release (docked, sit) overrides the hold.
  check(ShouldReturnBodyLeaseAfterAutonomy(input(100000, 99999, 180, true)),
        "release request returns inside the hold");

  // No autonomy since the lease was taken (teleop-only use): unchanged, return.
  check(ShouldReturnBodyLeaseAfterAutonomy(input(100000, 0, 180, false)),
        "no autonomy activity returns immediately");

  // Large hold does not overflow.
  check(!ShouldReturnBodyLeaseAfterAutonomy(input(100000, 99000, 2000000000, false)),
        "large hold_s does not overflow");

  if (failures != 0) {
    std::cerr << failures << " check(s) failed" << std::endl;
    return 1;
  }
  std::cout << "autonomy_lease_hold_test passed" << std::endl;
  return 0;
}
