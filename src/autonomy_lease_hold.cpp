#include "formant_spot_adapter/autonomy_lease_hold.hpp"

namespace fsa {

bool ShouldReturnBodyLeaseAfterAutonomy(const AutonomyLeaseHoldInput& in) {
  if (in.hold_s <= 0) return true;
  if (in.release_requested) return true;
  if (in.last_autonomy_activity_ms <= 0) return true;
  const long long hold_ms = static_cast<long long>(in.hold_s) * 1000LL;
  return (in.now_ms - in.last_autonomy_activity_ms) >= hold_ms;
}

}  // namespace fsa
