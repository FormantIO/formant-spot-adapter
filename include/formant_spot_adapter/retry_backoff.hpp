#pragma once

#include <algorithm>

namespace fsa {

// Delay before the next attempt after `failures` consecutive failures:
// initial, 2x, 4x ... capped at max. 0 failures (or fewer) means "now".
inline long long backoff_delay_ms(int failures, long long initial_ms, long long max_ms) {
  if (failures <= 0) return 0;
  const long long cap = std::max(1LL, max_ms);
  long long delay = std::max(1LL, initial_ms);
  for (int i = 1; i < failures && delay < cap; ++i) delay *= 2;
  return std::min(delay, cap);
}

}  // namespace fsa
