#include "formant_spot_adapter/command_cancel.hpp"

#include <algorithm>

#include "formant_spot_adapter/command_result.hpp"

namespace fsa {

bool is_stop_command(const std::string& command) { return command == "spot.graphnav.cancel"; }

bool stoppable_command(const std::string& command) { return is_nav_command(command); }

QueueAdmission admit_command(const std::string& command, size_t queue_depth, size_t max_depth) {
  if (is_stop_command(command)) return QueueAdmission::kFront;
  return queue_depth >= max_depth ? QueueAdmission::kReject : QueueAdmission::kBack;
}

bool stopped_since_enqueue(const std::string& command, unsigned long long stop_seq_at_enqueue,
                           unsigned long long stop_seq_now) {
  return stoppable_command(command) && stop_seq_at_enqueue != stop_seq_now;
}

std::string stop_arrival_log(const std::string& command, const std::string& running) {
  if (running.empty()) {
    return "[command] " + command +
           " received with no queued command running; nothing to interrupt, it runs next";
  }
  if (!stoppable_command(running)) {
    return "[command] " + command + " received while " + running +
           " runs; that command is not a walk or dock, so " + command + " runs after it";
  }
  return "[command] " + command + " received: stopping the running " + running + " now";
}

WaitEnd cancellable_wait(long long timeout_ms, long long poll_ms, long long slice_ms,
                         const CancellableWaitHooks& hooks) {
  poll_ms = std::max(1LL, poll_ms);
  slice_ms = std::max(1LL, std::min(slice_ms, poll_ms));
  const auto is_running = [&]() { return !hooks.running || hooks.running(); };
  const auto is_cancelled = [&]() { return hooks.cancelled && hooks.cancelled(); };
  const long long start = hooks.now_ms();
  for (;;) {
    if (!is_running()) return WaitEnd::kStopped;
    if (is_cancelled()) return WaitEnd::kCancelled;
    if (hooks.now_ms() - start > timeout_ms) return WaitEnd::kTimedOut;
    switch (hooks.poll()) {
      case PollStep::kSucceeded:
        return WaitEnd::kSucceeded;
      case PollStep::kFailed:
        return WaitEnd::kFailed;
      case PollStep::kContinue:
        break;
    }
    for (long long slept = 0; slept < poll_ms; slept += slice_ms) {
      hooks.sleep_ms(std::min(slice_ms, poll_ms - slept));
      if (!is_running()) return WaitEnd::kStopped;
      if (is_cancelled()) return WaitEnd::kCancelled;
    }
  }
}

}  // namespace fsa
