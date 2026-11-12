#pragma once

#include <functional>
#include <string>

namespace fsa {

// Spot refuses to walk with ROBOT_IMPAIRED while a critical fault is active,
// for example a body camera that drops to USB2 and comes back a few seconds
// later (DEV-17754). Instead of failing the command at once, the adapter holds
// a walk or dock command that failed that way for a bounded window and retries
// it once as soon as Spot's robot state shows the fault cleared.

// The commands that are held and retried: the walk and dock commands, the
// same set the command-result grammar treats as navigation.
bool impaired_retry_applies(const std::string& command);

// True for a queued command that moves or stops the robot. One arriving while
// a command is held means the operator or the mission has moved on, so the
// held command is not retried.
bool supersedes_held_command(const std::string& command);

// One poll of Spot's robot state, as the hold sees it.
struct ImpairmentSample {
  bool read_ok{false};   // the robot state was read on this poll
  bool estopped{false};  // any E-Stop is asserted
  // A critical or unclearable fault, or a motor power error: the same rule
  // that sets the adapter's degraded state.
  bool degraded{false};
  // The fault behind it, e.g. "camera_server: right camera failed to initialize".
  std::string fault;
};

enum class ImpairedHoldEnd {
  kCleared,     // a robot state read after the failure shows no fault: retry once
  kNotCleared,  // the window ran out with the fault still active (or unreadable)
  kCancelled,   // a cancel or newer command, teleop, an E-Stop, a lost connection, shutdown
};

struct ImpairedHoldResult {
  ImpairedHoldEnd end{ImpairedHoldEnd::kNotCleared};
  // The last fault seen during the hold, or the failure's own text if none was.
  std::string fault;
  // Why the hold was cancelled (kCancelled only).
  std::string cancel_reason;
  long long waited_ms{0};
};

struct ImpairedHoldHooks {
  std::function<long long()> now_ms;
  std::function<void(long long)> sleep_ms;
  std::function<ImpairmentSample()> sample;
  // Non-empty when the hold must stop without retrying; the text says why.
  std::function<std::string()> cancel_reason;
};

// Holds a command that failed with ROBOT_IMPAIRED. Polls the robot state every
// poll_ms, starting one poll_ms after the failure so Spot's state has caught up
// with its refusal, and returns kCleared on the first read that shows no fault.
// Gives up with kNotCleared once window_ms has passed, and returns kCancelled
// as soon as cancel_reason says so or an E-Stop is seen. A window of 0 or less
// returns kNotCleared at once without polling.
ImpairedHoldResult hold_until_impairment_clears(long long window_ms,
                                                long long poll_ms,
                                                const std::string& failure_fault,
                                                const ImpairedHoldHooks& hooks);

// The command-result line (mission grammar) for a held command that is not
// retried: ABORTED when the fault did not clear, STOPPED when cancelled.
std::string impaired_hold_result_text(const ImpairedHoldResult& result, long long window_ms);

// The command-result line when the one retry was refused as impaired again.
std::string impaired_again_result_text(const std::string& fault);

}  // namespace fsa
