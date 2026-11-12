#pragma once

#include <functional>
#include <string>
#include <vector>

namespace fsa {

// Spot refuses to walk with ROBOT_IMPAIRED while a critical fault is active,
// for example a body camera that drops to USB2 and comes back a few seconds
// later (DEV-17754). Instead of failing the command at once, the adapter holds
// a walk or dock command that failed that way for a bounded window and retries
// it once when Spot's robot state shows the fault gone. The retry must never
// run against something a person or another client did in the meantime, so
// the hold ends without a retry on any sign of that.

// The commands that are held and retried: the walk and dock commands, the
// same set the command-result grammar treats as navigation.
bool impaired_retry_applies(const std::string& command);

// True for a queued command that means the operator or the mission has moved
// on (it moves or stops the robot, or changes its map, localization or arm),
// so a held command must not be retried.
bool supersedes_held_command(const std::string& command);

// True when a GraphNav error text is the ROBOT_IMPAIRED status: it names
// STATUS_ROBOT_IMPAIRED, or reads "status=5" with no further digit (the
// adapter's "NavigateTo failed status=5"), so "status=50" does not match.
bool error_text_says_robot_impaired(const std::string& error_text);

// The longest hold impaired_retry_window_sec may ask for.
constexpr int kMaxImpairedRetryWindowSec = 60;
// impaired_retry_window_sec clamped to 0..kMaxImpairedRetryWindowSec. Inline
// so config.cpp can use it without linking this module.
inline int clamp_impaired_retry_window_sec(int seconds) {
  return seconds < 0 ? 0 : (seconds > kMaxImpairedRetryWindowSec ? kMaxImpairedRetryWindowSec : seconds);
}

// What can change under a held command. Taken when the command starts (before
// its first attempt) and again on every poll of the hold.
struct HoldWatch {
  bool running{true};
  bool connected{true};
  // Bumped on every transition to connected: a reconnect between two polls
  // still shows up.
  unsigned long long connect_generation{0};
  // Bumped whenever the adapter loses or gives up the body lease (it was taken
  // by another client, a retain failed, the connection dropped).
  unsigned long long lease_epoch{0};
  // Bumped on every teleop button press and every teleop twist that asks for
  // motion, whether or not the adapter acted on it.
  unsigned long long operator_actions{0};
  // Bumped when a spot.graphnav.cancel arrives (see command_cancel.hpp). For
  // the command-start watch it is the value when the command was queued.
  unsigned long long stop_seq{0};
  std::vector<std::string> queued_commands;
};

// Why the hold must end without a retry, or "" to keep holding.
std::string hold_cancel_reason(const HoldWatch& at_command_start, const HoldWatch& now);

// One poll of Spot's robot state, as the hold sees it.
struct ImpairmentSample {
  bool read_ok{false};   // the robot state was read on this poll
  bool estopped{false};  // any E-Stop is asserted
  // "on", "off", "powering_on", "powering_off", "error" or "unknown".
  std::string motor_power;
  // A critical system fault, an unclearable behavior fault, or a motor power
  // error: the rule that sets the adapter's degraded state.
  bool degraded{false};
  // An unclearable behavior fault: it will not clear on its own.
  bool unclearable{false};
  // A fault that GraphNav named as the reason for the refusal is still listed
  // in the robot state (system, service or behavior fault). Catches causes the
  // degraded rule ignores, such as service faults.
  bool cause_present{false};
  // The fault to report, e.g. "camera_server: right camera failed to initialize".
  std::string fault;
};

enum class SampleVerdict {
  kClear,       // nothing stops the robot from walking on this read
  kWait,        // still impaired, unreadable, or powering on
  kPoweredOff,  // motors off or powering off: someone turned them off; do not retry
  kEstopped,    // an E-Stop is asserted; do not retry
  kUnclearable  // will not clear in the window; give up now
};
SampleVerdict classify_impairment_sample(const ImpairmentSample& sample);

// True if any of cause_ids is in present_ids.
bool any_cause_present(const std::vector<std::string>& cause_ids,
                       const std::vector<std::string>& present_ids);

// Consecutive clear reads needed before the retry, so a fault that flaps for
// one poll does not count as cleared.
constexpr int kClearReadsRequired = 2;

enum class ImpairedHoldEnd {
  kCleared,     // kClearReadsRequired clear reads in a row: retry once
  kNotCleared,  // the window ran out, or an unclearable fault was seen
  kCancelled,   // see hold_cancel_reason, plus E-Stop and motor power off
};

struct ImpairedHoldResult {
  ImpairedHoldEnd end{ImpairedHoldEnd::kNotCleared};
  // The last fault seen during the hold, or the failure's own text if none was.
  std::string fault;
  // Why the hold was cancelled (kCancelled only).
  std::string cancel_reason;
  // kNotCleared because an unclearable fault was seen, not because time ran out.
  bool unclearable{false};
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
// poll_ms, starting one poll_ms after the failure, and returns kCleared after
// kClearReadsRequired clear reads in a row. Any read that is not clear resets
// the count. Gives up with kNotCleared once window_ms has passed or on an
// unclearable fault, and returns kCancelled as soon as cancel_reason says so,
// an E-Stop is seen or the motors are off. cancel_reason is checked before
// every read and again right before returning kCleared. A window of 0 or less
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

// The held state for spot.nav.state, e.g. "held: robot impaired (camera_server:
// right camera failed to initialize), retrying once when clear, 14s left".
std::string impaired_hold_status_text(const std::string& fault, long long remaining_ms);

}  // namespace fsa
