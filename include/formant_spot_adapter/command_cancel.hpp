#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace fsa {

// Commands run one at a time from a single queue, and a walk holds that queue
// until GraphNav reaches a terminal state (up to the command timeout plus
// 30 s). A spot.graphnav.cancel queued behind it would only run after the walk
// ended, so Formant could not stop the robot mid-walk. A cancel is now decided
// on arrival (decide_cancel): it flags the running walk or dock, which checks
// the flag every 100 ms, and marks matching queued commands so they never
// start (DEV-17754).
//
// A cancel may carry the Formant command id it is meant for, as parameter text
// "command_id=<uuid>". Then it stops only that command, running or queued, and
// does nothing if that command already finished or is unknown. A cancel with no
// command_id (an operator's) stops whatever walk or dock is running and every
// queued one.
//
// A robot that has started the final approach onto the dock (AutoDock) is
// never interrupted by a cancel; spot.return_and_dock can be stopped only while
// it walks to the dock waypoint. Teleop and the E-Stop remain the tools that
// stop everything.

// The commands that stop what is running on arrival. Only spot.graphnav.cancel:
// a queued spot.sit sent after a goto by a driver that does not wait for the
// result must not cancel that goto. The teleop sit and estop buttons already
// act at once on their own thread.
bool is_stop_command(const std::string& command);

// Commands a cancel can end: the walk and dock commands. spot.undock and
// spot.rotate_left/right are not among them.
bool stoppable_command(const std::string& command);

// True when a cancel may stop this running command now: it is a walk or dock
// and has not started docking onto the dock.
bool cancel_may_interrupt(const std::string& running_command, bool in_dock_phase);

enum class QueueAdmission { kBack, kFront, kReject };
// Where a new command goes: a stop goes to the front and is never rejected;
// anything else goes to the back, or is rejected when the queue is full.
QueueAdmission admit_command(const std::string& command, size_t queue_depth, size_t max_depth);

struct QueuedRef {
  std::string id;
  std::string command;
};

struct CancelInput {
  std::string target_id;   // the cancel's command_id parameter, "" when untargeted
  std::string running_id;  // the queued command running now, "" when none
  std::string running_command;
  bool running_in_dock_phase{false};
  std::vector<QueuedRef> queued;
};

struct CancelDecision {
  // Flag the running command so it stops itself.
  bool stop_running{false};
  // Queued commands to end without starting them.
  std::vector<std::string> cancel_queued_ids;
  // Queue the cancel command itself (untargeted: it also stops a navigation
  // that is not a queued command, such as the teleop return-and-dock). When
  // false, the cancel is answered at once with success and result_text.
  bool enqueue{false};
  bool success{true};
  std::string result_text;
  std::string log;
};

CancelDecision decide_cancel(const CancelInput& in);

// What the cancel command does when it runs from the queue, given whether a
// GraphNav navigation is still live. With nothing live it must not touch the
// body lease (another client may be driving), stamp autonomy activity (which
// would keep the lease for the autonomy hold) or clear a pending release.
struct CancelExecutionPlan {
  bool act{false};
  bool allow_lease_takeover{false};  // never: acquire only if nobody holds it
  bool stamp_autonomy{false};
  bool clear_release_request{false};
};
CancelExecutionPlan plan_cancel_execution(bool navigation_active);

// Whether a dock command may call AutoDock (which powers the motors on).
enum class DockStart { kStart, kStopOperatorAction, kStopPoweredOff };
// operator_actions_* are the operator-action counter at command start and now;
// motor_power_* are "on"/"off"/... when the dock command started and now. A
// robot whose motors were on and are now off was powered off by someone (the
// Formant estop button is a safe power off), so AutoDock must not power it
// back on. A robot that was off at the start is the dock's own job to power.
DockStart decide_dock_start(unsigned long long operator_actions_at_start,
                            unsigned long long operator_actions_now,
                            const std::string& motor_power_at_start,
                            const std::string& motor_power_now);

// The command-result line for a command a stop ended.
inline const char* stopped_by_cancel_result_text() { return "STOPPED(cancelled)"; }
// The command-result line for a dock command an operator action stopped.
inline const char* stopped_by_operator_result_text() { return "STOPPED(operator action)"; }

enum class PollStep { kContinue, kSucceeded, kFailed };
enum class WaitEnd { kSucceeded, kFailed, kCancelled, kTimedOut, kStopped };

struct CancellableWaitHooks {
  std::function<long long()> now_ms;
  std::function<void(long long)> sleep_ms;
  std::function<PollStep()> poll;   // one poll of the thing waited on
  std::function<bool()> cancelled;  // a cancel for this command arrived
  std::function<bool()> running;    // the adapter is still running
};

// Calls poll() every poll_ms until it returns kSucceeded or kFailed. Between
// polls it sleeps in slices of slice_ms and checks cancelled() and running()
// after every slice, so a stop is seen within one slice, not one poll. Returns
// kTimedOut once timeout_ms has passed since the start without a result.
WaitEnd cancellable_wait(long long timeout_ms, long long poll_ms, long long slice_ms,
                         const CancellableWaitHooks& hooks);

}  // namespace fsa
