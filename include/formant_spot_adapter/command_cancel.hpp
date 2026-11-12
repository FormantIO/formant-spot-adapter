#pragma once

#include <cstddef>
#include <functional>
#include <string>

namespace fsa {

// Commands run one at a time from a single queue, and a walk holds that queue
// until GraphNav reaches a terminal state (up to the command timeout plus
// 30 s). A spot.graphnav.cancel queued behind it would only run after the walk
// ended, so Formant could not stop the robot mid-walk. A stop command now
// takes effect on arrival: it bumps a stop counter that the running walk or
// dock (and an impaired hold) checks every 100 ms, it goes to the front of the
// queue, and it is accepted even when the queue is full (DEV-17754).

// The commands that stop what is running on arrival. Only spot.graphnav.cancel:
// a queued spot.sit sent after a goto by a driver that does not wait for the
// result must not cancel that goto. The teleop sit and estop buttons already
// act at once on their own thread.
bool is_stop_command(const std::string& command);

// Commands a stop ends when they are running or still queued from before it:
// the walk and dock commands.
bool stoppable_command(const std::string& command);

enum class QueueAdmission { kBack, kFront, kReject };
// Where a new command goes: a stop goes to the front and is never rejected;
// anything else goes to the back, or is rejected when the queue is full.
QueueAdmission admit_command(const std::string& command, size_t queue_depth, size_t max_depth);

// True if a stop arrived after this command was queued (its stop counter at
// enqueue differs from the current one) and the command is one a stop ends.
bool stopped_since_enqueue(const std::string& command, unsigned long long stop_seq_at_enqueue,
                           unsigned long long stop_seq_now);

// The log line when a stop command arrives. running is the queued command
// running right now, or "" when none is.
std::string stop_arrival_log(const std::string& command, const std::string& running);

// The command-result line for a command a stop ended.
inline const char* stopped_by_cancel_result_text() { return "STOPPED(cancelled)"; }

enum class PollStep { kContinue, kSucceeded, kFailed };
enum class WaitEnd { kSucceeded, kFailed, kCancelled, kTimedOut, kStopped };

struct CancellableWaitHooks {
  std::function<long long()> now_ms;
  std::function<void(long long)> sleep_ms;
  std::function<PollStep()> poll;        // one poll of the thing waited on
  std::function<bool()> cancelled;       // a stop command arrived
  std::function<bool()> running;         // the adapter is still running
};

// Calls poll() every poll_ms until it returns kSucceeded or kFailed. Between
// polls it sleeps in slices of slice_ms and checks cancelled() and running()
// after every slice, so a stop is seen within one slice, not one poll. Returns
// kTimedOut once timeout_ms has passed since the start without a result.
WaitEnd cancellable_wait(long long timeout_ms, long long poll_ms, long long slice_ms,
                         const CancellableWaitHooks& hooks);

}  // namespace fsa
