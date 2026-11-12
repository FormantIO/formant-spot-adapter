#include "formant_spot_adapter/command_cancel.hpp"

#include <algorithm>

#include "formant_spot_adapter/command_result.hpp"

namespace fsa {

bool is_stop_command(const std::string& command) { return command == "spot.graphnav.cancel"; }

bool stoppable_command(const std::string& command) { return is_nav_command(command); }

bool cancel_may_interrupt(const std::string& running_command, bool in_dock_phase) {
  return stoppable_command(running_command) && !in_dock_phase;
}

QueueAdmission admit_command(const std::string& command, size_t queue_depth, size_t max_depth) {
  if (is_stop_command(command)) return QueueAdmission::kFront;
  return queue_depth >= max_depth ? QueueAdmission::kReject : QueueAdmission::kBack;
}

CancelDecision decide_cancel(const CancelInput& in) {
  CancelDecision d;
  const std::string prefix = "[command] spot.graphnav.cancel";

  if (in.target_id.empty()) {
    // An operator's cancel: stop the running walk or dock and every queued one.
    for (const auto& q : in.queued) {
      if (stoppable_command(q.command)) d.cancel_queued_ids.push_back(q.id);
    }
    const std::string queued_note =
        d.cancel_queued_ids.empty()
            ? std::string()
            : "; " + std::to_string(d.cancel_queued_ids.size()) +
                  " queued walk/dock command(s) will not start";
    if (in.running_command.empty()) {
      d.log = prefix +
              " received with no queued command running; nothing to interrupt, it runs next" +
              queued_note;
    } else if (cancel_may_interrupt(in.running_command, in.running_in_dock_phase)) {
      d.stop_running = true;
      d.log = prefix + " received: stopping the running " + in.running_command + " now" +
              queued_note;
      return d;
    } else if (stoppable_command(in.running_command)) {
      d.log = prefix + " received while " + in.running_command +
              " is docking onto the dock; docking is not interrupted" + queued_note;
    } else {
      d.log = prefix + " received while " + in.running_command +
              " runs; that command is not a walk or dock, so the cancel runs after it" +
              queued_note;
    }
    // Nothing running to flag: run the cancel from the queue (it holds
    // position if some other navigation is live, otherwise it is ignored).
    d.enqueue = true;
    return d;
  }

  // Targeted: only the named command, and answered at once.
  const std::string target = " target=" + in.target_id;
  if (!in.running_id.empty() && in.running_id == in.target_id) {
    if (cancel_may_interrupt(in.running_command, in.running_in_dock_phase)) {
      d.stop_running = true;
      d.log = prefix + target + ": stopping the running " + in.running_command + " now";
    } else if (stoppable_command(in.running_command)) {
      d.success = false;
      d.result_text = "REJECTED(docking in progress)";
      d.log = prefix + target + ": " + in.running_command +
              " is docking onto the dock; docking is not interrupted";
    } else {
      d.success = false;
      d.result_text = "REJECTED(target is not a walk or dock)";
      d.log = prefix + target + ": " + in.running_command + " is not a walk or dock; not stopped";
    }
    return d;
  }
  for (const auto& q : in.queued) {
    if (q.id != in.target_id) continue;
    if (stoppable_command(q.command)) {
      d.cancel_queued_ids.push_back(q.id);
      d.success = true;
      d.result_text = cancel_done_text();
      d.log = prefix + target + ": queued " + q.command + " will not start";
    } else {
      d.success = false;
      d.result_text = "REJECTED(target is not a walk or dock)";
      d.log = prefix + target + ": queued " + q.command + " is not a walk or dock; not cancelled";
    }
    return d;
  }
  // Not running and not queued: it already finished, or it has not arrived
  // yet. Remember it so it does not start if it arrives later.
  d.remember_target = true;
  d.success = false;
  d.result_text = cancel_ignored_target_text();
  d.log = prefix + target + " ignored: target not running (remembered for 10 min in case it arrives later)";
  return d;
}

void CancelledTargets::Expire(long long now_ms) {
  while (!entries_.empty() && now_ms - entries_.front().at_ms > ttl_ms_) entries_.pop_front();
}

void CancelledTargets::Add(const std::string& id, long long now_ms) {
  if (id.empty()) return;
  Expire(now_ms);
  entries_.push_back({id, now_ms});
  while (entries_.size() > max_size_) entries_.pop_front();
}

bool CancelledTargets::Take(const std::string& id, long long now_ms) {
  Expire(now_ms);
  for (auto it = entries_.begin(); it != entries_.end(); ++it) {
    if (it->id != id) continue;
    entries_.erase(it);
    return true;
  }
  return false;
}

StopConfirmation confirm_stop(int max_attempts, long long settle_ms, const StopConfirmHooks& hooks) {
  StopConfirmation result;
  for (int attempt = 1; attempt <= std::max(1, max_attempts); ++attempt) {
    result.attempts = attempt;
    std::string error;
    if (!hooks.stop(&error)) {
      result.reason = "stop command failed: " + (error.empty() ? std::string("unknown error") : error);
      hooks.sleep_ms(settle_ms);
      continue;
    }
    hooks.sleep_ms(settle_ms);
    const StopFeedback fb = hooks.feedback();
    if (!fb.read_ok) {
      result.reason = "navigation feedback unavailable";
      continue;
    }
    if (!fb.following) {
      result.confirmed = true;
      result.reason.clear();
      return result;
    }
    result.reason = "still following route";
  }
  return result;
}

std::string cancel_not_confirmed_text(const std::string& reason) {
  return "STOPPED not confirmed: " + reason;
}

CancelExecutionPlan plan_cancel_execution(bool navigation_active) {
  CancelExecutionPlan plan;
  if (!navigation_active) return plan;
  plan.act = true;
  plan.stamp_autonomy = true;
  plan.clear_release_request = true;
  return plan;
}

DockStart decide_dock_start(unsigned long long operator_actions_at_start,
                            unsigned long long operator_actions_now,
                            const std::string& motor_power_at_start,
                            const std::string& motor_power_now) {
  if (operator_actions_now != operator_actions_at_start) return DockStart::kStopOperatorAction;
  if (motor_power_at_start == "on" && motor_power_now != "on") return DockStart::kStopPoweredOff;
  return DockStart::kStart;
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
