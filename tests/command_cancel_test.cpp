#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "formant_spot_adapter/command_cancel.hpp"
#include "formant_spot_adapter/impaired_retry.hpp"

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
  if (condition) return;
  ++failures;
  std::cerr << "command_cancel_test failed: " << message << std::endl;
}

void expect_eq(const std::string& got, const std::string& want, const std::string& message) {
  if (got == want) return;
  ++failures;
  std::cerr << "command_cancel_test failed: " << message << ": got \"" << got << "\" want \""
            << want << "\"" << std::endl;
}

bool has(const std::vector<std::string>& v, const std::string& s) {
  return std::find(v.begin(), v.end(), s) != v.end();
}

// A fake walk: GraphNav reports FOLLOWING_ROUTE until reaches_at_ms (0 = never).
struct FakeWalk {
  long long now{50000};
  long long reaches_at_ms{0};
  long long fails_at_ms{0};
  long long cancel_at_ms{0};  // a cancel for this walk arrives at this time (0 = never)
  long long stop_at_ms{0};    // the adapter stops at this time (0 = never)
  int polls{0};

  fsa::CancellableWaitHooks hooks() {
    fsa::CancellableWaitHooks h;
    h.now_ms = [this]() { return now; };
    h.sleep_ms = [this](long long ms) { now += ms; };
    h.poll = [this]() {
      ++polls;
      if (reaches_at_ms > 0 && now >= reaches_at_ms) return fsa::PollStep::kSucceeded;
      if (fails_at_ms > 0 && now >= fails_at_ms) return fsa::PollStep::kFailed;
      return fsa::PollStep::kContinue;
    };
    h.cancelled = [this]() { return cancel_at_ms > 0 && now >= cancel_at_ms; };
    h.running = [this]() { return !(stop_at_ms > 0 && now >= stop_at_ms); };
    return h;
  }
};

fsa::CancelInput walking(const std::string& target) {
  fsa::CancelInput in;
  in.target_id = target;
  in.running_id = "walk-1";
  in.running_command = "spot.waypoint.goto";
  in.queued = {{"walk-2", "spot.waypoint.goto"}, {"map-1", "spot.map.set_default"}};
  return in;
}

}  // namespace

int main() {
  using fsa::DockStart;
  using fsa::QueueAdmission;
  using fsa::WaitEnd;

  check(fsa::is_stop_command("spot.graphnav.cancel"), "cancel is a stop command");
  check(!fsa::is_stop_command("spot.sit"), "a queued sit does not cancel an earlier goto");
  for (const char* cmd : {"spot.waypoint.goto", "spot.waypoint.goto_straight", "spot.graphnav.goto_pose",
                          "spot.graphnav.goto_pose_straight", "spot.dock", "spot.return_and_dock"}) {
    check(fsa::stoppable_command(cmd), std::string(cmd) + " can be ended by a cancel");
  }
  for (const char* cmd : {"spot.undock", "spot.rotate_left", "spot.rotate_right", "spot.map.load"}) {
    check(!fsa::stoppable_command(cmd), std::string(cmd) + " is not stoppable by cancel");
  }

  // Never stop a robot that is docking onto the dock.
  check(fsa::cancel_may_interrupt("spot.return_and_dock", false), "return_and_dock walking: stoppable");
  check(!fsa::cancel_may_interrupt("spot.return_and_dock", true), "return_and_dock docking: not");
  check(!fsa::cancel_may_interrupt("spot.dock", true), "dock docking: not");
  check(fsa::cancel_may_interrupt("spot.dock", false), "dock before docking starts: stoppable");
  check(!fsa::cancel_may_interrupt("spot.undock", false), "undock: not");

  // A full queue still accepts a cancel, at the front.
  check(fsa::admit_command("spot.graphnav.cancel", 10, 10) == QueueAdmission::kFront,
        "full queue accepts a cancel");
  check(fsa::admit_command("spot.waypoint.goto", 10, 10) == QueueAdmission::kReject,
        "full queue rejects a goto");
  check(fsa::admit_command("spot.waypoint.goto", 3, 10) == QueueAdmission::kBack, "goto goes to the back");

  // Untargeted (operator) cancel during a walk: stop it and the queued walk.
  {
    const auto d = fsa::decide_cancel(walking(""));
    check(d.stop_running && !d.enqueue, "untargeted stops the running walk, answered after the stop");
    check(has(d.cancel_queued_ids, "walk-2") && !has(d.cancel_queued_ids, "map-1"),
          "untargeted cancels queued walks, not bookkeeping");
    expect_eq(d.log,
              "[command] spot.graphnav.cancel received: stopping the running spot.waypoint.goto now; 1 "
              "queued walk/dock command(s) will not start",
              "untargeted log");
  }
  // Untargeted with nothing running: no-op with a clear log, then it runs.
  {
    fsa::CancelInput in;
    const auto d = fsa::decide_cancel(in);
    check(!d.stop_running && d.enqueue && d.cancel_queued_ids.empty(), "nothing running");
    expect_eq(d.log,
              "[command] spot.graphnav.cancel received with no queued command running; nothing to "
              "interrupt, it runs next",
              "nothing-running log");
  }
  // Untargeted while docking onto the dock: docking continues.
  {
    fsa::CancelInput in;
    in.running_id = "dock-1";
    in.running_command = "spot.return_and_dock";
    in.running_in_dock_phase = true;
    const auto d = fsa::decide_cancel(in);
    check(!d.stop_running, "untargeted does not interrupt docking");
    check(d.log.find("docking is not interrupted") != std::string::npos, "docking log");
  }
  // Untargeted while return_and_dock walks to the dock waypoint: stopped.
  {
    fsa::CancelInput in;
    in.running_id = "dock-1";
    in.running_command = "spot.return_and_dock";
    check(fsa::decide_cancel(in).stop_running, "return_and_dock nav phase is stopped");
  }

  // Targeted at the running walk: stop it, answer at once.
  {
    const auto d = fsa::decide_cancel(walking("walk-1"));
    check(d.stop_running && !d.enqueue, "targeted stops its walk");
    check(d.result_text.empty(), "targeted at a running walk is not answered before the stop");
    check(d.cancel_queued_ids.empty(), "targeted leaves other queued walks alone");
  }
  // Targeted at a queued walk: only that one.
  {
    const auto d = fsa::decide_cancel(walking("walk-2"));
    check(!d.stop_running && !d.enqueue && has(d.cancel_queued_ids, "walk-2"), "targeted queued walk");
    check(d.success, "ending a queued walk counts as stopping something");
    expect_eq(d.result_text, "DONE: spot.graphnav.cancel", "queued target result");
  }
  // Targeted at a command that already finished, while a new walk runs: nothing.
  {
    const auto d = fsa::decide_cancel(walking("walk-0-finished"));
    check(!d.stop_running && !d.enqueue && d.cancel_queued_ids.empty(), "finished target does nothing");
    check(!d.success, "stopping nothing answers success=false");
    check(d.remember_target, "an unknown target is remembered in case it arrives later");
    expect_eq(d.result_text, "REJECTED(ignored: target not running)", "ignored result");
    check(d.log.find("ignored: target not running") != std::string::npos, "ignored log");
  }
  // Targeted at a return_and_dock that is docking: refused, docking continues.
  {
    fsa::CancelInput in;
    in.target_id = "dock-1";
    in.running_id = "dock-1";
    in.running_command = "spot.return_and_dock";
    in.running_in_dock_phase = true;
    const auto d = fsa::decide_cancel(in);
    check(!d.stop_running && !d.success, "targeted docking refused");
    expect_eq(d.result_text, "REJECTED(docking in progress)", "docking refusal text");
  }
  // Targeted at a running non-walk command: refused.
  {
    fsa::CancelInput in;
    in.target_id = "map-9";
    in.running_id = "map-9";
    in.running_command = "spot.map.load";
    const auto d = fsa::decide_cancel(in);
    check(!d.stop_running && !d.success, "targeted non-walk refused");
  }

  // A targeted cancel that arrives before its target: remembered, then the
  // target does not start; entries expire after 10 minutes and are bounded.
  {
    fsa::CancelledTargets targets;
    targets.Add("goto-7", 1000);
    check(!targets.Take("goto-8", 2000), "other ids start");
    check(targets.Take("goto-7", 5000), "the remembered target does not start");
    check(!targets.Take("goto-7", 6000), "taken once");
    targets.Add("goto-9", 0);
    check(!targets.Take("goto-9", 10 * 60 * 1000 + 1), "expired after 10 minutes");
    fsa::CancelledTargets small(10 * 60 * 1000, 3);
    for (int i = 0; i < 5; ++i) small.Add("id-" + std::to_string(i), 100 + i);
    check(small.size() == 3, "bounded size");
    check(!small.Take("id-0", 200) && small.Take("id-4", 200), "oldest dropped first");
    targets.Add("", 0);
    check(!targets.Take("", 0), "empty id ignored");
  }

  // Untargeted cancel with nothing running: queued; when it then finds nothing
  // live it answers the ignored text with success=false (adapter side).
  expect_eq(fsa::cancel_ignored_nothing_text(), "REJECTED(ignored: nothing running)", "nothing text");
  expect_eq(fsa::cancel_ignored_finished_text(), "REJECTED(ignored: command ended before the stop)",
            "finished-before-stop text");
  expect_eq(fsa::cancel_not_confirmed_text("still following route"),
            "STOPPED not confirmed: still following route", "not confirmed text");

  // Confirming the stop.
  {
    struct FakeStop {
      int stop_calls{0};
      int fail_stops{0};          // the first N stop commands fail
      int following_reads{0};     // the first N feedback reads say FOLLOWING_ROUTE
      bool reads_fail{false};
      long long slept{0};
      fsa::StopConfirmHooks hooks() {
        fsa::StopConfirmHooks h;
        h.stop = [this](std::string* error) {
          ++stop_calls;
          if (stop_calls <= fail_stops) {
            *error = "RobotCommand failed: lease error";
            return false;
          }
          return true;
        };
        h.feedback = [this]() {
          fsa::StopFeedback fb;
          fb.read_ok = !reads_fail;
          if (following_reads > 0) {
            --following_reads;
            fb.following = true;
          }
          return fb;
        };
        h.sleep_ms = [this](long long ms) { slept += ms; };
        return h;
      }
    };
    {
      FakeStop f;
      const auto r = fsa::confirm_stop(3, 200, f.hooks());
      check(r.confirmed && r.attempts == 1 && f.slept == 200, "confirmed on the first try within 0.2 s");
    }
    {
      FakeStop f;
      f.fail_stops = 100;  // StopMotion always fails
      const auto r = fsa::confirm_stop(3, 200, f.hooks());
      check(!r.confirmed && f.stop_calls == 3, "a failing StopMotion is retried, then not confirmed");
      expect_eq(r.reason, "stop command failed: RobotCommand failed: lease error", "failing stop reason");
    }
    {
      FakeStop f;
      f.fail_stops = 1;
      const auto r = fsa::confirm_stop(3, 200, f.hooks());
      check(r.confirmed && r.attempts == 2, "a stop that fails once is confirmed on the retry");
    }
    {
      FakeStop f;
      f.following_reads = 100;
      const auto r = fsa::confirm_stop(3, 200, f.hooks());
      check(!r.confirmed, "still FOLLOWING_ROUTE is not confirmed");
      expect_eq(r.reason, "still following route", "following reason");
    }
    {
      FakeStop f;
      f.following_reads = 1;
      check(fsa::confirm_stop(3, 200, f.hooks()).confirmed, "stops after a second stop command");
    }
    {
      FakeStop f;
      f.reads_fail = true;
      const auto r = fsa::confirm_stop(3, 200, f.hooks());
      check(!r.confirmed, "unreadable feedback is not confirmed");
      expect_eq(r.reason, "navigation feedback unavailable", "feedback reason");
    }
  }

  // A cancel that finds no live navigation still stops a walk whose earlier
  // stop was not confirmed, if it may still be walking.
  {
    using fsa::UnconfirmedWalk;
    const auto following = fsa::plan_cancel_execution(false, UnconfirmedWalk::kFollowing);
    check(!following.act && following.stop_unconfirmed_walk, "unconfirmed walk still following: stop it");
    check(!following.allow_lease_takeover && !following.stamp_autonomy && !following.clear_release_request,
          "stopping it does not touch the lease");
    const auto unreadable = fsa::plan_cancel_execution(false, UnconfirmedWalk::kUnreadable);
    check(unreadable.stop_unconfirmed_walk, "unconfirmed walk unreadable: stop it");
    const auto stopped = fsa::plan_cancel_execution(false, UnconfirmedWalk::kNotFollowing);
    check(!stopped.act && !stopped.stop_unconfirmed_walk && stopped.clear_unconfirmed,
          "unconfirmed walk now stopped: nothing to do, forget it");
    const auto none = fsa::plan_cancel_execution(false, UnconfirmedWalk::kNone);
    check(!none.act && !none.stop_unconfirmed_walk && !none.clear_unconfirmed, "nothing at all");
    const auto live = fsa::plan_cancel_execution(true, UnconfirmedWalk::kFollowing);
    check(live.act && !live.stop_unconfirmed_walk, "a live navigation is held as before");
  }
  // The internal hold's log: superseded, not FAILED, when there was nothing left.
  check(fsa::internal_hold_outcome(true, false) == fsa::InternalHoldOutcome::kDone, "hold done");
  check(fsa::internal_hold_outcome(false, true) == fsa::InternalHoldOutcome::kSuperseded,
        "nothing left to hold: superseded");
  check(fsa::internal_hold_outcome(false, false) == fsa::InternalHoldOutcome::kFailed, "hold failed");

  // The background poll never recovers or re-sends a cancelled navigation.
  check(fsa::poll_may_auto_recover(41, 0, false, false), "an impaired teleop navigation may be recovered once");
  check(!fsa::poll_may_auto_recover(41, 41, false, false), "a cancelled navigation is never recovered");
  check(!fsa::poll_may_auto_recover(41, 41, false, true), "cancelled and already recovered: no");
  check(fsa::poll_may_auto_recover(42, 41, false, false), "a later navigation is not blocked by an old cancel");
  check(!fsa::poll_may_auto_recover(42, 0, true, false), "never while a queued command runs");
  check(!fsa::poll_may_auto_recover(42, 0, false, true), "never twice");

  // The cancel command when it runs: with nothing live it must not touch the
  // lease, stamp autonomy activity or clear a pending release.
  {
    const auto idle = fsa::plan_cancel_execution(false);
    check(!idle.act && !idle.allow_lease_takeover && !idle.stamp_autonomy && !idle.clear_release_request,
          "idle cancel leaves the lease alone");
    const auto live = fsa::plan_cancel_execution(true);
    check(live.act && !live.allow_lease_takeover, "live cancel acts but never takes the lease over");
  }

  // Dock start: an operator action or a power-off since the command started
  // means AutoDock (which powers on) must not run.
  check(fsa::decide_dock_start(5, 5, "on", "on") == DockStart::kStart, "nothing changed: dock");
  check(fsa::decide_dock_start(5, 6, "on", "on") == DockStart::kStopOperatorAction,
        "estop or any button during preflight: no dock");
  check(fsa::decide_dock_start(5, 5, "on", "off") == DockStart::kStopPoweredOff,
        "motors turned off during preflight: no dock");
  check(fsa::decide_dock_start(5, 5, "on", "powering_off") == DockStart::kStopPoweredOff,
        "motors powering off: no dock");
  check(fsa::decide_dock_start(5, 5, "off", "off") == DockStart::kStart,
        "robot off from the start: the dock powers it on itself");
  expect_eq(fsa::stopped_by_cancel_result_text(), "STOPPED(cancelled)", "cancel result text");
  expect_eq(fsa::stopped_by_operator_result_text(), "STOPPED(operator action)", "operator result text");

  // A cancel during a walk ends it at once: within one 100 ms slice, not at the
  // next 1 s poll and not at the walk's 90 s timeout.
  {
    FakeWalk walk;
    const long long start = walk.now;
    walk.cancel_at_ms = start + 3250;
    const WaitEnd end = fsa::cancellable_wait(90000, 1000, 100, walk.hooks());
    check(end == WaitEnd::kCancelled, "cancel ends the walk");
    check(walk.now - start <= 3300, "seen within one slice, at " + std::to_string(walk.now - start));
  }
  {
    FakeWalk walk;
    walk.cancel_at_ms = walk.now;
    check(fsa::cancellable_wait(90000, 1000, 100, walk.hooks()) == WaitEnd::kCancelled && walk.polls == 0,
          "cancel before polling");
  }
  {
    FakeWalk walk;
    walk.reaches_at_ms = walk.now + 12000;
    check(fsa::cancellable_wait(90000, 1000, 100, walk.hooks()) == WaitEnd::kSucceeded, "arrives");
    check(walk.polls == 13, "polled once a second, polls=" + std::to_string(walk.polls));
  }
  {
    FakeWalk walk;
    walk.fails_at_ms = walk.now + 4000;
    check(fsa::cancellable_wait(90000, 1000, 100, walk.hooks()) == WaitEnd::kFailed, "fails");
  }
  {
    FakeWalk walk;
    const long long start = walk.now;
    check(fsa::cancellable_wait(5000, 1000, 100, walk.hooks()) == WaitEnd::kTimedOut, "times out");
    check(walk.now - start <= 6000, "timeout within one poll of the limit");
  }
  {
    FakeWalk walk;
    walk.stop_at_ms = walk.now + 1500;
    check(fsa::cancellable_wait(90000, 1000, 100, walk.hooks()) == WaitEnd::kStopped, "adapter stop");
  }

  // A cancel during an impaired hold ends it, and is reported first.
  {
    fsa::HoldWatch start;
    fsa::HoldWatch now = start;
    expect_eq(fsa::hold_cancel_reason(start, now), "", "no cancel keeps holding");
    now.cancelled = true;
    expect_eq(fsa::hold_cancel_reason(start, now), "cancelled by spot.graphnav.cancel", "cancel ends a hold");
    now.operator_actions += 1;
    expect_eq(fsa::hold_cancel_reason(start, now), "cancelled by spot.graphnav.cancel",
              "cancel is reported first");
  }

  if (failures != 0) {
    std::cerr << failures << " check(s) failed" << std::endl;
    return 1;
  }
  std::cout << "command_cancel_test passed" << std::endl;
  return 0;
}
