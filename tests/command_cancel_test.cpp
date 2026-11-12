#include <iostream>
#include <string>

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

// A fake walk: GraphNav reports FOLLOWING_ROUTE until reaches_at_ms (0 = never).
struct FakeWalk {
  long long now{50000};
  long long reaches_at_ms{0};
  long long fails_at_ms{0};
  long long cancel_at_ms{0};  // a spot.graphnav.cancel arrives at this time (0 = never)
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

}  // namespace

int main() {
  using fsa::QueueAdmission;
  using fsa::WaitEnd;

  check(fsa::is_stop_command("spot.graphnav.cancel"), "cancel is a stop command");
  check(!fsa::is_stop_command("spot.sit"), "a queued sit does not cancel an earlier goto");
  check(!fsa::is_stop_command("spot.waypoint.goto"), "goto is not a stop");
  for (const char* cmd : {"spot.waypoint.goto", "spot.waypoint.goto_straight", "spot.graphnav.goto_pose",
                          "spot.graphnav.goto_pose_straight", "spot.dock", "spot.return_and_dock"}) {
    check(fsa::stoppable_command(cmd), std::string(cmd) + " is ended by a stop");
  }
  check(!fsa::stoppable_command("spot.map.load"), "a map load is not ended by a stop");

  // A full queue still accepts a cancel, at the front.
  check(fsa::admit_command("spot.graphnav.cancel", 10, 10) == QueueAdmission::kFront,
        "full queue accepts a cancel");
  check(fsa::admit_command("spot.graphnav.cancel", 0, 10) == QueueAdmission::kFront,
        "cancel always goes to the front");
  check(fsa::admit_command("spot.waypoint.goto", 10, 10) == QueueAdmission::kReject,
        "full queue rejects a goto");
  check(fsa::admit_command("spot.waypoint.goto", 3, 10) == QueueAdmission::kBack, "goto goes to the back");

  // Which commands a stop ends.
  check(fsa::stopped_since_enqueue("spot.waypoint.goto", 4, 5), "goto queued before the stop is ended");
  check(!fsa::stopped_since_enqueue("spot.waypoint.goto", 5, 5), "goto queued after the stop runs");
  check(!fsa::stopped_since_enqueue("spot.map.load", 4, 5), "a map load is not ended");

  // Arrival logs: nothing running is a clear no-op.
  expect_eq(fsa::stop_arrival_log("spot.graphnav.cancel", ""),
            "[command] spot.graphnav.cancel received with no queued command running; nothing to "
            "interrupt, it runs next",
            "cancel with nothing running");
  expect_eq(fsa::stop_arrival_log("spot.graphnav.cancel", "spot.waypoint.goto"),
            "[command] spot.graphnav.cancel received: stopping the running spot.waypoint.goto now",
            "cancel during a walk");
  check(fsa::stop_arrival_log("spot.graphnav.cancel", "spot.map.load").find("runs after it") !=
            std::string::npos,
        "cancel during a non-walk command says it waits");
  expect_eq(fsa::stopped_by_cancel_result_text(), "STOPPED(cancelled)", "result text");

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
  // Cancel before the first poll: no feedback poll at all.
  {
    FakeWalk walk;
    walk.cancel_at_ms = walk.now;
    check(fsa::cancellable_wait(90000, 1000, 100, walk.hooks()) == WaitEnd::kCancelled && walk.polls == 0,
          "cancel before polling");
  }
  // Without a cancel the walk ends as GraphNav says.
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

  // A cancel during an impaired hold ends it (the stop counter is part of the
  // hold's watch), and wins over everything else that may have changed.
  {
    fsa::HoldWatch start;
    start.stop_seq = 9;
    fsa::HoldWatch now = start;
    expect_eq(fsa::hold_cancel_reason(start, now), "", "no stop keeps holding");
    now.stop_seq = 10;
    expect_eq(fsa::hold_cancel_reason(start, now), "cancelled by spot.graphnav.cancel", "stop ends a hold");
    now.operator_actions += 1;
    expect_eq(fsa::hold_cancel_reason(start, now), "cancelled by spot.graphnav.cancel",
              "stop is reported first");
  }

  if (failures != 0) {
    std::cerr << failures << " check(s) failed" << std::endl;
    return 1;
  }
  std::cout << "command_cancel_test passed" << std::endl;
  return 0;
}
