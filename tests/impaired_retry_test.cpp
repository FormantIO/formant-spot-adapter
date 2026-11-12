#include <iostream>
#include <string>
#include <vector>

#include "formant_spot_adapter/impaired_retry.hpp"

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
  if (condition) return;
  ++failures;
  std::cerr << "impaired_retry_test failed: " << message << std::endl;
}

void expect_eq(const std::string& got, const std::string& want, const std::string& message) {
  if (got == want) return;
  ++failures;
  std::cerr << "impaired_retry_test failed: " << message << ": got \"" << got << "\" want \""
            << want << "\"" << std::endl;
}

// A fake robot: a clock that only moves when the hold sleeps, and a script of
// what the robot state says at each time.
struct FakeRobot {
  long long now{100000};
  int polls{0};
  // The fault is active (degraded) while now < clears_at_ms.
  long long clears_at_ms{0};
  std::string fault{"camera_server: right camera failed to initialize"};
  bool estopped{false};
  bool reads_fail{false};
  // cancel_reason returns this once now >= cancel_at_ms (0 = never).
  long long cancel_at_ms{0};
  std::string cancel_text{"superseded by spot.graphnav.cancel"};

  fsa::ImpairedHoldHooks hooks() {
    fsa::ImpairedHoldHooks h;
    h.now_ms = [this]() { return now; };
    h.sleep_ms = [this](long long ms) { now += ms; };
    h.sample = [this]() {
      ++polls;
      fsa::ImpairmentSample s;
      s.read_ok = !reads_fail;
      s.estopped = estopped;
      s.degraded = now < clears_at_ms;
      if (s.degraded) s.fault = fault;
      return s;
    };
    h.cancel_reason = [this]() {
      return (cancel_at_ms > 0 && now >= cancel_at_ms) ? cancel_text : std::string();
    };
    return h;
  }
};

}  // namespace

int main() {
  using fsa::hold_until_impairment_clears;
  using fsa::ImpairedHoldEnd;

  // Which commands are held.
  for (const char* cmd : {"spot.waypoint.goto", "spot.waypoint.goto_straight", "spot.graphnav.goto_pose",
                          "spot.graphnav.goto_pose_straight", "spot.dock", "spot.return_and_dock"}) {
    check(fsa::impaired_retry_applies(cmd), std::string(cmd) + " is held and retried");
  }
  for (const char* cmd : {"spot.graphnav.cancel", "spot.stand", "spot.sit", "spot.undock",
                          "spot.map.load", "spot.localize"}) {
    check(!fsa::impaired_retry_applies(cmd), std::string(cmd) + " is not retried");
  }
  check(fsa::supersedes_held_command("spot.graphnav.cancel"), "cancel supersedes");
  check(fsa::supersedes_held_command("spot.waypoint.goto"), "a newer goto supersedes");
  check(fsa::supersedes_held_command("spot.sit"), "sit supersedes");
  check(!fsa::supersedes_held_command("spot.map.set_default"), "map bookkeeping does not");
  check(!fsa::supersedes_held_command("spot.waypoint.save"), "saving a waypoint does not");

  // Fault clears 6 s in: retried as soon as a poll sees it clear.
  {
    FakeRobot robot;
    robot.clears_at_ms = robot.now + 6000;
    const auto r = hold_until_impairment_clears(20000, 1000, "navigate failed: 5(STATUS_ROBOT_IMPAIRED)",
                                                robot.hooks());
    check(r.end == ImpairedHoldEnd::kCleared, "fault that clears in the window is retried");
    check(r.waited_ms == 6000, "retried on the first clear poll, waited " + std::to_string(r.waited_ms));
    expect_eq(r.fault, "camera_server: right camera failed to initialize", "fault seen during the hold");
  }

  // Already clear when the hold starts: one poll interval, then retry.
  {
    FakeRobot robot;
    const auto r = hold_until_impairment_clears(20000, 1000, "", robot.hooks());
    check(r.end == ImpairedHoldEnd::kCleared, "already clear is retried");
    check(r.waited_ms == 1000, "first poll is one interval after the failure");
    check(robot.polls == 1, "one poll");
  }

  // Fault never clears: give up at the window, never later, with the fault.
  {
    FakeRobot robot;
    robot.clears_at_ms = robot.now + 60000;
    const auto r = hold_until_impairment_clears(20000, 1000, "", robot.hooks());
    check(r.end == ImpairedHoldEnd::kNotCleared, "fault that outlasts the window is not retried");
    check(r.waited_ms == 20000, "gave up at exactly the window, waited " + std::to_string(r.waited_ms));
    check(robot.polls == 20, "polled once a second, polls=" + std::to_string(robot.polls));
    expect_eq(r.fault, "camera_server: right camera failed to initialize", "fault kept for the report");
    expect_eq(fsa::impaired_hold_result_text(r, 20000),
              "ABORTED(robot impaired: camera_server: right camera failed to initialize, not cleared "
              "within 20s)",
              "not-cleared result text");
  }

  // Clears exactly at the window edge: the last poll still counts.
  {
    FakeRobot robot;
    robot.clears_at_ms = robot.now + 20000;
    const auto r = hold_until_impairment_clears(20000, 1000, "", robot.hooks());
    check(r.end == ImpairedHoldEnd::kCleared, "clear on the last poll is retried");
  }

  // A window that is not a multiple of the poll still ends on time.
  {
    FakeRobot robot;
    robot.clears_at_ms = robot.now + 60000;
    const auto r = hold_until_impairment_clears(2500, 1000, "", robot.hooks());
    check(r.end == ImpairedHoldEnd::kNotCleared && r.waited_ms == 2500, "partial last poll interval");
  }

  // Robot state unreadable the whole time: not treated as clear.
  {
    FakeRobot robot;
    robot.reads_fail = true;
    const auto r = hold_until_impairment_clears(5000, 1000, "navigate failed: 5(STATUS_ROBOT_IMPAIRED)",
                                                robot.hooks());
    check(r.end == ImpairedHoldEnd::kNotCleared, "unreadable state is not clear");
    expect_eq(r.fault, "navigate failed: 5(STATUS_ROBOT_IMPAIRED)", "falls back to the failure text");
  }

  // Cancelled (newer command, cancel, teleop, lost connection) while waiting: no retry.
  {
    FakeRobot robot;
    robot.clears_at_ms = robot.now + 10000;
    robot.cancel_at_ms = robot.now + 3000;
    const auto r = hold_until_impairment_clears(20000, 1000, "", robot.hooks());
    check(r.end == ImpairedHoldEnd::kCancelled, "cancel stops the hold");
    check(r.waited_ms == 3000, "cancel seen right after the sleep that reached it");
    expect_eq(fsa::impaired_hold_result_text(r, 20000),
              "STOPPED(robot impaired: camera_server: right camera failed to initialize, superseded by "
              "spot.graphnav.cancel)",
              "cancelled result text");
  }

  // Cancelled before the first poll: no robot-state read at all.
  {
    FakeRobot robot;
    robot.cancel_at_ms = robot.now;
    robot.cancel_text = "adapter stopping";
    const auto r = hold_until_impairment_clears(20000, 1000, "", robot.hooks());
    check(r.end == ImpairedHoldEnd::kCancelled && robot.polls == 0, "cancel before polling");
  }

  // Cancel wins even if the fault clears in the same interval.
  {
    FakeRobot robot;
    robot.clears_at_ms = robot.now + 2000;
    robot.cancel_at_ms = robot.now + 2000;
    const auto r = hold_until_impairment_clears(20000, 1000, "", robot.hooks());
    check(r.end == ImpairedHoldEnd::kCancelled, "cancel checked before the clear sample");
  }

  // An E-Stop looks like "no fault" to the degraded rule, but must not retry.
  {
    FakeRobot robot;
    robot.estopped = true;
    const auto r = hold_until_impairment_clears(20000, 1000, "", robot.hooks());
    check(r.end == ImpairedHoldEnd::kCancelled, "E-Stop cancels");
    expect_eq(r.cancel_reason, "E-Stop asserted", "E-Stop reason");
  }

  // Window 0 turns the hold off.
  {
    FakeRobot robot;
    const auto r = hold_until_impairment_clears(0, 1000, "x", robot.hooks());
    check(r.end == ImpairedHoldEnd::kNotCleared && robot.polls == 0 && r.waited_ms == 0,
          "window 0 does not poll");
  }

  // Result text details.
  {
    fsa::ImpairedHoldResult r;
    r.end = ImpairedHoldEnd::kNotCleared;
    expect_eq(fsa::impaired_hold_result_text(r, 20000),
              "ABORTED(robot impaired: STATUS_ROBOT_IMPAIRED, not cleared within 20s)",
              "no fault text known");
    r.fault = "navigate failed: 5(STATUS_ROBOT_IMPAIRED)";
    expect_eq(fsa::impaired_hold_result_text(r, 1500),
              "ABORTED(robot impaired: navigate failed: 5STATUS_ROBOT_IMPAIRED, not cleared within 2s)",
              "parentheses stripped, window rounded up");
    r.fault = std::string(200, 'f');
    const std::string text = fsa::impaired_hold_result_text(r, 20000);
    check(text.size() <= 120 + 9, "long fault capped");
    check(text.find("not cleared within 20s)") != std::string::npos, "suffix survives a long fault");
    expect_eq(fsa::impaired_again_result_text("camera_server: right camera failed to initialize"),
              "ABORTED(robot impaired: camera_server: right camera failed to initialize, still impaired "
              "after one retry)",
              "impaired again text");
  }

  if (failures != 0) {
    std::cerr << failures << " check(s) failed" << std::endl;
    return 1;
  }
  std::cout << "impaired_retry_test passed" << std::endl;
  return 0;
}
