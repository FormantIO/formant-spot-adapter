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
  std::string motor_power{"on"};
  bool unclearable{false};
  // A GraphNav-named cause (e.g. a service fault the degraded rule ignores)
  // stays listed while now < cause_clears_at_ms.
  long long cause_clears_at_ms{0};
  // Times at which a read shows the fault again (a flapping camera).
  std::vector<long long> flap_at_ms;
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
      s.motor_power = motor_power;
      s.unclearable = unclearable;
      bool flapping = false;
      for (long long t : flap_at_ms) flapping = flapping || t == now;
      s.degraded = now < clears_at_ms || flapping;
      s.cause_present = now < cause_clears_at_ms;
      if (s.degraded) s.fault = fault;
      return s;
    };
    h.cancel_reason = [this]() {
      return (cancel_at_ms > 0 && now >= cancel_at_ms) ? cancel_text : std::string();
    };
    return h;
  }
};

fsa::HoldWatch base_watch() {
  fsa::HoldWatch w;
  w.connect_generation = 3;
  w.lease_epoch = 7;
  w.operator_actions = 42;
  return w;
}

}  // namespace

int main() {
  using fsa::hold_until_impairment_clears;
  using fsa::ImpairedHoldEnd;
  using fsa::SampleVerdict;

  // Which commands are held.
  for (const char* cmd : {"spot.waypoint.goto", "spot.waypoint.goto_straight", "spot.graphnav.goto_pose",
                          "spot.graphnav.goto_pose_straight", "spot.dock", "spot.return_and_dock"}) {
    check(fsa::impaired_retry_applies(cmd), std::string(cmd) + " is held and retried");
  }
  for (const char* cmd : {"spot.graphnav.cancel", "spot.stand", "spot.sit", "spot.undock",
                          "spot.map.load", "spot.localize"}) {
    check(!fsa::impaired_retry_applies(cmd), std::string(cmd) + " is not retried");
  }

  // Which queued commands end a hold.
  for (const char* cmd :
       {"spot.graphnav.cancel", "spot.waypoint.goto", "spot.return_and_dock", "spot.sit", "spot.stand",
        "spot.recover", "spot.undock", "spot.rotate_left", "spot.reset_arm", "spot.camera.calibrate",
        "spot.map.load", "spot.map.delete", "spot.map.start_mapping", "spot.map.create", "spot.localize",
        "spot.localize.confirm", "spot.robot.reboot", "spot.jetson.reboot"}) {
    check(fsa::supersedes_held_command(cmd), std::string(cmd) + " supersedes a held command");
  }
  check(!fsa::supersedes_held_command("spot.map.set_default"), "map default bookkeeping does not");
  check(!fsa::supersedes_held_command("spot.waypoint.save"), "saving a waypoint does not");
  check(!fsa::supersedes_held_command("spot.waypoint.delete"), "deleting a waypoint does not");

  // Window clamp.
  check(fsa::clamp_impaired_retry_window_sec(20) == 20, "20 kept");
  check(fsa::clamp_impaired_retry_window_sec(-5) == 0, "negative clamped to 0");
  check(fsa::clamp_impaired_retry_window_sec(3600) == 60, "an hour clamped to 60");

  // Exact status matching.
  check(fsa::error_text_says_robot_impaired("5(STATUS_ROBOT_IMPAIRED)"), "SDK status text");
  check(fsa::error_text_says_robot_impaired("NavigateTo failed status=5"), "adapter text, end");
  check(fsa::error_text_says_robot_impaired("NavigateTo failed status=5 extra"), "adapter text, space");
  check(!fsa::error_text_says_robot_impaired("NavigateTo failed status=50"), "status=50 is not 5");
  check(!fsa::error_text_says_robot_impaired("NavigateTo failed status=15"), "status=15 is not 5");
  check(!fsa::error_text_says_robot_impaired("NavigateTo failed status=13"), "not localized");

  // hold_cancel_reason: every reason, each on its own.
  {
    const fsa::HoldWatch start = base_watch();
    fsa::HoldWatch now = start;
    expect_eq(fsa::hold_cancel_reason(start, now), "", "nothing changed keeps holding");
    now.queued_commands = {"spot.map.set_default", "spot.waypoint.save"};
    expect_eq(fsa::hold_cancel_reason(start, now), "", "bookkeeping commands keep holding");

    now = start;
    now.running = false;
    expect_eq(fsa::hold_cancel_reason(start, now), "adapter stopping", "shutdown");
    now = start;
    now.connected = false;
    expect_eq(fsa::hold_cancel_reason(start, now), "Spot connection lost", "disconnected");
    now = start;
    now.connect_generation += 1;
    expect_eq(fsa::hold_cancel_reason(start, now), "Spot reconnected",
              "reconnect between polls is seen");
    now = start;
    now.lease_epoch += 1;
    expect_eq(fsa::hold_cancel_reason(start, now), "body lease lost or taken",
              "lease taken on the tablet");
    now = start;
    now.operator_actions += 1;
    expect_eq(fsa::hold_cancel_reason(start, now), "operator used teleop or a button",
              "Formant estop, sit, dock button or teleop twist");
    now = start;
    now.queued_commands = {"spot.map.set_default", "spot.graphnav.cancel"};
    expect_eq(fsa::hold_cancel_reason(start, now), "superseded by spot.graphnav.cancel",
              "queued cancel");
    now = start;
    now.queued_commands = {"spot.reset_arm"};
    expect_eq(fsa::hold_cancel_reason(start, now), "superseded by spot.reset_arm", "queued arm reset");
  }

  // classify_impairment_sample.
  {
    fsa::ImpairmentSample s;
    check(fsa::classify_impairment_sample(s) == SampleVerdict::kWait, "unread is wait");
    s.read_ok = true;
    s.motor_power = "on";
    check(fsa::classify_impairment_sample(s) == SampleVerdict::kClear, "on and no fault is clear");
    s.motor_power = "powering_on";
    check(fsa::classify_impairment_sample(s) == SampleVerdict::kWait, "powering on is not clear");
    s.motor_power = "unknown";
    check(fsa::classify_impairment_sample(s) == SampleVerdict::kWait, "unknown power is not clear");
    s.motor_power = "off";
    check(fsa::classify_impairment_sample(s) == SampleVerdict::kPoweredOff, "motors off");
    s.motor_power = "powering_off";
    check(fsa::classify_impairment_sample(s) == SampleVerdict::kPoweredOff, "motors powering off");
    s.motor_power = "on";
    s.cause_present = true;
    check(fsa::classify_impairment_sample(s) == SampleVerdict::kWait,
          "a GraphNav-named service fault still listed is not clear");
    s.cause_present = false;
    s.degraded = true;
    check(fsa::classify_impairment_sample(s) == SampleVerdict::kWait, "degraded is wait");
    s.unclearable = true;
    check(fsa::classify_impairment_sample(s) == SampleVerdict::kUnclearable, "unclearable");
    s.estopped = true;
    s.motor_power = "off";
    check(fsa::classify_impairment_sample(s) == SampleVerdict::kEstopped, "E-Stop wins");
  }
  check(fsa::any_cause_present({"camera_server"}, {"other", "camera_server"}), "cause present");
  check(!fsa::any_cause_present({"camera_server"}, {"other"}), "cause gone");
  check(!fsa::any_cause_present({""}, {""}), "empty ids never match");
  check(!fsa::any_cause_present({}, {"camera_server"}), "no named cause");

  // Fault clears 6 s in: retried after two clear reads (7 s).
  {
    FakeRobot robot;
    robot.clears_at_ms = robot.now + 6000;
    const auto r = hold_until_impairment_clears(20000, 1000, "navigate failed: 5(STATUS_ROBOT_IMPAIRED)",
                                                robot.hooks());
    check(r.end == ImpairedHoldEnd::kCleared, "fault that clears in the window is retried");
    check(r.waited_ms == 7000, "retried on the second clear read, waited " + std::to_string(r.waited_ms));
    expect_eq(r.fault, "camera_server: right camera failed to initialize", "fault seen during the hold");
  }

  // Already clear when the hold starts: two reads, then retry.
  {
    FakeRobot robot;
    const auto r = hold_until_impairment_clears(20000, 1000, "", robot.hooks());
    check(r.end == ImpairedHoldEnd::kCleared, "already clear is retried");
    check(r.waited_ms == 2000 && robot.polls == 2, "two reads one second apart");
  }

  // A camera that flaps: clear, faulted, clear, clear. One clear read is not enough.
  {
    FakeRobot robot;
    robot.clears_at_ms = robot.now + 3000;
    robot.flap_at_ms = {robot.now + 5000};
    const auto r = hold_until_impairment_clears(20000, 1000, "", robot.hooks());
    // Reads: 1-2 s faulted, 3 s clear, 4 s clear -> cleared at 4 s.
    check(r.end == ImpairedHoldEnd::kCleared && r.waited_ms == 4000,
          "two clear reads in a row at 3 s and 4 s, waited " + std::to_string(r.waited_ms));
  }
  {
    FakeRobot robot;
    robot.clears_at_ms = robot.now + 3000;
    robot.flap_at_ms = {robot.now + 4000};
    const auto r = hold_until_impairment_clears(20000, 1000, "", robot.hooks());
    // 3 s clear, 4 s faulted (count resets), 5 s clear, 6 s clear.
    check(r.end == ImpairedHoldEnd::kCleared && r.waited_ms == 6000,
          "a flap resets the clear count, waited " + std::to_string(r.waited_ms));
  }

  // A service fault GraphNav named, which the degraded rule ignores, holds too.
  {
    FakeRobot robot;
    robot.cause_clears_at_ms = robot.now + 8000;
    const auto r = hold_until_impairment_clears(20000, 1000, "", robot.hooks());
    check(r.end == ImpairedHoldEnd::kCleared && r.waited_ms == 9000,
          "waits for the named cause, waited " + std::to_string(r.waited_ms));
  }

  // Fault never clears: give up at the window, never later, with the fault.
  {
    FakeRobot robot;
    robot.clears_at_ms = robot.now + 60000;
    const auto r = hold_until_impairment_clears(20000, 1000, "", robot.hooks());
    check(r.end == ImpairedHoldEnd::kNotCleared && !r.unclearable, "outlasts the window");
    check(r.waited_ms == 20000, "gave up at exactly the window, waited " + std::to_string(r.waited_ms));
    check(robot.polls == 20, "polled once a second, polls=" + std::to_string(robot.polls));
    expect_eq(fsa::impaired_hold_result_text(r, 20000),
              "ABORTED(robot impaired: camera_server: right camera failed to initialize, not cleared "
              "within 20s)",
              "not-cleared result text");
  }

  // One clear read on the last poll is not enough.
  {
    FakeRobot robot;
    robot.clears_at_ms = robot.now + 20000;
    const auto r = hold_until_impairment_clears(20000, 1000, "", robot.hooks());
    check(r.end == ImpairedHoldEnd::kNotCleared, "a single clear read at the window edge is not a retry");
  }

  // Unclearable fault: give up at once.
  {
    FakeRobot robot;
    robot.clears_at_ms = robot.now + 60000;
    robot.unclearable = true;
    const auto r = hold_until_impairment_clears(20000, 1000, "", robot.hooks());
    check(r.end == ImpairedHoldEnd::kNotCleared && r.unclearable && robot.polls == 1,
          "unclearable stops on the first read");
    expect_eq(fsa::impaired_hold_result_text(r, 20000),
              "ABORTED(robot impaired: camera_server: right camera failed to initialize, unclearable fault)",
              "unclearable result text");
  }

  // Motors turned off during the hold (the Formant estop button is a safe
  // power off, not an E-Stop): cancel, never retry.
  {
    FakeRobot robot;
    robot.motor_power = "powering_off";
    const auto r = hold_until_impairment_clears(20000, 1000, "", robot.hooks());
    check(r.end == ImpairedHoldEnd::kCancelled, "motors powering off cancels");
    expect_eq(r.cancel_reason, "robot powered off", "power-off reason");
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

  // Cancelled while waiting: no retry.
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
    robot.cancel_at_ms = robot.now + 2000;
    const auto r = hold_until_impairment_clears(20000, 1000, "", robot.hooks());
    check(r.end == ImpairedHoldEnd::kCancelled, "cancel checked before the clearing read");
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

  // Result and status text details.
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
    expect_eq(fsa::impaired_hold_status_text("camera_server: right camera failed to initialize", 13200),
              "held: robot impaired (camera_server: right camera failed to initialize), retrying once when "
              "clear, 14s left",
              "held status text");
  }

  if (failures != 0) {
    std::cerr << failures << " check(s) failed" << std::endl;
    return 1;
  }
  std::cout << "impaired_retry_test passed" << std::endl;
  return 0;
}
