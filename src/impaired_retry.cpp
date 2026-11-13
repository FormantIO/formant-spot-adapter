#include "formant_spot_adapter/impaired_retry.hpp"

#include <algorithm>
#include <cctype>

#include "formant_spot_adapter/command_result.hpp"

namespace fsa {

namespace {

const char* const kUnknownImpairment = "STATUS_ROBOT_IMPAIRED";

// Capped so the suffix that says what happened survives grammar_reason's
// 120-character limit.
std::string fault_or_unknown(const std::string& fault) {
  constexpr size_t kMaxFaultChars = 70;
  if (fault.empty()) return kUnknownImpairment;
  return fault.size() > kMaxFaultChars ? fault.substr(0, kMaxFaultChars) : fault;
}

}  // namespace

bool impaired_retry_applies(const std::string& command) {
  return is_nav_command(command);
}

bool supersedes_held_command(const std::string& command) {
  if (is_nav_command(command)) return true;
  if (command.rfind("spot.map.", 0) == 0) return command != "spot.map.set_default";
  return command == "spot.graphnav.cancel" || command == "spot.sit" || command == "spot.stand" ||
         command == "spot.recover" || command == "spot.undock" || command == "spot.rotate_left" ||
         command == "spot.rotate_right" || command == "spot.reset_arm" ||
         command == "spot.camera.calibrate" || command == "spot.localize" ||
         command == "spot.localize.confirm" || command == "spot.robot.reboot" ||
         command == "spot.jetson.reboot";
}

bool error_text_says_robot_impaired(const std::string& error_text) {
  if (error_text.find("STATUS_ROBOT_IMPAIRED") != std::string::npos) return true;
  const std::string needle = "status=5";
  for (size_t pos = error_text.find(needle); pos != std::string::npos;
       pos = error_text.find(needle, pos + 1)) {
    const size_t after = pos + needle.size();
    if (after >= error_text.size() || !std::isdigit(static_cast<unsigned char>(error_text[after]))) {
      return true;
    }
  }
  return false;
}

std::string hold_cancel_reason(const HoldWatch& at_command_start, const HoldWatch& now) {
  if (now.cancelled && !at_command_start.cancelled) return "cancelled by spot.graphnav.cancel";
  if (!now.running) return "adapter stopping";
  if (!now.connected) return "Spot connection lost";
  if (now.connect_generation != at_command_start.connect_generation) return "Spot reconnected";
  if (now.lease_epoch != at_command_start.lease_epoch) return "body lease lost or taken";
  if (now.operator_actions != at_command_start.operator_actions) {
    return "operator used teleop or a button";
  }
  for (const auto& queued : now.queued_commands) {
    if (supersedes_held_command(queued)) return "superseded by " + queued;
  }
  return std::string();
}

SampleVerdict classify_impairment_sample(const ImpairmentSample& sample) {
  if (!sample.read_ok) return SampleVerdict::kWait;
  if (sample.estopped) return SampleVerdict::kEstopped;
  if (sample.motor_power == "off" || sample.motor_power == "powering_off") {
    return SampleVerdict::kPoweredOff;
  }
  if (sample.unclearable) return SampleVerdict::kUnclearable;
  if (sample.degraded || sample.cause_present) return SampleVerdict::kWait;
  // Only a robot whose motors are on counts as clear: a retry must never be
  // what powers the robot back on.
  if (sample.motor_power != "on") return SampleVerdict::kWait;
  return SampleVerdict::kClear;
}

bool any_cause_present(const std::vector<std::string>& cause_ids,
                       const std::vector<std::string>& present_ids) {
  for (const auto& cause : cause_ids) {
    if (cause.empty()) continue;
    if (std::find(present_ids.begin(), present_ids.end(), cause) != present_ids.end()) return true;
  }
  return false;
}

ImpairedHoldResult hold_until_impairment_clears(long long window_ms,
                                                long long poll_ms,
                                                const std::string& failure_fault,
                                                const ImpairedHoldHooks& hooks) {
  ImpairedHoldResult result;
  result.fault = failure_fault;
  if (window_ms <= 0) return result;
  poll_ms = std::max(1LL, poll_ms);

  const long long start = hooks.now_ms();
  const auto cancel_why = [&]() {
    return hooks.cancel_reason ? hooks.cancel_reason() : std::string();
  };
  const auto cancelled = [&](const std::string& why) {
    result.end = ImpairedHoldEnd::kCancelled;
    result.cancel_reason = why;
    result.waited_ms = hooks.now_ms() - start;
    return result;
  };
  const auto not_cleared = [&](bool unclearable) {
    result.end = ImpairedHoldEnd::kNotCleared;
    result.unclearable = unclearable;
    result.waited_ms = hooks.now_ms() - start;
    return result;
  };

  int clear_reads = 0;
  for (;;) {
    std::string why = cancel_why();
    if (!why.empty()) return cancelled(why);
    const long long elapsed = hooks.now_ms() - start;
    if (elapsed >= window_ms) return not_cleared(false);
    hooks.sleep_ms(std::min(poll_ms, window_ms - elapsed));

    why = cancel_why();
    if (!why.empty()) return cancelled(why);
    const ImpairmentSample sample = hooks.sample();
    if (sample.read_ok && !sample.fault.empty()) result.fault = sample.fault;
    switch (classify_impairment_sample(sample)) {
      case SampleVerdict::kEstopped:
        return cancelled("E-Stop asserted");
      case SampleVerdict::kPoweredOff:
        return cancelled("robot powered off");
      case SampleVerdict::kUnclearable:
        return not_cleared(true);
      case SampleVerdict::kWait:
        clear_reads = 0;
        continue;
      case SampleVerdict::kClear:
        if (++clear_reads < kClearReadsRequired) continue;
        why = cancel_why();
        if (!why.empty()) return cancelled(why);
        result.end = ImpairedHoldEnd::kCleared;
        result.waited_ms = hooks.now_ms() - start;
        return result;
    }
  }
}

std::string impaired_hold_result_text(const ImpairedHoldResult& result, long long window_ms) {
  const std::string fault = fault_or_unknown(result.fault);
  if (result.end == ImpairedHoldEnd::kCancelled) {
    return "STOPPED(" + grammar_reason("robot impaired: " + fault + ", " + result.cancel_reason) +
           ")";
  }
  if (result.unclearable) {
    return "ABORTED(" + grammar_reason("robot impaired: " + fault + ", unclearable fault") + ")";
  }
  const long long window_s = std::max(0LL, (window_ms + 999) / 1000);
  return "ABORTED(" +
         grammar_reason("robot impaired: " + fault + ", not cleared within " +
                        std::to_string(window_s) + "s") +
         ")";
}

std::string impaired_again_result_text(const std::string& fault) {
  return "ABORTED(" +
         grammar_reason("robot impaired: " + fault_or_unknown(fault) +
                        ", still impaired after one retry") +
         ")";
}

std::string impaired_hold_status_text(const std::string& fault, long long remaining_ms) {
  const long long remaining_s = std::max(0LL, (remaining_ms + 999) / 1000);
  return "held: robot impaired (" + fault_or_unknown(fault) + "), retrying once when clear, " +
         std::to_string(remaining_s) + "s left";
}

}  // namespace fsa
