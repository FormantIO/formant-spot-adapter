#include "formant_spot_adapter/impaired_retry.hpp"

#include <algorithm>

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
  return is_nav_command(command) || command == "spot.graphnav.cancel" || command == "spot.sit" ||
         command == "spot.stand" || command == "spot.recover" || command == "spot.undock" ||
         command == "spot.rotate_left" || command == "spot.rotate_right" ||
         command == "spot.robot.reboot" || command == "spot.jetson.reboot";
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
  const auto cancelled = [&](const std::string& why) {
    result.end = ImpairedHoldEnd::kCancelled;
    result.cancel_reason = why;
    result.waited_ms = hooks.now_ms() - start;
    return result;
  };

  for (;;) {
    std::string why = hooks.cancel_reason ? hooks.cancel_reason() : std::string();
    if (!why.empty()) return cancelled(why);
    const long long elapsed = hooks.now_ms() - start;
    if (elapsed >= window_ms) {
      result.end = ImpairedHoldEnd::kNotCleared;
      result.waited_ms = elapsed;
      return result;
    }
    hooks.sleep_ms(std::min(poll_ms, window_ms - elapsed));

    why = hooks.cancel_reason ? hooks.cancel_reason() : std::string();
    if (!why.empty()) return cancelled(why);
    const ImpairmentSample sample = hooks.sample();
    if (!sample.read_ok) continue;  // unreadable is not clear; keep waiting
    if (sample.estopped) return cancelled("E-Stop asserted");
    if (sample.degraded) {
      if (!sample.fault.empty()) result.fault = sample.fault;
      continue;
    }
    result.end = ImpairedHoldEnd::kCleared;
    result.waited_ms = hooks.now_ms() - start;
    return result;
  }
}

std::string impaired_hold_result_text(const ImpairedHoldResult& result, long long window_ms) {
  const std::string fault = fault_or_unknown(result.fault);
  if (result.end == ImpairedHoldEnd::kCancelled) {
    return "STOPPED(" + grammar_reason("robot impaired: " + fault + ", " + result.cancel_reason) +
           ")";
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

}  // namespace fsa
