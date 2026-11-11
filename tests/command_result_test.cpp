#include <cstdlib>
#include <iostream>
#include <string>

#include "formant_spot_adapter/command_result.hpp"

using fsa::command_result_text;
using fsa::NavOutcome;

namespace {
int failures = 0;
void expect_eq(const std::string& got, const std::string& want, const char* what) {
  if (got != want) {
    std::cerr << "FAIL " << what << ": got \"" << got << "\" want \"" << want << "\"\n";
    ++failures;
  }
}
}  // namespace

int main() {
  expect_eq(command_result_text("spot.waypoint.goto", true, {"reached", "Robot reached it.", "Wood Pile", true}),
            "ARRIVED: Wood Pile", "goto arrived");
  expect_eq(command_result_text("spot.waypoint.goto", false,
                                {"failed", "Navigation ended with STATUS_STUCK.", "Wood Pile", true}),
            "ABORTED(Navigation ended with STATUS_STUCK)", "goto stuck");
  expect_eq(command_result_text("spot.graphnav.goto_pose", false,
                                {"interrupted", "Spot connection lost: deadline (5s)", "", true}),
            "STOPPED(Spot connection lost: deadline 5s)", "interrupted, parentheses stripped");
  expect_eq(command_result_text("spot.waypoint.goto", false, {"failed", "", "Table", true}),
            "ABORTED(navigation failed)", "no reason");
  expect_eq(command_result_text("spot.dock", true, {"reached", "", "Wood Pile", false}), "ARRIVED: dock",
            "dock ignores the last goto's name");
  expect_eq(command_result_text("spot.stand", true, {}), "DONE: spot.stand", "other command done");
  expect_eq(command_result_text("spot.sit", false, {}), "ABORTED(spot.sit failed)", "other command failed");
  const std::string long_reason(300, 'x');
  expect_eq(command_result_text("spot.waypoint.goto", false, {"failed", long_reason, "", true}),
            "ABORTED(" + std::string(120, 'x') + ")", "reason capped at 120");
  // Failed before navigating: the last command's stuck reason is not this one's.
  expect_eq(command_result_text("spot.waypoint.goto", false,
                                {"failed", "Navigation ended with STATUS_STUCK.", "Old", false}),
            "ABORTED(spot.waypoint.goto failed)", "stale nav state ignored");
  expect_eq(command_result_text("spot.waypoint.goto", true, {"reached", "", "Old", false}),
            "ARRIVED: spot.waypoint.goto", "stale target not named");
  // return_and_dock reached the dock waypoint, then the dock step failed.
  expect_eq(command_result_text("spot.return_and_dock", false, {"reached", "Reached goal", "Dock", true}),
            "ABORTED(spot.return_and_dock failed)", "dock phase failure");
  if (failures == 0) std::cout << "command_result_test: all passed\n";
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
