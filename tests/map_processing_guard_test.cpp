#include <iostream>
#include <string>

#include "formant_spot_adapter/map_processing_guard.hpp"

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
  if (condition) return;
  ++failures;
  std::cerr << "map_processing_guard_test failed: " << message << std::endl;
}

bool contains(const std::string& haystack, const std::string& needle) {
  return haystack.find(needle) != std::string::npos;
}

void test_block_reason() {
  fsa::MapProcessingActivity idle;
  check(fsa::MapProcessingBlockReason(idle).empty(), "idle robot may process");

  fsa::MapProcessingActivity nav;
  nav.navigation_active = true;
  check(contains(fsa::MapProcessingBlockReason(nav), "navigation"), "navigation blocks");

  fsa::MapProcessingActivity dock;
  dock.docking_in_progress = true;
  check(contains(fsa::MapProcessingBlockReason(dock), "dock"), "dock/undock blocks");

  fsa::MapProcessingActivity queued;
  queued.dock_requested = true;
  check(contains(fsa::MapProcessingBlockReason(queued), "dock request"), "queued dock blocks");

  fsa::MapProcessingActivity restore;
  restore.pending_restore_map_id = "site_a";
  check(contains(fsa::MapProcessingBlockReason(restore), "site_a"), "pending restore blocks");

  fsa::MapProcessingActivity all;
  all.navigation_active = true;
  all.docking_in_progress = true;
  check(!fsa::MapProcessingBlockReason(all).empty(), "several activities block");
}

void test_session_ownership() {
  fsa::ProcessTopologySessionClaim opened;
  opened.session_existed_before = false;
  opened.session_open_after = true;
  check(fsa::ShouldDiscardSessionAfterFailedProcessing(opened),
        "a session the command opened is discarded");

  fsa::ProcessTopologySessionClaim reused;
  reused.session_existed_before = true;
  reused.session_open_after = true;
  check(!fsa::ShouldDiscardSessionAfterFailedProcessing(reused),
        "a reused session (e.g. from a failed stop_mapping) is kept");

  fsa::ProcessTopologySessionClaim none;
  check(!fsa::ShouldDiscardSessionAfterFailedProcessing(none), "no session, nothing to discard");
}

void test_status() {
  check(!fsa::ProcessTopologyStatusError(0).empty(), "unset status fails");
  check(fsa::ProcessTopologyStatusError(1).empty(), "STATUS_OK passes");
  check(!fsa::ProcessTopologyStatusError(2).empty(), "missing snapshots fails");
  check(!fsa::ProcessTopologyStatusError(3).empty(), "invalid graph fails");
  check(!fsa::ProcessTopologyStatusError(4).empty(), "map modified fails");
  check(fsa::ProcessTopologyStatusError(99).empty(), "unknown future status accepted like the Python SDK");
}

}  // namespace

int main() {
  test_block_reason();
  test_session_ownership();
  test_status();
  if (failures > 0) {
    std::cerr << failures << " failure(s)" << std::endl;
    return 1;
  }
  std::cout << "map_processing_guard_test passed" << std::endl;
  return 0;
}
