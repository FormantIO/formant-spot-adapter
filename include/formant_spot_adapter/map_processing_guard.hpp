#pragma once

#include <string>

namespace fsa {

// What else the adapter is doing when loop closure is about to rewrite the live
// GraphNav graph (ProcessTopology with modify_map_on_server). Navigation and
// docking read that graph, and a failed commit restores the saved map with
// UploadGraphMap, so processing must not start while any of them is active.
struct MapProcessingActivity {
  bool navigation_active{false};
  bool docking_in_progress{false};  // dock or undock running
  bool dock_requested{false};       // dock / return_and_dock queued, not started yet
  // DockLoop has taken a dock request and is working on it (preflight,
  // return-to-dock navigation, docking). Set before DockLoop checks for map
  // processing, so between this and the processing flag one side always sees
  // the other.
  bool dock_loop_busy{false};
  std::string pending_restore_map_id;
};

// Empty when processing may start; otherwise why it may not.
std::string MapProcessingBlockReason(const MapProcessingActivity& activity);

// Which mapping session spot.map.process_topology is working in. It reuses a
// session that already applies to the active map (for example one left by a
// stop_mapping that failed) and only owns a session it opened itself.
struct ProcessTopologySessionClaim {
  bool session_existed_before{false};
  bool session_open_after{false};
};

// After processing failed: true only for a session this command opened, so a
// pre-existing session (and its staged waypoint edits) is left alone.
bool ShouldDiscardSessionAfterFailedProcessing(const ProcessTopologySessionClaim& claim);

// ProcessTopologyResponse.Status checked per streamed response, the same way
// the Python SDK does (bosdyn/client/map_processing.py,
// _process_topology_streamed_errors): STATUS_UNKNOWN (0, unset) and the three
// error statuses fail; anything else is accepted. Empty means OK.
std::string ProcessTopologyStatusError(int status);

}  // namespace fsa
