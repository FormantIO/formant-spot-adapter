#include "formant_spot_adapter/map_processing_guard.hpp"

namespace fsa {

std::string MapProcessingBlockReason(const MapProcessingActivity& activity) {
  if (activity.navigation_active) return "graphnav navigation in progress";
  if (activity.docking_in_progress) return "dock or undock in progress";
  if (activity.dock_requested) return "dock request pending";
  if (activity.dock_loop_busy) return "dock or return_and_dock attempt in progress";
  if (!activity.pending_restore_map_id.empty()) {
    return "map " + activity.pending_restore_map_id + " is still being restored to the robot";
  }
  return "";
}

bool ShouldDiscardSessionAfterFailedProcessing(const ProcessTopologySessionClaim& claim) {
  return claim.session_open_after && !claim.session_existed_before;
}

std::string ProcessTopologyStatusError(int status) {
  // Values from bosdyn/api/graph_nav/map_processing.proto.
  switch (status) {
    case 0:
      return "status unset (STATUS_UNKNOWN)";
    case 2:
      return "missing waypoint snapshots (STATUS_MISSING_WAYPOINT_SNAPSHOTS)";
    case 3:
      return "invalid graph (STATUS_INVALID_GRAPH)";
    case 4:
      return "map modified during processing (STATUS_MAP_MODIFIED_DURING_PROCESSING)";
    default:
      return "";
  }
}

}  // namespace fsa
