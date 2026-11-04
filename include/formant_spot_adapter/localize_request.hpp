#pragma once

#include "bosdyn/api/geometry.pb.h"
#include "bosdyn/api/graph_nav/graph_nav.pb.h"
#include "formant_spot_adapter/localize_recovery.hpp"

namespace fsa {

// Turns a WaypointLocalizeSpec into the GraphNav SetLocalizationRequest. Kept
// apart from SpotClient so the exact request fields can be unit tested.
// `ko_tform_body` is only used when spec.send_current_ko_tform_body is set and it
// is not null.
::bosdyn::api::graph_nav::SetLocalizationRequest BuildWaypointSetLocalizationRequest(
    const WaypointLocalizeSpec& spec, const ::bosdyn::api::SE3Pose* ko_tform_body);

}  // namespace fsa
