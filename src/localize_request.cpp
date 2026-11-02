#include "formant_spot_adapter/localize_request.hpp"

namespace fsa {

::bosdyn::api::graph_nav::SetLocalizationRequest BuildWaypointSetLocalizationRequest(
    const WaypointLocalizeSpec& spec, const ::bosdyn::api::SE3Pose* ko_tform_body) {
  using ::bosdyn::api::graph_nav::SetLocalizationRequest;
  SetLocalizationRequest req;
  auto* guess = req.mutable_initial_guess();
  guess->set_waypoint_id(spec.waypoint_id);
  auto* tform = guess->mutable_waypoint_tform_body();
  tform->mutable_position()->set_x(spec.waypoint_tform_body.x);
  tform->mutable_position()->set_y(spec.waypoint_tform_body.y);
  tform->mutable_position()->set_z(spec.waypoint_tform_body.z);
  tform->mutable_rotation()->set_x(spec.waypoint_tform_body.qx);
  tform->mutable_rotation()->set_y(spec.waypoint_tform_body.qy);
  tform->mutable_rotation()->set_z(spec.waypoint_tform_body.qz);
  tform->mutable_rotation()->set_w(spec.waypoint_tform_body.qw);
  req.set_max_distance(spec.max_distance_m);
  req.set_max_yaw(spec.max_yaw_rad);
  req.set_fiducial_init(SetLocalizationRequest::FIDUCIAL_INIT_NO_FIDUCIAL);
  req.set_do_ambiguity_check(spec.do_ambiguity_check);
  if (spec.refine_with_visual_features) {
    // With verify_refinement_quality a failed visual refinement fails the request
    // (STATUS_VISUAL_ALIGNMENT_FAILED); without it GraphNav falls back to the
    // point-cloud result.
    req.mutable_refine_with_visual_features()->set_verify_refinement_quality(
        spec.verify_visual_refinement);
  }
  if (spec.send_current_ko_tform_body && ko_tform_body) {
    *req.mutable_ko_tform_body() = *ko_tform_body;
  }
  return req;
}

}  // namespace fsa
