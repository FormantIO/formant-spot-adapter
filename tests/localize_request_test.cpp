#include <cmath>
#include <iostream>
#include <string>

#include "formant_spot_adapter/localize_request.hpp"

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
  if (condition) return;
  ++failures;
  std::cerr << "localize_request_test failed: " << message << std::endl;
}

bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

using ::bosdyn::api::graph_nav::SetLocalizationRequest;

}  // namespace

int main() {
  ::bosdyn::api::SE3Pose odom;
  odom.mutable_position()->set_x(4.0);
  odom.mutable_rotation()->set_w(1.0);

  {
    fsa::WaypointLocalizeLimits limits;
    limits.max_distance_m = 0.75;
    limits.max_yaw_rad = 0.4;
    const auto spec = fsa::OperatorLocalizeSpec("aglow-weasel", limits, true);
    const SetLocalizationRequest req = fsa::BuildWaypointSetLocalizationRequest(spec, &odom);
    check(req.fiducial_init() == SetLocalizationRequest::FIDUCIAL_INIT_NO_FIDUCIAL,
          "fiducials are not used");
    check(req.initial_guess().waypoint_id() == "aglow-weasel", "guess waypoint id");
    const auto& t = req.initial_guess().waypoint_tform_body();
    check(near(t.position().x(), 0) && near(t.position().y(), 0) && near(t.position().z(), 0) &&
              near(t.rotation().w(), 1) && near(t.rotation().z(), 0),
          "operator guess is identity waypoint_tform_body");
    check(near(req.max_distance(), 0.75) && near(req.max_yaw(), 0.4), "search window");
    check(req.do_ambiguity_check(), "ambiguity check requested");
    check(req.refinement_case() == SetLocalizationRequest::kRefineWithVisualFeatures,
          "visual feature refinement selected");
    check(req.refine_with_visual_features().verify_refinement_quality(),
          "first operator pass verifies visual alignment");
    check(req.has_ko_tform_body() && near(req.ko_tform_body().position().x(), 4.0),
          "operator request carries current ko_tform_body");
    check(!req.restrict_fiducial_detections_to_target_waypoint() && req.use_fiducial_id() == 0,
          "no fiducial fields set");
  }
  {
    const auto spec = fsa::OperatorLocalizeSpec("wp", fsa::WaypointLocalizeLimits{}, false);
    const auto req = fsa::BuildWaypointSetLocalizationRequest(spec, &odom);
    check(req.refinement_case() == SetLocalizationRequest::kRefineWithVisualFeatures &&
              !req.refine_with_visual_features().verify_refinement_quality(),
          "operator retry refines without verification");
  }
  {
    const auto spec = fsa::OperatorLocalizeSpec("wp", fsa::WaypointLocalizeLimits{}, true);
    const auto req = fsa::BuildWaypointSetLocalizationRequest(spec, nullptr);
    check(!req.has_ko_tform_body(), "no ko_tform_body when odom is unavailable");
  }
  {
    fsa::LastLocalization saved;
    saved.map_id = "m";
    saved.waypoint_id = "wp-9";
    saved.has_waypoint_tform_body = true;
    saved.x = 1.5;
    saved.y = -0.5;
    saved.qz = std::sin(0.25);
    saved.qw = std::cos(0.25);
    const auto spec = fsa::RecoveryLocalizeSpec(saved);
    const auto req = fsa::BuildWaypointSetLocalizationRequest(spec, &odom);
    check(req.fiducial_init() == SetLocalizationRequest::FIDUCIAL_INIT_NO_FIDUCIAL,
          "recovery does not use fiducials");
    check(req.initial_guess().waypoint_id() == "wp-9", "recovery guess waypoint");
    const auto& t = req.initial_guess().waypoint_tform_body();
    check(near(t.position().x(), 1.5) && near(t.position().y(), -0.5) &&
              near(t.rotation().z(), std::sin(0.25)) && near(t.rotation().w(), std::cos(0.25)),
          "recovery guess is the saved offset");
    check(near(req.max_distance(), 0.5), "recovery default search distance");
    check(req.do_ambiguity_check(), "recovery ambiguity check");
    check(req.refinement_case() == SetLocalizationRequest::kRefineWithVisualFeatures &&
              req.refine_with_visual_features().verify_refinement_quality(),
          "recovery verifies visual alignment");
    check(!req.has_ko_tform_body(), "recovery never sends ko_tform_body, even if odom is given");
  }
  {
    // Unverified second pass after a reboot: standing when saved, sitting now.
    fsa::LastLocalization saved;
    saved.map_id = "m";
    saved.waypoint_id = "wp-9";
    saved.has_waypoint_tform_body = true;
    saved.x = 1.5;
    saved.z = 0.01;
    saved.has_body_height = true;
    saved.body_height_m = 0.52;
    auto spec = fsa::RecoveryLocalizeSpec(saved, false, true, 0.22);
    spec.neighbors.push_back({"wp-10", fsa::LocalizePose{2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}});
    const auto req = fsa::BuildWaypointSetLocalizationRequest(spec, &odom);
    check(req.fiducial_init() == SetLocalizationRequest::FIDUCIAL_INIT_NO_FIDUCIAL,
          "second pass does not use fiducials");
    check(req.refinement_case() == SetLocalizationRequest::kRefineWithVisualFeatures &&
              !req.refine_with_visual_features().verify_refinement_quality(),
          "second pass refines without verification");
    check(near(req.initial_guess().waypoint_tform_body().position().z(), 0.01 - 0.30),
          "guess height follows the body height change");
    check(req.initial_guess().waypoint_id() == "wp-9",
          "neighbors are for measuring the result only, never sent");
    check(near(req.max_distance(), 0.5), "second pass keeps the same search distance");
    check(!req.has_ko_tform_body(), "second pass never sends ko_tform_body");
  }

  if (failures == 0) std::cout << "localize_request_test: all checks passed" << std::endl;
  return failures == 0 ? 0 : 1;
}
