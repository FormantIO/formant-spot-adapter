#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "formant_spot_adapter/localize_recovery.hpp"

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
  if (condition) return;
  ++failures;
  std::cerr << "localize_recovery_test failed: " << message << std::endl;
}

bool contains(const std::string& haystack, const std::string& needle) {
  return haystack.find(needle) != std::string::npos;
}

bool near(double a, double b) { return std::fabs(a - b) < 1e-6; }

fsa::LastLocalization rec(const std::string& map_id, const std::string& waypoint_id) {
  fsa::LastLocalization r;
  r.map_id = map_id;
  r.waypoint_id = waypoint_id;
  r.updated_ms = 1762100000000LL;
  return r;
}

fsa::LastLocalization with_pose(fsa::LastLocalization r, double x, double y, double yaw_rad) {
  r.has_waypoint_tform_body = true;
  r.x = x;
  r.y = y;
  r.z = 0.0;
  r.qx = 0.0;
  r.qy = 0.0;
  r.qz = std::sin(yaw_rad / 2.0);
  r.qw = std::cos(yaw_rad / 2.0);
  return r;
}

void test_command_params() {
  {
    const auto p = fsa::ParseLocalizeCommandParams("kitchen");
    check(p.error.empty(), "bare name is accepted: " + p.error);
    check(p.waypoint_name == "kitchen", "bare value is the waypoint name");
    check(p.waypoint_id.empty(), "no waypoint id from a bare name");
    const fsa::WaypointLocalizeLimits defaults;
    check(near(p.limits.max_distance_m, defaults.max_distance_m), "default max distance");
    check(near(p.limits.max_yaw_rad, defaults.max_yaw_rad), "default max yaw");
  }
  {
    const auto p = fsa::ParseLocalizeCommandParams("  charging dock  ");
    check(p.waypoint_name == "charging dock", "bare name with spaces is trimmed");
  }
  {
    const auto p = fsa::ParseLocalizeCommandParams("name=lights_out_04, map_uuid=abc-123");
    check(p.error.empty(), "name + map_uuid ok: " + p.error);
    check(p.waypoint_name == "lights_out_04", "name= is read");
    check(p.map_uuid == "abc-123", "map_uuid is read");
  }
  {
    const auto p = fsa::ParseLocalizeCommandParams("waypoint_id=wp-9f3e");
    check(p.error.empty(), "waypoint_id ok: " + p.error);
    check(p.waypoint_id == "wp-9f3e", "waypoint_id is read");
    check(p.waypoint_name.empty(), "waypoint_id does not also produce a name");
  }
  {
    const auto p = fsa::ParseLocalizeCommandParams("waypoint_id=wp-1; name=kitchen");
    check(p.waypoint_id == "wp-1" && p.waypoint_name.empty(), "waypoint_id wins over name");
  }
  {
    const auto p = fsa::ParseLocalizeCommandParams(
        "{\"waypoint\": \"kitchen\", \"map_uuid\": \"u-1\", \"max_distance\": 1.5, "
        "\"max_yaw_deg\": 45}");
    // 1.5 m and 45 deg are the caps, so they are still accepted.
    check(p.error.empty(), "JSON ok: " + p.error);
    check(p.waypoint_name == "kitchen", "JSON waypoint is read");
    check(p.map_uuid == "u-1", "JSON map_uuid is read");
    check(near(p.limits.max_distance_m, 1.5), "JSON max_distance is read");
    check(near(p.limits.max_yaw_rad, 45.0 * 3.14159265358979323846 / 180.0),
          "max_yaw_deg is converted to radians");
  }
  {
    const auto p = fsa::ParseLocalizeCommandParams("");
    check(contains(p.error, "missing waypoint"), "empty text is rejected: " + p.error);
  }
  {
    const auto p = fsa::ParseLocalizeCommandParams("map_uuid=abc");
    check(contains(p.error, "missing waypoint"), "map_uuid alone is rejected: " + p.error);
  }
  {
    const auto p = fsa::ParseLocalizeCommandParams("name=kitchen, max_distance=1.6");
    check(contains(p.error, "max_distance"), "max_distance above 1.5 m is rejected: " + p.error);
  }
  {
    const auto p = fsa::ParseLocalizeCommandParams("name=kitchen, max_yaw_deg=46");
    check(contains(p.error, "max_yaw_deg"), "max_yaw_deg above 45 is rejected: " + p.error);
  }
  {
    const auto p = fsa::ParseLocalizeCommandParams("name=kitchen, max_distance=abc");
    check(contains(p.error, "max_distance"), "non-numeric max_distance is rejected");
  }
  {
    const auto p = fsa::ParseLocalizeCommandParams("name=kitchen, max_distance=-1");
    check(contains(p.error, "max_distance"), "negative max_distance is rejected");
  }
  {
    const auto p = fsa::ParseLocalizeCommandParams("name=kitchen, max_yaw_deg=0");
    check(contains(p.error, "max_yaw_deg"), "zero max_yaw_deg is rejected");
  }
}

void test_serialization() {
  {
    const auto r = with_pose(rec("demo_map", "wp-abc"), 1.25, -0.5, 0.7);
    fsa::LastLocalization back;
    check(fsa::ParseLastLocalization(fsa::SerializeLastLocalization(r), &back), "round trip parses");
    check(back.map_id == "demo_map" && back.waypoint_id == "wp-abc", "ids round trip");
    check(back.has_waypoint_tform_body, "pose round trips");
    check(near(back.x, 1.25) && near(back.y, -0.5), "position round trips");
    check(near(back.qz, std::sin(0.35)) && near(back.qw, std::cos(0.35)), "rotation round trips");
    check(back.updated_ms == 1762100000000LL, "timestamp round trips");
  }
  {
    const auto r = rec("demo_map", "wp-abc");
    fsa::LastLocalization back;
    check(fsa::ParseLastLocalization(fsa::SerializeLastLocalization(r), &back), "no-pose parses");
    check(!back.has_waypoint_tform_body, "missing pose stays missing");
    check(near(back.qw, 1.0), "missing pose is identity");
  }
  {
    fsa::LastLocalization back;
    check(!fsa::ParseLastLocalization("map_id=m\n", &back), "missing waypoint_id is rejected");
    check(!fsa::ParseLastLocalization("waypoint_id=w\n", &back), "missing map_id is rejected");
    check(!fsa::ParseLastLocalization("", &back), "empty text is rejected");
  }
  {
    fsa::LastLocalization back;
    check(fsa::ParseLastLocalization(
              "# comment\nmap_id=m\nwaypoint_id=w\nfuture_key=1\nwaypoint_tform_body=1,2\n",
              &back),
          "unknown keys and a malformed pose are tolerated");
    check(!back.has_waypoint_tform_body, "malformed pose is dropped, not half-read");
  }
  {
    fsa::LastLocalization back;
    check(fsa::ParseLastLocalization("map_id=m\nwaypoint_id=w\nwaypoint_tform_body=0,0,0,0,0,0,0\n",
                                     &back),
          "zero quaternion still parses ids");
    check(!back.has_waypoint_tform_body, "zero quaternion is dropped");
  }
  {
    fsa::LastLocalization back;
    check(fsa::ParseLastLocalization("map_id=m\nwaypoint_id=w\nwaypoint_tform_body=0,0,0,0,0,0,2\n",
                                     &back),
          "non-unit quaternion parses");
    check(near(back.qw, 1.0), "non-unit quaternion is normalized");
  }
}

void test_file_io() {
  const auto dir = std::filesystem::temp_directory_path() / "fsa_localize_recovery_test";
  std::filesystem::remove_all(dir);
  const std::string path = (dir / "graphnav" / "last_localization.txt").string();

  fsa::LastLocalization missing;
  check(!fsa::LoadLastLocalization(path, &missing), "missing file loads as false");

  const auto r = with_pose(rec("demo_map", "wp-1"), 0.3, 0.1, -1.0);
  std::string error;
  check(fsa::SaveLastLocalization(path, r, &error), "save creates directories: " + error);
  check(!std::filesystem::exists(path + ".tmp"), "temporary file is renamed away");
  fsa::LastLocalization back;
  check(fsa::LoadLastLocalization(path, &back), "saved file loads");
  check(back.waypoint_id == "wp-1" && near(back.x, 0.3), "loaded content matches");

  auto r2 = rec("demo_map", "wp-2");
  check(fsa::SaveLastLocalization(path, r2, &error), "second save overwrites");
  check(fsa::LoadLastLocalization(path, &back) && back.waypoint_id == "wp-2",
        "overwrite replaces the old record");
  check(!back.has_waypoint_tform_body, "overwrite drops the old pose");

  check(!fsa::SaveLastLocalization(path, rec("demo_map", ""), &error), "empty waypoint not saved");
  check(!fsa::SaveLastLocalization(path, rec("demo\nmap", "wp"), &error), "newline id not saved");
  check(fsa::LoadLastLocalization(path, &back) && back.waypoint_id == "wp-2",
        "a refused save leaves the file intact");

  std::filesystem::remove_all(dir);
}

void test_throttle() {
  fsa::LastLocalizationThrottle t(5000, 0.5, 0.2617993877991494);
  const auto a = with_pose(rec("m", "wp-a"), 0.0, 0.0, 0.0);
  check(t.ShouldPersist(a, 1000), "first localization is persisted");
  t.MarkPersisted(a, 1000);
  check(!t.ShouldPersist(a, 20000), "unchanged localization is not rewritten");
  const auto b = with_pose(rec("m", "wp-b"), 0.0, 0.0, 0.0);
  check(!t.ShouldPersist(b, 3000), "a new waypoint waits for the minimum interval");
  check(t.ShouldPersist(b, 6000), "a new waypoint is persisted after the interval");
  t.MarkPersisted(b, 6000);
  check(!t.ShouldPersist(with_pose(rec("m", "wp-b"), 0.3, 0.0, 0.0), 20000),
        "small move is not rewritten");
  check(t.ShouldPersist(with_pose(rec("m", "wp-b"), 0.6, 0.0, 0.0), 20000),
        "move past the threshold is rewritten");
  check(!t.ShouldPersist(with_pose(rec("m", "wp-b"), 0.0, 0.0, 0.1), 20000),
        "small turn is not rewritten");
  check(t.ShouldPersist(with_pose(rec("m", "wp-b"), 0.0, 0.0, 0.5), 20000),
        "turn past the threshold is rewritten");
  check(t.ShouldPersist(with_pose(rec("other", "wp-b"), 0.0, 0.0, 0.0), 20000),
        "map change is rewritten");
  check(!t.ShouldPersist(rec("m", ""), 20000), "an empty waypoint is never persisted");

  fsa::LastLocalizationThrottle seeded;
  seeded.Seed(a);
  check(!seeded.ShouldPersist(a, 100), "seeded from disk: same record is not rewritten at startup");
  check(seeded.ShouldPersist(b, 100), "seeded from disk: a different waypoint is written at once");
}

void test_fiducial_errors() {
  check(fsa::IsNoUsableFiducialError("SetLocalization failed status=STATUS_NO_MATCHING_FIDUCIAL (8)"),
        "no matching fiducial is recognized");
  check(fsa::IsNoUsableFiducialError("status=STATUS_FIDUCIAL_TOO_FAR_AWAY"), "too far is recognized");
  check(fsa::IsNoUsableFiducialError("status=status_fiducial_too_old"), "case insensitive");
  check(fsa::IsNoUsableFiducialError("STATUS_FIDUCIAL_POSE_UNCERTAIN"), "uncertain is recognized");
  check(!fsa::IsNoUsableFiducialError("SetLocalization failed status=STATUS_ROBOT_IMPAIRED (2)"),
        "impaired is not a fiducial problem");
  check(!fsa::IsNoUsableFiducialError("STATUS_NO_MAP_LOADED"), "no map is not a fiducial problem");
  check(!fsa::IsNoUsableFiducialError(""), "empty error is not a fiducial problem");
}

fsa::WaypointRecoveryInputs walt_after_reboot() {
  fsa::WaypointRecoveryInputs in;
  in.restored_map_id = "demo_map";
  in.fiducial_localization_ok = false;
  in.fiducial_error = "SetLocalization failed status=STATUS_NO_MATCHING_FIDUCIAL (8)";
  in.has_last_localization = true;
  in.last = rec("demo_map", "wp-9m-from-dock");
  in.last_waypoint_in_map = true;
  in.now_ms = in.last.updated_ms + 10LL * 60 * 1000;  // ten minutes later
  in.max_record_age_ms = 24LL * 3600 * 1000;
  in.restore_delay_ms = 5000;
  in.max_restore_delay_ms = 60000;
  return in;
}

fsa::LocalizeResponseFacts facts(const std::string& status, const std::string& quality,
                                 bool ambiguity = false) {
  fsa::LocalizeResponseFacts f;
  f.status = status;
  f.quality_check = quality;
  f.suspected_ambiguity = ambiguity;
  f.ambiguity_check_requested = true;
  return f;
}

void test_acceptance() {
  const auto op = fsa::OperatorAcceptancePolicy();
  const auto rec = fsa::RecoveryAcceptancePolicy();
  check(op.allow_skipped_with_small_correction, "operator policy allows the small-correction rule");
  check(!rec.allow_skipped_with_small_correction, "recovery never uses the small-correction rule");
  {
    const auto a = fsa::EvaluateWaypointLocalization(facts("STATUS_OK", "QUALITY_CHECK_SUCCESS"), rec);
    check(a.ok && a.applied && a.basis == "quality_check", "quality success is accepted");
  }
  {
    const auto a = fsa::EvaluateWaypointLocalization(
        facts("STATUS_OK", "QUALITY_CHECK_POOR_POINT_CLOUD_MATCH"), op);
    check(!a.ok && a.applied, "poor point cloud match is applied but rejected");
    check(contains(a.reason, "POOR_POINT_CLOUD_MATCH"), "reason names the quality: " + a.reason);
  }
  for (const char* q : {"QUALITY_CHECK_POOR_GRAVITY_ALIGNMENT", "QUALITY_CHECK_BAD_HEIGHT",
                        "QUALITY_CHECK_UNKNOWN"}) {
    auto f = facts("STATUS_OK", q);
    f.visual_verification_requested = true;
    f.has_correction = true;
    check(!fsa::EvaluateWaypointLocalization(f, op).ok,
          std::string(q) + " is rejected even with visual verification");
  }
  {
    auto f = facts("STATUS_OK", "QUALITY_CHECK_SUCCESS", true);
    f.visual_verification_requested = true;
    const auto a = fsa::EvaluateWaypointLocalization(f, op);
    check(!a.ok && a.applied && contains(a.reason, "ambiguous"),
          "suspected ambiguity is always rejected: " + a.reason);
  }
  {
    const auto a = fsa::EvaluateWaypointLocalization(
        facts("STATUS_VISUAL_ALIGNMENT_FAILED", "QUALITY_CHECK_UNKNOWN"), op);
    check(!a.ok && !a.applied && contains(a.reason, "VISUAL_ALIGNMENT_FAILED"),
          "visual alignment failure is a failure, not applied: " + a.reason);
  }
  {
    const auto a = fsa::EvaluateWaypointLocalization(facts("STATUS_FAILED", ""), op);
    check(!a.ok && !a.applied, "STATUS_FAILED is not applied");
  }
  // QUALITY_CHECK_SKIPPED: what Walt's dock waypoint returned on the robot.
  {
    auto f = facts("STATUS_OK", "QUALITY_CHECK_SKIPPED");
    f.visual_verification_requested = true;
    const auto a = fsa::EvaluateWaypointLocalization(f, rec);
    check(a.ok && a.basis == "visual_verification",
          "skipped + verified visual alignment is accepted, also for recovery");
  }
  {
    auto f = facts("STATUS_OK", "QUALITY_CHECK_SKIPPED");
    f.has_correction = true;
    f.correction_m = 0.05;
    const auto a = fsa::EvaluateWaypointLocalization(f, rec);
    check(!a.ok && a.applied && contains(a.reason, "visual verification was not used"),
          "recovery rejects skipped without visual verification: " + a.reason);
  }
  {
    auto f = facts("STATUS_OK", "QUALITY_CHECK_SKIPPED");
    f.has_correction = true;
    f.correction_m = 0.05;
    f.correction_yaw_rad = 0.02;
    const auto a = fsa::EvaluateWaypointLocalization(f, op);
    check(a.ok && a.basis == "operator_small_correction",
          "operator: skipped + no ambiguity + 5 cm correction is accepted");
  }
  {
    auto f = facts("STATUS_OK", "QUALITY_CHECK_SKIPPED");
    f.has_correction = true;
    f.correction_m = 0.30;
    const auto a = fsa::EvaluateWaypointLocalization(f, op);
    check(!a.ok && a.applied && contains(a.reason, "moved the pose"),
          "operator: correction over 25 cm is rejected: " + a.reason);
  }
  {
    auto f = facts("STATUS_OK", "QUALITY_CHECK_SKIPPED");
    f.has_correction = true;
    f.correction_yaw_rad = 0.2;  // about 11.5 deg
    check(!fsa::EvaluateWaypointLocalization(f, op).ok, "operator: correction over 10 deg rejected");
  }
  {
    auto f = facts("STATUS_OK", "QUALITY_CHECK_SKIPPED");
    f.has_correction = false;
    const auto a = fsa::EvaluateWaypointLocalization(f, op);
    check(!a.ok && contains(a.reason, "different waypoint"),
          "operator: unknown correction (other waypoint) is rejected: " + a.reason);
  }
  {
    auto f = facts("STATUS_OK", "QUALITY_CHECK_SKIPPED");
    f.ambiguity_check_requested = false;
    f.has_correction = true;
    check(!fsa::EvaluateWaypointLocalization(f, op).ok,
          "operator: skipped without an ambiguity check is rejected");
  }
  {
    auto f = facts("STATUS_OK", "QUALITY_CHECK_SKIPPED", true);
    f.has_correction = true;
    check(!fsa::EvaluateWaypointLocalization(f, op).ok, "operator: skipped + ambiguity rejected");
  }
}

void test_pose_correction() {
  fsa::LocalizePose guess;
  fsa::LocalizePose got{0.03, -0.04, 0.0, 0.0, 0.0, std::sin(0.05), std::cos(0.05)};
  double d = 0.0;
  double yaw = 0.0;
  fsa::PoseCorrection(guess, got, &d, &yaw);
  check(near(d, 0.05), "correction distance is 5 cm");
  check(std::fabs(yaw - 0.1) < 1e-9, "correction yaw is 0.1 rad");
  fsa::LocalizePose turned{0, 0, 0, 0, 0, std::sin(3.1), std::cos(3.1)};  // yaw 6.2 rad
  fsa::PoseCorrection(guess, turned, &d, &yaw);
  check(std::fabs(yaw - (2 * 3.14159265358979323846 - 6.2)) < 1e-9, "yaw difference wraps");
}

void test_specs() {
  fsa::WaypointLocalizeLimits limits;
  limits.max_distance_m = 1.0;
  limits.max_yaw_rad = 0.5;
  const auto first = fsa::OperatorLocalizeSpec("wp-1", limits, true);
  check(first.waypoint_id == "wp-1", "operator spec waypoint");
  check(near(first.waypoint_tform_body.qw, 1.0) && near(first.waypoint_tform_body.x, 0.0),
        "operator guess is identity: robot at the waypoint");
  check(near(first.max_distance_m, 1.0) && near(first.max_yaw_rad, 0.5), "operator limits");
  check(first.refine_with_visual_features && first.verify_visual_refinement,
        "operator first pass verifies visual alignment");
  check(first.do_ambiguity_check, "operator ambiguity check on");
  check(first.send_current_ko_tform_body, "operator guess is about now: send ko_tform_body");
  const auto retry = fsa::OperatorLocalizeSpec("wp-1", limits, false);
  check(retry.refine_with_visual_features && !retry.verify_visual_refinement,
        "operator retry refines visually without verification");

  const auto saved = with_pose(rec("m", "wp-9"), 1.0, 2.0, 0.3);
  const auto r = fsa::RecoveryLocalizeSpec(saved);
  check(r.waypoint_id == "wp-9", "recovery spec waypoint");
  check(near(r.waypoint_tform_body.x, 1.0) && near(r.waypoint_tform_body.y, 2.0) &&
            near(r.waypoint_tform_body.qz, saved.qz),
        "recovery guess is the saved offset");
  check(r.refine_with_visual_features && r.verify_visual_refinement,
        "recovery verifies visual alignment");
  check(r.do_ambiguity_check, "recovery ambiguity check on");
  check(!r.send_current_ko_tform_body, "recovery omits ko_tform_body (odom reset by reboot)");
  const fsa::WaypointLocalizeLimits defaults;
  check(near(r.max_distance_m, defaults.max_distance_m), "recovery uses default limits");
  const auto r2 = fsa::RecoveryLocalizeSpec(rec("m", "wp-9"));
  check(near(r2.waypoint_tform_body.qw, 1.0), "recovery without saved pose uses identity");
}

void test_confirmation() {
  fsa::LocalizationConfirmation c;
  std::string refusal;
  check(c.confirmed(), "starts confirmed");
  check(c.AllowsPersist(), "confirmed localization may be saved");
  check(c.NavigationBlockReason("spot.waypoint.goto").empty(), "confirmed: navigation allowed");
  c.OnAutomaticLocalization("wp-1");
  check(!c.confirmed() && !c.AllowsPersist(), "automatic recovery is unconfirmed, not saved");
  const std::string reason = c.NavigationBlockReason("spot.return_and_dock");
  check(contains(reason, "spot.return_and_dock refused"), "block names the action: " + reason);
  check(contains(reason, "spot.localize.confirm"), "block says how to confirm: " + reason);
  check(contains(reason, "wp-1"), "block names the waypoint: " + reason);
  check(c.TryOperatorConfirm(false, &refusal), "operator confirm unblocks an automatic guess");
  check(c.confirmed() && c.pending_reason().empty(), "confirmed after operator confirm");

  c.OnAutomaticLocalization("wp-1");
  c.OnFiducialLocalization();
  check(c.confirmed(), "fiducial localization unblocks");

  c.OnAutomaticLocalization("wp-1");
  c.OnOperatorLocalize();
  check(c.confirmed(), "a successful operator spot.localize unblocks");

  c.OnRejectedLocalization("wp-2", "GraphNav quality check QUALITY_CHECK_POOR_POINT_CLOUD_MATCH");
  check(!c.confirmed(), "a rejected but applied localization blocks");
  check(contains(c.NavigationBlockReason("spot.waypoint.goto"), "POOR_POINT_CLOUD_MATCH"),
        "block carries the rejection reason");
  check(!c.TryOperatorConfirm(false, &refusal), "a rejected localization needs force to confirm");
  check(contains(refusal, "force=true"), "refusal explains force: " + refusal);
  check(!c.confirmed(), "refused confirm leaves it blocked");
  check(c.TryOperatorConfirm(true, &refusal), "force=true confirms a rejected localization");
  check(c.confirmed(), "confirmed after a forced confirm");

  c.OnRejectedLocalization("wp-2", "ambiguous");
  c.OnLocalizationCleared();
  check(c.confirmed(), "once cleared there is nothing left to confirm");
}

// The restore clears its pending flag before recovery runs. Everything that could
// act on a freshly applied pose must stay blocked from the moment fiducial
// localization fails until recovery has finished.
void test_recovery_ordering() {
  std::string refusal;
  {
    // Recovery accepted: blocked throughout, then unconfirmed until a person confirms.
    fsa::LocalizationConfirmation c;
    c.OnLocalizationCleared();  // map uploaded
    c.OnRecoveryStarted();      // fiducial failed, before pending is cleared
    check(c.recovery_in_progress(), "recovery in progress after start");
    check(contains(c.NavigationBlockReason("spot.waypoint.goto"), "in progress"),
          "navigation blocked as soon as recovery starts");
    check(!c.LocalizeCommandBlockReason().empty(), "spot.localize blocked during recovery");
    check(!c.AllowsPersist(), "nothing saved during recovery");
    check(!c.TryOperatorConfirm(true, &refusal), "even a forced confirm waits for recovery");
    c.OnAutomaticLocalization("wp-1");  // SetLocalization accepted
    check(!c.NavigationBlockReason("spot.waypoint.goto").empty(),
          "still blocked between SetLocalization and the end of recovery");
    check(!c.AllowsPersist(), "applied pose not saved before recovery ends");
    check(!c.TryOperatorConfirm(false, &refusal), "no confirm before recovery ends");
    c.OnRecoveryFinished();
    check(!c.recovery_in_progress(), "recovery finished");
    check(!c.confirmed(), "accepted recovery is still unconfirmed");
    check(c.LocalizeCommandBlockReason().empty(), "spot.localize allowed after recovery");
    check(c.TryOperatorConfirm(false, &refusal), "confirm allowed after recovery");
    check(c.confirmed() && c.AllowsPersist(), "confirmed and saveable");
  }
  {
    // Recovery rejected and cleared: blocked until the clear is done, then free.
    fsa::LocalizationConfirmation c;
    c.OnRecoveryStarted();
    c.OnLocalizationCleared();  // ClearGraph + re-upload done
    check(!c.NavigationBlockReason("spot.waypoint.goto").empty(),
          "blocked after the clear while recovery is still running");
    c.OnRecoveryFinished();
    check(c.confirmed(), "unlocalized robot after a cleared rejection has nothing to confirm");
  }
  {
    // Recovery skipped: nothing applied.
    fsa::LocalizationConfirmation c;
    c.OnRecoveryStarted();
    c.OnRecoveryFinished();
    check(c.confirmed(), "skipped recovery leaves nothing to confirm");
  }
  {
    // Recovery rejected and the clear failed: stays blocked, needs force.
    fsa::LocalizationConfirmation c;
    c.OnRecoveryStarted();
    c.OnRejectedLocalization("wp-3", "GraphNav quality check QUALITY_CHECK_SKIPPED");
    c.OnRecoveryFinished();
    check(!c.confirmed(), "uncleared rejection stays blocked");
    check(!c.TryOperatorConfirm(false, &refusal), "uncleared rejection needs force");
  }
  {
    // Operator spot.localize: same window, so the 1 Hz persist and navigation
    // cannot use a pose that is about to be rejected and cleared.
    fsa::LocalizationConfirmation c;
    check(c.TryBeginOperatorAttempt(&refusal), "operator attempt starts when idle");
    check(!c.AllowsPersist(), "nothing saved during an operator attempt");
    check(!c.NavigationBlockReason("spot.waypoint.goto").empty(),
          "navigation blocked during an operator attempt");
    check(!c.TryBeginOperatorAttempt(&refusal), "a second attempt is refused");
    check(contains(refusal, "in progress"), "refusal says why: " + refusal);
    check(!c.TryOperatorConfirm(true, &refusal), "no confirm during an operator attempt");
    c.OnLocalizationCleared();  // result rejected, cleared
    check(!c.AllowsPersist(), "still nothing saved after the clear until the attempt ends");
    c.OnRecoveryFinished();
    check(c.confirmed() && c.AllowsPersist(), "cleared operator attempt leaves nothing to confirm");
  }
  {
    // A failed operator attempt that applied nothing keeps the earlier state.
    fsa::LocalizationConfirmation c;
    c.OnAutomaticLocalization("wp-1");
    check(c.TryBeginOperatorAttempt(&refusal), "operator attempt allowed while unconfirmed");
    c.OnRecoveryFinished();
    check(!c.confirmed() && contains(c.pending_reason(), "wp-1"),
          "failed attempt does not confirm the automatic guess");
  }
  {
    // Accepted operator attempt.
    fsa::LocalizationConfirmation c;
    c.OnAutomaticLocalization("wp-1");
    check(c.TryBeginOperatorAttempt(&refusal), "attempt begins");
    c.OnOperatorLocalize();
    check(!c.AllowsPersist(), "not saved until the attempt window closes");
    c.OnRecoveryFinished();
    check(c.confirmed(), "accepted operator attempt confirms");
  }
  {
    // Operator attempt refused while restart recovery runs.
    fsa::LocalizationConfirmation c;
    c.OnRecoveryStarted();
    check(!c.TryBeginOperatorAttempt(&refusal), "operator attempt refused during recovery");
  }
}

void test_recovery_decision() {
  {
    const auto plan = fsa::DecideWaypointRecovery(walt_after_reboot());
    check(plan.attempt, "rebooted away from the dock with a saved waypoint: attempt");
    check(contains(plan.reason, "wp-9m-from-dock"), "reason names the waypoint: " + plan.reason);
  }
  {
    auto in = walt_after_reboot();
    in.fiducial_localization_ok = true;
    check(!fsa::DecideWaypointRecovery(in).attempt, "fiducial success: no recovery");
  }
  {
    auto in = walt_after_reboot();
    in.fiducial_error = "SetLocalization failed status=STATUS_ROBOT_IMPAIRED (2)";
    const auto plan = fsa::DecideWaypointRecovery(in);
    check(!plan.attempt, "impaired robot: no recovery");
    check(contains(plan.reason, "ROBOT_IMPAIRED"), "reason carries the error: " + plan.reason);
  }
  {
    auto in = walt_after_reboot();
    in.already_attempted = true;
    check(!fsa::DecideWaypointRecovery(in).attempt, "only once per restore");
  }
  {
    auto in = walt_after_reboot();
    in.has_last_localization = false;
    const auto plan = fsa::DecideWaypointRecovery(in);
    check(!plan.attempt && contains(plan.reason, "no saved"), "nothing saved: no recovery");
  }
  {
    auto in = walt_after_reboot();
    in.last.map_id = "old_map";
    const auto plan = fsa::DecideWaypointRecovery(in);
    check(!plan.attempt && contains(plan.reason, "old_map"), "saved for another map: no recovery");
  }
  {
    auto in = walt_after_reboot();
    in.last_waypoint_in_map = false;
    const auto plan = fsa::DecideWaypointRecovery(in);
    check(!plan.attempt && contains(plan.reason, "not in the restored map"),
          "waypoint deleted from the map: no recovery");
  }
  {
    auto in = walt_after_reboot();
    in.now_ms = in.last.updated_ms + 25LL * 3600 * 1000;
    const auto plan = fsa::DecideWaypointRecovery(in);
    check(!plan.attempt && contains(plan.reason, "s old"), "record older than 24 h: " + plan.reason);
  }
  {
    auto in = walt_after_reboot();
    in.now_ms = in.last.updated_ms + 23LL * 3600 * 1000;
    check(fsa::DecideWaypointRecovery(in).attempt, "record just under 24 h is used");
  }
  {
    auto in = walt_after_reboot();
    in.max_record_age_ms = 0;
    const auto plan = fsa::DecideWaypointRecovery(in);
    check(!plan.attempt && contains(plan.reason, "turned off"), "age limit 0 turns recovery off");
  }
  {
    auto in = walt_after_reboot();
    in.last.updated_ms = 0;
    check(!fsa::DecideWaypointRecovery(in).attempt, "record without a timestamp is not used");
  }
  {
    auto in = walt_after_reboot();
    in.now_ms = in.last.updated_ms - 10LL * 60 * 1000;
    const auto plan = fsa::DecideWaypointRecovery(in);
    check(!plan.attempt && contains(plan.reason, "future"), "record from the future: " + plan.reason);
  }
  {
    auto in = walt_after_reboot();
    in.restore_delay_ms = 61000;
    const auto plan = fsa::DecideWaypointRecovery(in);
    check(!plan.attempt && contains(plan.reason, "may have been moved"),
          "restore delayed past 60 s (lease contention): " + plan.reason);
  }
  {
    auto in = walt_after_reboot();
    in.restore_delay_ms = 59000;
    check(fsa::DecideWaypointRecovery(in).attempt, "restore within 60 s still recovers");
  }
}

}  // namespace

int main() {
  test_command_params();
  test_serialization();
  test_file_io();
  test_throttle();
  test_fiducial_errors();
  test_acceptance();
  test_pose_correction();
  test_specs();
  test_confirmation();
  test_recovery_ordering();
  test_recovery_decision();

  if (failures == 0) std::cout << "localize_recovery_test: all checks passed" << std::endl;
  return failures == 0 ? 0 : 1;
}
