#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

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
    // force is off unless asked for: a mistyped waypoint on a localized,
    // confirmed robot must not be able to clear its localization.
    const auto plain = fsa::ParseLocalizeCommandParams("kitchen");
    check(!plain.force, "force defaults to false");
    const auto forced = fsa::ParseLocalizeCommandParams("name=kitchen, force=true");
    check(forced.error.empty() && forced.force, "force=true is read: " + forced.error);
    check(!fsa::ParseLocalizeCommandParams("name=kitchen, force=false").force, "force=false is read");
    check(fsa::ParseLocalizeCommandParams("name=kitchen, force=1").force, "force=1 is read");
    const auto bad = fsa::ParseLocalizeCommandParams("name=kitchen, force=maybe");
    check(!bad.error.empty(), "a non-boolean force is an error, not a silent true");
  }
  {
    // Config values reach a long long cast, and these come from JSON or an env
    // var, so NaN/inf/huge must not be undefined behaviour.
    check(fsa::SecondsToMsClamped(1.5) == 1500, "seconds to ms");
    check(fsa::SecondsToMsClamped(0.0) == 0, "zero stays zero");
    check(fsa::SecondsToMsClamped(-5.0) == 0, "negative reads as no limit");
    check(fsa::SecondsToMsClamped(std::nan("")) == 0, "NaN reads as no limit");
    check(fsa::SecondsToMsClamped(std::numeric_limits<double>::infinity()) == 0,
          "infinity reads as no limit, not a cast of inf");
    check(fsa::SecondsToMsClamped(1e300) == 100LL * 24 * 60 * 60 * 1000, "1e300 is capped");
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
  const long long startup = a.updated_ms + 1000;
  check(!seeded.ShouldPersist(a, startup),
        "seeded from disk: same fresh record is not rewritten at startup");
  check(seeded.ShouldPersist(b, startup), "seeded from disk: a different waypoint is written at once");
}

fsa::LocalizeResponseFacts facts(const std::string& status, const std::string& quality,
                                 bool ambiguity = false);

void test_fiducial_errors() {
  // Nov 12, Walt rebooted 2 m from his dock, no tag in view: the nearest-fiducial
  // attempt answered plain STATUS_FAILED. That must not block recovery.
  check(fsa::FiducialFailureBlocksWaypointRecovery(
            "SetLocalization failed status=STATUS_FAILED (5)").empty(),
        "STATUS_FAILED (no tag in view with FIDUCIAL_INIT_NEAREST) allows recovery");
  for (const char* e : {"SetLocalization failed status=STATUS_NO_MATCHING_FIDUCIAL (8)",
                        "status=STATUS_FIDUCIAL_TOO_FAR_AWAY", "status=status_fiducial_too_old",
                        "STATUS_FIDUCIAL_POSE_UNCERTAIN",
                        "SetLocalization failed status=STATUS_ROBOT_IMPAIRED (2)",
                        "SetLocalization failed status=STATUS_ABORTED (4)",
                        "rpc error: deadline exceeded", ""}) {
    check(fsa::FiducialFailureBlocksWaypointRecovery(e).empty(),
          std::string("a waypoint guess may help after: ") + e);
  }
  const std::string no_map =
      fsa::FiducialFailureBlocksWaypointRecovery("SetLocalization failed status=STATUS_NO_MAP_LOADED (12)");
  check(contains(no_map, "no map"), "no map loaded blocks recovery: " + no_map);
  const std::string sensors =
      fsa::FiducialFailureBlocksWaypointRecovery("status=status_incompatible_sensors");
  check(contains(sensors, "sensor"), "incompatible sensors block recovery (case insensitive): " +
                                         sensors);
}

void test_throttle_refresh() {
  // A robot that stands still keeps its record fresh, so a reboot an hour later
  // still finds a record younger than the age limit.
  fsa::LastLocalizationThrottle t(5000, 0.5, 0.2617993877991494, 60000);
  const auto a = with_pose(rec("m", "wp-a"), 0.0, 0.0, 0.0);
  t.MarkPersisted(a, 1000);
  check(!t.ShouldPersist(a, 30000), "unchanged, 29 s later: not rewritten");
  check(t.ShouldPersist(a, 61000), "unchanged, 60 s later: refreshed");
  t.MarkPersisted(a, 61000);
  check(!t.ShouldPersist(a, 100000), "refresh resets the clock");
  check(t.ShouldPersist(a, 121000), "and refreshes again a minute later");

  fsa::LastLocalizationThrottle off(5000, 0.5, 0.2617993877991494, 0);
  off.MarkPersisted(a, 1000);
  check(!off.ShouldPersist(a, 10LL * 3600 * 1000), "refresh interval 0: never refreshed");

  // Seeded from disk: the record's own timestamp counts as the last write.
  fsa::LastLocalizationThrottle seeded;
  auto old_rec = a;
  old_rec.updated_ms = 1000000;
  seeded.Seed(old_rec);
  check(!seeded.ShouldPersist(a, 1000000 + 30000), "fresh record on disk: not rewritten at once");
  check(seeded.ShouldPersist(a, 1000000 + 2LL * 3600 * 1000),
        "stale record on disk and the robot is still localized: refreshed");
  check(seeded.ShouldPersist(a, 500000), "record stamped ahead of the clock: rewritten");
  fsa::LastLocalizationThrottle back;
  back.MarkPersisted(a, 1000000);
  check(back.ShouldPersist(a, 900000), "clock set back after a write: rewritten, not stuck");
}

void test_body_height_serialization() {
  auto r = with_pose(rec("m", "wp"), 0.1, 0.2, 0.0);
  r.has_body_height = true;
  r.body_height_m = 0.523;
  fsa::LastLocalization back;
  check(fsa::ParseLastLocalization(fsa::SerializeLastLocalization(r), &back), "parses");
  check(back.has_body_height && near(back.body_height_m, 0.523), "body height round trips");
  fsa::LastLocalization old_format;
  check(fsa::ParseLastLocalization("map_id=m\nwaypoint_id=w\nupdated_ms=5\n", &old_format),
        "a record written before body_height_m existed still parses");
  check(!old_format.has_body_height, "and has no body height");
  fsa::LastLocalization bad;
  check(fsa::ParseLastLocalization("map_id=m\nwaypoint_id=w\nbody_height_m=nan\n", &bad) &&
            !bad.has_body_height,
        "a non-finite body height is dropped");
}

void test_recovery_seed() {
  auto saved = with_pose(rec("m", "wp-9"), 1.0, -0.4, 0.3);
  saved.z = 0.02;
  {
    const auto p = fsa::RecoverySeedPose(saved, false, 0.0);
    check(near(p.x, 1.0) && near(p.y, -0.4) && near(p.z, 0.02),
          "seed is the saved offset from the waypoint");
    check(near(p.qz, saved.qz) && near(p.qw, saved.qw), "seed keeps the saved heading");
  }
  saved.has_body_height = true;
  saved.body_height_m = 0.52;  // standing when saved
  {
    const auto p = fsa::RecoverySeedPose(saved, true, 0.22);  // sitting after the reboot
    check(near(p.z, 0.02 - 0.30), "sitting after the reboot: guess drops by the height change");
    check(near(p.x, 1.0) && near(p.y, -0.4), "height correction leaves x, y alone");
  }
  {
    const auto p = fsa::RecoverySeedPose(saved, false, 0.0);
    check(near(p.z, 0.02), "current height unknown: saved z used");
  }
  {
    const auto p = fsa::RecoverySeedPose(saved, true, 3.5);
    check(near(p.z, 0.02), "implausible height change (over 1 m) is ignored");
  }
  {
    const auto p = fsa::RecoverySeedPose(saved, true, std::numeric_limits<double>::quiet_NaN());
    check(near(p.z, 0.02), "non-finite current height is ignored");
  }
  {
    const auto p = fsa::RecoverySeedPose(rec("m", "wp-9"), true, 0.2);
    check(near(p.x, 0.0) && near(p.z, 0.0) && near(p.qw, 1.0),
          "no saved offset: identity at the waypoint");
  }
  {
    const auto spec = fsa::RecoveryLocalizeSpec(saved, false, true, 0.22);
    check(!spec.verify_visual_refinement && spec.refine_with_visual_features,
          "unverified pass still refines visually");
    check(near(spec.waypoint_tform_body.z, 0.02 - 0.30), "spec carries the corrected seed");
    check(!spec.send_current_ko_tform_body, "never odom: the reboot reset it");
    check(spec.do_ambiguity_check, "ambiguity check on in both passes");
    const fsa::WaypointLocalizeLimits defaults;
    check(near(spec.max_distance_m, defaults.max_distance_m) &&
              near(spec.max_yaw_rad, defaults.max_yaw_rad),
          "same search window in both passes");
  }
}

void test_pose_math() {
  const double yaw = 0.5 * 3.14159265358979323846;  // 90 deg
  const fsa::LocalizePose a_tform_b{1.0, 2.0, 0.0, 0.0, 0.0, std::sin(yaw / 2), std::cos(yaw / 2)};
  const fsa::LocalizePose b_tform_c{1.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0};
  const auto a_tform_c = fsa::ComposePoses(a_tform_b, b_tform_c);
  check(near(a_tform_c.x, 1.0) && near(a_tform_c.y, 3.0) && near(a_tform_c.z, 0.5),
        "compose rotates the second offset by the first heading");
  check(near(a_tform_c.qz, std::sin(yaw / 2)) && near(a_tform_c.qw, std::cos(yaw / 2)),
        "compose keeps the heading");
  const auto back = fsa::ComposePoses(a_tform_b, fsa::InvertPose(a_tform_b));
  check(near(back.x, 0.0) && near(back.y, 0.0) && near(back.z, 0.0) && near(std::fabs(back.qw), 1.0),
        "a pose composed with its inverse is identity");
  const auto inv = fsa::InvertPose(a_tform_b);
  check(near(inv.x, -2.0) && near(inv.y, 1.0), "inverse position is rotated back");
}

void test_neighbor_correction() {
  // wp-a -> wp-b is 2 m straight ahead; wp-c -> wp-a is 1 m ahead of wp-c.
  std::vector<fsa::WaypointEdgeTform> edges = {
      {"wp-a", "wp-b", {2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}},
      {"wp-c", "wp-a", {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}},
      {"wp-b", "wp-d", {5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}},
  };
  const auto n = fsa::NeighborTforms("wp-a", edges);
  check(n.size() == 2, "two neighbors one edge away (wp-d is two away)");
  fsa::WaypointLocalizeSpec spec;
  spec.waypoint_id = "wp-a";
  spec.waypoint_tform_body = {1.9, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};  // guess: near wp-b
  spec.neighbors = n;
  fsa::LocalizePose out;
  // GraphNav answers relative to wp-b, 0.05 m behind it.
  check(fsa::ResultInGuessFrame(spec, "wp-b", {-0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}, &out),
        "a result on a neighbor is expressed in the guess frame");
  check(near(out.x, 1.95) && near(out.y, 0.0), "via the edge transform");
  double xy = 0.0;
  double z = 0.0;
  double dyaw = 0.0;
  fsa::PoseCorrection(spec.waypoint_tform_body, out, &xy, &z, &dyaw);
  check(near(xy, 0.05), "so the correction is measurable: 5 cm");
  check(fsa::ResultInGuessFrame(spec, "wp-c", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}, &out) &&
            near(out.x, -1.0),
        "edge stored the other way round is inverted");
  check(fsa::ResultInGuessFrame(spec, "wp-a", spec.waypoint_tform_body, &out) && near(out.x, 1.9),
        "result on the guessed waypoint is used as is");
  check(!fsa::ResultInGuessFrame(spec, "wp-d", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}, &out),
        "a waypoint two edges away is not measurable");
}

void test_recovery_passes() {
  const auto first = fsa::RecoveryAcceptancePolicy();
  const auto second = fsa::RecoveryAcceptancePolicy(true);
  check(!first.allow_skipped_with_small_correction, "verified pass: no small-correction rule");
  check(second.allow_skipped_with_small_correction, "unverified pass: small-correction rule");
  check(near(second.max_correction_m, 0.25), "near means within 0.25 m of the saved pose");
  check(fsa::RecoveryNeedsUnverifiedPass(false, false, "STATUS_VISUAL_ALIGNMENT_FAILED"),
        "visual alignment failed, nothing applied: run the unverified pass");
  check(!fsa::RecoveryNeedsUnverifiedPass(true, true, "STATUS_OK"), "accepted: no second pass");
  check(!fsa::RecoveryNeedsUnverifiedPass(false, true, "STATUS_OK"),
        "applied but rejected: no second pass (it gets cleared)");
  check(!fsa::RecoveryNeedsUnverifiedPass(false, false, "STATUS_FAILED"),
        "no match: no second pass");

  auto f = facts("STATUS_OK", "QUALITY_CHECK_SKIPPED");
  f.visual_verification_requested = false;
  f.has_correction = true;
  f.correction_m = 0.08;
  f.correction_yaw_rad = 0.05;
  {
    const auto a = fsa::EvaluateWaypointLocalization(f, second);
    check(a.ok && a.basis == "saved_pose_small_correction",
          "unverified pass: found where it was saved, accepted: " + a.basis + a.reason);
  }
  {
    const auto a = fsa::EvaluateWaypointLocalization(f, first);
    check(!a.ok && a.applied, "same facts under the verified-pass policy: rejected");
  }
  {
    auto far = f;
    far.correction_m = 0.4;
    const auto a = fsa::EvaluateWaypointLocalization(far, second);
    check(!a.ok && a.applied && contains(a.reason, "0.4"),
          "moved 0.4 m from the saved pose: rejected: " + a.reason);
  }
  {
    auto turned = f;
    turned.correction_yaw_rad = 0.3;
    check(!fsa::EvaluateWaypointLocalization(turned, second).ok, "turned 17 deg: rejected");
  }
  {
    auto unknown = f;
    unknown.has_correction = false;
    check(!fsa::EvaluateWaypointLocalization(unknown, second).ok,
          "correction unknown (not the saved waypoint or a neighbor): rejected");
  }
  {
    auto amb = f;
    amb.suspected_ambiguity = true;
    check(!fsa::EvaluateWaypointLocalization(amb, second).ok, "ambiguity: rejected");
  }
  {
    auto poor = f;
    poor.quality_check = "QUALITY_CHECK_BAD_HEIGHT";
    check(!fsa::EvaluateWaypointLocalization(poor, second).ok, "bad height: rejected");
  }
  {
    const auto a = fsa::EvaluateWaypointLocalization(f, fsa::OperatorAcceptancePolicy());
    check(a.ok && a.basis == "operator_small_correction", "operator basis unchanged");
  }
}

void test_recovery_retry() {
  const fsa::RecoveryRetryPolicy policy;
  {
    const auto d = fsa::DecideRecoveryRetry(false, "STATUS_ROBOT_IMPAIRED", 1, policy);
    check(d.retry && d.delay_ms == 10000, "impaired right after the reboot: retry in 10 s");
    check(contains(d.reason, "attempt 2 of 6"), "reason counts attempts: " + d.reason);
  }
  check(fsa::DecideRecoveryRetry(false, "", 1, policy).retry, "no answer (rpc, time sync): retry");
  check(fsa::DecideRecoveryRetry(false, "STATUS_UNKNOWN", 2, policy).retry, "unknown: retry");
  check(fsa::DecideRecoveryRetry(false, "STATUS_ABORTED", 2, policy).retry, "aborted: retry");
  {
    const auto d = fsa::DecideRecoveryRetry(false, "STATUS_ROBOT_IMPAIRED", 6, policy);
    check(!d.retry && contains(d.reason, "giving up"), "six attempts: give up: " + d.reason);
  }
  for (const char* st : {"STATUS_FAILED", "STATUS_VISUAL_ALIGNMENT_FAILED", "STATUS_UNKNOWN_WAYPOINT",
                         "STATUS_NO_MAP_LOADED", "STATUS_INCOMPATIBLE_SENSORS"}) {
    check(!fsa::DecideRecoveryRetry(false, st, 1, policy).retry,
          std::string("final, no further guessing: ") + st);
  }
  check(!fsa::DecideRecoveryRetry(true, "STATUS_OK", 1, policy).retry,
        "applied but rejected: final");
  fsa::RecoveryRetryPolicy off;
  off.retry_interval_ms = 0;
  check(!fsa::DecideRecoveryRetry(false, "STATUS_ROBOT_IMPAIRED", 1, off).retry,
        "interval 0 turns retries off");
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
                                 bool ambiguity) {
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
    f.correction_m = 0.05;
    f.correction_z_m = 0.4;
    check(fsa::EvaluateWaypointLocalization(f, op).ok,
          "operator: the 25 cm bound is horizontal; height is not bounded");
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
  fsa::LocalizePose got{0.03, -0.04, 0.12, 0.0, 0.0, std::sin(0.05), std::cos(0.05)};
  double xy = 0.0;
  double z = 0.0;
  double yaw = 0.0;
  fsa::PoseCorrection(guess, got, &xy, &z, &yaw);
  check(near(xy, 0.05), "correction is horizontal: 5 cm, height excluded");
  check(near(z, 0.12), "height difference reported separately");
  check(std::fabs(yaw - 0.1) < 1e-9, "correction yaw is 0.1 rad");
  fsa::LocalizePose turned{0, 0, 0, 0, 0, std::sin(3.1), std::cos(3.1)};  // yaw 6.2 rad
  fsa::PoseCorrection(guess, turned, &xy, &z, &yaw);
  check(std::fabs(yaw - (2 * 3.14159265358979323846 - 6.2)) < 1e-9, "yaw difference wraps");
}

void test_detail_and_confirmation_need() {
  fsa::LocalizeDetailFields f;
  f.status = "STATUS_OK";
  f.quality_check = "QUALITY_CHECK_SKIPPED";
  f.accepted_by = "operator_small_correction";
  f.visual_verification = false;
  f.has_correction = true;
  f.correction_xy_m = 0.0423;
  f.correction_z_m = -0.011;
  f.correction_yaw_rad = 0.05;
  const std::string d = fsa::FormatLocalizeDetail(f);
  check(contains(d, "status=STATUS_OK"), "detail has status: " + d);
  check(contains(d, "quality=QUALITY_CHECK_SKIPPED"), "detail has quality: " + d);
  check(contains(d, "accepted_by=operator_small_correction"), "detail has accepted_by: " + d);
  check(contains(d, "visual_verification=false"), "detail says whether verification ran: " + d);
  check(contains(d, "correction_m=0.042"), "detail has horizontal correction: " + d);
  check(contains(d, "correction_z_m=-0.011"), "detail has height correction: " + d);
  check(contains(d, "correction_deg=2.9"), "detail has yaw correction in degrees: " + d);
  fsa::LocalizeDetailFields rejected;
  rejected.status = "STATUS_VISUAL_ALIGNMENT_FAILED";
  rejected.visual_verification = true;
  const std::string r = fsa::FormatLocalizeDetail(rejected);
  check(contains(r, "accepted_by=none") && contains(r, "visual_verification=true") &&
            contains(r, "correction_m=unknown") && contains(r, "correction_z_m=unknown") &&
            contains(r, "correction_deg=unknown"),
        "rejected detail still has every field: " + r);

  check(fsa::OperatorAcceptNeedsConfirmation("operator_small_correction", false),
        "small-correction accept on the unverified retry needs confirmation");
  check(!fsa::OperatorAcceptNeedsConfirmation("operator_small_correction", true),
        "small-correction accept on the verified attempt does not");
  check(!fsa::OperatorAcceptNeedsConfirmation("visual_verification", true),
        "visual verification accept does not");
  check(!fsa::OperatorAcceptNeedsConfirmation("quality_check", false), "quality success does not");

  fsa::LocalizationConfirmation c;
  c.OnUnverifiedOperatorLocalization("aglow-weasel",
                                     "visual alignment failed; accepted on small correction");
  check(!c.confirmed() && !c.AllowsPersist(), "unverified operator accept is unconfirmed");
  const std::string why = c.NavigationBlockReason("spot.waypoint.goto");
  check(contains(why, "visual alignment failed; accepted on small correction"),
        "block explains the weaker evidence: " + why);
  std::string refusal;
  check(c.TryOperatorConfirm(false, &refusal), "plain confirm is enough (not a rejection)");
  check(c.confirmed(), "confirmed after spot.localize.confirm");
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
    // Nov 12: Walt rebooted about 2 m from his dock, no tag in view. The old
    // gate skipped this ("a reason a waypoint guess does not fix").
    auto in = walt_after_reboot();
    in.fiducial_error = "SetLocalization failed status=STATUS_FAILED (5)";
    const auto plan = fsa::DecideWaypointRecovery(in);
    check(plan.attempt, "STATUS_FAILED with no tag in view: attempt");
    check(contains(plan.reason, "STATUS_FAILED") && contains(plan.reason, "wp-9m-from-dock"),
          "reason carries the fiducial error and the waypoint: " + plan.reason);
  }
  {
    auto in = walt_after_reboot();
    in.fiducial_error = "SetLocalization failed status=STATUS_ROBOT_IMPAIRED (2)";
    check(fsa::DecideWaypointRecovery(in).attempt,
          "impaired right after a reboot: attempt (and retry while impaired)");
  }
  {
    auto in = walt_after_reboot();
    in.fiducial_error = "SetLocalization failed status=STATUS_NO_MAP_LOADED (12)";
    const auto plan = fsa::DecideWaypointRecovery(in);
    check(!plan.attempt && contains(plan.reason, "NO_MAP_LOADED"),
          "no map loaded: no recovery: " + plan.reason);
  }
  {
    // Standing still for two hours before the reboot. The record was refreshed
    // a minute before the reboot, so it is fresh even though nothing moved.
    fsa::LastLocalizationThrottle t;
    auto saved = with_pose(rec("demo_map", "wp-near-dock"), 1.8, 0.4, 0.2);
    const long long t0 = saved.updated_ms;
    t.MarkPersisted(saved, t0);
    long long last_write = t0;
    for (long long now = t0; now <= t0 + 2LL * 3600 * 1000; now += 1000) {
      if (t.ShouldPersist(saved, now)) {
        t.MarkPersisted(saved, now);
        last_write = now;
      }
    }
    saved.updated_ms = last_write;
    auto in = walt_after_reboot();
    in.fiducial_error = "SetLocalization failed status=STATUS_FAILED (5)";
    in.last = saved;
    in.last.map_id = "demo_map";
    in.now_ms = t0 + 2LL * 3600 * 1000 + 4LL * 60 * 1000;  // body back about 4 min later
    in.max_record_age_ms = 1800LL * 1000;                 // default 30 min
    const auto plan = fsa::DecideWaypointRecovery(in);
    check(plan.attempt, "idle two hours, record refreshed while localized: attempt: " + plan.reason);
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
  test_throttle_refresh();
  test_body_height_serialization();
  test_recovery_seed();
  test_pose_math();
  test_neighbor_correction();
  test_recovery_passes();
  test_recovery_retry();
  test_acceptance();
  test_pose_correction();
  test_detail_and_confirmation_need();
  test_specs();
  test_confirmation();
  test_recovery_ordering();
  test_recovery_decision();

  if (failures == 0) std::cout << "localize_recovery_test: all checks passed" << std::endl;
  return failures == 0 ? 0 : 1;
}
