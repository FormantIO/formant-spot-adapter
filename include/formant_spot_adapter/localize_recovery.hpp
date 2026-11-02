#pragma once

#include <string>

namespace fsa {

// Localizing Spot on its GraphNav map without a fiducial in view.
//
// The adapter normally localizes with FIDUCIAL_INIT_NEAREST. A map with a single
// fiducial (the dock tag) can only be localized that way while the robot can see
// the dock. After a reboot away from the dock the robot stays unlocalized and can
// neither navigate nor dock. The pieces here let the robot localize from "I am at
// waypoint X" instead: an operator command (`spot.localize`) and an automatic
// one-shot recovery that reuses the last waypoint the robot was localized at.

// Search window GraphNav uses around the initial guess. Boston Dynamics' own
// "initialize at waypoint" example uses 0.2 m / 20 deg; the defaults are a little
// wider so a robot standing near, not exactly on, the waypoint still localizes.
struct WaypointLocalizeLimits {
  double max_distance_m{0.5};
  double max_yaw_rad{0.3490658503988659};  // 20 degrees
};

// Hard bounds on operator-supplied limits. A large search window lets GraphNav
// match the wrong part of a repetitive map.
constexpr double kMaxLocalizeDistanceM = 1.5;
constexpr double kMaxLocalizeYawDeg = 45.0;

// What makes a SetLocalization answer trustworthy (Spot SDK 5.1).
//
// GraphNav applies a STATUS_OK localization even when its own point-cloud quality
// check fails, so STATUS_OK alone is not enough. The 5.1 API has no field that
// forces that quality check to run (no refine_algorithm; the `refinement` oneof
// only offers refine_fiducial_result_with_icp, which needs a fiducial, and
// refine_with_visual_features). It reports QUALITY_CHECK_SKIPPED when it "wasn't
// enough data to make a determination", which is what a NO_FIDUCIAL waypoint
// guess got on the robot. The checks that remain under the client's control:
//   - refine_with_visual_features with verify_refinement_quality: GraphNav
//     returns STATUS_VISUAL_ALIGNMENT_FAILED (nothing applied) when visual
//     alignment fails or its pose solution is unreliable, so STATUS_OK means
//     that independent check passed;
//   - do_ambiguity_check: GraphNav reports a suspected_ambiguity when a nearby
//     pose (for example turned 180 degrees) fits as well;
//   - how far GraphNav moved the pose from the guess.
//
// Acceptance, in order:
//   1. STATUS_OK is required; anything else applied nothing.
//   2. A suspected ambiguity is always rejected.
//   3. QUALITY_CHECK_SUCCESS is accepted.
//   4. QUALITY_CHECK_SKIPPED is accepted when visual verification was requested
//      (STATUS_OK then means it passed), or, only where the policy allows it (an
//      operator who asserted the waypoint), when the ambiguity check was
//      requested and the pose moved at most max_correction from the guess.
//   5. Every other quality result (poor point cloud match, poor gravity
//      alignment, bad height, unknown) is rejected.
// Trade-off of 4b: a small correction does not prove the point clouds matched;
// a poorly constrained match can also stay near the guess. It is accepted only
// on a person's word that the robot is at that waypoint, with the ambiguity check
// as the remaining automatic guard. Restart recovery never uses it.

struct LocalizePose {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

// Everything that goes into one SetLocalization request at a waypoint.
struct WaypointLocalizeSpec {
  std::string waypoint_id;
  // Robot pose relative to the waypoint. Identity means "at the waypoint".
  LocalizePose waypoint_tform_body;
  double max_distance_m{0.5};
  double max_yaw_rad{0.3490658503988659};
  bool refine_with_visual_features{true};
  // Fail with STATUS_VISUAL_ALIGNMENT_FAILED instead of falling back to the
  // point-cloud result when visual refinement fails.
  bool verify_visual_refinement{true};
  bool do_ambiguity_check{true};
  // Send the robot's current odom pose as ko_tform_body. Only correct when the
  // guess is about the robot's pose right now; a guess saved before a reboot
  // refers to an odom frame that no longer exists.
  bool send_current_ko_tform_body{true};
};

struct LocalizeAcceptancePolicy {
  // Rule 4b above. Operator command only.
  bool allow_skipped_with_small_correction{false};
  double max_correction_m{0.25};
  double max_correction_yaw_rad{0.17453292519943295};  // 10 degrees
};

// What the request asked for and what GraphNav answered.
struct LocalizeResponseFacts {
  std::string status;
  std::string quality_check;
  bool suspected_ambiguity{false};
  bool ambiguity_check_requested{false};
  bool visual_verification_requested{false};
  // How far the result moved from the guess. Only known when GraphNav kept the
  // guessed waypoint as the reference.
  bool has_correction{false};
  double correction_m{0.0};
  double correction_yaw_rad{0.0};
};

struct LocalizeAcceptance {
  bool ok{false};
  // GraphNav applied a localization (STATUS_OK). When `applied && !ok` the robot
  // is now localized to a result that was rejected.
  bool applied{false};
  // Why the result was rejected, empty when ok.
  std::string reason;
  // Which check accepted it: quality_check, visual_verification or
  // operator_small_correction. Empty when rejected.
  std::string basis;
};
LocalizeAcceptance EvaluateWaypointLocalization(const LocalizeResponseFacts& facts,
                                                const LocalizeAcceptancePolicy& policy);

// Distance and absolute yaw difference between two poses in the same frame.
void PoseCorrection(const LocalizePose& guess, const LocalizePose& result, double* out_distance_m,
                    double* out_yaw_rad);

// Request specs. The operator command first asks for visual verification; if that
// fails (STATUS_VISUAL_ALIGNMENT_FAILED, nothing applied) it retries once without
// it, under the operator policy. Restart recovery makes one verified attempt.
WaypointLocalizeSpec OperatorLocalizeSpec(const std::string& waypoint_id,
                                          const WaypointLocalizeLimits& limits,
                                          bool verify_visual_refinement);
LocalizeAcceptancePolicy OperatorAcceptancePolicy();
struct LastLocalization;
WaypointLocalizeSpec RecoveryLocalizeSpec(const LastLocalization& last);
LocalizeAcceptancePolicy RecoveryAcceptancePolicy();

// Parameters of the `spot.localize` command.
struct LocalizeCommandParams {
  // Exactly one of these is set when `error` is empty.
  std::string waypoint_id;
  std::string waypoint_name;
  // Optional: the map the operator expects to be loaded.
  std::string map_uuid;
  WaypointLocalizeLimits limits;
  // Why the parameters were rejected, written for an operator. Empty when ok.
  std::string error;
};

// Accepts `<name>`, `name=<name>`, `waypoint_id=<id>`, plus optional
// `map_uuid=<uuid>`, `max_distance=<m>` and `max_yaw_deg=<deg>`, as key=value
// pairs or a JSON object.
LocalizeCommandParams ParseLocalizeCommandParams(const std::string& text);

// The last place the robot was localized, saved so a later restart can reuse it
// as an initial guess.
struct LastLocalization {
  // Adapter map id (the name under data/graphnav/maps), not the GraphNav map uuid.
  std::string map_id;
  std::string waypoint_id;
  // Robot pose relative to the waypoint at that time. Identity when unknown.
  bool has_waypoint_tform_body{false};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
  // Wall-clock time of the save, milliseconds since the Unix epoch.
  long long updated_ms{0};
};

// One `key=value` per line. Unknown keys are ignored when reading so the file
// can grow later.
std::string SerializeLastLocalization(const LastLocalization& rec);
// False when the text has no map_id or waypoint_id.
bool ParseLastLocalization(const std::string& text, LastLocalization* out);

// Writes through a temporary file and a rename so a crash mid-write never
// leaves a half-written file behind.
bool SaveLastLocalization(const std::string& path, const LastLocalization& rec,
                          std::string* error);
bool LoadLastLocalization(const std::string& path, LastLocalization* out);

// Decides when a fresh localization is worth writing to disk. GraphNav reports
// localization every poll; the file only needs to change when the robot is at a
// different waypoint or has moved noticeably relative to it, and never more often
// than `min_interval_ms`.
class LastLocalizationThrottle {
 public:
  explicit LastLocalizationThrottle(long long min_interval_ms = 5000,
                                    double min_move_m = 0.5,
                                    double min_turn_rad = 0.2617993877991494 /* 15 deg */);

  bool ShouldPersist(const LastLocalization& current, long long now_ms) const;
  void MarkPersisted(const LastLocalization& rec, long long now_ms);
  // Starts from what is already on disk so a restart does not rewrite it at once.
  void Seed(const LastLocalization& rec);

 private:
  long long min_interval_ms_;
  double min_move_m_;
  double min_turn_rad_;
  bool has_last_{false};
  LastLocalization last_;
  long long last_write_ms_{0};
};

// True for SetLocalization errors that mean "no usable fiducial": the robot sees
// no tag that is in the map, or the tag it sees is too far, too old or uncertain.
bool IsNoUsableFiducialError(const std::string& error_text);

struct WaypointRecoveryInputs {
  // The map that was just restored and the error from the fiducial attempt.
  std::string restored_map_id;
  bool fiducial_localization_ok{false};
  std::string fiducial_error;
  // Recovery already ran for this restore.
  bool already_attempted{false};
  bool has_last_localization{false};
  LastLocalization last;
  // The saved waypoint still exists in the restored map.
  bool last_waypoint_in_map{false};
  // Wall-clock now, and how old a saved record may be. A limit of 0 or less
  // turns recovery off.
  long long now_ms{0};
  long long max_record_age_ms{86400000LL};
  // Time between the restore being scheduled and it running. A long delay (for
  // example waiting for the lease) means the robot may have been moved.
  long long restore_delay_ms{0};
  long long max_restore_delay_ms{60000LL};
};

struct WaypointRecoveryPlan {
  bool attempt{false};
  // Why recovery is attempted or skipped, for the log.
  std::string reason;
};

// After a restart the adapter re-uploads the map and tries fiducial localization.
// When that fails only because no fiducial is usable, it may try once more with the
// last saved waypoint as the initial guess. It never moves the robot.
WaypointRecoveryPlan DecideWaypointRecovery(const WaypointRecoveryInputs& in);

// Tracks whether the current localization came from an automatic guess that a
// person has not confirmed. While unconfirmed, GraphNav navigation commands are
// refused, because a wrong guess sends the robot to the wrong place.
//
// Restart recovery is bracketed by OnRecoveryStarted / OnRecoveryFinished. In
// between, navigation, spot.localize, operator confirmation and saving the
// localization are all refused, so nothing can act on (or overwrite) a pose
// that recovery has applied but not yet judged or cleared.
class LocalizationConfirmation {
 public:
  bool confirmed() const { return state_ == State::kConfirmed && !recovery_in_progress_; }
  bool recovery_in_progress() const { return recovery_in_progress_; }
  // Why it is unconfirmed, empty when confirmed.
  std::string pending_reason() const;

  // Fiducial localization failed during a restore and recovery is about to run.
  void OnRecoveryStarted();
  // An operator spot.localize is about to call SetLocalization. Opens the same
  // window as OnRecoveryStarted (close it with OnRecoveryFinished) but keeps the
  // current state, so a refused or failed attempt changes nothing. Returns false
  // with `refusal` set when another attempt is already running.
  bool TryBeginOperatorAttempt(std::string* refusal);
  // Recovery is done (attempted, skipped or failed). If nothing was applied the
  // robot is simply unlocalized and there is nothing to confirm.
  void OnRecoveryFinished();
  // Automatic restart recovery left the robot localized at `waypoint_id`.
  void OnAutomaticLocalization(const std::string& waypoint_id);
  // GraphNav applied a localization that failed its quality or ambiguity check
  // and it could not be cleared.
  void OnRejectedLocalization(const std::string& waypoint_id, const std::string& reason);
  // An operator ran spot.localize.confirm. Refused while recovery runs, and for a
  // rejected localization unless `force`. Returns false with `refusal` set.
  bool TryOperatorConfirm(bool force, std::string* refusal);
  // A person chose the waypoint with spot.localize and it was accepted.
  void OnOperatorLocalize();
  // Fiducial localization succeeded; the dock tag fixes the pose independently.
  void OnFiducialLocalization();
  // The robot has no localization any more (graph cleared or re-uploaded).
  void OnLocalizationCleared();

  // Empty when `action` (a navigation command) may run; otherwise a refusal.
  std::string NavigationBlockReason(const std::string& action) const;
  // Empty when spot.localize may run; otherwise a refusal.
  std::string LocalizeCommandBlockReason() const;
  // Whether the current localization may be saved for the next restart.
  bool AllowsPersist() const { return confirmed(); }

 private:
  enum class State { kConfirmed, kRecovering, kAutomatic, kRejected };
  State state_{State::kConfirmed};
  bool recovery_in_progress_{false};
  std::string waypoint_id_;
  std::string rejection_;
};

}  // namespace fsa
