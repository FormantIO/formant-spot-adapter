#pragma once

#include <string>
#include <vector>

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
//      operator who asserted the waypoint, or restart recovery's unverified
//      second pass at the last saved pose), when the ambiguity check was
//      requested and the pose moved at most max_correction (horizontal x/y and
//      yaw) from the guess. If that happened on the retry without visual
//      verification, the localization still needs spot.localize.confirm.
//   5. Every other quality result (poor point cloud match, poor gravity
//      alignment, bad height, unknown) is rejected.
// Trade-off of 4b: a small correction does not prove the point clouds matched;
// a poorly constrained match can also stay near the guess. It is accepted on a
// person's word that the robot is at that waypoint, or by restart recovery when
// the guess is the robot's last saved pose; the ambiguity check is the remaining
// automatic guard, and a recovery result is never trusted for navigation until
// it is confirmed (or replaced by a fiducial localization).

struct LocalizePose {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

// A waypoint joined to the guessed waypoint by one map edge, and where it sits in
// the guessed waypoint's frame. GraphNav can answer with a neighbor of the
// waypoint it was given; this lets the adapter still measure how far the result
// is from the guess.
struct NeighborWaypointTform {
  std::string waypoint_id;
  LocalizePose guess_tform_waypoint;
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
  // Not sent to the robot. Used only to measure the correction when GraphNav
  // localizes to one of these neighbors instead of `waypoint_id`.
  std::vector<NeighborWaypointTform> neighbors;
};

struct LocalizeAcceptancePolicy {
  // Rule 4b above. Operator command only.
  bool allow_skipped_with_small_correction{false};
  double max_correction_m{0.25};
  double max_correction_yaw_rad{0.17453292519943295};  // 10 degrees
  // The `basis` reported when rule 4b accepts a result.
  std::string small_correction_basis{"operator_small_correction"};
};

// What the request asked for and what GraphNav answered.
struct LocalizeResponseFacts {
  std::string status;
  std::string quality_check;
  bool suspected_ambiguity{false};
  bool ambiguity_check_requested{false};
  bool visual_verification_requested{false};
  // How far the result moved from the guess. Only known when GraphNav kept the
  // guessed waypoint as the reference. correction_m is horizontal (x, y) only;
  // height is reported separately and not bounded.
  bool has_correction{false};
  double correction_m{0.0};
  double correction_z_m{0.0};
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

// Horizontal (x, y) distance, height difference and absolute yaw difference
// between two poses in the same frame.
void PoseCorrection(const LocalizePose& guess, const LocalizePose& result, double* out_xy_m,
                    double* out_z_m, double* out_yaw_rad);

// SE3 helpers (quaternions are x, y, z, w). ComposePoses(a_tform_b, b_tform_c)
// is a_tform_c; InvertPose(a_tform_b) is b_tform_a.
LocalizePose ComposePoses(const LocalizePose& a_tform_b, const LocalizePose& b_tform_c);
LocalizePose InvertPose(const LocalizePose& a_tform_b);

// One map edge: `from_tform_to` is the `to` waypoint in the `from` waypoint's frame.
struct WaypointEdgeTform {
  std::string from_waypoint_id;
  std::string to_waypoint_id;
  LocalizePose from_tform_to;
};
// Every waypoint one edge away from `waypoint_id`, in its frame.
std::vector<NeighborWaypointTform> NeighborTforms(const std::string& waypoint_id,
                                                  const std::vector<WaypointEdgeTform>& edges);

// The robot pose GraphNav returned (`result_waypoint_tform_body`, relative to
// `result_waypoint_id`) expressed in the guessed waypoint's frame. False when the
// result waypoint is neither the guess nor one of `spec.neighbors`.
bool ResultInGuessFrame(const WaypointLocalizeSpec& spec, const std::string& result_waypoint_id,
                        const LocalizePose& result_waypoint_tform_body, LocalizePose* out);

// Everything localize_detail reports about one SetLocalization attempt.
struct LocalizeDetailFields {
  std::string status;
  std::string quality_check;
  // Empty when the result was rejected.
  std::string accepted_by;
  bool visual_verification{false};
  bool has_correction{false};
  double correction_xy_m{0.0};
  double correction_z_m{0.0};
  double correction_yaw_rad{0.0};
};
// "status=... quality=... accepted_by=... visual_verification=true|false
// correction_m=... correction_z_m=... correction_deg=..." (correction fields are
// "unknown" when GraphNav localized to a different waypoint than the guess).
std::string FormatLocalizeDetail(const LocalizeDetailFields& fields);

// Request specs. The operator command first asks for visual verification; if that
// fails (STATUS_VISUAL_ALIGNMENT_FAILED, nothing applied) it retries once without
// it, under the operator policy. Restart recovery makes one verified attempt.
WaypointLocalizeSpec OperatorLocalizeSpec(const std::string& waypoint_id,
                                          const WaypointLocalizeLimits& limits,
                                          bool verify_visual_refinement);
LocalizeAcceptancePolicy OperatorAcceptancePolicy();
// An accepted operator result still needs spot.localize.confirm when it was
// accepted only on a small correction without verified visual alignment.
bool OperatorAcceptNeedsConfirmation(const std::string& accepted_by, bool visual_verification);
struct LastLocalization;
// Restart recovery's initial guess: the saved offset from the saved waypoint, in
// the waypoint's frame (never odom, which a reboot resets). When the body height
// above ground is known both at save time and now, z moves by the difference, so
// a robot that was standing when saved and sits after the reboot is not guessed
// 0.3 m above itself (GraphNav reports QUALITY_CHECK_BAD_HEIGHT for a prior whose
// height is off). The correction is clamped to +-0.4 m (about the sit/stand
// difference), and is only made when both heights were read in a settled
// behavior state (BodyHeightUsable); the caller passes has_current_body_height
// false otherwise.
LocalizePose RecoverySeedPose(const LastLocalization& last, bool has_current_body_height,
                              double current_body_height_m);
// First pass verifies visual alignment. If GraphNav answers
// STATUS_VISUAL_ALIGNMENT_FAILED (nothing applied), the second pass repeats the
// same guess without verification under RecoveryAcceptancePolicy(true).
WaypointLocalizeSpec RecoveryLocalizeSpec(const LastLocalization& last,
                                          bool verify_visual_refinement = true,
                                          bool has_current_body_height = false,
                                          double current_body_height_m = 0.0);
// First pass: QUALITY_CHECK_SUCCESS, or SKIPPED with verified visual alignment.
// Unverified second pass: also SKIPPED when the ambiguity check passed and the
// result is within 0.25 m / 10 deg of the saved pose (on the saved waypoint or a
// neighbor), i.e. GraphNav found the robot where it was last known to be. That
// basis is reported as "saved_pose_unverified": it is close to accepting the
// saved pose itself, so an operator should check the pose before confirming.
// Every recovery result stays unconfirmed either way (LocalizationConfirmation).
LocalizeAcceptancePolicy RecoveryAcceptancePolicy(bool unverified_pass = false);
// True when the first (verified) pass failed only on visual alignment, so the
// unverified second pass should run.
bool RecoveryNeedsUnverifiedPass(bool accepted, bool applied, const std::string& status);

// Retrying a recovery attempt that applied nothing because the robot or GraphNav
// was not ready yet (impaired right after a reboot, for example a camera still
// initializing; the request aborted; the RPC or time sync failed). A clean "no
// match" (STATUS_FAILED, STATUS_VISUAL_ALIGNMENT_FAILED after both passes), an
// unknown waypoint, a missing map and a result that was applied but rejected are
// final. The same saved guess is reused each time; nothing is widened.
struct RecoveryRetryPolicy {
  int max_attempts{6};
  long long retry_interval_ms{10000};
};
struct RecoveryRetryDecision {
  bool retry{false};
  long long delay_ms{0};
  std::string reason;
};
// `attempts_made` counts the attempts so far, including the one that just failed.
// `status` is the GraphNav status name ("" when the request never got an answer).
RecoveryRetryDecision DecideRecoveryRetry(bool applied, const std::string& status,
                                          int attempts_made, const RecoveryRetryPolicy& policy);

// Commands that move the body (walks, docks, rotate, stand, sit, self-right).
// Any of them after the first recovery attempt drops pending retries: the saved
// guess may no longer describe where the robot is.
bool IsMotionCommand(const std::string& command);

// Whether a scheduled recovery retry may run now. The retry reuses the guess
// from attempt 1, so it is only valid while nothing could have moved the robot.
struct RecoveryRetryCheck {
  long long now_ms{0};
  long long first_attempt_ms{0};
  // Same limit as the restore delay: retries end this long after attempt 1.
  // 0 means no limit.
  long long max_window_ms{60000};
  // Bumped on every walk or dock command, teleop motion or button, and seen
  // GraphNav navigation or docking.
  unsigned long long motion_epoch_at_first{0};
  unsigned long long motion_epoch_now{0};
  // Odometry at attempt 1 and now.
  bool has_odom_at_first{false};
  LocalizePose odom_at_first;
  bool has_odom_now{false};
  LocalizePose odom_now;
  // A spot.map.* command is running or queued, or recording or map processing
  // is active.
  bool map_activity{false};
};
enum class RecoveryRetryGate { kRun, kWait, kDrop };
struct RecoveryRetryGateDecision {
  RecoveryRetryGate gate{RecoveryRetryGate::kDrop};
  std::string reason;
};
RecoveryRetryGateDecision CheckRecoveryRetry(const RecoveryRetryCheck& in);

// Parameters of the `spot.localize` command.
/**
 * Seconds (from config or an env var) as milliseconds, clamped.
 *
 * `getenv_double_or` and the JSON config both accept NaN, inf and absurd
 * values, and a double-to-int64 cast out of range is undefined behaviour. A
 * non-finite or negative value becomes 0, which every caller reads as "no
 * limit set", and anything past 100 days is capped.
 */
long long SecondsToMsClamped(double seconds);

struct LocalizeCommandParams {
  // Exactly one of these is set when `error` is empty.
  std::string waypoint_id;
  std::string waypoint_name;
  // Optional: the map the operator expects to be loaded.
  std::string map_uuid;
  WaypointLocalizeLimits limits;
  /**
   * `force=true`: relocalize even when the robot is already localized and
   * confirmed.
   *
   * Without it, a mistyped waypoint on a healthy robot can strand it: GraphNav
   * answers STATUS_OK with a quality or ambiguity rejection, and the recovery
   * path then clears the graph and re-uploads the map, so the good
   * (fiducial-derived) localization is destroyed and the robot is left
   * unlocalized until a fiducial or another `spot.localize` succeeds. The
   * robot still does not move either way; this is about not throwing away a
   * localization nobody asked to replace.
   */
  bool force{false};
  // Why the parameters were rejected, written for an operator. Empty when ok.
  std::string error;
};

// Accepts `<name>`, `name=<name>`, `waypoint_id=<id>`, plus optional
// `map_uuid=<uuid>`, `max_distance=<m>`, `max_yaw_deg=<deg>` and
// `force=true`, as key=value pairs or a JSON object.
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
  // Height of the body above the ground plane estimate ("gpe" frame) at save
  // time. Lets restart recovery correct the guess when the robot was standing
  // when saved and is sitting after the reboot (or the other way round).
  bool has_body_height{false};
  double body_height_m{0.0};
  // Wall-clock time of the save, milliseconds since the Unix epoch. While the
  // robot stays localized and odometry shows the body still, this is refreshed
  // about once a minute (see LastLocalizationThrottle), so it means "last time
  // the robot was seen, unmoved, at this pose": in practice the time since it
  // lost power. Restart recovery's age limit uses it.
  long long updated_ms{0};
  // When the saved pose last changed: a new waypoint or offset, or odometry
  // showing the body moved. Refreshes leave it alone. 0 means unknown (a file
  // written before this field existed); read back as updated_ms.
  long long moved_ms{0};
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

// Odometry, body height and behavior from one robot state read.
struct BodyPoseSample {
  bool has_odom{false};
  LocalizePose odom_tform_body;
  bool has_body_height{false};
  double body_height_m{0.0};
  // "not_ready", "transition", "standing", "stepping" or "unknown".
  std::string behavior_state;
};

// Whether a body height read in this behavior state describes a settled pose
// (sitting with motors off, or standing still). Mid sit/stand transition or
// while stepping it does not, and is not used.
bool BodyHeightUsable(const std::string& behavior_state);

// Empty when `now` is within `max_m` horizontally and `max_yaw_rad` in yaw of
// `before` (both odom_tform_body); otherwise how far the body moved. Height is
// ignored: sitting down or standing up is not a move.
std::string BodyMovedReason(const LocalizePose& before, const LocalizePose& now,
                            double max_m = 0.1, double max_yaw_rad = 0.08726646259971647);

// Decides when a fresh localization is worth writing to disk. GraphNav reports
// localization every poll. A write is due:
//   - kChanged: first record, a different waypoint or map, the pose moved 0.5 m
//     or turned 15 deg, or odometry shows the body moved 0.1 m / 5 deg since the
//     last write. Never more often than `min_interval_ms`. The record's moved_ms
//     becomes the write time.
//   - kRefresh: `refresh_interval_ms` since the saved timestamp and odometry has
//     shown the body still (within 0.1 m / 5 deg) since the last write or for the
//     whole refresh interval. moved_ms is kept. No odometry, no refresh.
// A refresh interval of 0 turns refreshes off. A saved stamp ahead of the clock
// (the clock was set back) is rewritten as kChanged.
class LastLocalizationThrottle {
 public:
  enum class Reason { kNone, kChanged, kRefresh };

  explicit LastLocalizationThrottle(long long min_interval_ms = 5000,
                                    double min_move_m = 0.5,
                                    double min_turn_rad = 0.2617993877991494 /* 15 deg */,
                                    long long refresh_interval_ms = 60000);

  // Call on every poll, before Decide.
  void ObserveOdom(long long now_ms, bool has_odom, const LocalizePose& odom_tform_body);
  Reason Decide(const LastLocalization& current, long long now_ms) const;
  bool ShouldPersist(const LastLocalization& current, long long now_ms) const {
    return Decide(current, now_ms) != Reason::kNone;
  }
  // `rec.moved_ms` is what was written.
  void MarkPersisted(const LastLocalization& rec, long long now_ms);
  // Starts from what is already on disk. The same record is rewritten once its
  // own timestamp is due a refresh (and the body is seen still); a different one
  // is written at once.
  void Seed(const LastLocalization& rec);
  // moved_ms of the last record written or seeded.
  long long last_moved_ms() const { return last_.moved_ms; }

 private:
  long long min_interval_ms_;
  double min_move_m_;
  double min_turn_rad_;
  long long refresh_interval_ms_;
  bool has_last_{false};
  LastLocalization last_;
  // When this process last wrote (0: not yet), for min_interval_ms.
  long long last_write_ms_{0};
  // Timestamp of the record on disk, for refresh_interval_ms.
  long long last_stamp_ms_{0};
  // Latest odometry, and the stretch over which it has been still.
  bool has_odom_{false};
  LocalizePose odom_;
  LocalizePose still_anchor_;
  long long still_since_ms_{0};
  // Odometry when the last write happened.
  bool has_odom_at_write_{false};
  LocalizePose odom_at_write_;
};

// Whether a failed nearest-fiducial localization leaves anything for a waypoint
// guess to do. Empty when restart recovery may try the saved waypoint; otherwise
// why not.
//
// FIDUCIAL_INIT_NEAREST with no tag in view answers plain STATUS_FAILED ("failed
// to localize for some other reason"); the proto documents NO_MATCHING_FIDUCIAL,
// FIDUCIAL_TOO_FAR_AWAY, FIDUCIAL_TOO_OLD and FIDUCIAL_POSE_UNCERTAIN only for
// use_fiducial_id. So the "no usable fiducial" statuses cannot be the gate: Walt
// rebooted 2 m from his dock on Nov 12 got STATUS_FAILED and recovery was
// skipped. Only failures that a waypoint guess cannot change block recovery:
// no map loaded, or a map recorded with different sensors. A robot that is
// impaired right after a reboot is tried (and retried, see DecideRecoveryRetry).
std::string FiducialFailureBlocksWaypointRecovery(const std::string& error_text);

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
// When that fails for any reason a waypoint guess could fix (see
// FiducialFailureBlocksWaypointRecovery), it tries the last saved waypoint as the
// initial guess. It never moves the robot.
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
  // `note` (for example why the evidence is weaker) is shown in pending_reason.
  void OnAutomaticLocalization(const std::string& waypoint_id, const std::string& note = "");
  // A scheduled recovery retry is about to call SetLocalization. Same window as
  // TryBeginOperatorAttempt: the current state is kept, so a retry that finds
  // the robot already localized and stops changes nothing.
  bool TryBeginRecoveryRetry(std::string* refusal) { return TryBeginOperatorAttempt(refusal); }
  // An operator spot.localize was accepted on weaker evidence (for example a small
  // correction after visual alignment failed). It needs spot.localize.confirm.
  void OnUnverifiedOperatorLocalization(const std::string& waypoint_id, const std::string& why);
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
  // Why an accepted localization still needs confirming; empty for restart recovery.
  std::string unverified_note_;
};

}  // namespace fsa
