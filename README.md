# formant-spot-adapter

C++ adapter for Spot + Formant Agent teleoperation without ROS.

This repository is maintained by Formant. It is not affiliated with or endorsed by
Boston Dynamics.

## Repository Basics

- License: Apache-2.0
- Contributing: [`CONTRIBUTING.md`](CONTRIBUTING.md)
- Security reporting: [`SECURITY.md`](SECURITY.md)
- Conduct expectations: [`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md)

## What It Does

- Teleop motion from Formant twist control, with optional adapter-side gamepad (`Joy`) mapping.
- Stand/Sit, E-Stop, Recover, Dock, locomotion mode, and explicit arm reset controls.
- Image publishing from hand, left, right, and back cameras.
- Dock readiness status publishing.
- Stateful mode publishing (walk/stairs/crawl).
- Formant command handling for Jetson reboot and robot body reboot.
- Manual camera calibration command.

## Formant I/O Reference

### Streams received by adapter (Formant -> adapter)

| Stream name (default) | Type | Used for | Notes |
|---|---|---|---|
| `Joystick` | `Twist` | Drive + body pitch | `linear.x` forward/back, `linear.y` strafe, `angular.z` yaw, `angular.y` body pitch |
| `Gamepad` | `Joy` | Optional gamepad bridge | Default joy input stream. Adapter maps controller axes/buttons onto the same motion + action handlers used by `Joystick`/button streams. Default face-button mapping is `A`=`Stand`, `B`=`Sit`, `X`=`Walk`, `Y`=`Dock` |
| `Buttons` | `Bitset` | Multi-button input | Recognized keys: `Stand`, `Sit`, `E-Stop`, `Recover`, `Dock`, `Walk`, `Stairs`, `Crawl`, `Reset Arm` |
| `Stand` | `Bitset`/boolean-to-device | Stand command | Any `true` bit triggers stand |
| `Sit` | `Bitset`/boolean-to-device | Sit command | Any `true` bit triggers sit |
| `E-Stop` | `Bitset`/boolean-to-device | Safe power-off command | Requires lease |
| `Recover` | `Bitset`/boolean-to-device | Self-right command | Requires lease |
| `Walk` | `Bitset`/boolean-to-device | Set locomotion mode | Does not stand robot |
| `Stairs` | `Bitset`/boolean-to-device | Set locomotion mode | Does not stand robot |
| `Crawl` | `Bitset`/boolean-to-device | Set locomotion mode | Does not stand robot |
| `Reset Arm` | `Bitset`/boolean-to-device | Send an explicit arm stow command | Requires lease to execute immediately |
| `Dock` | `Bitset`/boolean-to-device | Start autodock | Uses configured dock id or discovery |

Additional control channels (not streams):
- Teleop heartbeat from Formant agent controls session active/inactive behavior.
- Command channel supports `spot.jetson.reboot`, `spot.robot.reboot`, `spot.camera.calibrate`,
  `spot.stand`, `spot.sit`, `spot.recover`, `spot.dock`, `spot.undock`, `spot.reset_arm`,
  `spot.graphnav.*`, `spot.map.*`, and `spot.waypoint.*`.

### Streams published by adapter (adapter -> Formant)

| Stream name (default) | Type | Content |
|---|---|---|
| `spot.hand.image` | Image (JPEG) | Hand camera (`hand_color_image`) |
| `spot.left.image` | Image (JPEG) | Left fisheye (`left_fisheye_image`) |
| `spot.right.image` | Image (JPEG) | Right fisheye (`right_fisheye_image`) |
| `spot.back.image` | Image (JPEG) | Back fisheye (`back_fisheye_image`) |
| `spot.status` | Bitset | `Has lease`, `Robot available`, `Robot degraded`, `Teleop running`, `Teleop active`, `Docking` |
| `spot.connection` | Text (JSON) | Spot connection health (`state`, `connected`, `degraded_non_estop`, `degraded_reason`, reconnect/attempt timestamps, last error) |
| `spot.connection_state` | Text | One-word connection state: `disconnected`, `connecting`, `connected` |
| `spot.localization` | Text (JSON) | GraphNav localization status (`localized`, `waypoint_id`, `error`) |
| `spot.localization.graphnav` | Localization | Formant typed localization stream with a live occupancy-grid patch in the GraphNav seed frame; intended for the Formant localization viewer |
| `spot.localization.graphnav.image` | Image (JPEG) | Rendered local GraphNav occupancy/localization overlay image, published at stable output FPS from cached frames |
| `spot.localization.graphnav.global` | Localization | Formant typed global GraphNav localization stream combining live seed-frame robot pose with the stitched saved-site map; default 2 Hz |
| `spot.localization.graphnav.global.image` | Image (JPEG) | Rendered global GraphNav map image with waypoint overlays and live robot pose when available |
| `spot.localization.graphnav.global.image.meta` | Text (JSON) | Companion metadata for the rendered global GraphNav image including draw rect, render scale, resolution, and `seed_tform_grid` for click-to-go UIs |
| `spot.map.graphnav` | Localization payload carrying map-only data | Dedicated stitched GraphNav map payload for future top-level map transport |
| `spot.graphnav.metadata` | Text (JSON) | Full waypoint/edge/object metadata for GraphNav maps. Useful for diagnostics, but may be too large for some iframe text-query paths; prefer a smaller UI-specific overlay stream if needed |
| `spot.graphnav.overlay` | Text (JSON) | Small UI-safe waypoint overlay stream for the active GraphNav map. Publishes saved/operator waypoint names only (`map_uuid`, `current_waypoint_id`, `dock_waypoint_id`, `waypoints`). |
| `spot.nav.state` | Text (JSON) | Active GraphNav target/mode/map context plus lifecycle/terminal result and current seed-frame robot pose for external map UIs |
| `spot.can_dock` | Bitset | `Can dock` (published at 0.2 Hz / every 5s) |
| `spot.mode_state` | Bitset | `Walk`, `Stairs`, `Crawl` |
| `spot.commanded_motion_mode` | Text | One-word commanded locomotion mode: `walk`, `stairs`, `crawl` |
| `spot.docking_state` | Text | One-word docking state: `unknown`, `docked`, `docking`, `undocked`, `undocking` |
| `spot.waypoints` | Text | Newline-separated waypoint names for the active map |
| `spot.waypoint.current` | Text | Waypoint name if robot is within 1 ft of a saved waypoint, else empty string; checks every 1s, publishes every 10s |
| `spot.maps` | Text | Newline-separated saved map IDs |
| `spot.map.current` | Text | Active/current map ID (`none` if unset); publishes every 10s and on change |
| `spot.map.default` | Text | Default map ID (`none` if unset); publishes every 10s and on change |
| `spot.robot_state.power` | Text (JSON) | Motor power state, estop summary, battery availability |
| `spot.motor_power_state` | Text | One-word motor power state: `off`, `on`, `powering_on`, `powering_off`, `error`, `unknown` |
| `spot.robot_state.battery` | Numeric | Battery percentage (0-100) when available |
| `spot.robot_state.body_pitch_rad` | Numeric | Measured body pitch in radians (`odom` frame) |
| `spot.behavior_state` | Text | One-word behavior state: `not_ready`, `transition`, `standing`, `stepping`, `unknown` |
| `spot.shore_power_state` | Text | One-word shore-power state: `on`, `off`, `unknown` |
| `spot.faults.system` | Text (JSON) | Active system faults summary |
| `spot.faults.behavior` | Text (JSON) | Active behavior faults summary |
| `spot.faults.service` | Text (JSON) | Active service faults summary |
| `spot.fault.events` | Text | Simplified fault event feed (`FAULT OPEN/CHANGED/CLEARED` only) |
| `spot.nav.feedback` | Text (JSON) | Low-level GraphNav command status/route-following diagnostics |
| `spot.adapter.log` | Text | Buffered adapter event log batches (1 Hz) |

Per-stream enable/disable is configured in `config/formant-spot-adapter.json` via the
`streamControls` array. When a stream is disabled there, the adapter suppresses publication, and
for high-cost streams such as cameras/localization/map imagery it also avoids starting the
associated worker loops entirely. If you rename a stream from its default, use the renamed stream
value in `streamControls`.

## Runtime Behavior

- Starts passive. Lease is acquired only while teleop session is active.
- Adapter stays online if Spot is unavailable and retries robot connection in the background.
- While Spot is unavailable, robot actions are rejected and availability is published on `spot.connection`.
- Adapter applies soft non-E-Stop recovery gating: teleop motion/dock commands are blocked only for critical/unclearable robot faults or motor power error.
- For GraphNav nav commands, adapter prechecks localization and attempts fiducial relocalization before failing.
- `spot.localization.graphnav` is a live local terrain/occupancy patch around the robot, not a
  stitched full-site map. See
  [`docs/formant-localization-map-analysis.md`](docs/formant-localization-map-analysis.md)
  for the implementation notes and next steps.
- `spot.localization.graphnav.global.image`, `spot.localization.graphnav.global.image.meta`,
  `spot.graphnav.overlay`, and `spot.nav.state` form the core backend contract for an external
  GraphNav map UI.
- `spot.graphnav.metadata` remains available as a richer diagnostic stream, but iframe-based UIs
  should treat it as optional unless they use a transport path that safely handles larger JSON
  payloads. Production map UIs should prefer `spot.graphnav.overlay`.
- `ui/map-navigation` contains a GitHub Pages-hosted custom module that consumes that backend
  contract from inside a Formant iframe, renders the global map image, prefers waypoint-first
  navigation, and issues `spot.waypoint.goto` / `spot.graphnav.goto_pose` commands plus
  `spot.graphnav.cancel` (hold-position override), `spot.undock`, and
  `spot.return_and_dock` actions.
- `spot.localization.graphnav.image` is a rendered 16:9 visualization of the same local patch with a robot
  footprint, heading arrow, scale bar, and status HUD. The adapter renders only when the Spot data
  changes, then republishes the cached JPEG at a stable output FPS. For camera-like playback in
  Formant, the `spot.localization.graphnav.image` stream also needs Formant-side video encoding/realtime
  stream configuration; otherwise the agent treats it as a throttled telemetry image stream.
- `spot.left.image`, `spot.right.image`, and `spot.back.image` use the same cached-frame publish
  pattern: the adapter polls Spot at a lower configured rate, then republishes the latest JPEG at a
  stable output FPS. The right camera is rotated 180 degrees by default.
- `spot.front.image` is rotated 90 degrees clockwise by default. Set
  `frontImageRollDegrees` only to override that default for a specific robot.
- If lease is already held, adapter attempts takeover (`TakeLease`) to recover from stale owners.
- On teleop inactivity: sends zero velocity, then returns lease unless GraphNav nav is active.
- Heartbeat timeout behavior is zero-velocity only (no auto-sit).
- The adapter does not automatically command the arm on lease acquire, teleop activity, lease return, or shutdown.
- Dock command runs in background loop with status/error logging.
- GraphNav maps are persisted under `graphnavStoreDir` and can be loaded/deleted via commands.
- Waypoint aliases are map-scoped (`name -> waypoint_id`) and published on `spot.waypoints`.
- Teleop motion uses held-command resend (smoother than packet-by-packet pulse behavior), deadband
  rescaling, response curves, and adapter-side slew limiting.

## GraphNav Commands

All commands are namespaced as `spot.*` and accept optional text parameters.

Parameter text is `key=value` pairs separated by `,` `;` or newlines, or a JSON object. A
command's main parameter can also be given as a bare value (`kitchen` instead of
`name=kitchen`). A parameter that is not present is treated as missing: text meant for one
parameter is never used as the value of another.

Additional non-GraphNav command-channel actions:
- `spot.stand`: stand command; command path does not require active teleop heartbeat.
- `spot.sit`: sit command; command path does not require active teleop heartbeat.
- `spot.recover`: self-right command; command path does not require active teleop heartbeat.
- `spot.dock`: run autodock procedure; command path does not require active teleop heartbeat.
  - Refused before the robot moves unless the dock's AprilTag (tag id = dock id) was detected
    within `dock_preflight_max_fiducial_age_sec` (default 5 s) and is within
    `dock_preflight_max_distance_m` (default 5 m). It keeps looking for up to
    `dock_preflight_wait_sec` (default 4 s) before refusing, so a tag that is briefly out of view
    after a turn does not cost the dock. The refusal is logged with the reason, the tag's age,
    distance and bearing. Set the age limit to 0 to turn the preflight off. The same
    preflight runs for `spot.return_and_dock` (after it reaches the dock waypoint) and the teleop
    dock button. Every allowed dock also logs these measurements (`preflight ok ...`).
- `spot.undock`: undock from the current dock and move to prep pose; command path does not require active teleop heartbeat.
  - No parameters.
  - Fails if the robot is not currently docked.
- `spot.return_and_dock`: navigate to saved dock waypoint for active map, then run dock.
  - No parameters.
  - Fails if no dock waypoint is saved yet for the active/default map.
  - Successful manual `spot.dock` learns/saves current localized waypoint as dock waypoint. If
    the `dock_home_alias` waypoint name (default `charging dock`, matched ignoring case) is missing
    or was on the old dock waypoint, it moves to the new one, so
    `spot.waypoint.goto name="charging dock"` and `spot.return_and_dock` keep going to the same
    place. A name that points somewhere else is left alone and a warning is logged. Set
    `dock_home_alias` to `""` in the JSON config to turn the sync off.
- `spot.rotate_left`: rotate left in place by a requested angle. Parameter: `degrees`.
- `spot.rotate_right`: rotate right in place by a requested angle. Parameter: `degrees`.
- `spot.reset_arm`: explicit arm reset/stow; command path does not require active teleop heartbeat.

- `spot.map.create`: create a new empty map context and make it active.
  - Parameter examples: `name=warehouse_a`
- `spot.map.load`: load a saved map into GraphNav.
  - Parameter examples: `map_id=warehouse_a` or `{"map_id":"warehouse_a"}`
- `spot.map.set_default`: set default map ID in adapter state.
  - Parameter examples: `map_id=warehouse_a`
- `spot.map.delete`: delete saved map and map-scoped aliases.
  - Parameter examples: `map_id=warehouse_a`
- `spot.map.start_mapping`: start GraphNav recording.
- `spot.map.stop_mapping`: stop recording and persist current map to disk.
- `spot.waypoint.save`: create waypoint at current pose and bind alias.
  - Parameter examples: `name=dock_entry`
- `spot.waypoint.update`: same as save, but intended to rebind an existing alias name.
  - Parameter examples: `name=dock_entry`
- `spot.waypoint.delete`: delete alias mapping (does not mutate GraphNav graph topology).
  - Parameter examples: `name=dock_entry`
- `spot.waypoint.goto`: resolve alias/label or accept `waypoint_id=<id>` directly; optional `map_uuid=<uuid>` validation is supported for UI-issued commands.
- `spot.waypoint.goto_straight`: same as above, but uses straight-line-biased travel params.
- `spot.graphnav.goto_pose`: accept `map_uuid=<uuid>, x=<seed_x>, y=<seed_y>` and optional
  `yaw_rad=<rad>` / `yaw_deg=<deg>` to navigate to a seed-frame target on the active map.
- `spot.graphnav.goto_pose_straight`: same as `spot.graphnav.goto_pose`, with straight-line-biased travel params.
  - Parameter examples: `name=dock_entry`
- `spot.graphnav.cancel`: override any active GraphNav route with a new goal at the robot's
  current localized pose, effectively acting as a hold-position override rather than a hard stop.
  - No parameters.
  - Requires a valid current localization and active lease.

- `spot.localize`: localize on the active map from "the robot is at this waypoint", without
  using fiducials. Use it when Spot is unlocalized away from the dock, for example after a reboot
  where the only fiducial in the map (the dock tag) is out of view.
  - Parameter examples: `kitchen`, `name=lights_out_04`, `waypoint_id=<id>`
  - Optional: `map_uuid=<uuid>` (rejected if it is not the active map), `max_distance=<m>`
    (search radius around the waypoint, default 0.5, max 1.5), `max_yaw_deg=<deg>` (default 20,
    max 45).
  - Calls GraphNav SetLocalization with `FIDUCIAL_INIT_NO_FIDUCIAL`, an initial guess of identity
    `waypoint_tform_body` at that waypoint, the current `ko_tform_body`, `do_ambiguity_check`, and
    `refine_with_visual_features` with `verify_refinement_quality`. If visual alignment fails
    (`STATUS_VISUAL_ALIGNMENT_FAILED`, nothing applied) it retries once without verification.
  - Acceptance. GraphNav applies a `STATUS_OK` localization even when its own point-cloud quality
    check fails, and for a waypoint guess on Spot SDK 5.1 it can skip that check entirely
    (`QUALITY_CHECK_SKIPPED`, seen on the robot). The 5.1 API has no request field that forces the
    check to run: there is no refine algorithm option, and the only other refinement,
    `refine_fiducial_result_with_icp`, needs a fiducial. So the adapter accepts a result when:
    - the status is `STATUS_OK` and GraphNav reports no suspected ambiguity, and
    - the quality check is `QUALITY_CHECK_SUCCESS`, or
    - it is `QUALITY_CHECK_SKIPPED` and verified visual alignment passed, or
    - (this command only, on the retry) it is `QUALITY_CHECK_SKIPPED`, the ambiguity check was
      requested, GraphNav kept the guessed waypoint, and it moved the pose at most 25 cm and
      10 deg from the guess.
    Any other quality result is rejected. Trade-off of the last rule: a small correction does not
    prove the point clouds matched, since a poorly constrained match can also stay near the guess.
    It rests on the operator's statement that the robot is at that waypoint, with the ambiguity
    check as the remaining automatic guard, and restart recovery never uses it. The log and
    `localize_detail` name the rule that accepted (`accepted_by=quality_check`,
    `visual_verification` or `operator_small_correction`) and the correction.
  - A rejected result that GraphNav applied is cleared the same way restart recovery does
    (below). If it cannot be cleared, navigation stays blocked until it is confirmed or replaced.
  - No lease needed for the localization itself and the robot does not move. Rejected while
    navigating, mapping, while the map is still being restored, or while restart recovery runs.
    The result is published on `spot.nav.state`.
- `spot.localize.confirm`: an operator confirms the robot's current localization after checking
  its position on the map. Clears the navigation block described below. Fails if the robot is not
  localized or restart recovery is still running. A localization that failed GraphNav's own
  checks and could not be cleared is only confirmed with `force=true`.

Restart localization recovery:
- The adapter saves the last confirmed localization (waypoint plus the robot's offset from it) to
  `<graphnav_store_dir>/last_localization.txt`, at most every 5 s and only when the waypoint
  changes or the robot moves 0.5 m / turns 15 deg. Nothing is saved while a restore is pending, a
  map is recording, or the localization is unconfirmed.
- The map restore runs at adapter start. It also runs while the adapter keeps running after a
  reconnect (a Spot body reboot drops the connection) when Spot's GraphNav map stats show an empty
  graph on two checks 5 s apart within 2 minutes of the reconnect and no other client holds the
  body lease. A graph another client cleared on purpose without a reconnect is left alone.
- If the restore's nearest-fiducial localization fails because no fiducial is usable
  (`STATUS_NO_MATCHING_FIDUCIAL`, `FIDUCIAL_TOO_FAR_AWAY`, `FIDUCIAL_TOO_OLD`,
  `FIDUCIAL_POSE_UNCERTAIN`), the adapter tries SetLocalization once at the saved waypoint and
  offset, with visual refinement verification on and without `ko_tform_body` (the odom frame from
  before the reboot no longer exists). It accepts only `QUALITY_CHECK_SUCCESS`, or
  `QUALITY_CHECK_SKIPPED` with verified visual alignment, and never a suspected ambiguity. A map
  without usable visual data therefore cannot be recovered automatically; use `spot.localize`. It is skipped when the saved record is for another map, its
  waypoint is gone, it is older than `localize_recovery_max_record_age_sec` (default 86400, 0 turns
  recovery off), or the restore ran more than `localize_recovery_max_restore_delay_sec` (default
  60) after it could first start (scheduled, or Spot's API answering after a power cycle,
  whichever is later), for example because another client held the lease.
- From the moment the restore's fiducial localization fails until recovery has finished,
  navigation, `spot.localize`, `spot.localize.confirm` and saving the localization are refused, so
  nothing can act on a pose recovery has applied but not yet accepted or cleared. The same window
  covers each `spot.localize` attempt.
- Navigation is also refused while a map restore is pending ("map ... is still being restored to
  the robot" in `spot.adapter.log`). The restore retries every 5 s and stays pending while the map
  file under `<graphnav_store_dir>/maps/<map_id>` cannot be read, the upload fails, or another
  client (for example the tablet) holds the body lease. To clear it, release the lease on the
  other client, or fix or replace the map files, or load a different map with `spot.map.load`.
  If none of these works, restart the adapter.
- An accepted recovery leaves the localization unconfirmed (`localize_confirmed=false` on
  `spot.nav.state`). A recovery result that fails the quality or ambiguity check is cleared:
  GraphNav has no clear-localization call, so the adapter runs ClearGraph (which clears
  localization) and re-uploads the same map, leaving the robot on its map but unlocalized. This
  needs the body lease; if it cannot be taken, the localization stays and is marked unconfirmed
  (confirming it then needs `force=true`). If the clear works but the re-upload fails, the normal
  restore retries the upload without running recovery again.
- While the localization is unconfirmed, GraphNav navigation (`spot.waypoint.goto`,
  `spot.waypoint.goto_straight`, `spot.graphnav.goto_pose`, `spot.graphnav.goto_pose_straight`,
  `spot.return_and_dock`, the teleop return-and-dock) is refused with a reason in the log. Each
  refused request first tries a nearest-fiducial localization, so a robot that can see the dock tag
  is unblocked automatically. `spot.localize.confirm`, a successful `spot.localize`, or any
  successful fiducial localization also unblock it. `spot.graphnav.cancel` (hold position) is not
  blocked.
- Recovery never moves the robot. If it is skipped or fails, drive the robot to a known waypoint
  and use `spot.localize`, or drive it back in view of the dock.

Note: `spot.waypoint.save`/`spot.waypoint.update` can be called while not actively mapping; the
adapter runs a temporary start/create/stop recording sequence.

Command responses:
- Responses are terminal: success/failure is returned after the action reaches a terminal state.
- Long-running commands (`spot.camera.calibrate`, `spot.waypoint.goto`, `spot.dock`,
  `spot.undock`, `spot.return_and_dock`, `spot.graphnav.cancel`) keep the response pending until
  completion.

## Configuration Model

- Local runtime config: `config/formant-spot-adapter.json` (materialized from the example on first run/deploy; not tracked)
- Secrets: `config/formant-spot-adapter.env` (`SPOT_USERNAME`, `SPOT_PASSWORD`) (materialized from the example on first run/deploy; not tracked)
- Tracked templates: `config/formant-spot-adapter.json.example`, `config/formant-spot-adapter.env.example`

Important:
- `config/formant-spot-adapter.json.example` is the canonical baseline configuration for this repository.
- Current runtime `load_config()` takes secrets plus `FORMANT_AGENT_TARGET` from env and all other runtime params from JSON.
- If you want to change stream names, arm pose, docking config, or camera config, update JSON.

### Teleop Tuning

Formant joystick UI modules publish normalized `Twist` values, normally in `[-1, 1]`, with the
teleop view owning which UI axis maps to each twist field. The adapter treats those values as
unitless operator intent and maps them to Spot body velocity using:

```text
clamp [-1, 1] -> deadband with rescale -> response curve -> max velocity -> slew limit
```

The deadband is rescaled rather than simply zeroed, so motion ramps continuously from zero instead
of jumping to the deadband percentage. Slew limits are applied in the adapter's resend loop before
each Spot velocity command. A `teleopIdleTimeoutMs` default of `1000` ms is used because the
Formant teleop joystick can debounce repeated identical values; normal joystick release still sends
a zero command immediately.

The JSON config values are the baseline. Formant application configuration may override the runtime
teleop tuning without restarting the adapter. Missing application config keys leave the JSON/default
values in effect. The adapter reads application configuration at startup and again when the Formant
agent gRPC streams reconnect, which is how agent configuration changes propagate after the agent
restarts. Values are parsed into typed tuning fields, validated against conservative bounds, and
invalid values are ignored.

| Application config key | Default | Meaning |
|---|---:|---|
| `spot.teleop.max_vx_mps` | `0.8` | Max forward/back velocity sent to Spot |
| `spot.teleop.max_vy_mps` | `0.5` | Max strafe velocity sent to Spot |
| `spot.teleop.max_wz_rps` | `1.2` | Max yaw rate sent to Spot |
| `spot.teleop.max_body_pitch_rad` | `0.25` | Max body pitch offset |
| `spot.teleop.deadband` | `0.08` | Normalized input deadband before rescaling |
| `spot.teleop.idle_timeout_ms` | `1000` | Stale input timeout before zeroing held motion |
| `spot.teleop.translation_response_curve` | `1.4` | Curve exponent for forward/strafe axes |
| `spot.teleop.rotation_response_curve` | `1.2` | Curve exponent for yaw/body-pitch axes |
| `spot.teleop.linear_accel_limit_mps2` | `0.8` | Forward/back acceleration limit, `0` disables |
| `spot.teleop.strafe_accel_limit_mps2` | `0.6` | Strafe acceleration limit, `0` disables |
| `spot.teleop.angular_accel_limit_rps2` | `1.5` | Yaw acceleration limit, `0` disables |
| `spot.teleop.body_pitch_rate_limit_radps` | `0.5` | Body-pitch rate limit, `0` disables |

For the optional `Gamepad`/`Joy` bridge, these application config keys override the adapter-side
browser gamepad mapping:

| Application config key | Default |
|---|---:|
| `spot.teleop.joy_axis_forward` | `1` |
| `spot.teleop.joy_axis_strafe` | `0` |
| `spot.teleop.joy_axis_yaw` | `2` |
| `spot.teleop.joy_axis_body_pitch` | `3` |
| `spot.teleop.joy_axis_forward_inverted` | `true` |
| `spot.teleop.joy_axis_strafe_inverted` | `true` |
| `spot.teleop.joy_axis_yaw_inverted` | `true` |
| `spot.teleop.joy_axis_body_pitch_inverted` | `true` |
| `spot.teleop.joy_button_stand` | `0` |
| `spot.teleop.joy_button_sit` | `1` |
| `spot.teleop.joy_button_walk` | `2` |
| `spot.teleop.joy_button_dock` | `3` |
| `spot.teleop.joy_button_reset_arm` | `-1` |
| `spot.teleop.joy_button_recover` | `-1` |
| `spot.teleop.joy_button_stairs` | `-1` |
| `spot.teleop.joy_button_crawl` | `-1` |
| `spot.teleop.joy_button_estop` | `-1` |

## Deployment Modes

This repository supports two deployment modes:

- Host service mode: build and run the adapter directly on the device with the systemd-backed scripts in `scripts/`.
- Docker mode: build and run the adapter container from the checked-out repository on the device with the Docker scripts in `scripts/`.

Only one deployment mode should be active on a device at a time.

## Quickstart (Jetson / Ubuntu)

This repository is intended to run on NVIDIA Jetson devices with an Ubuntu-based
userspace. Other Linux environments are out of scope for this repository and
are not documented or supported by the bootstrap or deployment scripts.

1. Install deps and build:

```bash
./scripts/bootstrap_ubuntu.sh
./scripts/build.sh
```

`./scripts/bootstrap_ubuntu.sh` installs the Ubuntu packages and ensures a
local Spot C++ SDK checkout is available at `third_party/spot-cpp-sdk`. By
default the setup is pinned to Spot C++ SDK `v5.1.0`. If a local archive such
as `bosdyn/spot-cpp-sdk-5.1.0.zip` is present, the setup uses it; otherwise it
clones the pinned upstream tag from Boston Dynamics.

The Formant protobuf definitions used by this adapter are already vendored in
`proto/protos/`, so no additional Formant dependency fetch is required during
setup or build. Maintainers can refresh that vendored snapshot with
`./scripts/sync_formant_protos.sh` when intentionally updating to a newer
public Formant proto revision.

2. Run directly once to materialize local config files:

```bash
./scripts/run.sh
```

On first run, the script creates local `config/formant-spot-adapter.env` and
`config/formant-spot-adapter.json` files from the tracked example files if they
do not already exist, then exits if credentials are still set to the example
placeholders.

3. Edit the local config files as needed:

```bash
nano config/formant-spot-adapter.env
nano config/formant-spot-adapter.json
```

At minimum, set `SPOT_USERNAME` and `SPOT_PASSWORD` in
`config/formant-spot-adapter.env` before rerunning `./scripts/run.sh`. Change
`config/formant-spot-adapter.json` if `spotHost` or other runtime defaults need
to differ from the example values.

4. Smoke-test locally or deploy as a persistent service:

```bash
./scripts/run.sh
```

or:

```bash
./scripts/deploy.sh
```

`./scripts/deploy.sh` uses the same local env/config files, creates them from
the tracked examples if they are missing, and refuses to start the service with
placeholder credentials.

## Docker Deployment

The supported container workflow is to build and manage the adapter directly
from a checked-out copy of this repository on the target device.

The intended topology is:

- Formant Agent in its own container on the same host
- This adapter in a separate container on the same host
- Both containers using host networking
- The host still having LAN reachability to the Spot robot

Container deployment notes:

- With `network_mode: host`, the default Formant agent target remains `localhost:5501`.
- The container runtime mounts `./config` and `./data` into the adapter container.
- `./data` must be persistent if you want GraphNav state and saved map artifacts to survive container recreation.
- Container logs go to `docker logs`; the container path does not write repo log files by default.
- `spot.jetson.reboot` is not supported in container deployment because it is a host reboot command.
- The current Docker build follows the repository's Jetson/aarch64-focused build
  assumptions. Build it on the same class of target machine that will run it.
- Repo-local Docker management scripts use plain `docker`.
- A sample [`compose.yaml`](compose.yaml) is included only as a reference for larger setups; it is not the canonical management path for this repository.

Prerequisites:

- Docker installed on the target device
- This repository checked out on the target device
- The Formant Agent container running separately on the same host with host networking
- `config/formant-spot-adapter.env` populated with real `SPOT_USERNAME` and `SPOT_PASSWORD`
- `config/formant-spot-adapter.json` populated with the correct `spotHost`

Canonical Docker deployment lifecycle from the checked-out repository:

1. Build, stop the host service if present, and deploy the Docker container:

```bash
./scripts/docker_deploy.sh
```

2. Watch logs:

```bash
./scripts/docker_logs.sh
```

3. Check container status:

```bash
./scripts/docker_status.sh
```

4. Stop and remove the adapter container:

```bash
./scripts/docker_uninstall.sh
```

5. Pull latest from git, rebuild, and redeploy:

```bash
./scripts/docker_update.sh
```

The `docker_deploy.sh` script:

- disables and stops the host systemd service if it exists
- rebuilds the adapter image from the current checkout
- recreates the adapter container

The Docker container runs with:

- host networking
- mounted `./config` at `/opt/formant-spot-adapter/config`
- mounted `./data` at `/opt/formant-spot-adapter/data`
- automatic restart policy `unless-stopped`

Low-level Docker scripts remain available for manual control:

- `./scripts/docker_build.sh`
- `./scripts/docker_up.sh`
- `./scripts/docker_down.sh`

If you want a one-shot rebuild and replace without the higher-level deploy wrapper:

```bash
./scripts/docker_up.sh --build
```

## Host Service Lifecycle Scripts

- Canonical deploy/redeploy path (build + install/update service + start/restart):

```bash
./scripts/deploy.sh
```

- `./scripts/deploy.sh` is the supported service-install path for a checkout of
  this repository. It generates the systemd unit with the correct working
  directory and local config paths for the current machine.

- Pull latest from git, then redeploy:

```bash
PULL_FROM_GIT=1 ./scripts/deploy.sh
```

- `./scripts/update.sh` remains as a compatibility wrapper around `deploy.sh` with `PULL_FROM_GIT=1` by default.

- Remove service unit and stop service:

```bash
./scripts/uninstall.sh
```

- Check service status:

```bash
./scripts/service_status.sh
```

- Tail service logs:

```bash
./scripts/service_logs.sh
```

- Install/update log rotation policy only:

```bash
./scripts/setup_logrotate.sh
```

## Arm State Utility

```bash
./scripts/arm_state.sh
```

Optional frame:

```bash
BASE_FRAME=vision ./scripts/arm_state.sh
```

## Camera Calibration Utility

Inspect camera-calibration readiness and feedback:

```bash
./scripts/camera_calibration.sh status
```

Start camera calibration with the official Spot SDK path:

```bash
./scripts/camera_calibration.sh start
```

Cancel a running camera calibration:

```bash
./scripts/camera_calibration.sh cancel
```

The utility refuses to start by default if the robot is docked/on shore power, not standing,
or has active `camera_server` init faults such as `failed to initialize` / `enumerated USB2`.
Use `--force` only if you have verified the robot is in the correct physical setup.

The adapter command path now uses the same basic preflight logic for `spot.camera.calibrate`:
it rejects calibration when Spot is docked/on shore power, not standing, motor power is off,
an E-Stop is active, or `camera_server` still has active init faults.

## Notes

- Formant agent target default is `localhost:5501`.
- This project intentionally does not use ROS.
- The supported target environment is Jetson with an Ubuntu-based userspace.
- The setup/build flow expects the Boston Dynamics Spot C++ SDK as an external
  dependency under its own license. The repo fetches the pinned SDK automatically
  into `third_party/spot-cpp-sdk` during bootstrap/build if it is missing.
- Default Spot SDK pin: `v5.1.0`. You can override with `SPOT_SDK_REF=vX.Y.Z`.
- If you change the SDK pin for an existing checkout, rerun with
  `FORCE_REINSTALL_SPOT_SDK=1` or point `SPOT_CPP_SDK_DIR` at a different SDK path.
- If you already have a local SDK checkout, point the build at it with
  `SPOT_CPP_SDK_DIR=/path/to/spot-cpp-sdk ./scripts/build.sh`.
- Formant protos are vendored from the public `FormantIO/formant` repo at the
  pinned commit recorded in [`proto/formant_vendor.lock`](proto/formant_vendor.lock),
  so users do not need any extra setup step for them.
- Maintainers can refresh the vendored Formant proto set with
  `./scripts/sync_formant_protos.sh [ref]`.
- For UI examples, see `docs/formant-streams.md`.
- For detailed implementation history/behavior notes, see `docs/graphnav-teleop-implementation-notes.md`.
