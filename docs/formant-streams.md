# Formant Configuration for Teleop

Configure these streams in your Formant teleop view/device config.

Runtime stream enablement is controlled in `config/formant-spot-adapter.json` with
`streamControls`. Setting `"enabled": false` disables publication for that stream, and image/map
streams also stop their source polling/render loops so they no longer consume runtime resources.
If you rename a stream from its default, use the renamed stream name in `streamControls`.

Runtime config precedence is adapter defaults, then JSON, then valid environment overrides.
Invalid numeric/boolean environment overrides are ignored and logged at adapter startup.

## Control Streams (Formant -> Adapter)

- `Joystick` (type: `geometry_msgs/Twist` equivalent RTC twist stream)
- This is the primary driving control.
- Two joystick UI modules can both publish to `Joystick`.
- Default mapping:
- `linear.x` = forward/back
- `linear.y` = strafe left/right
- `angular.z` = yaw left/right
- `angular.y` = body pitch up/down (clamped by `maxBodyPitchRad`)

- `Gamepad` (type: `sensor_msgs/Joy` equivalent RTC joy stream)
- Default adapter joy input stream for the teleop app `joy-bridge` module / browser gamepad support.
- Adapter-side mapping translates it into the same motion path used by `Joystick`.
- Default axis mapping:
- axis `1` (inverted) -> `linear.x`
- axis `0` (inverted) -> `linear.y`
- axis `2` (inverted) -> `angular.z`
- axis `3` (inverted) -> `angular.y`
- Default button mapping (standard browser/Xbox face buttons):
- button `0` (`A`) -> `Stand`
- button `1` (`B`) -> `Sit`
- button `2` (`X`) -> `Walk`
- button `3` (`Y`) -> `Dock`
- `Recover`, `E-Stop`, `Reset Arm`, `Stairs`, and `Crawl` are intentionally unmapped by default.
- `teleopJoyStream` defaults to `Gamepad`.

## Teleop Motion Tuning

Formant joystick modules publish normalized `Twist` values, normally in `[-1, 1]`. The adapter maps
those unitless values to Spot velocity commands with clamp, deadband rescaling, response curves,
speed caps, and slew limits. Normal joystick release still sends zero immediately; the stale input
timeout is for lost/repeated input transport cases.

Baseline values live in `config/formant-spot-adapter.json`. Formant application configuration can
override runtime tuning without an adapter restart. Missing keys use the baseline values, and invalid
values are ignored. The adapter reads application configuration at startup and on Formant agent
stream reconnects after agent configuration changes; it does not poll application configuration in
the control loop.

| Application config key | Default |
|---|---:|
| `spot.teleop.max_vx_mps` | `0.8` |
| `spot.teleop.max_vy_mps` | `0.5` |
| `spot.teleop.max_wz_rps` | `1.2` |
| `spot.teleop.max_body_pitch_rad` | `0.25` |
| `spot.teleop.deadband` | `0.08` |
| `spot.teleop.idle_timeout_ms` | `1000` |
| `spot.teleop.translation_response_curve` | `1.4` |
| `spot.teleop.rotation_response_curve` | `1.2` |
| `spot.teleop.linear_accel_limit_mps2` | `0.8` |
| `spot.teleop.strafe_accel_limit_mps2` | `0.6` |
| `spot.teleop.angular_accel_limit_rps2` | `1.5` |
| `spot.teleop.body_pitch_rate_limit_radps` | `0.5` |

For the optional `Gamepad`/`Joy` bridge, the same application configuration map can override
`spot.teleop.joy_axis_*`, `spot.teleop.joy_axis_*_inverted`, and `spot.teleop.joy_button_*`
parameters documented in the repository `README.md`.

- `Buttons` (type: Bitset)
- Include at least two buttons:
- `Stand`
- `Sit`

- `Stand` (type: boolean-to-device / bitset true click)
- Preferred for simple teleop UI button wiring.
- Any `true` bitset value on this stream triggers stand.

- `Sit` (type: boolean-to-device / bitset true click)
- Preferred for simple teleop UI button wiring.
- Any `true` bitset value on this stream triggers sit.

- `E-Stop` (type: boolean-to-device / bitset true click)
- Triggers emergency stop behavior (safe power-off command).
- Requires lease ownership.

- `Recover` (type: boolean-to-device / bitset true click)
- Triggers self-right recovery behavior.
- Requires active teleop session and lease ownership.

- `Walk` (type: boolean-to-device / bitset true click)
- Selects walk locomotion mode.
- Does not stand the robot.

- `Stairs` (type: boolean-to-device / bitset true click)
- Selects stairs locomotion mode.
- Does not stand the robot.

- `Crawl` (type: boolean-to-device / bitset true click)
- Selects crawl locomotion mode.
- Does not stand the robot.

- `Dock` (type: boolean-to-device / bitset true click)
- Triggers auto-docking command.
- If `dockStationId` is `-1`, adapter auto-discovers dock ID only when exactly one dock ID is configured.

- `Reset Arm` (type: boolean-to-device / bitset true click)
- Sends Spot arm to stowed/retracted pose.
- Requires active teleop session and lease ownership.

## Telemetry Streams (Adapter -> Formant)

- `spot.hand.image` (type: image/video)
- Source camera: Spot arm camera (`hand_color_image`)
- Configure module as camera/video view in teleop.
- Auto-rotation is applied here when wrist orientation threshold is crossed.

- `spot.left.image` (type: image/video)
- Source camera: `left_fisheye_image` (default configurable)
- Adapter republishes the latest cached JPEG at the configured surround-camera output FPS.

- `spot.right.image` (type: image/video)
- Source camera: `right_fisheye_image` (default configurable)
- Adapter republishes the latest cached JPEG at the configured surround-camera output FPS.
- Rotated 180 degrees by default to correct the current Spot right-camera orientation.

- `spot.back.image` (type: image/video)
- Source camera: `back_fisheye_image` (default configurable)
- Adapter republishes the latest cached JPEG at the configured surround-camera output FPS.

- `spot.front.image` (type: image/video)
- Source cameras: `frontleft_fisheye_image` + `frontright_fisheye_image` (default configurable)
- Adapter fetches both front fisheyes in one Spot image batch, rectifies them into a single
  forward-looking stitched view, and republishes the cached composite at the configured front-image
  output FPS.
- By default the adapter rotates the stitched image 90 degrees clockwise.
  `frontImageRollDegrees` remains available as a manual override when a deployment needs a
  different fixed roll.
- Designed for low-latency teleop use. Default output is 15 FPS with 15 Hz Spot polling.
- If one front camera is degraded, the adapter keeps publishing a single-camera rectified fallback
  instead of dropping the stream immediately.

- `spot.status` (type: bitset, 0.2 Hz / every 5s)
- Published keys:
- `Has lease`
- `Robot available`
- `Robot degraded`
- `Teleop running`
- `Teleop active`
- `Docking`

- `spot.connection` (type: text/json, 0.2 Hz / every 5s)
- Spot connectivity health and reconnect telemetry.
- Fields include `state`, `connected`, `degraded_non_estop`, `degraded_reason`,
  `reconnect_attempt`, `last_success_ms`, `last_attempt_ms`, and `error`.

- `spot.connection_state` (type: text, 0.2 Hz / every 5s)
- One-word Spot connection state for teleop/UI bindings.
- Values: `disconnected`, `connecting`, `connected`.

- `spot.localization` (type: text/json, 0.2 Hz / every 5s)
- GraphNav localization status.
- Fields: `localized`, `waypoint_id`, `error`.

- `spot.localization.graphnav` (type: localization, 0.2 Hz / every 5s)
- Typed Formant localization stream for the 3D/localization viewer.
- Publishes GraphNav `seed_tform_body` plus a live occupancy-grid patch derived from
  Spot live terrain maps.
- Current implementation is a local patch around the robot, not a stitched global map.

- `spot.localization.graphnav.image` (type: image/jpeg, default 15 FPS output / 2 Hz Spot refresh)
- Rendered 16:9 visualization of the GraphNav local occupancy patch for straightforward image/video
  viewing in Formant.
- Includes occupancy overlay, robot footprint, heading arrow, scale bar, current waypoint/map
  labels, and a status HUD.
- The adapter renders only when the Spot localization patch updates, then republishes the cached
  JPEG at the configured output FPS.
- To behave like a camera/video stream instead of a throttled telemetry image, configure
  `spot.localization.graphnav.image` in Formant with video encoding enabled or add it as a realtime stream.

- `spot.localization.graphnav.global` (type: localization, default 2 Hz)
- Typed Formant localization stream for the stitched saved GraphNav site map plus live seed-frame
  robot pose.

- `spot.localization.graphnav.global.image` (type: image/jpeg, default 15 FPS output / 2 Hz refresh)
- Rendered global GraphNav map image with stitched occupancy, waypoint/edge overlays, and live robot
  pose when available.

- `spot.localization.graphnav.global.image.meta` (type: text/json, 0.2 Hz / every 5s + on change)
- Companion metadata for `spot.localization.graphnav.global.image`.
- Includes `map_id`, `map_uuid`, canvas size, draw rect, render scale, map resolution, and
  `seed_tform_grid` so a UI can convert image pixels into seed-frame navigation targets without
  reverse-engineering the server render math.

- `spot.map.graphnav` (type: localization payload carrying map-only data, on change)
- Dedicated stitched GraphNav map payload published without a live pose so the transport can be
  upgraded to a true top-level map datapoint later without changing upstream map assembly logic.

- `spot.graphnav.metadata` (type: text/json, every 5s + on change)
- Full waypoint, edge, and related GraphNav metadata intended for diagnostics and id-based
  navigation support.
- For iframe-based map UIs, this stream can be too large for reliable text query paths. Prefer
  `spot.localization.graphnav.global.image`, `spot.localization.graphnav.global.image.meta`,
  `spot.graphnav.overlay`, and `spot.nav.state` as the core navigation contract.

- `spot.graphnav.overlay` (type: text/json, every 5s + on change)
- UI-safe GraphNav overlay stream for operator map modules.
- Includes only saved/operator-facing waypoint names, not every raw GraphNav node label.
- Includes:
  - `map_id`
  - `map_uuid`
  - `current_waypoint_id`
  - `dock_waypoint_id`
  - `waypoints: [{id, name, label, x, y, is_dock}]`
  - `edges: [{from_waypoint_id, to_waypoint_id}]` between emitted waypoint IDs
- Intended as the primary waypoint stream for iframe-based navigation UIs.
- UI command dispatch should prefer `waypoint_id` from this stream over display names.

- `spot.nav.state` (type: text/json, 1 Hz + on change)
- High-level GraphNav navigation state for external map UIs.
- Includes `active`, `command_id`, `request_id`, `command_request_id`, `status`,
  `status_name`, `phase`, `terminal_result`, `terminal_reason`, `mode`, `map_id`, `map_uuid`,
  `target_waypoint_id`, `target_name`, target pose fields when available, plus current
  localization fields: `localized`, `current_waypoint_id`, `has_current_seed_pose`,
  `current_seed_x`, `current_seed_y`, `current_seed_z`, and `current_seed_yaw_rad`.
- `phase` may be `starting` before the Boston Dynamics GraphNav command id is available.

- `spot.can_dock` (type: bitset, 0.2 Hz / every 5s)
- Key: `Can dock` (boolean)
- Indicates whether robot is currently in a dockable state for the resolved dock ID.

- `spot.mode_state` (type: bitset, 0.2 Hz / every 5s)
- Published keys:
- `Walk`
- `Stairs`
- `Crawl`

- `spot.commanded_motion_mode` (type: text, 0.2 Hz / every 5s)
- One-word commanded locomotion mode used for future teleop velocity commands.
- Values: `walk`, `stairs`, `crawl`.

- `spot.docking_state` (type: text, 0.2 Hz / every 5s)
- One-word docking state from the Spot docking service.
- Values: `unknown`, `docked`, `docking`, `undocked`, `undocking`.

- `spot.robot_state.power` (type: text/json, 0.2 Hz / every 5s)
- Fields include motor power state, estop aggregate, and battery availability.

- `spot.motor_power_state` (type: text, 0.2 Hz / every 5s)
- One-word motor power state.
- Values: `off`, `on`, `powering_on`, `powering_off`, `error`, `unknown`.

- `spot.robot_state.battery` (type: numeric, 0.2 Hz / every 5s)
- Battery percentage in range 0-100 when available from robot state.

- `spot.robot_state.body_pitch_rad` (type: numeric, 0.2 Hz / every 5s)
- Measured body pitch in radians (derived from `odom` -> `body` transform).

- `spot.behavior_state` (type: text, 0.2 Hz / every 5s)
- One-word high-level robot behavior state from Spot robot state.
- Values: `not_ready`, `transition`, `standing`, `stepping`, `unknown`.

- `spot.shore_power_state` (type: text, 0.2 Hz / every 5s)
- One-word shore-power state.
- Values: `on`, `off`, `unknown`.

- `spot.faults.system` (type: text/json, 0.2 Hz / every 5s)
- Active system fault summary (`count`, list of `id`, `severity`, `message`).

- `spot.faults.behavior` (type: text/json, 0.2 Hz / every 5s)
- Active behavior fault summary.

- `spot.faults.service` (type: text/json, 0.2 Hz / every 5s)
- Active service fault summary.

- `spot.fault.events` (type: text, event-driven)
- Simplified fault feed for operators.
- Emits only on fault changes:
  - `FAULT OPEN ...`
  - `FAULT CHANGED ...`
  - `FAULT CLEARED ...`

- `spot.nav.feedback` (type: text/json, ~2 Hz while nav command active)
- GraphNav feedback snapshot (`status`, `remaining_route_length_m`, route/blockage context).
- Intended as low-level diagnostics; use `spot.nav.state` as the primary UI state stream.

- `spot.adapter.log` (type: text, 1 Hz batched)
- Buffered adapter-side operational log lines (command/recovery events).

- `spot.waypoints` (type: text, 1 Hz)
- Newline-separated waypoint names for the currently loaded GraphNav map.
- Names come from adapter aliases (`spot.waypoint.save/update`) and fall back to GraphNav labels.

- `spot.waypoint.current` (type: text, 0.1 Hz / every 10s)
- Current waypoint name when robot is localized within 1 ft of a saved waypoint.
- Empty string when robot is not within 1 ft of a saved waypoint.
- Internal proximity check runs every 1s.

- `spot.maps` (type: text, 0.2 Hz / every 5s)
- Newline-separated saved map IDs discovered in adapter map storage.

- `spot.map.current` (type: text, 0.1 Hz / every 10s + on change)
- Active/current map ID (`none` when unset).

- `spot.map.default` (type: text, 0.1 Hz / every 10s + on change)
- Default map ID (`none` when unset).

- `spot.map.progress` (type: text/json, 0.2 Hz / every 5s)
- Recording/map statistics summary from GraphNav recording status.

- `spot.map.progress.waypoints` (type: numeric, 0.2 Hz / every 5s)
- Waypoint count in current loaded graph.

- `spot.map.progress.path_length_m` (type: numeric, 0.2 Hz / every 5s)
- Total recorded path length in meters.

- `spot.map.progress.fiducials` (type: numeric, 0.2 Hz / every 5s)
- Count of visible fiducials in recording status.

## Commands (Formant -> Adapter)

- `spot.jetson.reboot` (command)
- Reboots the Jetson host running this adapter.
- Adapter sends command success response, then executes `systemctl reboot`.

- `spot.robot.reboot` (command)
- Requests Spot body safe power-cycle reboot via power service helper.
- Command returns success/failure response based on SDK result.

- `spot.camera.calibrate` (command)
- Starts Spot camera calibration routine via SpotCheck service.
- Does not require active teleop session; adapter acquires lease when needed.

- `spot.stand` (command)
- Sends stand command.
- Does not require active teleop session; adapter acquires lease when needed.

- `spot.sit` (command)
- Sends sit command.
- Does not require active teleop session; adapter acquires lease when needed.

- `spot.recover` (command)
- Sends self-right command.
- Does not require active teleop session; adapter acquires lease when needed.

- `spot.dock` (command)
- Runs autodock procedure.
- Does not require active teleop session.

- `spot.undock` (command)
- Undocks from the current dock and moves Spot to the prep pose.
- Does not require active teleop session.
- Fails if the robot is not currently docked.

- `spot.return_and_dock` (command)
- Navigates to the saved dock waypoint for the active/default map, then runs autodock.
- Does not require active teleop session.
- Fails if no saved dock waypoint exists yet.
- Dock waypoint is learned automatically after successful manual `spot.dock`.

- `spot.rotate_left` (command)
- Rotates Spot left in place by a requested angle.
- Parameter text: `degrees=<float>`.
- Does not require active teleop session; adapter acquires lease when needed.

- `spot.rotate_right` (command)
- Rotates Spot right in place by a requested angle.
- Parameter text: `degrees=<float>`.
- Does not require active teleop session; adapter acquires lease when needed.

- `spot.reset_arm` (command)
- Requests arm stow/reset.
- Does not require active teleop session; adapter acquires lease when needed.

- `spot.map.create` (command)
- Creates a new empty map context, clears loaded GraphNav map, and sets it active.
- Parameter text: `name=<id>` (also accepts `map_id=<id>`).
- Rejects if the map already exists.

- `spot.map.load` (command)
- Loads a saved GraphNav map into Spot.
- Parameter text: `map_id=<id>` or JSON like `{"map_id":"<id>"}`.

- `spot.map.set_default` (command)
- Stores a default map ID in adapter state.
- Parameter text: `map_id=<id>`.

- `spot.map.delete` (command)
- Deletes a saved map from adapter storage.
- Parameter text: `map_id=<id>`.

- `spot.map.start_mapping` (command)
- Starts GraphNav recording.

- `spot.map.stop_mapping` (command)
- Stops recording and persists current GraphNav map to adapter storage.

- `spot.waypoint.save` (command)
- Creates a GraphNav waypoint at current robot location and stores alias.
- Parameter text: `name=<alias>`.
- If mapping is not active, adapter performs a temporary start/create/stop recording cycle.

- `spot.waypoint.update` (command)
- Rebinds alias to a newly created GraphNav waypoint at current location.
- Parameter text: `name=<alias>`.
- If mapping is not active, adapter performs a temporary start/create/stop recording cycle.

- `spot.waypoint.delete` (command)
- Deletes alias from adapter map state (does not delete underlying GraphNav waypoint object).
- Parameter text: `name=<alias>`.

- `spot.waypoint.goto` (command)
- Resolves alias/label or accepts `waypoint_id=<id>` directly and sends NavigateTo command.
- Preferred payload: `{"map_uuid":"<uuid>","waypoint_id":"<id>","request_id":"<request>"}`.
- Legacy parameter text is still accepted: `name=<alias_or_label>` or `waypoint_id=<id>`.
- Optional `map_uuid` rejects stale UI commands against the wrong loaded map.
- `request_id` is echoed in `spot.nav.state` while that request is starting, active, or terminal.

- `spot.waypoint.goto_straight` (command)
- Same target resolution as `spot.waypoint.goto`, but with straight-line-biased travel params.
- Same JSON and legacy parameter contract as `spot.waypoint.goto`.

- `spot.graphnav.goto_pose` (command)
- Navigates to a seed-frame pose on the active GraphNav map.
- Preferred payload:
  `{"map_uuid":"<uuid>","x":1.23,"y":4.56,"yaw_deg":90,"request_id":"<request>"}`.
- Legacy parameter text is still accepted: `map_uuid=<uuid>, x=<seed_x>, y=<seed_y>`.
- Optional heading parameters: `yaw_rad` or `yaw_deg`.
- Adapter resolves the nearest anchored waypoint, converts the seed-frame goal into that waypoint's
  flattened SE2 frame, and issues GraphNav navigation with the offset goal.

- `spot.graphnav.goto_pose_straight` (command)
- Same seed-frame goal contract as `spot.graphnav.goto_pose`, but with straight-line-biased travel params.

- `spot.graphnav.cancel` (command)
- Optional payload: `{"request_id":"<request>"}`.
- The adapter starts a hold-position GraphNav request at the robot's current localized pose. If cancel arrives
  while another navigation command is still starting, the cancel request latches until the original command id is known.
- Overrides active GraphNav navigation with a new goal at the robot's current localized pose.
- Intended as the operator-facing hold-position override for map UIs.
- Processed on a high-priority path so it can interrupt an active long-running GraphNav command.

Command response behavior:
- Formant command responses are terminal; success/failure is returned when the action reaches completion.
- Long-running commands (`spot.camera.calibrate`, `spot.waypoint.goto`,
  `spot.waypoint.goto_straight`, `spot.graphnav.goto_pose`,
  `spot.graphnav.goto_pose_straight`, `spot.graphnav.cancel`,
  `spot.dock`, `spot.undock`, `spot.return_and_dock`) keep the response pending until terminal
  success/failure.

## Recommended Teleop UI Layout

- Video panels subscribed to `spot.hand.image`, `spot.left.image`, `spot.right.image`, and `spot.back.image`.
- Left joystick and right joystick both driving `Joystick`.
- Two buttons in `Buttons` bitset: `Stand`, `Sit`.
- Or, use direct buttons publishing to streams `Stand`, `Sit`, `E-Stop`, `Recover`, `Walk`, `Stairs`, `Crawl`, `Reset Arm`, and `Dock`.

## Notes

- Adapter starts in passive mode: no lease is acquired until a teleop heartbeat is active.
- Adapter remains online when Spot is unavailable and continuously attempts background reconnect.
- While disconnected, robot actions are rejected and connection state is published to `spot.connection`.
- Adapter applies soft non-E-Stop recovery gating and blocks motion/dock only for
  critical/unclearable faults or motor power error.
- GraphNav command navigation prechecks localization and attempts fiducial relocalization once
  before failing a command.
- If teleop heartbeat stops, adapter sends zero velocity, then releases lease.
- Heartbeat timeout behavior is zero-velocity command only.
- No auto-sit on heartbeat timeout.
- Adapter enforces twist deadband and uses held-command resend for smoother teleop control.
- Very short pitch-only zero pulses are filtered to reduce oscillation from pulsed joystick transport.
- Surround-camera JPEG frames are republished from cached images, and the right camera is rotated
  180 degrees by default.
- The adapter does not automatically command the arm on lease acquire, teleop activity, lease return,
  or shutdown. Pressing `Reset Arm` or sending `spot.reset_arm` is the explicit arm stow path.
- Locomotion mode is stateful and defaults to `Walk`. `Walk`/`Stairs`/`Crawl` buttons only change mode.
