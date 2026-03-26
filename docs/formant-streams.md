# Formant Configuration for Teleop

Configure these streams in your Formant teleop view/device config.

## Control Streams (Formant -> Adapter)

- `Joystick` (type: `geometry_msgs/Twist` equivalent RTC twist stream)
- This is the primary driving control.
- Two joystick UI modules can both publish to `Joystick`.
- Default mapping:
- `linear.x` = forward/back
- `linear.y` = strafe left/right
- `angular.z` = yaw left/right
- `angular.y` = body pitch up/down (clamped by `maxBodyPitchRad`)

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

- `spot.right.image` (type: image/video)
- Source camera: `right_fisheye_image` (default configurable)

- `spot.back.image` (type: image/video)
- Source camera: `back_fisheye_image` (default configurable)

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

- `spot.localization` (type: text/json, 0.2 Hz / every 5s)
- GraphNav localization status.
- Fields: `localized`, `waypoint_id`, `error`.

- `spot.localization.graphnav` (type: localization, 0.2 Hz / every 5s)
- Typed Formant localization stream for the 3D/localization viewer.
- Publishes GraphNav `seed_tform_body` plus a live occupancy-grid patch derived from
  Spot live terrain maps.
- Current implementation is a local patch around the robot, not a stitched global map.

- `spot.localization.image` (type: image/jpeg, default 15 FPS output / 2 Hz Spot refresh)
- Rendered 16:9 visualization of the GraphNav local occupancy patch for straightforward image/video
  viewing in Formant.
- Includes occupancy overlay, robot footprint, heading arrow, scale bar, current waypoint/map
  labels, and a status HUD.
- The adapter renders only when the Spot localization patch updates, then republishes the cached
  JPEG at the configured output FPS.
- To behave like a camera/video stream instead of a throttled telemetry image, configure
  `spot.localization.image` in Formant with video encoding enabled or add it as a realtime stream.

- `spot.can_dock` (type: bitset, 0.2 Hz / every 5s)
- Key: `Can dock` (boolean)
- Indicates whether robot is currently in a dockable state for the resolved dock ID.

- `spot.mode_state` (type: bitset, 0.2 Hz / every 5s)
- Published keys:
- `Walk`
- `Stairs`
- `Crawl`

- `spot.robot_state.power` (type: text/json, 0.2 Hz / every 5s)
- Fields include motor power state, estop aggregate, and battery availability.

- `spot.robot_state.battery` (type: numeric, 0.2 Hz / every 5s)
- Battery percentage in range 0-100 when available from robot state.

- `spot.robot_state.body_pitch_rad` (type: numeric, 0.2 Hz / every 5s)
- Measured body pitch in radians (derived from `odom` -> `body` transform).

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
- Resolves alias (or existing GraphNav waypoint label) and sends NavigateTo command.
- Parameter text: `name=<alias_or_label>`.

Command response behavior:
- Formant command responses are terminal; success/failure is returned when the action reaches completion.
- Long-running commands (`spot.camera.calibrate`, `spot.waypoint.goto`, `spot.dock`,
  `spot.return_and_dock`) keep the response pending until terminal success/failure.

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
- If teleop heartbeat stops, adapter sends zero velocity, stows arm, then releases lease.
- Heartbeat timeout behavior is zero-velocity command only.
- No auto-sit on heartbeat timeout.
- Adapter enforces twist deadband and uses held-command resend for smoother teleop control.
- Very short pitch-only zero pulses are filtered to reduce oscillation from pulsed joystick transport.
- Camera auto-rotation can be enabled via `cameraAutoRotate` and uses `arm0.wr1` threshold
  `cameraRotateWr1ThresholdRad` to rotate hand-camera frames 90 degrees CCW when wrist is turned.
- Arm hold mode is always active while lease is owned:
  - adapter continuously keeps the arm stowed/retracted
  - pressing `Reset Arm` immediately requests stow
  - adapter re-issues stow every `armHoldIntervalMs` (default 2500 ms)
- Locomotion mode is stateful and defaults to `Walk`. `Walk`/`Stairs`/`Crawl` buttons only change mode.
