# Formant Configuration for Teleop

Configure these streams in your Formant teleop view/device config.

## Control Streams (Formant -> Adapter)

1. `Joystick` (type: `geometry_msgs/Twist` equivalent RTC twist stream)
- This is the primary driving control.
- Two joystick UI modules can both publish to `Joystick`.
- Default mapping:
- `linear.x` = forward/back
- `linear.y` = strafe left/right
- `angular.z` = yaw left/right
- `angular.y` = body pitch up/down (clamped by `maxBodyPitchRad`)

2. `Buttons` (type: Bitset)
- Include at least two buttons:
- `Stand`
- `Sit`

3. `Stand` (type: boolean-to-device / bitset true click)
- Preferred for simple teleop UI button wiring.
- Any `true` bitset value on this stream triggers stand.

4. `Sit` (type: boolean-to-device / bitset true click)
- Preferred for simple teleop UI button wiring.
- Any `true` bitset value on this stream triggers sit.

5. `E-Stop` (type: boolean-to-device / bitset true click)
- Triggers emergency stop behavior (safe power-off command).
- Requires lease ownership.

6. `Recover` (type: boolean-to-device / bitset true click)
- Triggers self-right recovery behavior.
- Requires active teleop session and lease ownership.

7. `Walk` (type: boolean-to-device / bitset true click)
- Selects walk locomotion mode.
- Does not stand the robot.

8. `Stairs` (type: boolean-to-device / bitset true click)
- Selects stairs locomotion mode.
- Does not stand the robot.

9. `Crawl` (type: boolean-to-device / bitset true click)
- Selects crawl locomotion mode.
- Does not stand the robot.

10. `Dock` (type: boolean-to-device / bitset true click)
- Triggers auto-docking command.
- If `dockStationId` is `-1`, adapter auto-discovers dock ID only when exactly one dock ID is configured.

11. `Reset Arm` (type: boolean-to-device / bitset true click)
- Sends Spot arm to stowed/retracted pose.
- Requires active teleop session and lease ownership.

## Telemetry Streams (Adapter -> Formant)

1. `spot.hand.image` (type: image/video)
- Source camera: Spot arm camera (`hand_color_image`)
- Configure module as camera/video view in teleop.
- Auto-rotation is applied here when wrist orientation threshold is crossed.

2. `spot.left.image` (type: image/video)
- Source camera: `left_fisheye_image` (default configurable)

3. `spot.right.image` (type: image/video)
- Source camera: `right_fisheye_image` (default configurable)

4. `spot.back.image` (type: image/video)
- Source camera: `back_fisheye_image` (default configurable)

5. `spot.status` (type: bitset, 0.2 Hz / every 5s)
- Published keys:
- `Has lease`
- `Robot available`
- `Robot degraded`
- `Teleop running`
- `Teleop active`
- `Docking`

6. `spot.connection` (type: text/json, 0.2 Hz / every 5s)
- Spot connectivity health and reconnect telemetry.
- Fields include `state`, `connected`, `degraded_non_estop`, `degraded_reason`,
  `reconnect_attempt`, `last_success_ms`, `last_attempt_ms`, and `error`.

7. `spot.can_dock` (type: bitset, 0.2 Hz / every 5s)
- Key: `Can dock` (boolean)
- Indicates whether robot is currently in a dockable state for the resolved dock ID.

8. `spot.mode_state` (type: bitset, 0.2 Hz / every 5s)
- Published keys:
- `Walk`
- `Stairs`
- `Crawl`

9. `spot.robot_state.power` (type: text/json, 0.2 Hz / every 5s)
- Fields include motor power state, estop aggregate, and battery availability.

10. `spot.robot_state.battery` (type: numeric, 0.2 Hz / every 5s)
- Battery percentage in range 0-100 when available from robot state.

11. `spot.robot_state.body_pitch_rad` (type: numeric, 0.2 Hz / every 5s)
- Measured body pitch in radians (derived from `odom` -> `body` transform).

12. `spot.faults.system` (type: text/json, 0.2 Hz / every 5s)
- Active system fault summary (`count`, list of `id`, `severity`, `message`).

13. `spot.faults.behavior` (type: text/json, 0.2 Hz / every 5s)
- Active behavior fault summary.

14. `spot.faults.service` (type: text/json, 0.2 Hz / every 5s)
- Active service fault summary.

15. `spot.fault.events` (type: text, event-driven + periodic summary)
- Simplified fault feed for operators.
- Emits only on fault changes:
  - `FAULT OPEN ...`
  - `FAULT CHANGED ...`
  - `FAULT CLEARED ...`
- Also emits periodic summary:
  - `FAULT SUMMARY active=<n> elevated=<n> critical=<n>`

16. `spot.nav.feedback` (type: text/json, ~2 Hz while nav command active)
- GraphNav feedback snapshot (`status`, `remaining_route_length_m`, route/blockage context).

17. `spot.adapter.log` (type: text, 1 Hz batched)
- Buffered adapter-side operational log lines (command/recovery events).

18. `spot.waypoints` (type: text, 1 Hz)
- Newline-separated waypoint names for the currently loaded GraphNav map.
- Names come from adapter aliases (`spot.waypoint.save/update`) and fall back to GraphNav labels.

19. `spot.maps` (type: text, 0.2 Hz / every 5s)
- Newline-separated saved map IDs discovered in adapter map storage.

20. `spot.map.progress` (type: text/json, 0.2 Hz / every 5s)
- Recording/map statistics summary from GraphNav recording status.

21. `spot.map.progress.waypoints` (type: numeric, 0.2 Hz / every 5s)
- Waypoint count in current loaded graph.

22. `spot.map.progress.path_length_m` (type: numeric, 0.2 Hz / every 5s)
- Total recorded path length in meters.

23. `spot.map.progress.fiducials` (type: numeric, 0.2 Hz / every 5s)
- Count of visible fiducials in recording status.

## Commands (Formant -> Adapter)

1. `spot.jetson.reboot` (command)
- Reboots the Jetson host running this adapter.
- Adapter sends command success response, then executes `systemctl reboot`.

2. `spot.robot.reboot` (command)
- Requests Spot body safe power-cycle reboot via power service helper.
- Command returns success/failure response based on SDK result.

3. `spot.camera.calibrate` (command)
- Starts Spot camera calibration routine via SpotCheck service.
- Requires active teleop session with body lease (adapter will reject otherwise).

4. `spot.map.load` (command)
- Loads a saved GraphNav map into Spot.
- Parameter text: `map_id=<id>` or JSON like `{"map_id":"<id>"}`.

5. `spot.map.set_default` (command)
- Stores a default map ID in adapter state.
- Parameter text: `map_id=<id>`.

6. `spot.map.delete` (command)
- Deletes a saved map from adapter storage.
- Parameter text: `map_id=<id>`.

7. `spot.map.start_mapping` (command)
- Starts GraphNav recording.

8. `spot.map.stop_mapping` (command)
- Stops recording and persists current GraphNav map to adapter storage.

9. `spot.waypoint.save` (command)
- Creates a GraphNav waypoint at current robot location and stores alias.
- Parameter text: `name=<alias>`.
- If mapping is not active, adapter performs a temporary start/create/stop recording cycle.

10. `spot.waypoint.update` (command)
- Rebinds alias to a newly created GraphNav waypoint at current location.
- Parameter text: `name=<alias>`.
- If mapping is not active, adapter performs a temporary start/create/stop recording cycle.

11. `spot.waypoint.delete` (command)
- Deletes alias from adapter map state (does not delete underlying GraphNav waypoint object).
- Parameter text: `name=<alias>`.

12. `spot.waypoint.goto` (command)
- Resolves alias (or existing GraphNav waypoint label) and sends NavigateTo command.
- Parameter text: `name=<alias_or_label>`.

## Recommended Teleop UI Layout

1. Video panels subscribed to `spot.hand.image`, `spot.left.image`, `spot.right.image`, and `spot.back.image`.
2. Left joystick and right joystick both driving `Joystick`.
3. Two buttons in `Buttons` bitset: `Stand`, `Sit`.
4. Or, use direct buttons publishing to streams `Stand`, `Sit`, `E-Stop`, `Recover`, `Walk`, `Stairs`, `Crawl`, `Reset Arm`, and `Dock`.

## Notes

- Adapter starts in passive mode: no lease is acquired until a teleop heartbeat is active.
- Adapter remains online when Spot is unavailable and continuously attempts background reconnect.
- While disconnected, robot actions are rejected and connection state is published to `spot.connection`.
- Adapter applies soft non-E-Stop recovery gating and blocks motion/dock only for
  critical/unclearable faults or motor power error.
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
