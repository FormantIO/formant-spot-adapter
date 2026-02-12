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
- `Teleop running`
- `Teleop active`
- `Docking`

6. `spot.can_dock` (type: bitset, 0.2 Hz / every 5s)
- Key: `Can dock` (boolean)
- Indicates whether robot is currently in a dockable state for the resolved dock ID.

7. `spot.mode_state` (type: bitset, 0.2 Hz / every 5s)
- Published keys:
- `Walk`
- `Stairs`
- `Crawl`

## Commands (Formant -> Adapter)

1. `spot.jetson.reboot` (command)
- Reboots the Jetson host running this adapter.
- Adapter sends command success response, then executes `systemctl reboot`.

2. `spot.robot.reboot` (command)
- Reserved for robot-body reboot flow.
- Currently returns unsuccessful response (TBD implementation).

## Recommended Teleop UI Layout

1. Video panels subscribed to `spot.hand.image`, `spot.left.image`, `spot.right.image`, and `spot.back.image`.
2. Left joystick and right joystick both driving `Joystick`.
3. Two buttons in `Buttons` bitset: `Stand`, `Sit`.
4. Or, use direct buttons publishing to streams `Stand`, `Sit`, `E-Stop`, `Recover`, `Walk`, `Stairs`, `Crawl`, `Reset Arm`, and `Dock`.

## Notes

- Adapter starts in passive mode: no lease is acquired until a teleop heartbeat is active.
- If teleop heartbeat stops, adapter sends zero velocity, stows arm, then releases lease.
- Heartbeat timeout behavior is zero-velocity command only.
- No auto-sit on heartbeat timeout.
- Adapter enforces a twist deadband and teleop idle timeout, so motion is forced to zero when joystick input is idle.
- Camera auto-rotation can be enabled via `cameraAutoRotate` and uses `arm0.wr1` threshold
  `cameraRotateWr1ThresholdRad` to rotate hand-camera frames 90 degrees CCW when wrist is turned.
- Arm hold mode is always active while lease is owned:
  - adapter continuously keeps the arm stowed/retracted
  - pressing `Reset Arm` immediately requests stow
  - adapter re-issues stow every `armHoldIntervalMs` (default 2500 ms)
- Locomotion mode is stateful and defaults to `Walk`. `Walk`/`Stairs`/`Crawl` buttons only change mode.
