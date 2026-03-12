# formant-spot-adapter

C++ adapter for Boston Dynamics Spot + Formant Agent teleoperation without ROS.

## What It Does

- Teleop motion from Formant twist control.
- Stand/Sit, E-Stop, Recover, Dock, locomotion mode, and arm mode controls.
- Arm stow hold while lease is active.
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
| `Buttons` | `Bitset` | Multi-button input | Recognized keys: `Stand`, `Sit`, `E-Stop`, `Recover`, `Dock`, `Walk`, `Stairs`, `Crawl`, `Reset Arm` |
| `Stand` | `Bitset`/boolean-to-device | Stand command | Any `true` bit triggers stand |
| `Sit` | `Bitset`/boolean-to-device | Sit command | Any `true` bit triggers sit |
| `E-Stop` | `Bitset`/boolean-to-device | Safe power-off command | Requires lease |
| `Recover` | `Bitset`/boolean-to-device | Self-right command | Requires lease |
| `Walk` | `Bitset`/boolean-to-device | Set locomotion mode | Does not stand robot |
| `Stairs` | `Bitset`/boolean-to-device | Set locomotion mode | Does not stand robot |
| `Crawl` | `Bitset`/boolean-to-device | Set locomotion mode | Does not stand robot |
| `Reset Arm` | `Bitset`/boolean-to-device | Set arm mode to stow | Requires lease to execute immediately |
| `Dock` | `Bitset`/boolean-to-device | Start autodock | Uses configured dock id or discovery |

Additional control channels (not streams):
- Teleop heartbeat from Formant agent controls session active/inactive behavior.
- Command channel supports `spot.jetson.reboot`, `spot.robot.reboot`, `spot.camera.calibrate`,
  `spot.stand`, `spot.sit`, `spot.recover`, `spot.dock`, `spot.reset_arm`,
  `spot.map.*`, and `spot.waypoint.*`.

### Streams published by adapter (adapter -> Formant)

| Stream name (default) | Type | Content |
|---|---|---|
| `spot.hand.image` | Image (JPEG) | Hand camera (`hand_color_image`), optional 90deg CCW auto-rotation |
| `spot.left.image` | Image (JPEG) | Left fisheye (`left_fisheye_image`) |
| `spot.right.image` | Image (JPEG) | Right fisheye (`right_fisheye_image`) |
| `spot.back.image` | Image (JPEG) | Back fisheye (`back_fisheye_image`) |
| `spot.status` | Bitset | `Has lease`, `Robot available`, `Robot degraded`, `Teleop running`, `Teleop active`, `Docking` |
| `spot.connection` | Text (JSON) | Spot connection health (`state`, `connected`, `degraded_non_estop`, `degraded_reason`, reconnect/attempt timestamps, last error) |
| `spot.localization` | Text (JSON) | GraphNav localization status (`localized`, `waypoint_id`, `error`) |
| `spot.can_dock` | Bitset | `Can dock` (published at 0.2 Hz / every 5s) |
| `spot.mode_state` | Bitset | `Walk`, `Stairs`, `Crawl` |
| `spot.waypoints` | Text | Newline-separated waypoint names for the active map |
| `spot.waypoint.current` | Text | Waypoint name if robot is within 1 ft of a saved waypoint, else empty string; checks every 1s, publishes every 10s |
| `spot.maps` | Text | Newline-separated saved map IDs |
| `spot.map.current` | Text | Active/current map ID (`none` if unset); publishes every 10s and on change |
| `spot.map.default` | Text | Default map ID (`none` if unset); publishes every 10s and on change |
| `spot.robot_state.power` | Text (JSON) | Motor power state, estop summary, battery availability |
| `spot.robot_state.battery` | Numeric | Battery percentage (0-100) when available |
| `spot.robot_state.body_pitch_rad` | Numeric | Measured body pitch in radians (`odom` frame) |
| `spot.faults.system` | Text (JSON) | Active system faults summary |
| `spot.faults.behavior` | Text (JSON) | Active behavior faults summary |
| `spot.faults.service` | Text (JSON) | Active service faults summary |
| `spot.fault.events` | Text | Simplified fault event feed (`FAULT OPEN/CHANGED/CLEARED` only) |
| `spot.nav.feedback` | Text (JSON) | GraphNav command status/route-following diagnostics |
| `spot.adapter.log` | Text | Buffered adapter event log batches (1 Hz) |

## Runtime Behavior

- Starts passive. Lease is acquired only while teleop session is active.
- Adapter stays online if Spot is unavailable and retries robot connection in the background.
- While Spot is unavailable, robot actions are rejected and availability is published on `spot.connection`.
- Adapter applies soft non-E-Stop recovery gating: teleop motion/dock commands are blocked only for critical/unclearable robot faults or motor power error.
- For GraphNav nav commands, adapter prechecks localization and attempts fiducial relocalization before failing.
- If lease is already held, adapter attempts takeover (`TakeLease`) to recover from stale owners.
- On teleop inactivity: sends zero velocity, stows arm, then returns lease unless GraphNav nav is active.
- Heartbeat timeout behavior is zero-velocity only (no auto-sit).
- Arm hold loop continuously reissues stow at interval.
- Dock command runs in background loop with status/error logging.
- GraphNav maps are persisted under `graphnavStoreDir` and can be loaded/deleted via commands.
- Waypoint aliases are map-scoped (`name -> waypoint_id`) and published on `spot.waypoints`.
- Teleop motion uses held-command resend (smoother than packet-by-packet pulse behavior).

## GraphNav Commands

All commands are namespaced as `spot.*` and accept optional text parameters.

Additional non-GraphNav command-channel actions:
- `spot.stand`: stand command; command path does not require active teleop heartbeat.
- `spot.sit`: sit command; command path does not require active teleop heartbeat.
- `spot.recover`: self-right command; command path does not require active teleop heartbeat.
- `spot.dock`: run autodock procedure; command path does not require active teleop heartbeat.
- `spot.return_and_dock`: navigate to saved dock waypoint for active map, then run dock.
  - No parameters.
  - Fails if no dock waypoint is saved yet for the active/default map.
  - Successful manual `spot.dock` learns/saves current localized waypoint as dock waypoint.
- `spot.rotate_left`: rotate left in place by a requested angle. Parameter: `degrees`.
- `spot.rotate_right`: rotate right in place by a requested angle. Parameter: `degrees`.
- `spot.reset_arm`: arm reset/stow; command path does not require active teleop heartbeat.

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
- `spot.waypoint.goto`: resolve alias (or GraphNav waypoint label) and navigate to it.
  - Parameter examples: `name=dock_entry`

Note: `spot.waypoint.save`/`spot.waypoint.update` can be called while not actively mapping; the
adapter runs a temporary start/create/stop recording sequence.

Command responses:
- Responses are terminal: success/failure is returned after the action reaches a terminal state.
- Long-running commands (`spot.camera.calibrate`, `spot.waypoint.goto`, `spot.dock`,
  `spot.return_and_dock`) keep the response pending until completion.

## Configuration Model

- Non-secrets: `config/formant-spot-adapter.json`
- Secrets: `config/formant-spot-adapter.env` (`SPOT_USERNAME`, `SPOT_PASSWORD`)

Important:
- Current runtime `load_config()` takes secrets from env and all other runtime params from JSON.
- If you want to change stream names, arm pose, docking config, or camera config, update JSON.

## Quickstart (Ubuntu/Jetson)

1. Configure files:

```bash
cp config/formant-spot-adapter.env.example config/formant-spot-adapter.env
cp config/formant-spot-adapter.json.example config/formant-spot-adapter.json
nano config/formant-spot-adapter.env
nano config/formant-spot-adapter.json
```

2. Install deps and build:

```bash
./scripts/bootstrap_ubuntu.sh
./scripts/build.sh
```

3. Run directly:

```bash
./scripts/run.sh
```

4. Or deploy as a persistent service:

```bash
./scripts/deploy.sh
```

## Service Lifecycle Scripts

- First-time deploy (build + install/update service + enable/start):

```bash
./scripts/deploy.sh
```

- Update from git + rebuild + restart service:

```bash
./scripts/update.sh
```

- Rebuild/restart without pulling from git:

```bash
PULL_FROM_GIT=0 ./scripts/update.sh
```

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
- For UI examples, see `docs/formant-streams.md`.
- For detailed implementation history/behavior notes, see `docs/graphnav-teleop-implementation-notes.md`.
