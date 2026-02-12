# formant-spot-adapter

C++ adapter for Boston Dynamics Spot + Formant Agent teleoperation without ROS.

## What It Does

- Teleop motion from Formant twist control.
- Stand/Sit, E-Stop, Recover, Dock, locomotion mode, and arm mode controls.
- Arm stow hold while lease is active.
- Image publishing from hand, left, right, and back cameras.
- Dock readiness status publishing.
- Stateful mode publishing (walk/stairs/crawl).
- Formant command handling for Jetson reboot (robot reboot placeholder).

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
- Command channel supports `spot.jetson.reboot` and `spot.robot.reboot` (TBD).

### Streams published by adapter (adapter -> Formant)

| Stream name (default) | Type | Content |
|---|---|---|
| `spot.hand.image` | Image (JPEG) | Hand camera (`hand_color_image`), optional 90deg CCW auto-rotation |
| `spot.left.image` | Image (JPEG) | Left fisheye (`left_fisheye_image`) |
| `spot.right.image` | Image (JPEG) | Right fisheye (`right_fisheye_image`) |
| `spot.back.image` | Image (JPEG) | Back fisheye (`back_fisheye_image`) |
| `spot.status` | Bitset | `Has lease`, `Teleop running`, `Teleop active`, `Docking` |
| `spot.can_dock` | Bitset | `Can dock` (published at 0.2 Hz / every 5s) |
| `spot.mode_state` | Bitset | `Walk`, `Stairs`, `Crawl` |

## Runtime Behavior

- Starts passive. Lease is acquired only while teleop session is active.
- If lease is already held, adapter attempts takeover (`TakeLease`) to recover from stale owners.
- On teleop inactivity: sends zero velocity, stows arm, then returns lease.
- Heartbeat timeout behavior is zero-velocity only (no auto-sit).
- Arm hold loop continuously reissues stow at interval.
- Dock command runs in background loop with status/error logging.

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

4. Or install/start service:

```bash
sudo ./scripts/install.sh
sudo ./scripts/setup_service.sh
sudo systemctl enable --now formant-spot-adapter.service
```

## Arm State Utility

```bash
./scripts/arm_state.sh
```

Optional frame:

```bash
BASE_FRAME=vision ./scripts/arm_state.sh
```

## Notes

- Formant agent target default is `localhost:5501`.
- This project intentionally does not use ROS.
- For UI examples, see `docs/formant-streams.md`.
