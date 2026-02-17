# IMPLEMENTATION HISTORY

## 2026-02-17

### Completed: Production Service Lifecycle Scripts and Hardened systemd Unit

- Added operational scripts for persistent deployment and maintenance:
  - `scripts/deploy.sh`: build + service install/update + enable/start + restart.
  - `scripts/update.sh`: optional git pull + rebuild + service restart.
  - `scripts/uninstall.sh`: disable/stop/remove service unit.
  - `scripts/service_status.sh` and `scripts/service_logs.sh` for operations.
  - `scripts/setup_logrotate.sh` for persistent file-log retention.
- Hardened generated and templated systemd unit defaults:
  - Added explicit `ENV_FILE`/`CONFIG_PATH` environment wiring.
  - Added restart tuning and shutdown timeout.
  - Added baseline hardening (`NoNewPrivileges`, `PrivateTmp`, `ProtectSystem`,
    `ProtectHome`, `UMask`) and restricted writable paths to adapter `logs/` and `data/`.
- Updated `README.md` with the service lifecycle workflow.
- Added log rotation policy install (`/etc/logrotate.d/formant-spot-adapter`) for `logs/*.log`.
- Files:
  - `scripts/deploy.sh`
  - `scripts/update.sh`
  - `scripts/uninstall.sh`
  - `scripts/service_status.sh`
  - `scripts/service_logs.sh`
  - `scripts/setup_logrotate.sh`
  - `scripts/setup_service.sh`
  - `systemd/formant-spot-adapter.service`
  - `README.md`
  - `PLAN.md`

### Completed: Soft Automatic Non-E-Stop Safety Recovery

- Added non-E-Stop degraded-state handling driven by robot diagnostics.
- Behavior now:
  - `spot.status` publishes `Robot degraded`.
  - `spot.connection` publishes `degraded_non_estop` and `degraded_reason`.
  - Motion commands are soft-gated when Spot reports:
    - critical system faults,
    - critical service faults,
    - unclearable behavior faults, or
    - motor power error.
  - E-Stop is explicitly excluded from this soft recovery path.
  - When degraded is active, desired twist is cleared and motion is stopped.
  - Dock requests are rejected while degraded is active.
- Files:
  - `src/adapter.cpp`
  - `include/formant_spot_adapter/adapter.hpp`
  - `README.md`
  - `docs/formant-streams.md`

### Completed: Fix Docking Control Freeze Caused by Long API Mutex Hold

- Refactored dock command execution to avoid holding `SpotClient::api_mu_` for the full
  auto-dock feedback loop.
- Behavior now:
  - Dock command and feedback RPCs are guarded by short-lived locks only.
  - Other adapter operations (state polling, teleop handlers) continue while docking runs.
- Files:
  - `src/spot_client.cpp`

### Completed: Enforce Twist Staleness Timeout In Teleop Loop

- Added explicit stale twist protection in the main motion loop.
- Behavior now:
  - If desired twist is older than the configured timeout window, the adapter invalidates desired twist.
  - It clears desired velocity/body-pitch state.
  - It issues a zero-velocity stop if the robot was moving.
  - It emits a rate-limited log message for stale twist timeout events.
- Files:
  - `src/adapter.cpp`

### Completed: Add Resilient Formant Stream Reconnection Logic

- Added reconnect loops for teleop, command, and heartbeat streams.
- Behavior now:
  - If any stream closes unexpectedly, the client retries connection with bounded exponential backoff and jitter.
  - Existing shutdown cancellation behavior is preserved (`TryCancel` still cleanly stops loops).
- Files:
  - `src/formant_agent_client.cpp`
  - `include/formant_spot_adapter/formant_agent_client.hpp`

### Completed: Harden Adapter Output Behavior Under Formant Backpressure

- Added bounded adapter log pending payload handling.
- Behavior now:
  - Pending adapter log payload is capped to a fixed max size.
  - Oldest content is truncated with an explicit truncation marker when cap is exceeded.
- Added camera-post backoff behavior.
- Behavior now:
  - On image post failures, sender applies exponential backoff.
  - Frames are dropped while in backoff to prevent pressure buildup.
- Files:
  - `src/adapter.cpp`

### Validation

- Built successfully after changes:
  - `./scripts/build.sh`
- Result:
  - `formant-spot-adapter` linked successfully.

### Completed: Spot Availability Monitoring, Publishing, and Safe Auto-Reconnect

- Added adapter-level Spot connectivity management with in-process reconnect behavior.
- Behavior now:
  - Adapter no longer exits if initial Spot connection fails.
  - Background connection loop retries Spot connection with bounded exponential backoff.
  - While connected, periodic health checks detect sustained failures and transition to disconnected state.
  - Teleop motion/lease/dock and robot commands are gated while disconnected.
  - Adapter clears active control intent and marks lease/motion inactive when connection is lost.
- Added availability publishing to Formant:
  - New `spot.connection` JSON status stream.
  - Added `Robot available` key to `spot.status` bitset.
- Updated docs to include availability semantics and the new stream.
- Files:
  - `src/adapter.cpp`
  - `include/formant_spot_adapter/adapter.hpp`
  - `README.md`
  - `docs/formant-streams.md`

### Validation

- Built successfully after changes:
  - `./scripts/build.sh`
- Runtime smoke:
  - `timeout 10s ./scripts/run.sh`
- Result:
  - Adapter starts and runs normally.
  - Connection status loop active; process no longer depends on one-shot init success.
