# IMPLEMENTATION HISTORY

## 2026-02-17

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
