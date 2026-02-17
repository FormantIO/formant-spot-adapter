# GraphNav + Teleop Implementation Notes

This document summarizes the mapping/navigation and teleop-control work implemented in this adapter.

## Scope Implemented

- GraphNav map lifecycle commands under `spot.map.*`
- GraphNav waypoint alias workflow under `spot.waypoint.*`
- Persistent map/alias storage on adapter filesystem
- Continuous publishing of map/waypoint lists
- Navigation diagnostics and impaired auto-recovery behavior
- Teleop motion smoothing for pulsed joystick updates
- Additional robot-state telemetry for body pitch

## Map + Waypoint Command Set

All commands are namespaced `spot.*`.

### Map commands

- `spot.map.load` (text param: `map_id`)
- `spot.map.set_default` (text param: `map_id`)
- `spot.map.delete` (text param: `map_id`)
- `spot.map.start_mapping` (no params)
- `spot.map.stop_mapping` (no params)

### Waypoint commands

- `spot.waypoint.save` (text param: `name`)
- `spot.waypoint.update` (text param: `name`)
- `spot.waypoint.delete` (text param: `name`)
- `spot.waypoint.goto` (text param: `name`)

## Persistence Model

- Graph files and snapshots are stored under: `graphnavStoreDir/maps/<map_id>/`
- Adapter state (active/default map + aliases) is stored in: `graphnavStoreDir/state.txt`
- Waypoint aliases are map-scoped (`name -> waypoint_id`) and intentionally adapter-managed.

## Current Runtime Behavior

- If no active/default map exists, adapter auto-creates initial map ID and sets it as default.
- Default map is auto-selected as active at startup when needed.
- Active map restore is deferred until a lease is available (avoids lease wallet errors at boot).
- `spot.waypoint.save` / `spot.waypoint.update` works even when not currently recording:
  adapter starts a temporary recording session, creates waypoint, then stops recording.
- `spot.map.start_mapping` and `spot.map.stop_mapping` can be repeated on the same selected map to
  continue extending that map (append behavior).

## Published Streams Added/Used

- `spot.waypoints` (text, newline-separated aliases/labels for active map)
- `spot.maps` (text, newline-separated map IDs)
- `spot.map.progress` (json text from recording status)
- `spot.map.progress.waypoints` (numeric)
- `spot.map.progress.path_length_m` (numeric)
- `spot.map.progress.fiducials` (numeric)
- `spot.nav.feedback` (json text with GraphNav navigation feedback state)
- `spot.robot_state.body_pitch_rad` (numeric)

## Navigation Reliability Improvements

- Adapter tracks current GraphNav `command_id` and polls feedback.
- Teleop inactivity lease-return logic is suppressed while GraphNav navigation is active, so the
  adapter does not interrupt autonomous nav.
- On `spot.waypoint.goto`, if navigate request fails with robot-impaired status, adapter attempts:
  self-right -> stand -> one retry navigate.
- If navigation feedback later reports `STATUS_ROBOT_IMPAIRED`, adapter attempts one auto-recover
  retry for the current target waypoint.

## Teleop Motion Smoothing

- Teleop switched from packet-by-packet velocity sends to held-command behavior:
  latest twist is cached and continuously resent in the run loop while session/lease are valid.
- This removes stutter when joystick transport is pulsed.
- Manual held twist is cleared when navigation command starts.
- Short pitch-only zero dropouts are filtered (`~80ms`) to avoid pushup-like oscillation from
  jittery neutral pulses while still allowing immediate translational stop.

## Observed Integration Caveats

- Formant stream throttling (`grpc_code=8`) can hide diagnostics in cloud streams, especially
  `spot.adapter.log`. Local adapter stdout/stderr remains the most reliable debug source.
- If joystick mapping sends mostly zeros, robot behavior follows zeros regardless of hold logic.
  Expected joystick mapping is:
  - `linear.x` forward/back
  - `linear.y` strafe
  - `angular.z` yaw
  - `angular.y` body pitch

## Recommended Validation Checklist

1. Confirm Formant joystick axes match expected twist fields.
2. Validate `spot.waypoint.save` while mapping is off.
3. Validate map switch (`spot.map.load`) updates `spot.waypoints` output.
4. Validate default behavior by restart and observing active map restore.
5. Validate `spot.waypoint.goto` with temporary teleop inactivity; robot should continue nav.
6. Validate pitch hold/release and confirm no rapid oscillation from input pulse jitter.
