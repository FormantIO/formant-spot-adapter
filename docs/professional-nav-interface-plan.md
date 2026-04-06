# Professional Spot Navigation Interface Plan

## Purpose

Define a production-grade navigation interface for Spot inside Formant custom modules.

This document covers:

- product intent
- operator workflows
- UI design direction
- backend contract changes
- implementation phases
- rollout and validation

This plan is for the local repository at `formant-spot-adapter`. It is not a remote deployment artifact.

## Background

The current custom module at `ui/map-navigation` is a good prototype:

- it authenticates inside a Formant iframe
- subscribes to realtime map image, map image metadata, and nav state
- renders the global GraphNav map
- converts clicks into seed-frame goals
- sends `spot.graphnav.goto_pose`

That is enough to prove the contract, but it is not yet a professional operator surface.

The production target should borrow structural lessons from:

- QGroundControl: clear separation of planning and execution
- DJI mission planning tools: waypoint-first workflows and route clarity
- Boston Dynamics Orbit: map as the operational center of gravity

The result should still be Spot-specific and grounded in the actual adapter commands and streams we have.

## Product Goals

- Make waypoint navigation easy, fast, and low-risk.
- Support arbitrary point navigation when needed, but treat it as an advanced workflow.
- Make robot readiness obvious before any motion command can be sent.
- Show live execution state clearly enough that operators trust the interface.
- Keep the main interface concise enough for real operational use.
- Split authoring/admin tasks away from the main nav surface.

## Non-Goals

- Full mission-editor parity with QGroundControl
- Full fleet/site management parity with Orbit
- Generic robot administration in the main navigation module
- Map recording and map lifecycle management inside the primary operator UI

## Operator Use Cases

Primary use cases:

- send Spot to a saved waypoint
- send Spot to dock / return and dock
- undock Spot
- send Spot to an arbitrary point on the current map
- monitor live nav progress and intervene when needed
- recover from common execution failures by switching to teleop or recovery actions

Secondary use cases:

- inspect current robot location on the map
- inspect current localization and map context
- verify which waypoint / dock / target is active

Authoring/admin use cases should live elsewhere:

- create/load/delete maps
- start/stop mapping
- save/update/delete waypoints
- configure dock waypoint

## Design Principles

### 1. Waypoint-first, point-second

Named waypoints are safer, more legible, and more auditable than free-click targets.

The main UI should make waypoint dispatch the default and arbitrary point navigation a secondary mode.

### 2. Readiness before action

Operators should never have to guess whether navigation is currently safe to issue.

If the robot is not ready, the UI should explain why in plain language and disable movement actions.

### 3. Outcome over dispatch

The interface must not imply success when a command is merely accepted by Formant.

The UI should reflect actual command lifecycle:

- queued
- accepted
- navigating
- reached
- failed
- canceled

### 4. Concise by default

Operational modules should surface the minimum high-value information first. Diagnostics belong behind an info panel or in a separate module.

### 5. Explicit mode transitions

The module should have clear states:

- observe
- select
- confirm
- execute
- resolve failure

### 6. Split operational surfaces

Keep the main navigation module focused. Use separate modules for:

- navigation operations
- map/waypoint authoring
- teleop fallback

## Recommended Product Shape

### Module 1: Spot Navigation

This is the primary operator module.

Responsibilities:

- show live map
- show live robot pose
- show waypoint overlays
- select waypoint or point target
- confirm target
- dispatch navigation
- show nav progress and current execution state
- allow cancel / return-and-dock / undock / open teleop

### Module 2: Spot Map Authoring

This is a separate tool for technicians and deployment workflows.

Responsibilities:

- map load/default/delete
- map recording start/stop
- waypoint save/update/delete
- dock waypoint management

### Module 3: Spot Teleop

This remains the live fallback/control surface.

Responsibilities:

- cameras
- joystick
- stand/sit/recover
- locomotion mode
- dock / undock / reset arm

## Navigation Module Layout

### Top Status Strip

Always visible:

- current map name
- localized / not localized
- connection state
- docking state
- motor power state
- battery
- current waypoint
- active nav status when relevant

This strip should be compact and color-disciplined. It should not look like a dashboard wall.

### Main Map Canvas

Always visible:

- global GraphNav map image
- robot marker with heading
- current active target marker
- waypoint markers
- dock markers visually distinct from standard waypoints

Optional overlays:

- edges between waypoints
- route segment preview for selected waypoint

### Right-Side Context Panel

Mode-dependent:

- default mode: searchable waypoint list, dock surfaced near top, current waypoint highlighted
- point mode: selected pose summary and heading controls
- active nav mode: destination, route length, progress, retry/cancel/open teleop
- failure mode: failure reason, recommended next actions

### Bottom Interaction Bar

Small, explicit state cue:

- no target: prompt to select waypoint or point mode
- target selected: confirm / cancel
- nav active: active mission summary and cancel control

## Recommended Primary Interaction Model

### Default Mode: Waypoint Navigation

Default operator flow:

1. choose a waypoint from the list or click a waypoint marker
2. review summary
3. confirm navigation
4. monitor execution

The interface should show:

- waypoint name
- dock/not dock
- current map
- whether the robot is already there

### Advanced Mode: Go To Point

Point mode should be explicit, not the default click behavior.

Flow:

1. operator enters point mode
2. clicks the map
3. target marker appears
4. operator chooses heading behavior
5. operator confirms or cancels

Recommended heading options:

- face path
- keep current heading
- custom heading

Do not default to exposing a raw degree input first.

### Active Navigation Mode

When navigation is active, the interface should prioritize:

- destination
- current nav state
- remaining route length
- auto-recovery status
- cancel
- return to teleop

Avoid allowing accidental reselection during active execution unless the operator explicitly chooses `Replace Target`.

## Readiness and Safety Gating

Before enabling movement actions, require:

- current map image metadata is fresh
- current nav state is fresh
- `spot.connection_state = connected`
- `spot.motor_power_state = on`
- `spot.docking_state = undocked` for movement actions
- `spot.nav.state.localized = true`
- target `map_uuid` matches the currently active map

Recommended additional soft warnings:

- `spot.behavior_state != standing`
- active faults present
- low battery
- robot currently docked but operator is issuing a movement command

If movement is blocked, the UI should tell the operator exactly why.

Example:

- `Cannot navigate: robot is docked`
- `Cannot navigate: robot is not localized`
- `Cannot navigate: motor power is off`

## Backend Contract Changes Recommended

The current backend is close, but not complete enough for a professional UI.

### 1. Add a UI-safe waypoint overlay stream

Add a small JSON stream, proposed name:

- `spot.graphnav.overlay`

Fields:

- `map_id`
- `map_uuid`
- `current_waypoint_id`
- `dock_waypoint_id`
- `waypoints: [{id, name, x, y, is_dock}]`
- `edges: [{from_waypoint_id, to_waypoint_id}]`

This should replace use of the large `spot.graphnav.metadata` blob for the iframe UI.

### 2. Add navigation cancel command

Add:

- `spot.graphnav.cancel`

This is required for a complete execution surface.

### 3. Extend nav state

Extend `spot.nav.state` with:

- `terminal_result`
- `failure_reason`
- `updated_at`
- `resolved_anchor_waypoint_id`
- `resolved_anchor_waypoint_name`
- `command_request_id` or equivalent correlation token

### 4. Preserve UI-safe render metadata

Keep:

- `spot.localization.graphnav.global.image`
- `spot.localization.graphnav.global.image.meta`
- `spot.nav.state`

These are the core map-navigation contract.

### 5. Use one-word state streams as operator gates

Use the existing streams directly in the UI:

- `spot.connection_state`
- `spot.docking_state`
- `spot.motor_power_state`
- `spot.behavior_state`
- `spot.commanded_motion_mode`
- `spot.shore_power_state`

## UI Engineering Requirements

### Correct Click Geometry

The current prototype uses simple bounding-rect scaling. The production UI should use the true rendered content transform so clicks remain correct under iframe resizing and letterboxing.

### Realtime-Only State Path

The navigation module should use realtime streams for:

- map image
- map image metadata
- nav state
- overlay stream
- readiness streams

Polling should not be part of the primary runtime path.

### Explicit Staleness Handling

Each live input should have freshness tracking. The UI should distinguish:

- never received
- stale
- live

### Error Model

The module should distinguish:

- auth/context failure
- realtime connection failure
- missing stream configuration
- robot not ready
- command rejected before execution
- nav failed during execution

Each class needs different operator messaging.

### Accessibility and Ergonomics

- target sizes should work on a touchscreen
- waypoint selection should work with mouse and keyboard
- colors should not be the only status signal
- text should remain readable over the map
- the module should remain usable at teleop panel sizes, not just full-screen

## Visual Design Direction

The visual tone should be:

- industrial
- calm
- precise
- not cluttered

Avoid:

- dashboard noise
- decorative flourishes that fight map legibility
- unnecessary panels open by default

Preferred visual character:

- restrained dark or neutral background
- bright but limited accent colors
- clear status colors with semantic discipline
- strong map readability first

## Implementation Plan

### Phase 1: Core Hardening

Goal:

- make the current prototype operationally trustworthy

Work:

- fix click geometry under resize/letterboxing
- add freshness tracking for all live inputs
- gate movement actions on readiness
- convert success messaging from dispatch-based to outcome-based
- improve error surfaces
- add tests for pixel-to-seed and seed-to-pixel round-trips in the UI layer

Exit criteria:

- point selection remains correct across responsive sizes
- no movement command can be sent while obvious blockers are present
- UI clearly distinguishes pending vs active vs terminal nav states

### Phase 2: Waypoint-First Production UX

Goal:

- make waypoint dispatch the primary workflow

Work:

- add `spot.graphnav.overlay`
- add searchable waypoint list and waypoint marker selection
- visually distinguish dock waypoint
- add current waypoint highlighting
- move arbitrary point navigation behind an explicit mode toggle

Exit criteria:

- operators can navigate by waypoint without needing arbitrary coordinates
- overlay stream is small and reliable in iframe realtime usage

### Phase 3: Execution Control

Goal:

- make live navigation manageable, not just initiatable

Work:

- add `spot.graphnav.cancel`
- add replace-target flow
- surface terminal failure reason and recommended next actions
- add open-teleop affordance from active nav state
- add return-and-dock as a first-class action

Exit criteria:

- operator can safely start, monitor, cancel, and resolve nav actions

### Phase 4: Authoring Split

Goal:

- keep navigation UI clean by moving map-admin tasks out

Work:

- create separate map authoring module
- expose map load/default/delete
- expose waypoint save/update/delete
- expose mapping start/stop
- expose dock waypoint assignment

Exit criteria:

- main nav module remains concise
- deployment/admin actions are still available in a separate tool

### Phase 5: Validation and Rollout

Goal:

- prove production readiness under real operating conditions

Work:

- live testing on docked, undocked, localized, and non-localized states
- command/result correlation checks
- stale-stream behavior testing
- resize/layout testing in real Formant iframe sizes
- operator feedback pass with at least one non-author user

Exit criteria:

- operators can complete common workflows without coaching
- command failures are understandable
- no known unsafe or ambiguous states remain in the core flow

## Validation Checklist

- waypoint selection is correct
- point selection is correct
- current robot marker is correct
- map/target mismatch is blocked
- docked robot cannot be sent to a movement command without undocking
- non-localized robot cannot be sent to nav
- active navigation shows live progress
- canceled navigation clears correctly
- failed navigation explains why
- switching maps invalidates stale selections
- module remains usable at realistic teleop panel sizes

## Success Criteria

This interface is successful when:

- a trained operator can use it confidently without needing backend knowledge
- the UI defaults to safe actions
- waypoint navigation is the fastest path
- arbitrary point navigation is available but controlled
- live nav state is understandable at a glance
- admin complexity is separated from runtime operation

## Recommended Immediate Next Step

Start with Phase 1 and Phase 2 together:

- harden the current module
- add the small overlay stream
- make waypoint navigation the primary workflow

That will produce the biggest jump from prototype to professional interface without overextending into mission-authoring scope too early.
