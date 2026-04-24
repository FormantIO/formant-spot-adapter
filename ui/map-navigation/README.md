# Spot Map Navigation Module

This directory contains a static single-page application intended to be hosted on GitHub Pages and embedded in Formant as a custom module.

## What it does

- Uses Formant iframe auth context via `@formant/data-sdk`
- Resolves the current device from Formant module context
- Opens a Formant realtime connection and subscribes to the Spot GraphNav map image, image metadata, overlay, and nav state streams
- Renders the live robot pose, waypoint overlay, active target, and readiness state
- Makes saved waypoint navigation the primary workflow, with arbitrary point navigation as an advanced mode
- Sends `spot.waypoint.goto`, `spot.graphnav.goto_pose`, `spot.graphnav.cancel`,
  `spot.return_and_dock`, and `spot.undock` through Formant. Waypoint navigation prefers stable
  `waypoint_id` values from `spot.graphnav.overlay`.
- Uses JSON command payloads with generated `request_id` values for navigation actions, then matches
  those IDs against `spot.nav.state` so UI notices clear only when the adapter has accepted that request.

By default the module depends on:

- `spot.localization.graphnav.global.image`
- `spot.localization.graphnav.global.image.meta`
- `spot.graphnav.overlay`
- `spot.nav.state`
- `spot.connection_state`
- `spot.docking_state`
- `spot.motor_power_state`
- `spot.behavior_state`

Recommended module configuration:

- `Global Map Image Stream`: `spot.localization.graphnav.global.image`
- `Global Map Image Metadata Stream`: `spot.localization.graphnav.global.image.meta`
- `GraphNav Overlay Stream`: `spot.graphnav.overlay`
- `Navigation State Stream`: `spot.nav.state`
- `Connection State Stream`: `spot.connection_state`
- `Docking State Stream`: `spot.docking_state`
- `Motor Power State Stream`: `spot.motor_power_state`
- `Behavior State Stream`: `spot.behavior_state`
- `Waypoint Goto Command Name`: `spot.waypoint.goto`
- `Goto Pose Command Name`: `spot.graphnav.goto_pose`
- `Cancel Navigation Command Name`: `spot.graphnav.cancel`
- `Return And Dock Command Name`: `spot.return_and_dock`
- `Undock Command Name`: `spot.undock`

`spot.graphnav.metadata` is intentionally not part of the default module contract. It can be too
large for some downstream query paths. The adapter publishes `spot.graphnav.overlay` instead as a
small UI-safe JSON stream for waypoint IDs, labels, dock status, and edge rendering.

## Local development

```bash
cd ui/map-navigation
npm install
npm run dev
```

Open the resulting URL from a Formant custom module, or provide Formant auth/device query parameters during development.

## Build

```bash
cd ui/map-navigation
npm run build
```

The app uses relative asset paths so the built output can be published directly on GitHub Pages.
