# Spot Map Navigation Module

This directory contains a static single-page application intended to be hosted on GitHub Pages and embedded in Formant as a custom module.

## What it does

- Uses Formant iframe auth context via `@formant/data-sdk`
- Resolves the current device from Formant module context
- Loads the Spot GraphNav map image and companion metadata streams
- Renders the live robot pose and optional waypoint overlays
- Converts map clicks into seed-frame `(x, y, yaw)` targets
- Sends `spot.graphnav.goto_pose` commands back through Formant

By default the module only depends on:

- `spot.localization.graphnav.global.image`
- `spot.localization.graphnav.global.image.meta`
- `spot.nav.state`

`spot.graphnav.metadata` is optional and disabled by default because large text/json payloads can
be truncated by downstream query paths. Only configure an overlay metadata stream if it is kept
small enough for reliable iframe queries.

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
