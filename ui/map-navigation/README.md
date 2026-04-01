# Spot Map Navigation Module

This directory contains a static single-page application intended to be hosted on GitHub Pages and embedded in Formant as a custom module.

## What it does

- Uses Formant iframe auth context via `@formant/data-sdk`
- Resolves the current device from Formant module context
- Loads the Spot GraphNav map image and companion metadata streams
- Renders waypoint and live-pose overlays
- Converts map clicks into seed-frame `(x, y, yaw)` targets
- Sends `spot.graphnav.goto_pose` commands back through Formant

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
