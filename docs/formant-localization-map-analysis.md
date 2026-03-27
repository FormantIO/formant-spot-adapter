# Formant Localization + Map Analysis

## Current State

- Formant already supports typed localization uploads with an occupancy grid in
  `Localization.map.occupancy_grid`.
- This adapter previously could not send that type because its older local
  Formant proto snapshot only exposed text/numeric/image payloads. The repo now
  vendors the public upstream Formant protos under
  [`proto/protos/model/v1/datapoint.proto`](../proto/protos/model/v1/datapoint.proto)
  and [`proto/protos/model/v1/navigation.proto`](../proto/protos/model/v1/navigation.proto).
- Spot already exposes map-related data in the local SDK checkout:
  - `GetLocalizationState(... request_live_terrain_maps=true)` returns live
    `WaypointSnapshot.robot_local_grids`.
  - `GetLocalizationState(... request_live_point_cloud=true)` can return the
    live point cloud used for localization.
  - `GetLocalizationState(... request_live_world_objects=true)` can return live
    world objects, including AprilTags/fiducials.
  - Downloaded GraphNav maps already contain waypoint snapshots with
    `robot_local_grids`, `point_cloud`, and `objects`.

## What Is Implemented Here

- Added typed Formant `Localization`, `Map`, `OccupancyGrid`, `Transform`, and
  `Odometry` messages through the vendored upstream Formant proto model.
- Added `FormantAgentClient::PostLocalization(...)`.
- Added `SpotClient::GetLocalizationMapSnapshot(...)`, which:
  - requests GraphNav localization plus live terrain maps,
  - prefers `no_step` local grids, then `obstacle_distance`, then
    `terrain_valid`/`terrain`,
  - decodes Spot `LocalGrid` cell formats and RLE/raw encodings,
  - converts the chosen grid into Formant occupancy values:
    - occupied: `100`
    - free: `0`
    - unknown: `-1`
  - places the grid in the GraphNav seed frame using `seed_tform_body` and the
    local-grid frame tree snapshot.
- The adapter now publishes a typed Formant localization stream on
  `spot.localization.graphnav`.

## Important Limitation

- `spot.localization.graphnav` is a live localized patch around the robot, not a
  stitched building-scale occupancy map.
- It should be enough to verify that Formant renders Spot occupancy-grid data
  correctly and that the frame math is right.
- It is not yet a persistent floorplan of the full GraphNav map.

## Best Next Steps

1. Validate the new stream in Formant’s localization viewer.
2. Inspect which live local-grid types your robot actually returns at runtime.
   The code is prepared for the common Spot names, but runtime confirmation is
   still useful.
3. Add live point-cloud overlays to the same localization datapoint.
   Formant supports `Localization.point_clouds`, and Spot exposes live
   localization point clouds plus recorded GraphNav snapshot point clouds.
4. Publish fiducials/world objects as overlays or auxiliary streams.
   The SDK already exposes AprilTag IDs, pose status, covariance, docks, and
   other world-object properties.
5. Build a stitched global map from downloaded GraphNav data.
   There are two realistic paths:
   - Rasterize anchored waypoint snapshot local grids into a shared seed frame.
   - Rasterize anchored GraphNav point clouds into a 2D occupancy map.

## Other Useful Spot Data For This Feature

- Live world objects / fiducials:
  - AprilTag ID
  - filtered/raw fiducial frame names
  - pose quality status
  - detection covariance
- GPS localization state:
  - available from `GetLocalizationState(... request_gps_state=true)` when GPS
    is configured
- Remote point cloud sensor status:
  - exposed in `GetLocalizationStateResponse.remote_cloud_status`
- GraphNav map anchoring:
  - waypoint anchors and anchored world objects in the seed frame

## Recommendation

- Treat `spot.localization.graphnav` as phase 1: verify Formant rendering and
  coordinate alignment.
- If the goal is a persistent site map, phase 2 should be a stitched GraphNav
  map built from downloaded waypoint snapshots rather than only live local
  terrain grids.
