import { App, Authentication, Fleet } from "@formant/data-sdk";
const DEFAULT_CONFIG = {
    mapImageStreamName: "spot.localization.graphnav.global.image",
    mapImageMetadataStreamName: "spot.localization.graphnav.global.image.meta",
    graphnavMetadataStreamName: "spot.graphnav.metadata",
    navStateStreamName: "spot.nav.state",
    gotoPoseCommandName: "spot.graphnav.goto_pose",
    pollIntervalMs: 2500,
    showWaypointLabels: true,
    defaultYawMode: "current",
    defaultYawDeg: 0
};
function isRecord(value) {
    return typeof value === "object" && value !== null;
}
function asString(value, fallback = "") {
    return typeof value === "string" ? value : fallback;
}
function asNumber(value, fallback = 0) {
    return typeof value === "number" && Number.isFinite(value) ? value : fallback;
}
function asBoolean(value, fallback = false) {
    return typeof value === "boolean" ? value : fallback;
}
function parseGraphNavMapImageMetadata(raw) {
    const value = JSON.parse(raw);
    const canvas = isRecord(value.canvas)
        ? {
            width: asNumber(value.canvas.width, 1280),
            height: asNumber(value.canvas.height, 720)
        }
        : undefined;
    const drawRect = isRecord(value.draw_rect)
        ? {
            x: asNumber(value.draw_rect.x),
            y: asNumber(value.draw_rect.y),
            width: asNumber(value.draw_rect.width),
            height: asNumber(value.draw_rect.height)
        }
        : undefined;
    const map = isRecord(value.map) && isRecord(value.map.seed_tform_grid)
        ? {
            width: asNumber(value.map.width),
            height: asNumber(value.map.height),
            resolution_m: asNumber(value.map.resolution_m),
            seed_tform_grid: {
                x: asNumber(value.map.seed_tform_grid.x),
                y: asNumber(value.map.seed_tform_grid.y),
                z: asNumber(value.map.seed_tform_grid.z),
                qw: asNumber(value.map.seed_tform_grid.qw, 1),
                qx: asNumber(value.map.seed_tform_grid.qx),
                qy: asNumber(value.map.seed_tform_grid.qy),
                qz: asNumber(value.map.seed_tform_grid.qz)
            }
        }
        : undefined;
    return {
        image_stream: asString(value.image_stream),
        metadata_stream: asString(value.metadata_stream),
        has_map: asBoolean(value.has_map),
        status_title: asString(value.status_title),
        status_detail: asString(value.status_detail),
        map_id: asString(value.map_id),
        map_uuid: asString(value.map_uuid),
        localized: asBoolean(value.localized),
        current_waypoint_id: asString(value.current_waypoint_id),
        has_live_pose_overlay: asBoolean(value.has_live_pose_overlay),
        current_seed_x: typeof value.current_seed_x === "number" ? value.current_seed_x : undefined,
        current_seed_y: typeof value.current_seed_y === "number" ? value.current_seed_y : undefined,
        current_seed_z: typeof value.current_seed_z === "number" ? value.current_seed_z : undefined,
        current_seed_yaw_rad: typeof value.current_seed_yaw_rad === "number" ? value.current_seed_yaw_rad : undefined,
        canvas,
        draw_rect: drawRect,
        render_scale: typeof value.render_scale === "number" ? value.render_scale : undefined,
        map
    };
}
function parseGraphNavMetadata(raw) {
    const value = JSON.parse(raw);
    const waypoints = Array.isArray(value.waypoints)
        ? value.waypoints
            .filter(isRecord)
            .map((waypoint) => ({
            id: asString(waypoint.id),
            label: asString(waypoint.label),
            display_name: asString(waypoint.display_name),
            is_dock: asBoolean(waypoint.is_dock),
            seed_tform_waypoint: isRecord(waypoint.seed_tform_waypoint)
                ? {
                    x: asNumber(waypoint.seed_tform_waypoint.x),
                    y: asNumber(waypoint.seed_tform_waypoint.y),
                    z: asNumber(waypoint.seed_tform_waypoint.z),
                    qw: asNumber(waypoint.seed_tform_waypoint.qw, 1),
                    qx: asNumber(waypoint.seed_tform_waypoint.qx),
                    qy: asNumber(waypoint.seed_tform_waypoint.qy),
                    qz: asNumber(waypoint.seed_tform_waypoint.qz)
                }
                : { x: 0, y: 0, z: 0, qw: 1, qx: 0, qy: 0, qz: 0 }
        }))
        : [];
    const edges = Array.isArray(value.edges)
        ? value.edges
            .filter(isRecord)
            .map((edge) => ({
            from_waypoint_id: asString(edge.from_waypoint_id),
            to_waypoint_id: asString(edge.to_waypoint_id)
        }))
        : [];
    return {
        map_id: asString(value.map_id),
        map_uuid: asString(value.map_uuid),
        waypoints,
        edges
    };
}
function parseNavState(raw) {
    const value = JSON.parse(raw);
    return {
        connected: asBoolean(value.connected),
        active: asBoolean(value.active),
        command_id: asNumber(value.command_id),
        status: asNumber(value.status),
        status_name: asString(value.status_name),
        remaining_route_length_m: asNumber(value.remaining_route_length_m),
        auto_recovered: asBoolean(value.auto_recovered),
        mode: asString(value.mode),
        target_waypoint_id: asString(value.target_waypoint_id),
        target_name: asString(value.target_name),
        map_id: asString(value.map_id),
        map_uuid: asString(value.map_uuid),
        localized: asBoolean(value.localized),
        current_waypoint_id: asString(value.current_waypoint_id),
        has_current_seed_pose: asBoolean(value.has_current_seed_pose),
        current_seed_x: typeof value.current_seed_x === "number" ? value.current_seed_x : undefined,
        current_seed_y: typeof value.current_seed_y === "number" ? value.current_seed_y : undefined,
        current_seed_z: typeof value.current_seed_z === "number" ? value.current_seed_z : undefined,
        current_seed_yaw_rad: typeof value.current_seed_yaw_rad === "number" ? value.current_seed_yaw_rad : undefined,
        has_seed_goal: asBoolean(value.has_seed_goal),
        target_seed_x: typeof value.target_seed_x === "number" ? value.target_seed_x : undefined,
        target_seed_y: typeof value.target_seed_y === "number" ? value.target_seed_y : undefined,
        target_seed_yaw_rad: typeof value.target_seed_yaw_rad === "number" ? value.target_seed_yaw_rad : undefined
    };
}
function getLastPoint(stream) {
    if (!stream || stream.points.length === 0)
        return undefined;
    return stream.points[stream.points.length - 1];
}
export function parseModuleConfig(raw) {
    if (!raw)
        return DEFAULT_CONFIG;
    try {
        const value = JSON.parse(raw);
        return {
            mapImageStreamName: value.mapImageStreamName || DEFAULT_CONFIG.mapImageStreamName,
            mapImageMetadataStreamName: value.mapImageMetadataStreamName || DEFAULT_CONFIG.mapImageMetadataStreamName,
            graphnavMetadataStreamName: value.graphnavMetadataStreamName || DEFAULT_CONFIG.graphnavMetadataStreamName,
            navStateStreamName: value.navStateStreamName || DEFAULT_CONFIG.navStateStreamName,
            gotoPoseCommandName: value.gotoPoseCommandName || DEFAULT_CONFIG.gotoPoseCommandName,
            pollIntervalMs: typeof value.pollIntervalMs === "number" && value.pollIntervalMs >= 500
                ? value.pollIntervalMs
                : DEFAULT_CONFIG.pollIntervalMs,
            showWaypointLabels: typeof value.showWaypointLabels === "boolean"
                ? value.showWaypointLabels
                : DEFAULT_CONFIG.showWaypointLabels,
            defaultYawMode: value.defaultYawMode === "fixed" ? "fixed" : DEFAULT_CONFIG.defaultYawMode,
            defaultYawDeg: typeof value.defaultYawDeg === "number"
                ? value.defaultYawDeg
                : DEFAULT_CONFIG.defaultYawDeg
        };
    }
    catch {
        return DEFAULT_CONFIG;
    }
}
export async function authenticateAndGetDevice() {
    const authenticated = await Authentication.waitTilAuthenticated();
    if (!authenticated) {
        throw new Error("Authentication failed. Open this module from Formant or include auth context in the URL.");
    }
    return Fleet.getCurrentDevice();
}
export async function getInitialModuleConfig() {
    return parseModuleConfig(await App.getCurrentModuleConfiguration());
}
export async function fetchStreamSnapshot(deviceId, config) {
    const end = new Date();
    const start = new Date(end.getTime() - (5 * 60 * 1000));
    const streams = await Fleet.queryTelemetry({
        deviceIds: [deviceId],
        names: [
            config.mapImageStreamName,
            config.mapImageMetadataStreamName,
            config.graphnavMetadataStreamName,
            config.navStateStreamName
        ],
        types: ["image", "text"],
        start: start.toISOString(),
        end: end.toISOString()
    });
    const byName = new Map(streams.map((stream) => [stream.name, stream]));
    const mapImagePoint = getLastPoint(byName.get(config.mapImageStreamName));
    const metaPoint = getLastPoint(byName.get(config.mapImageMetadataStreamName));
    const graphnavPoint = getLastPoint(byName.get(config.graphnavMetadataStreamName));
    const navStatePoint = getLastPoint(byName.get(config.navStateStreamName));
    return {
        mapImageUrl: isRecord(mapImagePoint?.[1]) && typeof mapImagePoint[1].url === "string"
            ? mapImagePoint[1].url
            : undefined,
        mapImageTime: mapImagePoint?.[0],
        mapImageMetadata: typeof metaPoint?.[1] === "string"
            ? parseGraphNavMapImageMetadata(metaPoint[1])
            : undefined,
        mapImageMetadataTime: metaPoint?.[0],
        graphnavMetadata: typeof graphnavPoint?.[1] === "string"
            ? parseGraphNavMetadata(graphnavPoint[1])
            : undefined,
        graphnavMetadataTime: graphnavPoint?.[0],
        navState: typeof navStatePoint?.[1] === "string"
            ? parseNavState(navStatePoint[1])
            : undefined,
        navStateTime: navStatePoint?.[0]
    };
}
export function formatGotoPosePayload(mapUuid, x, y, yawDeg) {
    return `map_uuid=${mapUuid}, x=${x.toFixed(3)}, y=${y.toFixed(3)}, yaw_deg=${yawDeg.toFixed(1)}`;
}
