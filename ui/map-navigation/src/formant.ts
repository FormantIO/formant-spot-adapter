import { App, Authentication, Device, Fleet, IStreamData } from "@formant/data-sdk";
import {
  GraphNavMapImageMetadata,
  GraphNavMetadata,
  ModuleConfig,
  NavState,
  StreamSnapshot
} from "./types";

const DEFAULT_CONFIG: ModuleConfig = {
  mapImageStreamName: "spot.localization.graphnav.global.image",
  mapImageMetadataStreamName: "spot.localization.graphnav.global.image.meta",
  graphnavMetadataStreamName: "",
  navStateStreamName: "spot.nav.state",
  gotoPoseCommandName: "spot.graphnav.goto_pose",
  pollIntervalMs: 2500,
  showWaypointLabels: true,
  defaultYawMode: "current",
  defaultYawDeg: 0
};

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null;
}

function asString(value: unknown, fallback = ""): string {
  return typeof value === "string" ? value : fallback;
}

function asNumber(value: unknown, fallback = 0): number {
  return typeof value === "number" && Number.isFinite(value) ? value : fallback;
}

function asBoolean(value: unknown, fallback = false): boolean {
  return typeof value === "boolean" ? value : fallback;
}

function parseJsonWithFallback<T>(
  raw: string,
  parser: (value: Record<string, unknown>) => T
): T | undefined {
  try {
    const value = JSON.parse(raw) as Record<string, unknown>;
    return parser(value);
  } catch {
    return undefined;
  }
}

function parseGraphNavMapImageMetadata(raw: string): GraphNavMapImageMetadata | undefined {
  return parseJsonWithFallback(raw, (value) => {
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
      current_seed_x:
        typeof value.current_seed_x === "number" ? value.current_seed_x : undefined,
      current_seed_y:
        typeof value.current_seed_y === "number" ? value.current_seed_y : undefined,
      current_seed_z:
        typeof value.current_seed_z === "number" ? value.current_seed_z : undefined,
      current_seed_yaw_rad:
        typeof value.current_seed_yaw_rad === "number" ? value.current_seed_yaw_rad : undefined,
      canvas,
      draw_rect: drawRect,
      render_scale:
        typeof value.render_scale === "number" ? value.render_scale : undefined,
      map
    };
  });
}

function parseGraphNavMetadata(raw: string): GraphNavMetadata | undefined {
  return parseJsonWithFallback(raw, (value) => {
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
  });
}

function parseNavState(raw: string): NavState {
  return (
    parseJsonWithFallback(raw, (value) => ({
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
      current_seed_x:
        typeof value.current_seed_x === "number" ? value.current_seed_x : undefined,
      current_seed_y:
        typeof value.current_seed_y === "number" ? value.current_seed_y : undefined,
      current_seed_z:
        typeof value.current_seed_z === "number" ? value.current_seed_z : undefined,
      current_seed_yaw_rad:
        typeof value.current_seed_yaw_rad === "number" ? value.current_seed_yaw_rad : undefined,
      has_seed_goal: asBoolean(value.has_seed_goal),
      target_seed_x:
        typeof value.target_seed_x === "number" ? value.target_seed_x : undefined,
      target_seed_y:
        typeof value.target_seed_y === "number" ? value.target_seed_y : undefined,
      target_seed_yaw_rad:
        typeof value.target_seed_yaw_rad === "number" ? value.target_seed_yaw_rad : undefined
    })) || {
      connected: false,
      active: false,
      command_id: 0,
      status: 0,
      status_name: "",
      remaining_route_length_m: 0,
      auto_recovered: false,
      mode: "",
      target_waypoint_id: "",
      target_name: "",
      map_id: "",
      map_uuid: "",
      localized: false,
      current_waypoint_id: "",
      has_current_seed_pose: false,
      has_seed_goal: false
    }
  );
}

function getLastPoint(stream?: IStreamData): [number, unknown] | undefined {
  if (!stream || stream.points.length === 0) return undefined;
  return stream.points[stream.points.length - 1];
}

export function parseModuleConfig(raw: string | undefined): ModuleConfig {
  if (!raw) return DEFAULT_CONFIG;

  try {
    const value = JSON.parse(raw) as Partial<ModuleConfig>;
    return {
      mapImageStreamName:
        typeof value.mapImageStreamName === "string"
          ? value.mapImageStreamName
          : DEFAULT_CONFIG.mapImageStreamName,
      mapImageMetadataStreamName:
        typeof value.mapImageMetadataStreamName === "string"
          ? value.mapImageMetadataStreamName
          : DEFAULT_CONFIG.mapImageMetadataStreamName,
      graphnavMetadataStreamName:
        typeof value.graphnavMetadataStreamName === "string"
          ? value.graphnavMetadataStreamName
          : DEFAULT_CONFIG.graphnavMetadataStreamName,
      navStateStreamName:
        typeof value.navStateStreamName === "string"
          ? value.navStateStreamName
          : DEFAULT_CONFIG.navStateStreamName,
      gotoPoseCommandName:
        typeof value.gotoPoseCommandName === "string"
          ? value.gotoPoseCommandName
          : DEFAULT_CONFIG.gotoPoseCommandName,
      pollIntervalMs:
        typeof value.pollIntervalMs === "number" && value.pollIntervalMs >= 500
          ? value.pollIntervalMs
          : DEFAULT_CONFIG.pollIntervalMs,
      showWaypointLabels:
        typeof value.showWaypointLabels === "boolean"
          ? value.showWaypointLabels
          : DEFAULT_CONFIG.showWaypointLabels,
      defaultYawMode:
        value.defaultYawMode === "fixed" ? "fixed" : DEFAULT_CONFIG.defaultYawMode,
      defaultYawDeg:
        typeof value.defaultYawDeg === "number"
          ? value.defaultYawDeg
          : DEFAULT_CONFIG.defaultYawDeg
    };
  } catch {
    return DEFAULT_CONFIG;
  }
}

export async function authenticateAndGetDevice(): Promise<Device> {
  const authenticated = await Authentication.waitTilAuthenticated();
  if (!authenticated) {
    throw new Error("Authentication failed. Open this module from Formant or include auth context in the URL.");
  }
  return Fleet.getCurrentDevice();
}

export async function getInitialModuleConfig(): Promise<ModuleConfig> {
  return parseModuleConfig(await App.getCurrentModuleConfiguration());
}

export async function fetchStreamSnapshot(
  deviceId: string,
  config: ModuleConfig
): Promise<StreamSnapshot> {
  const end = new Date();
  const start = new Date(end.getTime() - (5 * 60 * 1000));
  const names = [
    config.mapImageStreamName,
    config.mapImageMetadataStreamName,
    config.navStateStreamName
  ];
  if (config.graphnavMetadataStreamName) {
    names.push(config.graphnavMetadataStreamName);
  }
  const streams = await Fleet.queryTelemetry({
    deviceIds: [deviceId],
    names,
    types: ["image", "text"],
    start: start.toISOString(),
    end: end.toISOString()
  });

  const byName = new Map<string, IStreamData>(
    streams.map((stream: IStreamData) => [stream.name, stream])
  );

  const mapImagePoint = getLastPoint(byName.get(config.mapImageStreamName));
  const metaPoint = getLastPoint(byName.get(config.mapImageMetadataStreamName));
  const graphnavPoint = config.graphnavMetadataStreamName
    ? getLastPoint(byName.get(config.graphnavMetadataStreamName))
    : undefined;
  const navStatePoint = getLastPoint(byName.get(config.navStateStreamName));
  const warnings: string[] = [];
  const mapImageMetadata =
    typeof metaPoint?.[1] === "string"
      ? parseGraphNavMapImageMetadata(metaPoint[1])
      : undefined;
  if (typeof metaPoint?.[1] === "string" && !mapImageMetadata) {
    warnings.push(`Could not parse ${config.mapImageMetadataStreamName}`);
  }
  const graphnavMetadata =
    typeof graphnavPoint?.[1] === "string"
      ? parseGraphNavMetadata(graphnavPoint[1])
      : undefined;
  if (typeof graphnavPoint?.[1] === "string" && !graphnavMetadata) {
    warnings.push(`Could not parse ${config.graphnavMetadataStreamName}`);
  }
  const navState =
    typeof navStatePoint?.[1] === "string"
      ? parseNavState(navStatePoint[1])
      : undefined;

  return {
    mapImageUrl:
      isRecord(mapImagePoint?.[1]) && typeof mapImagePoint[1].url === "string"
        ? mapImagePoint[1].url
        : undefined,
    mapImageTime: mapImagePoint?.[0],
    mapImageMetadata,
    mapImageMetadataTime: metaPoint?.[0],
    graphnavMetadata,
    graphnavMetadataTime: graphnavPoint?.[0],
    navState,
    navStateTime: navStatePoint?.[0],
    warnings
  };
}

export function formatGotoPosePayload(
  mapUuid: string,
  x: number,
  y: number,
  yawDeg: number
): string {
  return `map_uuid=${mapUuid}, x=${x.toFixed(3)}, y=${y.toFixed(3)}, yaw_deg=${yawDeg.toFixed(1)}`;
}
