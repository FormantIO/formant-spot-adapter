import {
  App,
  Authentication,
  Device,
  Fleet,
  LiveUniverseData,
  RealtimeStreamSource
} from "@formant/data-sdk";
import {
  GraphNavMapImageMetadata,
  GraphNavMetadata,
  ModuleConfig,
  NavState,
  StreamSnapshot
} from "./types";

export const DEFAULT_MODULE_CONFIG: ModuleConfig = {
  mapImageStreamName: "spot.localization.graphnav.global.image",
  mapImageMetadataStreamName: "spot.localization.graphnav.global.image.meta",
  graphnavMetadataStreamName: "",
  navStateStreamName: "spot.nav.state",
  gotoPoseCommandName: "spot.graphnav.goto_pose",
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

function asConfiguredName(value: unknown, fallback: string): string {
  if (typeof value !== "string") return fallback;
  const trimmed = value.trim();
  return trimmed.includes(".") ? trimmed : fallback;
}

function asOptionalConfiguredName(value: unknown): string {
  if (typeof value !== "string") return "";
  const trimmed = value.trim();
  return trimmed.includes(".") ? trimmed : "";
}

function parseGraphNavMapImageMetadataValue(
  value: unknown
): GraphNavMapImageMetadata | undefined {
  if (!isRecord(value)) return undefined;
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
}

function parseGraphNavMetadataValue(value: unknown): GraphNavMetadata | undefined {
  if (!isRecord(value)) return undefined;
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

function parseNavStateValue(value: unknown): NavState {
  if (!isRecord(value)) {
    return {
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
    };
  }

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
  };
}

export function parseModuleConfig(raw: string | undefined): ModuleConfig {
  if (!raw) return DEFAULT_MODULE_CONFIG;

  try {
    const value = JSON.parse(raw) as Partial<ModuleConfig>;
    return {
      mapImageStreamName: asConfiguredName(
        value.mapImageStreamName,
        DEFAULT_MODULE_CONFIG.mapImageStreamName
      ),
      mapImageMetadataStreamName: asConfiguredName(
        value.mapImageMetadataStreamName,
        DEFAULT_MODULE_CONFIG.mapImageMetadataStreamName
      ),
      graphnavMetadataStreamName: asOptionalConfiguredName(value.graphnavMetadataStreamName),
      navStateStreamName: asConfiguredName(
        value.navStateStreamName,
        DEFAULT_MODULE_CONFIG.navStateStreamName
      ),
      gotoPoseCommandName: asConfiguredName(
        value.gotoPoseCommandName,
        DEFAULT_MODULE_CONFIG.gotoPoseCommandName
      ),
      showWaypointLabels:
        typeof value.showWaypointLabels === "boolean"
          ? value.showWaypointLabels
          : DEFAULT_MODULE_CONFIG.showWaypointLabels,
      defaultYawMode:
        value.defaultYawMode === "fixed" ? "fixed" : DEFAULT_MODULE_CONFIG.defaultYawMode,
      defaultYawDeg:
        typeof value.defaultYawDeg === "number"
          ? value.defaultYawDeg
          : DEFAULT_MODULE_CONFIG.defaultYawDeg
    };
  } catch {
    return DEFAULT_MODULE_CONFIG;
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

function buildRealtimeSource(name: string, streamType: string): RealtimeStreamSource {
  return {
    id: `${streamType}:${name}`,
    sourceType: "realtime",
    rosTopicName: name,
    streamType
  };
}

export function subscribeRealtimeSnapshot(
  deviceId: string,
  config: ModuleConfig,
  onPatch: (patch: Partial<StreamSnapshot>) => void
): () => void {
  const live = new LiveUniverseData();
  const cleanups: Array<() => void> = [];

  cleanups.push(
    live.subscribeToJson(
      deviceId,
      buildRealtimeSource(config.navStateStreamName, "json"),
      (value) => {
        onPatch({
          navState: parseNavStateValue(value),
          navStateTime: Date.now()
        });
      }
    )
  );

  cleanups.push(
    live.subscribeToJson(
      deviceId,
      buildRealtimeSource(config.mapImageMetadataStreamName, "json"),
      (value) => {
        const mapImageMetadata = parseGraphNavMapImageMetadataValue(value);
        if (!mapImageMetadata) {
          onPatch({
            warnings: [`Could not parse ${config.mapImageMetadataStreamName}`]
          });
          return;
        }
        onPatch({
          mapImageMetadata,
          mapImageMetadataTime: Date.now()
        });
      }
    )
  );

  cleanups.push(
    live.subscribeToVideo(
      deviceId,
      buildRealtimeSource(config.mapImageStreamName, "video"),
      (canvas) => {
        onPatch({
          mapImageCanvas: canvas,
          mapImageFrameVersion: Date.now(),
          mapImageTime: Date.now()
        });
      }
    )
  );

  if (config.graphnavMetadataStreamName) {
    cleanups.push(
      live.subscribeToJson(
        deviceId,
        buildRealtimeSource(config.graphnavMetadataStreamName, "json"),
        (value) => {
          const graphnavMetadata = parseGraphNavMetadataValue(value);
          if (!graphnavMetadata) {
            onPatch({
              warnings: [`Could not parse ${config.graphnavMetadataStreamName}`]
            });
            return;
          }
          onPatch({
            graphnavMetadata,
            graphnavMetadataTime: Date.now()
          });
        }
      )
    );
  }

  return () => {
    cleanups.forEach((cleanup) => {
      try {
        cleanup();
      } catch {
        // ignore cleanup errors from stale realtime subscriptions
      }
    });
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
