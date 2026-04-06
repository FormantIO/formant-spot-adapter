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
  GraphNavOverlay,
  ModuleConfig,
  NavState,
  StreamSnapshot
} from "./types";

export const DEFAULT_MODULE_CONFIG: ModuleConfig = {
  mapImageStreamName: "spot.localization.graphnav.global.image",
  mapImageMetadataStreamName: "spot.localization.graphnav.global.image.meta",
  overlayStreamName: "spot.graphnav.overlay",
  navStateStreamName: "spot.nav.state",
  connectionStateStreamName: "spot.connection_state",
  dockingStateStreamName: "spot.docking_state",
  motorPowerStateStreamName: "spot.motor_power_state",
  behaviorStateStreamName: "spot.behavior_state",
  waypointGotoCommandName: "spot.waypoint.goto",
  gotoPoseCommandName: "spot.graphnav.goto_pose",
  cancelNavCommandName: "spot.graphnav.cancel",
  returnAndDockCommandName: "spot.return_and_dock",
  undockCommandName: "spot.undock",
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

function parseGraphNavOverlayValue(value: unknown): GraphNavOverlay | undefined {
  if (!isRecord(value)) return undefined;
  const waypoints = Array.isArray(value.waypoints)
    ? value.waypoints
        .filter(isRecord)
        .map((waypoint) => ({
          id: asString(waypoint.id),
          name: asString(waypoint.name),
          label: asString(waypoint.label),
          x: asNumber(waypoint.x),
          y: asNumber(waypoint.y),
          is_dock: asBoolean(waypoint.is_dock)
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
    current_waypoint_id: asString(value.current_waypoint_id),
    dock_waypoint_id: asString(value.dock_waypoint_id),
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
    has_waypoint_goal: asBoolean(value.has_waypoint_goal),
    target_seed_x:
      typeof value.target_seed_x === "number" ? value.target_seed_x : undefined,
    target_seed_y:
      typeof value.target_seed_y === "number" ? value.target_seed_y : undefined,
    target_seed_yaw_rad:
      typeof value.target_seed_yaw_rad === "number" ? value.target_seed_yaw_rad : undefined,
    target_waypoint_goal_x:
      typeof value.target_waypoint_goal_x === "number"
        ? value.target_waypoint_goal_x
        : undefined,
    target_waypoint_goal_y:
      typeof value.target_waypoint_goal_y === "number"
        ? value.target_waypoint_goal_y
        : undefined,
    target_waypoint_goal_yaw_rad:
      typeof value.target_waypoint_goal_yaw_rad === "number"
        ? value.target_waypoint_goal_yaw_rad
        : undefined
  };
}

export function parseModuleConfig(raw: string | undefined): ModuleConfig {
  if (!raw) return DEFAULT_MODULE_CONFIG;

  try {
    const value = JSON.parse(raw) as Partial<ModuleConfig> & {
      graphnavMetadataStreamName?: string;
    };
    return {
      mapImageStreamName: asConfiguredName(
        value.mapImageStreamName,
        DEFAULT_MODULE_CONFIG.mapImageStreamName
      ),
      mapImageMetadataStreamName: asConfiguredName(
        value.mapImageMetadataStreamName,
        DEFAULT_MODULE_CONFIG.mapImageMetadataStreamName
      ),
      overlayStreamName: asConfiguredName(
        value.overlayStreamName ?? value.graphnavMetadataStreamName,
        DEFAULT_MODULE_CONFIG.overlayStreamName
      ),
      navStateStreamName: asConfiguredName(
        value.navStateStreamName,
        DEFAULT_MODULE_CONFIG.navStateStreamName
      ),
      connectionStateStreamName: asConfiguredName(
        value.connectionStateStreamName,
        DEFAULT_MODULE_CONFIG.connectionStateStreamName
      ),
      dockingStateStreamName: asConfiguredName(
        value.dockingStateStreamName,
        DEFAULT_MODULE_CONFIG.dockingStateStreamName
      ),
      motorPowerStateStreamName: asConfiguredName(
        value.motorPowerStateStreamName,
        DEFAULT_MODULE_CONFIG.motorPowerStateStreamName
      ),
      behaviorStateStreamName: asConfiguredName(
        value.behaviorStateStreamName,
        DEFAULT_MODULE_CONFIG.behaviorStateStreamName
      ),
      waypointGotoCommandName: asConfiguredName(
        value.waypointGotoCommandName,
        DEFAULT_MODULE_CONFIG.waypointGotoCommandName
      ),
      gotoPoseCommandName: asConfiguredName(
        value.gotoPoseCommandName,
        DEFAULT_MODULE_CONFIG.gotoPoseCommandName
      ),
      cancelNavCommandName: asConfiguredName(
        value.cancelNavCommandName,
        DEFAULT_MODULE_CONFIG.cancelNavCommandName
      ),
      returnAndDockCommandName: asConfiguredName(
        value.returnAndDockCommandName,
        DEFAULT_MODULE_CONFIG.returnAndDockCommandName
      ),
      undockCommandName: asConfiguredName(
        value.undockCommandName,
        DEFAULT_MODULE_CONFIG.undockCommandName
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
    throw new Error(
      "Authentication failed. Open this module from Formant or include auth context in the URL."
    );
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

function patchTextValue(
  key: keyof StreamSnapshot,
  timeKey: keyof StreamSnapshot,
  value: string | symbol,
  onPatch: (patch: Partial<StreamSnapshot>) => void
): void {
  if (typeof value !== "string") return;
  onPatch({
    [key]: value.trim(),
    [timeKey]: Date.now()
  });
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
    live.subscribeToJson(
      deviceId,
      buildRealtimeSource(config.overlayStreamName, "json"),
      (value) => {
        const overlay = parseGraphNavOverlayValue(value);
        if (!overlay) {
          onPatch({
            warnings: [`Could not parse ${config.overlayStreamName}`]
          });
          return;
        }
        onPatch({
          overlay,
          overlayTime: Date.now()
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

  cleanups.push(
    live.subscribeToText(
      deviceId,
      buildRealtimeSource(config.connectionStateStreamName, "text"),
      (value) => patchTextValue("connectionState", "connectionStateTime", value, onPatch)
    )
  );

  cleanups.push(
    live.subscribeToText(
      deviceId,
      buildRealtimeSource(config.dockingStateStreamName, "text"),
      (value) => patchTextValue("dockingState", "dockingStateTime", value, onPatch)
    )
  );

  cleanups.push(
    live.subscribeToText(
      deviceId,
      buildRealtimeSource(config.motorPowerStateStreamName, "text"),
      (value) => patchTextValue("motorPowerState", "motorPowerStateTime", value, onPatch)
    )
  );

  cleanups.push(
    live.subscribeToText(
      deviceId,
      buildRealtimeSource(config.behaviorStateStreamName, "text"),
      (value) => patchTextValue("behaviorState", "behaviorStateTime", value, onPatch)
    )
  );

  return () => {
    cleanups.forEach((cleanup) => {
      try {
        cleanup();
      } catch {
        // Ignore stale realtime cleanup errors.
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

export function formatWaypointGotoPayload(mapUuid: string, waypointId: string): string {
  return `map_uuid=${mapUuid}, waypoint_id=${waypointId}`;
}
