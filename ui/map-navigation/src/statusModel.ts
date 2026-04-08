import { TELEMETRY_POLL_INTERVAL_MS } from "./formant";
import { StreamSnapshot } from "./types";

export const MAP_VIDEO_MAX_AGE_MS = 20_000;
export const MAP_PRECISION_MAX_AGE_MS = 45_000;
export const NAV_STATE_MAX_AGE_MS = 12_000;
export const ROBOT_STATE_MAX_AGE_MS = 60_000;
export const FORMANT_BACKEND_MAX_AGE_MS = TELEMETRY_POLL_INTERVAL_MS * 3;
export const REALTIME_CONNECTION_MAX_AGE_MS = 12_000;
export const LIVE_SESSION_MAX_AGE_MS = 15_000;
export const INITIAL_CONNECT_TIMEOUT_MS = 10_000;
export const RECONNECT_TIMEOUT_MS = 10_000;

export type MetricTone = "good" | "neutral" | "caution" | "bad";
export type ConnectionPhase = "connecting" | "ready" | "failed";
export type ConnectionStepStatus = "pending" | "complete" | "failed";
export type LocalizationStatus = "localized" | "not_localized" | "unknown";
export type MapSurfaceMode = "live" | "video_only" | "catalog" | "unavailable";

export interface AsyncHealthState {
  status: "idle" | "pending" | "ready" | "failed";
  lastSuccessAt?: number;
  error?: string;
}

export interface ConnectionHealth {
  formantBackendHealthy: boolean;
  realtimeTransportHealthy: boolean;
  realtimeConnectionFresh: boolean;
  liveMapHealthy: boolean;
  liveNavigationDataHealthy: boolean;
  catalogHealthy: boolean;
  transportReady: boolean;
}

export interface ConnectionScreenState {
  phase: ConnectionPhase;
  title: string;
  detail: string;
  steps: Array<{
    label: string;
    detail: string;
    status: ConnectionStepStatus;
  }>;
}

export interface MapState {
  currentMapId: string;
  displayedMapId: string;
  displayedMapUuid: string;
  defaultMapId: string;
  savedCount: number;
  surfaceMode: MapSurfaceMode;
  videoFresh: boolean;
  precisionFresh: boolean;
  overlayFresh: boolean;
  catalogFresh: boolean;
  currentFresh: boolean;
  defaultFresh: boolean;
  headline: string;
  detail: string;
}

export interface StatusMetric {
  label: string;
  value: string;
  tone: MetricTone;
}

export interface NavigationSummary {
  title: string;
  detail: string;
  tone: MetricTone;
  preciseNavigationReady: boolean;
}

function formatStateLabel(value: string | undefined): string {
  if (!value) return "Unknown";
  return value
    .split(/[_\s]+/)
    .filter(Boolean)
    .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
    .join(" ");
}

function compactStateValue(value: string | undefined): string {
  switch (value) {
    case "connected":
      return "Live";
    case "connecting":
      return "Wait";
    case "disconnected":
      return "Off";
    case "undocked":
      return "Free";
    case "docked":
      return "Docked";
    case "docking":
      return "Docking";
    case "undocking":
      return "Leaving";
    case "on":
      return "On";
    case "off":
      return "Off";
    case "standing":
      return "Standing";
    case "stepping":
      return "Stepping";
    case "transition":
      return "Transition";
    case "not_ready":
      return "Idle";
    default:
      return formatStateLabel(value);
  }
}

export function isFresh(
  timestamp: number | undefined,
  maxAgeMs: number,
  now = Date.now()
): boolean {
  return Boolean(timestamp && now - timestamp <= maxAgeMs);
}

export function isBehaviorNavigationReady(value: string | undefined): boolean {
  return value === "standing" || value === "stepping" || value === "transition";
}

export function statusTone(value: string | undefined): MetricTone {
  switch (value) {
    case "connected":
    case "undocked":
    case "on":
    case "standing":
      return "good";
    case "connecting":
    case "docking":
    case "undocking":
    case "transition":
    case "stepping":
      return "caution";
    case "disconnected":
    case "error":
      return "bad";
    case "docked":
    case "off":
    case "not_ready":
      return "neutral";
    default:
      return "neutral";
  }
}

export function getLocalizationStatus(
  snapshot: StreamSnapshot,
  now = Date.now()
): LocalizationStatus {
  if (!isFresh(snapshot.navStateTime, NAV_STATE_MAX_AGE_MS, now)) return "unknown";
  return snapshot.navState?.localized ? "localized" : "not_localized";
}

export function deriveConnectionHealth(
  now: number,
  snapshot: StreamSnapshot,
  bootstrapState: AsyncHealthState,
  telemetryState: AsyncHealthState
): ConnectionHealth {
  const formantBackendHealthy =
    bootstrapState.status === "ready" &&
    Boolean(
      telemetryState.lastSuccessAt &&
        now - telemetryState.lastSuccessAt <= FORMANT_BACKEND_MAX_AGE_MS
    );
  const realtimeTransportHealthy = isFresh(
    snapshot.realtimeActivityTime,
    REALTIME_CONNECTION_MAX_AGE_MS,
    now
  );
  const realtimeConnectionFresh = isFresh(
    snapshot.realtimeConnectionStateTime,
    REALTIME_CONNECTION_MAX_AGE_MS,
    now
  );
  const liveMapHealthy = isFresh(snapshot.mapImageRealtimeTime, LIVE_SESSION_MAX_AGE_MS, now);
  const liveNavigationDataHealthy = [
    snapshot.navStateRealtimeTime,
    snapshot.mapImageMetadataRealtimeTime,
    snapshot.overlayRealtimeTime
  ].some((timestamp) => isFresh(timestamp, LIVE_SESSION_MAX_AGE_MS, now));
  const catalogHealthy = Boolean(
    telemetryState.lastSuccessAt &&
      now - telemetryState.lastSuccessAt <= FORMANT_BACKEND_MAX_AGE_MS
  );

  return {
    formantBackendHealthy,
    realtimeTransportHealthy,
    realtimeConnectionFresh,
    liveMapHealthy,
    liveNavigationDataHealthy,
    catalogHealthy,
    transportReady: bootstrapState.status === "ready" && realtimeTransportHealthy && liveMapHealthy
  };
}

export function deriveMapState(snapshot: StreamSnapshot, now = Date.now()): MapState {
  const currentMapId = snapshot.currentMapId || "";
  const displayedMapId =
    snapshot.mapImageMetadata?.map_id ||
    snapshot.overlay?.map_id ||
    snapshot.navState?.map_id ||
    "";
  const displayedMapUuid =
    snapshot.mapImageMetadata?.map_uuid ||
    snapshot.overlay?.map_uuid ||
    snapshot.navState?.map_uuid ||
    "";
  const defaultMapId = snapshot.defaultMapId || "";
  const ids = new Set<string>();
  (snapshot.maps || []).forEach((mapId) => {
    if (mapId) ids.add(mapId);
  });
  if (currentMapId) ids.add(currentMapId);
  if (displayedMapId) ids.add(displayedMapId);
  if (defaultMapId) ids.add(defaultMapId);

  const videoFresh = isFresh(snapshot.mapImageTime, MAP_VIDEO_MAX_AGE_MS, now);
  const precisionFresh = isFresh(
    snapshot.mapImageMetadataTime,
    MAP_PRECISION_MAX_AGE_MS,
    now
  );
  const overlayFresh = isFresh(snapshot.overlayTime, MAP_PRECISION_MAX_AGE_MS, now);
  const catalogFresh = isFresh(snapshot.mapsTime, ROBOT_STATE_MAX_AGE_MS, now);
  const currentFresh = !currentMapId || isFresh(snapshot.currentMapTime, ROBOT_STATE_MAX_AGE_MS, now);
  const defaultFresh = !defaultMapId || isFresh(snapshot.defaultMapTime, ROBOT_STATE_MAX_AGE_MS, now);

  let surfaceMode: MapSurfaceMode;
  if (videoFresh && precisionFresh) {
    surfaceMode = "live";
  } else if (videoFresh) {
    surfaceMode = "video_only";
  } else if (currentMapId) {
    surfaceMode = "catalog";
  } else {
    surfaceMode = "unavailable";
  }

  let headline = "No live map";
  let detail = "Load a saved map to begin.";
  if (surfaceMode === "live") {
    headline = "Live map ready";
    detail = "Live map video and navigation overlays are current.";
  } else if (surfaceMode === "video_only") {
    headline = "Live map visible";
    detail = "Video is live. Precision overlays are still catching up.";
  } else if (surfaceMode === "catalog") {
    headline = "Catalog only";
    detail = "Saved maps are available, but the live map view is offline.";
  }

  return {
    currentMapId: currentMapId || displayedMapId,
    displayedMapId,
    displayedMapUuid,
    defaultMapId,
    savedCount: ids.size,
    surfaceMode,
    videoFresh,
    precisionFresh,
    overlayFresh,
    catalogFresh,
    currentFresh,
    defaultFresh,
    headline,
    detail
  };
}

export function deriveRobotMetrics(
  snapshot: StreamSnapshot,
  now = Date.now()
): StatusMetric[] {
  const localization = getLocalizationStatus(snapshot, now);
  const localizationValue =
    localization === "localized"
      ? "Locked"
      : localization === "not_localized"
        ? "No lock"
        : "Unknown";

  return [
    {
      label: "Link",
      value: isFresh(snapshot.connectionStateTime, ROBOT_STATE_MAX_AGE_MS, now)
        ? compactStateValue(snapshot.connectionState)
        : "Unknown",
      tone: statusTone(snapshot.connectionState)
    },
    {
      label: "Local",
      value: localizationValue,
      tone:
        localization === "localized"
          ? "good"
          : localization === "not_localized"
            ? "caution"
            : "neutral"
    },
    {
      label: "Dock",
      value: isFresh(snapshot.dockingStateTime, ROBOT_STATE_MAX_AGE_MS, now)
        ? compactStateValue(snapshot.dockingState)
        : "Unknown",
      tone: statusTone(snapshot.dockingState)
    },
    {
      label: "Power",
      value: isFresh(snapshot.motorPowerStateTime, ROBOT_STATE_MAX_AGE_MS, now)
        ? compactStateValue(snapshot.motorPowerState)
        : "Unknown",
      tone: statusTone(snapshot.motorPowerState)
    },
    {
      label: "State",
      value: isFresh(snapshot.behaviorStateTime, ROBOT_STATE_MAX_AGE_MS, now)
        ? compactStateValue(snapshot.behaviorState)
        : "Unknown",
      tone: statusTone(snapshot.behaviorState)
    },
    {
      label: "Batt",
      value:
        typeof snapshot.batteryPct === "number"
          ? `${snapshot.batteryPct.toFixed(0)}%`
          : "Unknown",
      tone:
        typeof snapshot.batteryPct === "number" && snapshot.batteryPct <= 20
          ? "caution"
          : "neutral"
    }
  ];
}

export function deriveNavigationSummary(
  snapshot: StreamSnapshot,
  mapState: MapState,
  now = Date.now()
): NavigationSummary {
  const localization = getLocalizationStatus(snapshot, now);
  const connectionFresh = isFresh(snapshot.connectionStateTime, ROBOT_STATE_MAX_AGE_MS, now);
  const dockingFresh = isFresh(snapshot.dockingStateTime, ROBOT_STATE_MAX_AGE_MS, now);
  const motorFresh = isFresh(snapshot.motorPowerStateTime, ROBOT_STATE_MAX_AGE_MS, now);

  if (!mapState.currentMapId && mapState.surfaceMode === "unavailable") {
    return {
      title: "No map loaded",
      detail: "Load a saved map before using navigation tools.",
      tone: "caution",
      preciseNavigationReady: false
    };
  }

  if (mapState.surfaceMode === "catalog") {
    return {
      title: "Catalog only",
      detail: "Map management is available. Live navigation will return when the live map comes back.",
      tone: "neutral",
      preciseNavigationReady: false
    };
  }

  if (mapState.surfaceMode === "video_only") {
    return {
      title: "Precision syncing",
      detail: "The live map is visible. Precise overlays and route targeting are still catching up.",
      tone: "neutral",
      preciseNavigationReady: false
    };
  }

  if (!connectionFresh || snapshot.connectionState !== "connected") {
    return {
      title: "Robot connection unavailable",
      detail: "Reconnect the robot before sending a navigation command.",
      tone: "caution",
      preciseNavigationReady: false
    };
  }

  if (localization === "not_localized" || localization === "unknown") {
    return {
      title: "Localization required",
      detail: "The robot needs a fresh localization before route commands are reliable.",
      tone: "caution",
      preciseNavigationReady: false
    };
  }

  if (dockingFresh && snapshot.dockingState === "docked") {
    return {
      title: "Robot docked",
      detail: "Undock the robot before sending a navigation destination.",
      tone: "neutral",
      preciseNavigationReady: true
    };
  }

  if (motorFresh && snapshot.motorPowerState !== "on") {
    return {
      title: "Motor power off",
      detail: "Turn motor power on before sending a movement command.",
      tone: "neutral",
      preciseNavigationReady: true
    };
  }

  if (
    isFresh(snapshot.behaviorStateTime, ROBOT_STATE_MAX_AGE_MS, now) &&
    !isBehaviorNavigationReady(snapshot.behaviorState)
  ) {
    return {
      title: "Stand before navigating",
      detail: "The robot is not yet in a navigation-ready behavior state.",
      tone: "neutral",
      preciseNavigationReady: true
    };
  }

  return {
    title: "Ready for navigation",
    detail: "Select a saved waypoint or place a precise point target on the live map.",
    tone: "good",
    preciseNavigationReady: true
  };
}

export function buildConnectionScreenState(
  now: number,
  health: ConnectionHealth,
  snapshot: StreamSnapshot,
  bootstrapState: AsyncHealthState,
  telemetryState: AsyncHealthState,
  attemptStartedAt: number,
  hasReachedReady: boolean
): ConnectionScreenState {
  const timeoutMs = hasReachedReady ? RECONNECT_TIMEOUT_MS : INITIAL_CONNECT_TIMEOUT_MS;
  const timedOut = now - attemptStartedAt >= timeoutMs;
  const steps: ConnectionScreenState["steps"] = [
    {
      label: "Formant backend",
      detail:
        bootstrapState.status === "ready"
          ? "Authenticated and device context loaded."
          : bootstrapState.status === "pending"
            ? "Authenticating and locating the current device."
            : "Waiting for device context from Formant.",
      status: bootstrapState.status === "ready" ? "complete" : "pending"
    },
    {
      label: "Live session",
      detail: health.realtimeTransportHealthy
        ? "Realtime transport is active."
        : "Starting the realtime connection.",
      status: health.realtimeTransportHealthy ? "complete" : "pending"
    },
    {
      label: "Map video",
      detail: health.liveMapHealthy
        ? "Receiving live map video."
        : "Waiting for the first live map frame.",
      status: health.liveMapHealthy ? "complete" : "pending"
    },
    {
      label: "Catalog state",
      detail: health.catalogHealthy
        ? "Saved maps and sidebar state are available."
        : "Waiting for telemetry-backed catalog data.",
      status: health.catalogHealthy ? "complete" : "pending"
    }
  ];

  if (health.transportReady) {
    return {
      phase: "ready",
      title: "Connected",
      detail: "Formant and the live robot session are healthy.",
      steps
    };
  }

  if (bootstrapState.status === "failed") {
    return {
      phase: "failed",
      title: "Could not connect to Formant",
      detail:
        bootstrapState.error ||
        "Authentication or device bootstrap failed before the navigation session could start.",
      steps: steps.map((step, index) =>
        index === 0 ? { ...step, status: "failed" } : step
      )
    };
  }

  if (timedOut) {
    if (bootstrapState.status !== "ready") {
      return {
        phase: "failed",
        title: "Formant connection failed",
        detail:
          telemetryState.error ||
          "The module could not confirm a healthy connection to the Formant API backend.",
        steps: steps.map((step, index) =>
          index === 0 && step.status !== "complete"
            ? { ...step, status: "failed" }
            : step
        )
      };
    }

    if (!health.realtimeTransportHealthy) {
      return {
        phase: "failed",
        title: "Live robot connection timed out",
        detail: "The realtime transport never became active.",
        steps: steps.map((step, index) =>
          index === 1 && step.status !== "complete"
            ? { ...step, status: "failed" }
            : step
        )
      };
    }

    if (
      health.realtimeConnectionFresh &&
      snapshot.realtimeConnectionState &&
      snapshot.realtimeConnectionState !== "connected"
    ) {
      return {
        phase: "failed",
        title: "Robot is not connected",
        detail: "Formant is reachable, but the robot reported a disconnected live state.",
        steps: steps.map((step, index) =>
          index === 1 && step.status !== "complete"
            ? { ...step, status: "failed" }
            : step
        )
      };
    }

    return {
      phase: "failed",
      title: "Live map video is unavailable",
      detail: "The realtime session connected, but the live map video did not arrive in time.",
      steps: steps.map((step, index) =>
        index === 2 && step.status !== "complete"
          ? { ...step, status: "failed" }
          : step
      )
    };
  }

  if (bootstrapState.status !== "ready") {
    return {
      phase: "connecting",
      title: "Connecting to Formant",
      detail:
        bootstrapState.status === "pending"
          ? "Checking the Formant API backend and loading the current device."
          : "Waiting for a healthy Formant backend connection.",
      steps
    };
  }

  if (!health.realtimeTransportHealthy) {
    return {
      phase: "connecting",
      title: "Connecting to live robot session",
      detail: "Waiting for the realtime session to become active.",
      steps
    };
  }

  return {
    phase: "connecting",
    title: "Starting live map",
    detail: "The live session is active. Waiting for the first map frame before showing the interface.",
    steps
  };
}
