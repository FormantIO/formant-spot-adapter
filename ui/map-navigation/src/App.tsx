import { useEffect, useMemo, useRef, useState } from "react";
import {
  Alert,
  Box,
  Button,
  Chip,
  CircularProgress,
  Collapse,
  CssBaseline,
  Divider,
  List,
  ListItemButton,
  ListItemText,
  Stack,
  TextField,
  ThemeProvider,
  ToggleButton,
  ToggleButtonGroup,
  Typography,
  createTheme
} from "@mui/material";
import { App as FormantApp } from "@formant/data-sdk";
import {
  DEFAULT_MODULE_CONFIG,
  authenticateAndGetDevice,
  formatGotoPosePayload,
  formatMapIdPayload,
  formatWaypointGotoPayload,
  getInitialModuleConfig,
  loadTelemetrySnapshot,
  parseModuleConfig,
  subscribeRealtimeSnapshot,
  TELEMETRY_POLL_INTERVAL_MS
} from "./formant";
import { getCanvasSize, pixelToSeed, seedToPixel } from "./mapMath";
import {
  GraphNavOverlayWaypoint,
  ModuleConfig,
  NavState,
  StreamSnapshot,
  TargetPose
} from "./types";

const theme = createTheme({
  palette: {
    mode: "dark",
    primary: { main: "#2bb6ff" },
    secondary: { main: "#89f3c7" },
    warning: { main: "#ffb44d" },
    error: { main: "#ff6c7b" },
    background: {
      default: "#09131b",
      paper: "#101c25"
    }
  },
  shape: {
    borderRadius: 14
  },
  typography: {
    fontFamily: `"IBM Plex Sans", "Inter", sans-serif`,
    h6: { fontWeight: 600 },
    subtitle2: { fontWeight: 600 }
  }
});

type SelectionMode = "waypoints" | "point";
type PointHeadingMode = "current" | "path" | "custom";
type LocalizationStatus = "localized" | "not_localized" | "unknown";
type MapSurfaceStatus = "live" | "catalog" | "unavailable";
type SystemStatusSeverity = "success" | "info" | "warning";
type Selection =
  | { kind: "waypoint"; mapUuid: string; waypoint: GraphNavOverlayWaypoint }
  | { kind: "point"; mapUuid: string; target: TargetPose; headingMode: PointHeadingMode };

function isFresh(timestamp: number | undefined, maxAgeMs: number): boolean {
  return Boolean(timestamp && Date.now() - timestamp <= maxAgeMs);
}

function formatPose(target?: TargetPose): string {
  if (!target) return "No target selected";
  return `x=${target.x.toFixed(2)} m, y=${target.y.toFixed(2)} m, yaw=${target.yawDeg.toFixed(1)} deg`;
}

function getCurrentWaypointLabel(snapshot: StreamSnapshot): string {
  const currentWaypointId =
    snapshot.overlay?.current_waypoint_id || snapshot.navState?.current_waypoint_id || "";
  if (!currentWaypointId) return "Unknown waypoint";
  const waypoint = snapshot.overlay?.waypoints.find((entry) => entry.id === currentWaypointId);
  return waypoint?.name || waypoint?.label || currentWaypointId;
}

function getCurrentYawDeg(config: ModuleConfig, navState?: NavState): number {
  if (config.defaultYawMode === "current" && typeof navState?.current_seed_yaw_rad === "number") {
    return (navState.current_seed_yaw_rad * 180) / Math.PI;
  }
  return config.defaultYawDeg;
}

function computePathYawDeg(navState: NavState | undefined, target: TargetPose): number | undefined {
  if (
    typeof navState?.current_seed_x !== "number" ||
    typeof navState.current_seed_y !== "number"
  ) {
    return undefined;
  }
  const dx = target.x - navState.current_seed_x;
  const dy = target.y - navState.current_seed_y;
  if (Math.abs(dx) < 1e-6 && Math.abs(dy) < 1e-6) return undefined;
  return (Math.atan2(dy, dx) * 180) / Math.PI;
}

function statusTone(
  value: string | undefined
): "default" | "primary" | "secondary" | "warning" | "error" {
  switch (value) {
    case "connected":
    case "undocked":
    case "on":
    case "standing":
      return "secondary";
    case "connecting":
    case "docking":
    case "undocking":
    case "transition":
    case "stepping":
      return "warning";
    case "disconnected":
    case "error":
      return "error";
    case "docked":
    case "off":
    case "not_ready":
      return "default";
    default:
      return "default";
  }
}

function navLifecycleTone(
  result: string | undefined
): "default" | "primary" | "secondary" | "warning" | "error" {
  switch (result) {
    case "reached":
    case "held_position":
      return "secondary";
    case "interrupted":
      return "warning";
    case "failed":
      return "error";
    default:
      return "default";
  }
}

function isBehaviorNavigationReady(value: string | undefined): boolean {
  return value === "standing" || value === "stepping" || value === "transition";
}

function getLocalizationStatus(snapshot: StreamSnapshot): LocalizationStatus {
  if (!isFresh(snapshot.navStateTime, 4000)) return "unknown";
  return snapshot.navState?.localized ? "localized" : "not_localized";
}

function getMapSurfaceStatus(
  snapshot: StreamSnapshot,
  catalogCurrentMapId: string,
  displayedMapUuid: string
): MapSurfaceStatus {
  if (displayedMapUuid && isFresh(snapshot.mapImageMetadataTime, 8000)) {
    return "live";
  }
  if (catalogCurrentMapId) return "catalog";
  return "unavailable";
}

function buildSystemStatus(
  snapshot: StreamSnapshot,
  catalogCurrentMapId: string,
  displayedMapUuid: string
): { title: string; detail: string; severity: SystemStatusSeverity } {
  const connectionFresh = isFresh(snapshot.connectionStateTime, 45000);
  const dockingFresh = isFresh(snapshot.dockingStateTime, 45000);
  const motorFresh = isFresh(snapshot.motorPowerStateTime, 45000);
  const behaviorFresh = isFresh(snapshot.behaviorStateTime, 45000);
  const navFresh = isFresh(snapshot.navStateTime, 4000);
  const localizationStatus = getLocalizationStatus(snapshot);
  const mapSurfaceStatus = getMapSurfaceStatus(snapshot, catalogCurrentMapId, displayedMapUuid);

  if (!catalogCurrentMapId && mapSurfaceStatus === "unavailable") {
    return {
      title: "No map selected",
      detail: "Load a saved map to browse it or prepare navigation.",
      severity: "warning"
    };
  }

  if (mapSurfaceStatus !== "live") {
    return {
      title: "Map browsing available",
      detail: "Saved maps are available, but the live map view is unavailable.",
      severity: "info"
    };
  }

  if (!connectionFresh || !navFresh) {
    return {
      title: "Waiting for live robot state",
      detail: "The live map is visible, but robot state or navigation state is not current yet.",
      severity: "info"
    };
  }

  if (snapshot.connectionState !== "connected") {
    return {
      title: "Robot disconnected",
      detail: "Reconnect the robot to enable live navigation actions.",
      severity: "warning"
    };
  }

  if (localizationStatus === "not_localized") {
    return {
      title: "Robot not localized",
      detail: "Use GraphNav localization before sending navigation commands.",
      severity: "warning"
    };
  }

  if (dockingFresh && snapshot.dockingState === "docked") {
    return {
      title: "Robot docked",
      detail: "Map management remains available. Undock to enable movement commands.",
      severity: "info"
    };
  }

  if (motorFresh && snapshot.motorPowerState !== "on") {
    return {
      title: "Robot idle",
      detail: "Turn motor power on to enable movement commands.",
      severity: "info"
    };
  }

  if (behaviorFresh && !isBehaviorNavigationReady(snapshot.behaviorState)) {
    return {
      title: "Robot not ready to navigate",
      detail: "Stand the robot before sending navigation commands.",
      severity: "info"
    };
  }

  return {
    title: "Live navigation ready",
    detail: "Live map, localization, and robot state are ready for navigation.",
    severity: "success"
  };
}

function buildBaseNavigationIssues(
  snapshot: StreamSnapshot,
  targetMapUuid: string,
  requireLiveMapView: boolean
): string[] {
  const issues: string[] = [];
  const connectionFresh = isFresh(snapshot.connectionStateTime, 45000);
  const dockingFresh = isFresh(snapshot.dockingStateTime, 45000);
  const motorFresh = isFresh(snapshot.motorPowerStateTime, 45000);
  const behaviorFresh = isFresh(snapshot.behaviorStateTime, 45000);
  const navFresh = isFresh(snapshot.navStateTime, 4000);
  const mapFresh = isFresh(snapshot.mapImageMetadataTime, 8000);

  if (!targetMapUuid) {
    issues.push("Selected map is unavailable.");
  }
  if (!connectionFresh) {
    issues.push("Waiting for live robot connection.");
  } else if (snapshot.connectionState !== "connected") {
    issues.push("Robot is not connected.");
  }

  if (!navFresh) {
    issues.push("Waiting for live navigation state.");
  } else if (!snapshot.navState?.localized) {
    issues.push("Robot is not localized.");
  }

  if (!motorFresh || !dockingFresh || !behaviorFresh) {
    issues.push("Waiting for live robot state.");
  } else {
    if (snapshot.dockingState === "docked") issues.push("Undock the robot first.");
    if (snapshot.motorPowerState !== "on") issues.push("Turn motor power on.");
    if (!isBehaviorNavigationReady(snapshot.behaviorState)) {
      issues.push("Stand the robot before navigating.");
    }
  }

  if (requireLiveMapView && !mapFresh) {
    issues.push("Live map view is unavailable.");
  }

  return issues;
}

function buildWaypointNavigationIssues(
  snapshot: StreamSnapshot,
  selection: Extract<Selection, { kind: "waypoint" }>,
  displayedMapUuid: string
): string[] {
  const issues = buildBaseNavigationIssues(
    snapshot,
    selection.mapUuid || displayedMapUuid,
    false
  );
  if (!isFresh(snapshot.overlayTime, 15000)) {
    issues.push("Waypoint list is stale.");
  }
  if (!snapshot.overlay) {
    issues.push("Waypoint list is unavailable.");
  } else if (!snapshot.overlay.waypoints.some((waypoint) => waypoint.id === selection.waypoint.id)) {
    issues.push("Selected waypoint is no longer available.");
  }
  const overlayMapUuid = snapshot.overlay?.map_uuid || "";
  if (selection.mapUuid && overlayMapUuid && selection.mapUuid !== overlayMapUuid) {
    issues.push("Selected waypoint no longer matches the active map.");
  }
  return issues;
}

function buildPointNavigationIssues(
  snapshot: StreamSnapshot,
  selection: Extract<Selection, { kind: "point" }>,
  displayedMapUuid: string
): string[] {
  const issues = buildBaseNavigationIssues(
    snapshot,
    selection.mapUuid || displayedMapUuid,
    true
  );
  if (selection.mapUuid && displayedMapUuid && selection.mapUuid !== displayedMapUuid) {
    issues.push("Selected point no longer matches the displayed map.");
  }
  return issues;
}

function uniqueIssues(issues: string[]): string[] {
  return Array.from(new Set(issues));
}

function clientToSvgPoint(
  svg: SVGSVGElement,
  clientX: number,
  clientY: number
): { x: number; y: number } | null {
  const screenPoint = svg.createSVGPoint();
  screenPoint.x = clientX;
  screenPoint.y = clientY;
  const ctm = svg.getScreenCTM();
  if (!ctm) return null;
  const point = screenPoint.matrixTransform(ctm.inverse());
  return { x: point.x, y: point.y };
}

interface PoseMarkerProps {
  x: number;
  y: number;
  yawRad: number;
  fill: string;
  outline: string;
  size?: number;
}

function PoseMarker({ x, y, yawRad, fill, outline, size = 14 }: PoseMarkerProps) {
  const half = size / 2;
  const points = `${x},${y - size} ${x + half},${y + half} ${x},${y + half / 2} ${x - half},${y + half}`;
  return (
    <g transform={`rotate(${(yawRad * 180) / Math.PI} ${x} ${y})`}>
      <polygon points={points} fill={fill} stroke={outline} strokeWidth="2" />
    </g>
  );
}

interface MapViewProps {
  snapshot: StreamSnapshot;
  selectionMode: SelectionMode;
  selection?: Selection;
  showWaypointLabels: boolean;
  onSelectWaypoint: (waypoint: GraphNavOverlayWaypoint) => void;
  onSelectPoint: (target: TargetPose) => void;
}

function MapView({
  snapshot,
  selectionMode,
  selection,
  showWaypointLabels,
  onSelectWaypoint,
  onSelectPoint
}: MapViewProps) {
  const overlayRef = useRef<SVGSVGElement | null>(null);
  const realtimeCanvasRef = useRef<HTMLCanvasElement | null>(null);
  const canvas = getCanvasSize(snapshot.mapImageMetadata);

  useEffect(() => {
    if (!snapshot.mapImageCanvas || !realtimeCanvasRef.current) return;
    const target = realtimeCanvasRef.current;
    target.width = snapshot.mapImageCanvas.width;
    target.height = snapshot.mapImageCanvas.height;
    const context = target.getContext("2d");
    context?.drawImage(snapshot.mapImageCanvas, 0, 0);
  }, [snapshot.mapImageCanvas, snapshot.mapImageFrameVersion]);

  const waypointPixels = (snapshot.overlay?.waypoints || [])
    .map((waypoint) => {
      const point = seedToPixel(snapshot.mapImageMetadata, waypoint.x, waypoint.y);
      return point ? { waypoint, point } : null;
    })
    .filter(
      (entry): entry is { waypoint: GraphNavOverlayWaypoint; point: { x: number; y: number } } =>
        Boolean(entry)
    );

  const handleMapClick = (event: React.MouseEvent<SVGSVGElement>) => {
    if (selectionMode !== "point") return;
    const svg = overlayRef.current;
    if (!svg || !snapshot.mapImageMetadata) return;
    const point = clientToSvgPoint(svg, event.clientX, event.clientY);
    if (!point) return;
    const seed = pixelToSeed(snapshot.mapImageMetadata, point.x, point.y);
    if (!seed) return;
    onSelectPoint({
      x: seed.x,
      y: seed.y,
      yawDeg:
        typeof snapshot.navState?.current_seed_yaw_rad === "number"
          ? (snapshot.navState.current_seed_yaw_rad * 180) / Math.PI
          : 0
    });
  };

  const robotPixel =
    snapshot.navState?.has_current_seed_pose &&
    typeof snapshot.navState.current_seed_x === "number" &&
    typeof snapshot.navState.current_seed_y === "number"
      ? seedToPixel(
          snapshot.mapImageMetadata,
          snapshot.navState.current_seed_x,
          snapshot.navState.current_seed_y
        )
      : null;

  const activeTargetPixel =
    snapshot.navState?.has_seed_goal &&
    typeof snapshot.navState.target_seed_x === "number" &&
    typeof snapshot.navState.target_seed_y === "number"
      ? seedToPixel(
          snapshot.mapImageMetadata,
          snapshot.navState.target_seed_x,
          snapshot.navState.target_seed_y
        )
      : null;

  const selectedPixel =
    selection?.kind === "point"
      ? seedToPixel(snapshot.mapImageMetadata, selection.target.x, selection.target.y)
      : selection?.kind === "waypoint"
        ? seedToPixel(snapshot.mapImageMetadata, selection.waypoint.x, selection.waypoint.y)
        : null;

  return (
    <Box
      sx={{
        position: "relative",
        flex: 1,
        minHeight: 0,
        overflow: "hidden",
        background:
          "radial-gradient(circle at top, rgba(43,182,255,0.09), rgba(9,19,27,0.96) 52%)"
      }}
    >
      {snapshot.mapImageCanvas ? (
        <canvas
          ref={realtimeCanvasRef}
          style={{ width: "100%", height: "100%", objectFit: "contain", display: "block" }}
        />
      ) : (
        <Stack
          alignItems="center"
          justifyContent="center"
          sx={{ position: "absolute", inset: 0, textAlign: "center", p: 3 }}
          spacing={1}
        >
          <Typography variant="h6">
            {snapshot.mapImageMetadata?.status_title || "Waiting for GraphNav map"}
          </Typography>
          <Typography variant="body2" color="text.secondary">
            {snapshot.mapImageMetadata?.status_detail ||
              "The adapter has not published the global map image yet."}
          </Typography>
        </Stack>
      )}

      <svg
        ref={overlayRef}
        viewBox={`0 0 ${canvas.x} ${canvas.y}`}
        onClick={handleMapClick}
        style={{
          position: "absolute",
          inset: 0,
          width: "100%",
          height: "100%",
          cursor: selectionMode === "point" ? "crosshair" : "default"
        }}
      >
        {snapshot.mapImageMetadata?.draw_rect ? (
          <rect
            x={snapshot.mapImageMetadata.draw_rect.x}
            y={snapshot.mapImageMetadata.draw_rect.y}
            width={snapshot.mapImageMetadata.draw_rect.width}
            height={snapshot.mapImageMetadata.draw_rect.height}
            fill="none"
            stroke="rgba(255,255,255,0.13)"
            strokeDasharray="6 6"
          />
        ) : null}

        {waypointPixels.map(({ waypoint, point }) => {
          const isCurrent = snapshot.overlay?.current_waypoint_id === waypoint.id;
          const isSelected = selection?.kind === "waypoint" && selection.waypoint.id === waypoint.id;
          return (
            <g
              key={waypoint.id}
              onClick={(event) => {
                event.stopPropagation();
                onSelectWaypoint(waypoint);
              }}
              style={{ cursor: "pointer" }}
            >
              {isCurrent ? (
                <circle
                  cx={point.x}
                  cy={point.y}
                  r={waypoint.is_dock ? 12 : 10}
                  fill="rgba(137,243,199,0.18)"
                />
              ) : null}
              <circle
                cx={point.x}
                cy={point.y}
                r={isSelected ? 9 : waypoint.is_dock ? 7 : 5.5}
                fill={waypoint.is_dock ? "#ffb44d" : "#2bb6ff"}
                stroke={isSelected ? "#ffffff" : "rgba(255,255,255,0.82)"}
                strokeWidth={isSelected ? 2.2 : 1.4}
              />
              {showWaypointLabels ? (
                <text
                  x={point.x + 10}
                  y={point.y - 8}
                  fill="white"
                  fontSize="13"
                  fontFamily="IBM Plex Sans, sans-serif"
                  stroke="rgba(9,19,27,0.92)"
                  strokeWidth="3"
                  paintOrder="stroke"
                >
                  {waypoint.name}
                </text>
              ) : null}
            </g>
          );
        })}

        {robotPixel && typeof snapshot.navState?.current_seed_yaw_rad === "number" ? (
          <PoseMarker
            x={robotPixel.x}
            y={robotPixel.y}
            yawRad={snapshot.navState.current_seed_yaw_rad}
            fill="#89f3c7"
            outline="#071218"
          />
        ) : null}

        {activeTargetPixel && typeof snapshot.navState?.target_seed_yaw_rad === "number" ? (
          <PoseMarker
            x={activeTargetPixel.x}
            y={activeTargetPixel.y}
            yawRad={snapshot.navState.target_seed_yaw_rad}
            fill="#ff8f5a"
            outline="#251208"
            size={12}
          />
        ) : null}

        {selectedPixel ? (
          <g>
            <circle
              cx={selectedPixel.x}
              cy={selectedPixel.y}
              r="12"
              fill="none"
              stroke="#ffffff"
              strokeWidth="2"
            />
            <line
              x1={selectedPixel.x - 14}
              y1={selectedPixel.y}
              x2={selectedPixel.x + 14}
              y2={selectedPixel.y}
              stroke="#ffffff"
              strokeWidth="2"
            />
            <line
              x1={selectedPixel.x}
              y1={selectedPixel.y - 14}
              x2={selectedPixel.x}
              y2={selectedPixel.y + 14}
              stroke="#ffffff"
              strokeWidth="2"
            />
          </g>
        ) : null}
      </svg>
    </Box>
  );
}

const panelSurfaceSx = {
  backdropFilter: "blur(18px)",
  backgroundColor: "rgba(10, 19, 27, 0.82)",
  border: "1px solid rgba(255,255,255,0.06)",
  boxShadow: "0 18px 54px rgba(0,0,0,0.26)"
} as const;

export default function App() {
  const [device, setDevice] = useState<{ id: string; name: string }>();
  const [config, setConfig] = useState<ModuleConfig>(DEFAULT_MODULE_CONFIG);
  const [snapshot, setSnapshot] = useState<StreamSnapshot>({});
  const [selectionMode, setSelectionMode] = useState<SelectionMode>("waypoints");
  const [selection, setSelection] = useState<Selection>();
  const [availableCommands, setAvailableCommands] = useState<string[]>([]);
  const [waypointSearch, setWaypointSearch] = useState("");
  const [mapSearch, setMapSearch] = useState("");
  const [mapsExpanded, setMapsExpanded] = useState(false);
  const [loading, setLoading] = useState(true);
  const [pendingCommand, setPendingCommand] = useState<string>();
  const [error, setError] = useState<string>();
  const [commandError, setCommandError] = useState<string>();
  const [commandNotice, setCommandNotice] = useState<string>();

  useEffect(() => {
    let cancelled = false;
    let configCleanup: (() => void) | undefined;

    const bootstrap = async () => {
      try {
        const currentDevice = await authenticateAndGetDevice();
        const initialConfig = await getInitialModuleConfig();
        const commands = await currentDevice.getAvailableCommands().catch(() => []);
        if (cancelled) return;
        setDevice({ id: currentDevice.id, name: currentDevice.name });
        setConfig(initialConfig);
        setAvailableCommands(commands.map((command: { name: string }) => command.name));
        configCleanup = FormantApp.addModuleConfigurationListener((event: { configuration: string }) => {
          setConfig(parseModuleConfig(event.configuration));
        });
      } catch (bootstrapError) {
        if (cancelled) return;
        setError(
          bootstrapError instanceof Error ? bootstrapError.message : "Failed to initialize the module."
        );
      } finally {
        if (!cancelled) setLoading(false);
      }
    };

    void bootstrap();

    return () => {
      cancelled = true;
      configCleanup?.();
    };
  }, []);

  useEffect(() => {
    if (!device) return undefined;

    let cancelled = false;
    setError(undefined);
    const unsubscribe = subscribeRealtimeSnapshot(device.id, config, (patch) => {
      if (cancelled) return;
      setSnapshot((previous) => ({
        ...previous,
        ...patch
      }));
    });

    return () => {
      cancelled = true;
      unsubscribe();
    };
  }, [device, config]);

  useEffect(() => {
    if (!device) return undefined;

    let cancelled = false;

    const refreshTelemetry = async () => {
      try {
        const patch = await loadTelemetrySnapshot(device.id, config);
        if (cancelled) return;
        setSnapshot((previous) => ({
          ...previous,
          ...patch
        }));
      } catch (telemetryError) {
        console.warn("Failed to refresh telemetry snapshot.", telemetryError);
      }
    };

    void refreshTelemetry();
    const intervalId = window.setInterval(() => {
      void refreshTelemetry();
    }, TELEMETRY_POLL_INTERVAL_MS);

    return () => {
      cancelled = true;
      window.clearInterval(intervalId);
    };
  }, [device, config]);

  useEffect(() => {
    if (snapshot.navState?.active) {
      setSelection(undefined);
      setSelectionMode("waypoints");
    }
  }, [snapshot.navState?.active, snapshot.navState?.command_id]);

  useEffect(() => {
    if (snapshot.navState?.phase === "active" || snapshot.navState?.phase === "terminal") {
      setCommandNotice(undefined);
    }
  }, [snapshot.navState?.phase, snapshot.navState?.last_completed_command_id, snapshot.navState?.command_id]);

  const catalogCurrentMapId = snapshot.currentMapId || "";
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
  const currentMapId = catalogCurrentMapId || displayedMapId;
  const defaultMapId = snapshot.defaultMapId || "";
  const localizationStatus = getLocalizationStatus(snapshot);
  const mapSurfaceStatus = getMapSurfaceStatus(snapshot, catalogCurrentMapId, displayedMapUuid);
  const systemStatus = useMemo(
    () => buildSystemStatus(snapshot, catalogCurrentMapId, displayedMapUuid),
    [snapshot, catalogCurrentMapId, displayedMapUuid]
  );

  const mapLoadCommandAvailable = availableCommands.includes(config.mapLoadCommandName);
  const mapSetDefaultCommandAvailable = availableCommands.includes(config.mapSetDefaultCommandName);
  const waypointCommandAvailable = availableCommands.includes(config.waypointGotoCommandName);
  const gotoPoseCommandAvailable = availableCommands.includes(config.gotoPoseCommandName);
  const cancelCommandAvailable = availableCommands.includes(config.cancelNavCommandName);
  const returnAndDockCommandAvailable = availableCommands.includes(config.returnAndDockCommandName);
  const undockCommandAvailable = availableCommands.includes(config.undockCommandName);

  const sortedWaypoints = useMemo(() => {
    const currentWaypointId = snapshot.overlay?.current_waypoint_id || "";
    const filter = waypointSearch.trim().toLowerCase();
    return [...(snapshot.overlay?.waypoints || [])]
      .filter((waypoint) => {
        if (!filter) return true;
        return (
          waypoint.name.toLowerCase().includes(filter) ||
          waypoint.label.toLowerCase().includes(filter) ||
          waypoint.id.toLowerCase().includes(filter)
        );
      })
      .sort((left, right) => {
        if (left.is_dock !== right.is_dock) return left.is_dock ? -1 : 1;
        if ((left.id === currentWaypointId) !== (right.id === currentWaypointId)) {
          return left.id === currentWaypointId ? -1 : 1;
        }
        return left.name.localeCompare(right.name);
      });
  }, [snapshot.overlay?.waypoints, snapshot.overlay?.current_waypoint_id, waypointSearch]);

  const allMapIds = useMemo(() => {
    const ids = new Set<string>();
    (snapshot.maps || []).forEach((mapId) => {
      if (mapId) ids.add(mapId);
    });
    if (catalogCurrentMapId) ids.add(catalogCurrentMapId);
    if (displayedMapId) ids.add(displayedMapId);
    if (defaultMapId) ids.add(defaultMapId);
    return [...ids];
  }, [snapshot.maps, catalogCurrentMapId, displayedMapId, defaultMapId]);

  const sortedMaps = useMemo(() => {
    const filter = mapSearch.trim().toLowerCase();
    return [...allMapIds]
      .filter((mapId) => {
        if (!filter) return true;
        return mapId.toLowerCase().includes(filter);
      })
      .sort((left, right) => {
        const leftScore =
          (left === currentMapId ? 0 : 10) + (left === defaultMapId ? 0 : 1);
        const rightScore =
          (right === currentMapId ? 0 : 10) + (right === defaultMapId ? 0 : 1);
        if (leftScore !== rightScore) return leftScore - rightScore;
        return left.localeCompare(right);
      });
  }, [allMapIds, currentMapId, defaultMapId, mapSearch]);

  const selectionReadinessIssues = useMemo(() => {
    if (!selection) return [];
    if (selection.kind === "waypoint") {
      const issues = buildWaypointNavigationIssues(snapshot, selection, displayedMapUuid);
      if (!waypointCommandAvailable) issues.push("Waypoint goto command is unavailable.");
      if (snapshot.navState?.active) {
        issues.push("Hold or complete the active navigation before sending a new destination.");
      }
      return uniqueIssues(issues);
    }
    const issues = buildPointNavigationIssues(snapshot, selection, displayedMapUuid);
    if (!gotoPoseCommandAvailable) issues.push("Point goto command is unavailable.");
    if (snapshot.navState?.active) {
      issues.push("Hold or complete the active navigation before sending a new destination.");
    }
    return uniqueIssues(issues);
  }, [
    displayedMapUuid,
    gotoPoseCommandAvailable,
    selection,
    snapshot.overlay,
    snapshot.overlayTime,
    snapshot.navState?.active,
    waypointCommandAvailable
  ]);

  const connectionStateFresh = isFresh(snapshot.connectionStateTime, 45000);
  const dockingStateFresh = isFresh(snapshot.dockingStateTime, 45000);
  const motorStateFresh = isFresh(snapshot.motorPowerStateTime, 45000);
  const behaviorStateFresh = isFresh(snapshot.behaviorStateTime, 45000);
  const connectionReady =
    connectionStateFresh && snapshot.connectionState === "connected";
  const dockingReady = dockingStateFresh;
  const navReady = isFresh(snapshot.navStateTime, 4000);
  const motorReady =
    motorStateFresh && snapshot.motorPowerState === "on";
  const localizedReady = navReady && Boolean(snapshot.navState?.localized);
  const mapsReady = isFresh(snapshot.mapsTime, 60000);
  const currentMapReady = !catalogCurrentMapId || isFresh(snapshot.currentMapTime, 60000);
  const defaultMapReady = !defaultMapId || isFresh(snapshot.defaultMapTime, 60000);

  const mapChipLabel = currentMapId || "No map";
  const mapSurfaceLabel =
    mapSurfaceStatus === "live"
      ? "Live map"
      : mapSurfaceStatus === "catalog"
        ? "Catalog only"
        : "Map unavailable";
  const localizationChipLabel =
    localizationStatus === "localized"
      ? "Localized"
      : localizationStatus === "not_localized"
        ? "Not localized"
        : "Localization unknown";
  const connectionChipLabel = connectionStateFresh
    ? snapshot.connectionState || "connection unknown"
    : "connection unknown";
  const dockingChipLabel = dockingStateFresh
    ? snapshot.dockingState || "docking unknown"
    : "docking unknown";
  const motorChipLabel = motorStateFresh
    ? `motors ${snapshot.motorPowerState || "unknown"}`
    : "motors unknown";
  const behaviorChipLabel = behaviorStateFresh
    ? snapshot.behaviorState || "behavior unknown"
    : "behavior unknown";

  const activeNavLabel =
    snapshot.navState?.target_name ||
    (snapshot.navState?.target_waypoint_id
      ? `Waypoint ${snapshot.navState.target_waypoint_id}`
      : snapshot.navState?.has_seed_goal &&
          typeof snapshot.navState.target_seed_x === "number" &&
          typeof snapshot.navState.target_seed_y === "number"
        ? `x=${snapshot.navState.target_seed_x.toFixed(1)} m, y=${snapshot.navState.target_seed_y.toFixed(1)} m`
        : "No active destination");

  const currentPoseLabel =
    snapshot.navState?.has_current_seed_pose &&
    typeof snapshot.navState.current_seed_x === "number" &&
    typeof snapshot.navState.current_seed_y === "number"
      ? `x=${snapshot.navState.current_seed_x.toFixed(2)} m, y=${snapshot.navState.current_seed_y.toFixed(2)} m`
      : "No live pose";

  const batteryLabel =
    typeof snapshot.batteryPct === "number" ? `battery ${snapshot.batteryPct.toFixed(0)}%` : "battery";

  const navTerminalVisible =
    snapshot.navState?.phase === "terminal" && Boolean(snapshot.navState?.terminal_result);
  const mapsPanelVisible = mapsExpanded || !currentMapId || sortedMaps.length === 0;

  useEffect(() => {
    setSelection((current) => {
      if (!current) return current;
      if (current.mapUuid && displayedMapUuid && current.mapUuid !== displayedMapUuid) {
        return undefined;
      }
      if (current.kind === "waypoint" && snapshot.overlay) {
        const stillPresent = snapshot.overlay.waypoints.some(
          (waypoint) => waypoint.id === current.waypoint.id
        );
        if (!stillPresent) return undefined;
      }
      return current;
    });
  }, [displayedMapUuid, snapshot.overlay]);

  const sendCommand = async (
    name: string,
    payload: string | undefined,
    optimisticNotice: string,
    successNotice: string
  ) => {
    setPendingCommand(name);
    setCommandError(undefined);
    setCommandNotice(optimisticNotice);

    try {
      const currentDevice = await authenticateAndGetDevice();
      await currentDevice.sendCommand(name, payload);
      setCommandNotice(successNotice);
    } catch (deviceError) {
      setCommandError(
        deviceError instanceof Error ? deviceError.message : `Command ${name} failed.`
      );
    } finally {
      setPendingCommand((current) => (current === name ? undefined : current));
    }
  };

  const handleSelectWaypoint = (waypoint: GraphNavOverlayWaypoint) => {
    setSelection({
      kind: "waypoint",
      mapUuid: snapshot.overlay?.map_uuid || snapshot.mapImageMetadata?.map_uuid || "",
      waypoint
    });
    setSelectionMode("waypoints");
    setCommandError(undefined);
    setCommandNotice(undefined);
  };

  const handleLoadMap = (mapId: string) => {
    if (!mapId) return;
    setSelection(undefined);
    setSelectionMode("waypoints");
    setWaypointSearch("");
    setMapsExpanded(false);
    setCommandError(undefined);
    setCommandNotice(undefined);
    void sendCommand(
      config.mapLoadCommandName,
      formatMapIdPayload(mapId),
      `Loading ${mapId}...`,
      `Load command sent. Waiting for the active map to update...`
    );
  };

  const handleSetDefaultMap = (mapId: string) => {
    if (!mapId) return;
    setCommandError(undefined);
    setCommandNotice(undefined);
    void sendCommand(
      config.mapSetDefaultCommandName,
      formatMapIdPayload(mapId),
      `Setting ${mapId} as default...`,
      `Default map command sent. Waiting for the adapter state to update...`
    );
  };

  const handleSelectPoint = (target: TargetPose) => {
    const nextTarget = { ...target, yawDeg: getCurrentYawDeg(config, snapshot.navState) };
    setSelection({
      kind: "point",
      mapUuid: snapshot.mapImageMetadata?.map_uuid || "",
      target: nextTarget,
      headingMode: config.defaultYawMode === "fixed" ? "custom" : "current"
    });
    setCommandError(undefined);
    setCommandNotice(undefined);
  };

  const updatePointHeadingMode = (headingMode: PointHeadingMode) => {
    if (selection?.kind !== "point") return;
    let nextYawDeg = selection.target.yawDeg;
    if (headingMode === "current") {
      nextYawDeg = getCurrentYawDeg(config, snapshot.navState);
    } else if (headingMode === "path") {
      nextYawDeg = computePathYawDeg(snapshot.navState, selection.target) ?? nextYawDeg;
    }
    setSelection({
      kind: "point",
      mapUuid: selection.mapUuid,
      headingMode,
      target: {
        ...selection.target,
        yawDeg: nextYawDeg
      }
    });
  };

  const updatePointYawDeg = (value: number) => {
    if (selection?.kind !== "point" || !Number.isFinite(value)) return;
    setSelection({
      ...selection,
      headingMode: "custom",
      target: {
        ...selection.target,
        yawDeg: value
      }
    });
  };

  const confirmWaypointNavigation = () => {
    if (selection?.kind !== "waypoint" || !displayedMapUuid) return;
    const payload = formatWaypointGotoPayload(displayedMapUuid, selection.waypoint.id);
    void sendCommand(
      config.waypointGotoCommandName,
      payload,
      `Sending ${selection.waypoint.name} to Formant...`,
      `${selection.waypoint.name} command sent. Waiting for navigation state...`
    );
  };

  const confirmPointNavigation = () => {
    if (selection?.kind !== "point" || !displayedMapUuid) return;
    const payload = formatGotoPosePayload(
      displayedMapUuid,
      selection.target.x,
      selection.target.y,
      selection.target.yawDeg
    );
    void sendCommand(
      config.gotoPoseCommandName,
      payload,
      "Sending point target to Formant...",
      "Point target command sent. Waiting for navigation state..."
    );
  };

  const invokeAction = (name: string, optimisticNotice: string, successNotice: string) => {
    void sendCommand(name, undefined, optimisticNotice, successNotice);
  };

  const selectionReady = Boolean(selection) && selectionReadinessIssues.length === 0;
  const canLoadMaps = mapLoadCommandAvailable && connectionReady && !snapshot.navState?.active;
  const canSetDefaultMaps = mapSetDefaultCommandAvailable;

  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <Box
        sx={{
          width: "100vw",
          height: "100vh",
          bgcolor: "background.default",
          color: "text.primary"
        }}
      >
        {loading ? (
          <Stack alignItems="center" justifyContent="center" spacing={2} sx={{ width: "100%", height: "100%" }}>
            <CircularProgress size={28} color="primary" />
            <Typography variant="body2" color="text.secondary">
              Connecting
            </Typography>
          </Stack>
        ) : (
          <Box
            sx={{
              display: "flex",
              flexDirection: { xs: "column", md: "row" },
              width: "100%",
              height: "100%"
            }}
          >
            <Box sx={{ position: "relative", flex: 1, minHeight: { xs: "55%", md: 0 } }}>
              <MapView
                snapshot={snapshot}
                selectionMode={selectionMode}
                selection={selection}
                showWaypointLabels={config.showWaypointLabels}
                onSelectWaypoint={handleSelectWaypoint}
                onSelectPoint={handleSelectPoint}
              />

              <Stack
                direction="row"
                spacing={1}
                flexWrap="wrap"
                useFlexGap
                sx={{ position: "absolute", top: 12, left: 12, right: 12 }}
              >
                <Chip label={mapChipLabel} color="primary" />
                <Chip
                  label={mapSurfaceLabel}
                  color={
                    mapSurfaceStatus === "live"
                      ? "secondary"
                      : mapSurfaceStatus === "unavailable"
                        ? "warning"
                        : "default"
                  }
                  variant="outlined"
                />
                <Chip
                  label={localizationChipLabel}
                  color={
                    localizationStatus === "localized"
                      ? "secondary"
                      : localizationStatus === "not_localized"
                        ? "warning"
                        : "default"
                  }
                  variant={localizationStatus === "localized" ? "filled" : "outlined"}
                />
                <Chip
                  label={connectionChipLabel}
                  color={statusTone(snapshot.connectionState)}
                  variant="outlined"
                />
                <Chip
                  label={dockingChipLabel}
                  color={statusTone(snapshot.dockingState)}
                  variant="outlined"
                />
                <Chip
                  label={motorChipLabel}
                  color={statusTone(snapshot.motorPowerState)}
                  variant="outlined"
                />
                <Chip
                  label={behaviorChipLabel}
                  color={statusTone(snapshot.behaviorState)}
                  variant="outlined"
                />
                <Chip
                  label={batteryLabel}
                  color={
                    typeof snapshot.batteryPct === "number" && snapshot.batteryPct <= 20
                      ? "warning"
                      : "default"
                  }
                  variant={isFresh(snapshot.batteryTime, 15000) ? "outlined" : "filled"}
                />
                {snapshot.navState?.active && snapshot.navState.status_name ? (
                  <Chip label={snapshot.navState.status_name} color="warning" />
                ) : null}
                {navTerminalVisible ? (
                  <Chip
                    label={snapshot.navState?.terminal_result || "terminal"}
                    color={navLifecycleTone(snapshot.navState?.terminal_result)}
                  />
                ) : null}
              </Stack>
            </Box>

            <Box
              sx={{
                width: { xs: "100%", md: 368 },
                borderLeft: { md: "1px solid rgba(255,255,255,0.06)" },
                borderTop: { xs: "1px solid rgba(255,255,255,0.06)", md: "none" },
                ...panelSurfaceSx
              }}
            >
              <Stack spacing={2} sx={{ height: "100%", p: 2 }}>
                <Stack spacing={0.5}>
                  <Typography variant="subtitle2" color="text.secondary">
                    Spot Navigation
                  </Typography>
                  <Typography variant="h6">{device?.name || "Current device"}</Typography>
                  <Typography variant="body2" color="text.secondary">
                    Current waypoint: {getCurrentWaypointLabel(snapshot)}
                  </Typography>
                  <Typography variant="body2" color="text.secondary">
                    Pose: {currentPoseLabel}
                  </Typography>
                </Stack>

                <Box sx={{ p: 1.5, borderRadius: 2, bgcolor: "rgba(255,255,255,0.03)" }}>
                  <Stack spacing={1.25}>
                    <Stack direction="row" spacing={1} justifyContent="space-between" alignItems="flex-start">
                      <Stack spacing={0.35} sx={{ minWidth: 0, flex: 1 }}>
                        <Typography variant="subtitle2">Map</Typography>
                        <Typography
                          variant="body1"
                          sx={{
                            fontWeight: 600,
                            whiteSpace: "nowrap",
                            overflow: "hidden",
                            textOverflow: "ellipsis"
                          }}
                        >
                          {currentMapId || "No active map"}
                        </Typography>
                        <Typography variant="caption" color="text.secondary">
                          {defaultMapId
                            ? defaultMapId === currentMapId
                              ? "Current map is also the default."
                              : `Default map: ${defaultMapId}`
                            : allMapIds.length
                              ? "No default map configured."
                              : "No saved maps published yet."}
                        </Typography>
                        <Typography variant="caption" color="text.secondary">
                          {mapSurfaceStatus === "live"
                            ? "Live map view is available."
                            : currentMapId
                              ? "Map catalog is available; live map view is unavailable."
                              : "No live map view available."}
                        </Typography>
                      </Stack>
                      <Stack direction="row" spacing={1}>
                        {currentMapId && currentMapId !== defaultMapId ? (
                          <Button
                            size="small"
                            variant="text"
                            disabled={
                              !canSetDefaultMaps ||
                              pendingCommand === config.mapSetDefaultCommandName
                            }
                            onClick={() => handleSetDefaultMap(currentMapId)}
                          >
                            Set Default
                          </Button>
                        ) : null}
                        {allMapIds.length > 1 ? (
                          <Button
                            size="small"
                            variant="outlined"
                            onClick={() => setMapsExpanded((current) => !current)}
                          >
                            {mapsExpanded ? "Hide" : "Change"}
                          </Button>
                        ) : null}
                      </Stack>
                    </Stack>

                    <Stack direction="row" spacing={0.75} flexWrap="wrap" useFlexGap>
                      {currentMapId ? <Chip size="small" label="Active" color="primary" /> : null}
                      {defaultMapId === currentMapId && currentMapId ? (
                        <Chip size="small" label="Default" color="secondary" />
                      ) : null}
                      {allMapIds.length ? (
                        <Chip size="small" label={`${allMapIds.length} saved`} variant="outlined" />
                      ) : null}
                      <Chip
                        size="small"
                        label={mapSurfaceStatus === "live" ? "Live view" : "Catalog"}
                        variant="outlined"
                      />
                      {!mapsReady ? (
                        <Chip size="small" label="Catalog stale" color="warning" variant="outlined" />
                      ) : null}
                    </Stack>

                    <Collapse in={mapsPanelVisible} unmountOnExit>
                      <Stack spacing={1}>
                        {allMapIds.length > 6 ? (
                          <TextField
                            value={mapSearch}
                            onChange={(event) => setMapSearch(event.target.value)}
                            size="small"
                            label="Search maps"
                          />
                        ) : null}

                        <Box sx={{ maxHeight: 216, overflow: "auto" }}>
                          <Stack spacing={0.75}>
                            {sortedMaps.map((mapId) => {
                              const isCurrent = mapId === currentMapId;
                              const isDefault = mapId === defaultMapId;
                              return (
                                <Box
                                  key={mapId}
                                  sx={{
                                    p: 1.15,
                                    borderRadius: 1.5,
                                    bgcolor: isCurrent
                                      ? "rgba(43,182,255,0.08)"
                                      : "rgba(255,255,255,0.02)",
                                    border: "1px solid rgba(255,255,255,0.06)"
                                  }}
                                >
                                  <Stack
                                    direction="row"
                                    spacing={1}
                                    justifyContent="space-between"
                                    alignItems="center"
                                  >
                                    <Stack spacing={0.6} sx={{ minWidth: 0, flex: 1 }}>
                                      <Typography
                                        variant="body2"
                                        sx={{
                                          fontWeight: isCurrent ? 600 : 500,
                                          whiteSpace: "nowrap",
                                          overflow: "hidden",
                                          textOverflow: "ellipsis"
                                        }}
                                      >
                                        {mapId}
                                      </Typography>
                                      <Stack direction="row" spacing={0.5} flexWrap="wrap" useFlexGap>
                                        {isCurrent ? (
                                          <Chip size="small" label="Active" color="primary" />
                                        ) : null}
                                        {isDefault ? (
                                          <Chip
                                            size="small"
                                            label="Default"
                                            color={isCurrent ? "secondary" : "default"}
                                            variant={isCurrent ? "filled" : "outlined"}
                                          />
                                        ) : null}
                                      </Stack>
                                    </Stack>
                                    <Stack direction="row" spacing={0.75}>
                                      {!isCurrent ? (
                                        <Button
                                          size="small"
                                          variant="outlined"
                                          disabled={
                                            !canLoadMaps ||
                                            pendingCommand === config.mapLoadCommandName
                                          }
                                          onClick={() => handleLoadMap(mapId)}
                                        >
                                          Load
                                        </Button>
                                      ) : null}
                                      {!isDefault ? (
                                        <Button
                                          size="small"
                                          variant="text"
                                          disabled={
                                            !canSetDefaultMaps ||
                                            pendingCommand === config.mapSetDefaultCommandName
                                          }
                                          onClick={() => handleSetDefaultMap(mapId)}
                                        >
                                          Default
                                        </Button>
                                      ) : null}
                                    </Stack>
                                  </Stack>
                                </Box>
                              );
                            })}

                            {!sortedMaps.length ? (
                              <Typography variant="body2" color="text.secondary" sx={{ py: 1 }}>
                                {allMapIds.length
                                  ? "No saved maps match the current search."
                                  : "No saved maps have been published yet."}
                              </Typography>
                            ) : null}
                          </Stack>
                        </Box>

                        {!mapLoadCommandAvailable ? (
                          <Typography variant="caption" color="warning.main">
                            Map load command is unavailable on this device.
                          </Typography>
                        ) : null}
                        {!mapSetDefaultCommandAvailable ? (
                          <Typography variant="caption" color="warning.main">
                            Set-default command is unavailable on this device.
                          </Typography>
                        ) : null}
                        {snapshot.navState?.active ? (
                          <Typography variant="caption" color="text.secondary">
                            Finish the current navigation before loading a different map.
                          </Typography>
                        ) : null}
                        {!currentMapReady && currentMapId ? (
                          <Typography variant="caption" color="warning.main">
                            Active map state is stale. Wait for the map streams to settle before switching.
                          </Typography>
                        ) : null}
                        {!defaultMapReady && defaultMapId ? (
                          <Typography variant="caption" color="warning.main">
                            Default map state is stale.
                          </Typography>
                        ) : null}
                      </Stack>
                    </Collapse>
                  </Stack>
                </Box>

                <ToggleButtonGroup
                  exclusive
                  size="small"
                  value={selectionMode}
                  onChange={(_event, value: SelectionMode | null) => {
                    if (!value) return;
                    setSelectionMode(value);
                    setSelection(undefined);
                  }}
                >
                  <ToggleButton value="waypoints">Waypoints</ToggleButton>
                  <ToggleButton value="point">Point</ToggleButton>
                </ToggleButtonGroup>

                <Stack direction="row" spacing={1} flexWrap="wrap" useFlexGap>
                  <Button
                    variant="outlined"
                    size="small"
                    disabled={
                      !undockCommandAvailable ||
                      !connectionReady ||
                      !dockingReady ||
                      snapshot.dockingState !== "docked" ||
                      pendingCommand === config.undockCommandName
                    }
                    onClick={() =>
                      invokeAction(
                        config.undockCommandName,
                        "Sending undock command...",
                        "Undock command accepted."
                      )
                    }
                  >
                    Undock
                  </Button>
                  <Button
                    variant="outlined"
                    size="small"
                    disabled={
                      !returnAndDockCommandAvailable ||
                      !connectionReady ||
                      !dockingReady ||
                      !motorReady ||
                      !localizedReady ||
                      snapshot.dockingState !== "undocked" ||
                      pendingCommand === config.returnAndDockCommandName
                    }
                    onClick={() =>
                      invokeAction(
                        config.returnAndDockCommandName,
                        "Sending return-and-dock command...",
                        "Return-and-dock command accepted."
                      )
                    }
                  >
                    Return & Dock
                  </Button>
                  <Button
                    variant="outlined"
                    size="small"
                    color="warning"
                    disabled={
                      !cancelCommandAvailable ||
                      !navReady ||
                      !snapshot.navState?.active ||
                      pendingCommand === config.cancelNavCommandName
                    }
                    onClick={() =>
                      invokeAction(
                        config.cancelNavCommandName,
                        "Sending hold-position override...",
                        "Hold-position command sent. Waiting for navigation state..."
                      )
                    }
                  >
                    Hold Position
                  </Button>
                </Stack>

                <Box sx={{ p: 1.5, borderRadius: 2, bgcolor: "rgba(255,255,255,0.03)" }}>
                  <Stack spacing={0.75}>
                    <Stack direction="row" spacing={1} alignItems="center" useFlexGap flexWrap="wrap">
                      <Typography variant="subtitle2">System Status</Typography>
                      <Chip
                        size="small"
                        label={systemStatus.title}
                        color={
                          systemStatus.severity === "success"
                            ? "secondary"
                            : systemStatus.severity === "warning"
                              ? "warning"
                              : "default"
                        }
                        variant={systemStatus.severity === "success" ? "filled" : "outlined"}
                      />
                    </Stack>
                    <Typography variant="body2" color="text.secondary">
                      {systemStatus.detail}
                    </Typography>
                  </Stack>
                </Box>

                {snapshot.navState?.active ? (
                  <Box sx={{ p: 1.5, borderRadius: 2, bgcolor: "rgba(255,255,255,0.03)" }}>
                    <Stack spacing={0.75}>
                      <Typography variant="subtitle2">Active Navigation</Typography>
                      <Typography variant="body2">{activeNavLabel}</Typography>
                      <Typography variant="body2" color="text.secondary">
                        Status: {snapshot.navState.status_name || "Unknown"}
                      </Typography>
                      <Typography variant="body2" color="text.secondary">
                        Remaining route: {snapshot.navState.remaining_route_length_m.toFixed(2)} m
                      </Typography>
                      {snapshot.navState.auto_recovered ? (
                        <Chip label="Auto-recovered" color="warning" size="small" />
                      ) : null}
                    </Stack>
                  </Box>
                ) : null}

                {navTerminalVisible ? (
                  <Alert severity={snapshot.navState?.terminal_result === "failed" ? "error" : "info"}>
                    <Stack spacing={0.5}>
                      <Stack direction="row" spacing={1} alignItems="center" useFlexGap flexWrap="wrap">
                        <Typography variant="subtitle2">Last Navigation</Typography>
                        <Chip
                          size="small"
                          label={snapshot.navState?.terminal_result || "terminal"}
                          color={navLifecycleTone(snapshot.navState?.terminal_result)}
                        />
                      </Stack>
                      <Typography variant="body2">{activeNavLabel}</Typography>
                      <Typography variant="body2" color="text.secondary">
                        {snapshot.navState?.terminal_reason || "Navigation reached a terminal state."}
                      </Typography>
                    </Stack>
                  </Alert>
                ) : null}

                <Divider />

                {selectionMode === "waypoints" ? (
                  <>
                    <TextField
                      value={waypointSearch}
                      onChange={(event) => setWaypointSearch(event.target.value)}
                      size="small"
                      label="Search waypoints"
                    />
                    <Box sx={{ minHeight: 0, flex: 1, overflow: "auto" }}>
                      <List dense disablePadding>
                        {sortedWaypoints.map((waypoint) => {
                          const isSelected =
                            selection?.kind === "waypoint" && selection.waypoint.id === waypoint.id;
                          const secondary = [
                            waypoint.is_dock ? "Dock" : "",
                            snapshot.overlay?.current_waypoint_id === waypoint.id ? "Current" : ""
                          ]
                            .filter(Boolean)
                            .join(" · ");
                          return (
                            <ListItemButton
                              key={waypoint.id}
                              selected={isSelected}
                              onClick={() => handleSelectWaypoint(waypoint)}
                              sx={{ borderRadius: 1.5, mb: 0.5 }}
                            >
                              <ListItemText primary={waypoint.name} secondary={secondary || waypoint.id} />
                            </ListItemButton>
                          );
                        })}
                        {!sortedWaypoints.length ? (
                          <Typography variant="body2" color="text.secondary" sx={{ py: 1 }}>
                            No waypoints available for the current map.
                          </Typography>
                        ) : null}
                      </List>
                    </Box>
                  </>
                ) : (
                  <Box sx={{ p: 1.5, borderRadius: 2, bgcolor: "rgba(255,255,255,0.03)" }}>
                    <Stack spacing={1}>
                      <Typography variant="subtitle2">Point Navigation</Typography>
                      <Typography variant="body2" color="text.secondary">
                        Click the map to place a target. Point mode is intended for advanced use;
                        waypoint navigation is the safer default.
                      </Typography>
                    </Stack>
                  </Box>
                )}

                {selection ? (
                  <Box sx={{ p: 1.5, borderRadius: 2, bgcolor: "rgba(255,255,255,0.04)" }}>
                    {selection.kind === "waypoint" ? (
                      <Stack spacing={1}>
                        <Typography variant="subtitle2">Confirm Waypoint</Typography>
                        <Typography variant="body2">{selection.waypoint.name}</Typography>
                        <Typography variant="body2" color="text.secondary">
                          {selection.waypoint.is_dock ? "Dock waypoint" : "Standard waypoint"}
                        </Typography>
                        <Typography variant="caption" color="text.secondary">
                          {displayedMapUuid
                            ? formatWaypointGotoPayload(displayedMapUuid, selection.waypoint.id)
                            : "No command payload"}
                        </Typography>
                        {selectionReadinessIssues.length ? (
                          <Alert severity="warning">
                            <Stack spacing={0.5}>
                              {selectionReadinessIssues.map((issue) => (
                                <Typography key={issue} variant="body2">
                                  {issue}
                                </Typography>
                              ))}
                            </Stack>
                          </Alert>
                        ) : null}
                        <Stack direction="row" spacing={1}>
                          <Button
                            variant="contained"
                            onClick={confirmWaypointNavigation}
                            disabled={!selectionReady || pendingCommand === config.waypointGotoCommandName}
                          >
                            Confirm
                          </Button>
                          <Button variant="outlined" onClick={() => setSelection(undefined)}>
                            Cancel
                          </Button>
                        </Stack>
                      </Stack>
                    ) : (
                      <Stack spacing={1}>
                        <Typography variant="subtitle2">Confirm Point Target</Typography>
                        <Typography variant="body2" color="text.secondary">
                          {formatPose(selection.target)}
                        </Typography>
                        <ToggleButtonGroup
                          exclusive
                          size="small"
                          value={selection.headingMode}
                          onChange={(_event, value: PointHeadingMode | null) => {
                            if (value) updatePointHeadingMode(value);
                          }}
                        >
                          <ToggleButton value="current">Keep Heading</ToggleButton>
                          <ToggleButton value="path">Face Path</ToggleButton>
                          <ToggleButton value="custom">Custom</ToggleButton>
                        </ToggleButtonGroup>
                        {selection.headingMode === "custom" ? (
                          <TextField
                            label="Yaw (deg)"
                            type="number"
                            size="small"
                            value={selection.target.yawDeg}
                            onChange={(event) => updatePointYawDeg(Number(event.target.value))}
                            inputProps={{ step: 1 }}
                          />
                        ) : null}
                        <Typography variant="caption" color="text.secondary">
                          {displayedMapUuid
                            ? formatGotoPosePayload(
                                displayedMapUuid,
                                selection.target.x,
                                selection.target.y,
                                selection.target.yawDeg
                              )
                            : "No command payload"}
                        </Typography>
                        {selectionReadinessIssues.length ? (
                          <Alert severity="warning">
                            <Stack spacing={0.5}>
                              {selectionReadinessIssues.map((issue) => (
                                <Typography key={issue} variant="body2">
                                  {issue}
                                </Typography>
                              ))}
                            </Stack>
                          </Alert>
                        ) : null}
                        <Stack direction="row" spacing={1}>
                          <Button
                            variant="contained"
                            onClick={confirmPointNavigation}
                            disabled={!selectionReady || pendingCommand === config.gotoPoseCommandName}
                          >
                            Confirm
                          </Button>
                          <Button variant="outlined" onClick={() => setSelection(undefined)}>
                            Cancel
                          </Button>
                        </Stack>
                      </Stack>
                    )}
                  </Box>
                ) : null}

                {snapshot.warnings?.length ? (
                  <Alert severity="warning">
                    <Stack spacing={0.5}>
                      {snapshot.warnings.map((warning) => (
                        <Typography key={warning} variant="body2">
                          {warning}
                        </Typography>
                      ))}
                    </Stack>
                  </Alert>
                ) : null}

                {error ? <Alert severity="error">{error}</Alert> : null}
                {commandError ? <Alert severity="error">{commandError}</Alert> : null}
                {commandNotice ? <Alert severity="info">{commandNotice}</Alert> : null}

                <Box sx={{ mt: "auto" }}>
                  <Typography variant="caption" color="text.secondary">
                    Map view: {isFresh(snapshot.mapImageMetadataTime, 8000) ? "live" : "stale"} · Nav state: {isFresh(snapshot.navStateTime, 4000) ? "live" : "stale"} · Overlay: {isFresh(snapshot.overlayTime, 15000) ? "live" : "stale"} · Catalog: {mapsReady ? "ready" : "stale"}
                  </Typography>
                </Box>
              </Stack>
            </Box>
          </Box>
        )}
      </Box>
    </ThemeProvider>
  );
}
