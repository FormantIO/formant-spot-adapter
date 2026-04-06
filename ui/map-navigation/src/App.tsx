import { useEffect, useMemo, useRef, useState } from "react";
import {
  Alert,
  Box,
  Button,
  Chip,
  CircularProgress,
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
  formatWaypointGotoPayload,
  getInitialModuleConfig,
  parseModuleConfig,
  subscribeRealtimeSnapshot
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
type Selection =
  | { kind: "waypoint"; waypoint: GraphNavOverlayWaypoint }
  | { kind: "point"; target: TargetPose; headingMode: PointHeadingMode };

function isFresh(timestamp: number | undefined, maxAgeMs: number): boolean {
  return Boolean(timestamp && (Date.now() - timestamp) <= maxAgeMs);
}

function formatPose(target?: TargetPose): string {
  if (!target) return "No target selected";
  return `x=${target.x.toFixed(2)} m, y=${target.y.toFixed(2)} m, yaw=${target.yawDeg.toFixed(1)} deg`;
}

function getMapUuid(snapshot: StreamSnapshot): string {
  return (
    snapshot.mapImageMetadata?.map_uuid ||
    snapshot.navState?.map_uuid ||
    snapshot.overlay?.map_uuid ||
    ""
  );
}

function getMapLabel(snapshot: StreamSnapshot): string {
  return (
    snapshot.mapImageMetadata?.map_id ||
    snapshot.navState?.map_id ||
    snapshot.overlay?.map_id ||
    "Unknown map"
  );
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
    case "docked":
    case "off":
    case "error":
    case "not_ready":
      return "error";
    default:
      return "default";
  }
}

function buildMovementReadiness(snapshot: StreamSnapshot, mapUuid: string): string[] {
  const issues: string[] = [];
  if (!isFresh(snapshot.navStateTime, 4000)) issues.push("Navigation state is stale.");
  if (!isFresh(snapshot.mapImageMetadataTime, 8000)) issues.push("Map metadata is stale.");
  if (!isFresh(snapshot.connectionStateTime, 8000)) issues.push("Connection state is stale.");
  if (snapshot.connectionState !== "connected") issues.push("Robot is not connected.");
  if (snapshot.motorPowerState !== "on") issues.push("Motor power is not on.");
  if (snapshot.dockingState !== "undocked") issues.push("Robot must be undocked before moving.");
  if (!snapshot.navState?.localized) issues.push("Robot is not localized.");
  if (!mapUuid) issues.push("No active map is available.");
  return issues;
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
  const points = `${x},${y - size} ${x + half},${y + half} ${x},${y + (half / 2)} ${x - half},${y + half}`;
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
    .filter((entry): entry is { waypoint: GraphNavOverlayWaypoint; point: { x: number; y: number } } => Boolean(entry));

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

        {snapshot.overlay?.edges.map((edge, index) => {
          const from = waypointPixels.find((entry) => entry.waypoint.id === edge.from_waypoint_id);
          const to = waypointPixels.find((entry) => entry.waypoint.id === edge.to_waypoint_id);
          if (!from || !to) return null;
          return (
            <line
              key={`${edge.from_waypoint_id}-${edge.to_waypoint_id}-${index}`}
              x1={from.point.x}
              y1={from.point.y}
              x2={to.point.x}
              y2={to.point.y}
              stroke="rgba(43,182,255,0.36)"
              strokeWidth="1.5"
            />
          );
        })}

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
        ...patch,
        warnings: Array.from(
          new Set([...(previous.warnings || []), ...((patch.warnings as string[] | undefined) || [])])
        )
      }));
    });

    return () => {
      cancelled = true;
      unsubscribe();
    };
  }, [device, config]);

  useEffect(() => {
    if (snapshot.navState?.active) {
      setSelection(undefined);
      setSelectionMode("waypoints");
    }
  }, [snapshot.navState?.active, snapshot.navState?.command_id]);

  const mapUuid = getMapUuid(snapshot);
  const movementReadinessIssues = useMemo(
    () => buildMovementReadiness(snapshot, mapUuid),
    [snapshot, mapUuid]
  );

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
      ? `x=${snapshot.navState.current_seed_x.toFixed(2)} m, y=${snapshot.navState.current_seed_y.toFixed(
          2
        )} m`
      : "No live pose";

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
      void currentDevice
        .sendCommand(name, payload)
        .then(() => {
          setCommandNotice(successNotice);
        })
        .catch((commandErrorValue) => {
          setCommandError(
            commandErrorValue instanceof Error
              ? commandErrorValue.message
              : `Command ${name} failed.`
          );
        })
        .finally(() => {
          setPendingCommand((current) => (current === name ? undefined : current));
        });
    } catch (deviceError) {
      setPendingCommand(undefined);
      setCommandError(
        deviceError instanceof Error ? deviceError.message : `Command ${name} failed.`
      );
    }
  };

  const handleSelectWaypoint = (waypoint: GraphNavOverlayWaypoint) => {
    setSelection({ kind: "waypoint", waypoint });
    setSelectionMode("waypoints");
    setCommandError(undefined);
    setCommandNotice(undefined);
  };

  const handleSelectPoint = (target: TargetPose) => {
    const nextTarget = { ...target, yawDeg: getCurrentYawDeg(config, snapshot.navState) };
    setSelection({
      kind: "point",
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
    if (selection?.kind !== "waypoint" || !mapUuid) return;
    const payload = formatWaypointGotoPayload(mapUuid, selection.waypoint.id);
    void sendCommand(
      config.waypointGotoCommandName,
      payload,
      `Sent ${selection.waypoint.name}. Waiting for navigation state...`,
      `${selection.waypoint.name} navigation command accepted.`
    );
  };

  const confirmPointNavigation = () => {
    if (selection?.kind !== "point" || !mapUuid) return;
    const payload = formatGotoPosePayload(
      mapUuid,
      selection.target.x,
      selection.target.y,
      selection.target.yawDeg
    );
    void sendCommand(
      config.gotoPoseCommandName,
      payload,
      "Submitted point target. Waiting for navigation state...",
      "Point navigation command accepted."
    );
  };

  const invokeAction = (name: string, optimisticNotice: string, successNotice: string) => {
    void sendCommand(name, undefined, optimisticNotice, successNotice);
  };

  const selectionReady =
    selection?.kind === "waypoint"
      ? movementReadinessIssues.length === 0 && waypointCommandAvailable
      : selection?.kind === "point"
        ? movementReadinessIssues.length === 0 && gotoPoseCommandAvailable
        : false;

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
                <Chip label={getMapLabel(snapshot)} color="primary" />
                <Chip
                  label={snapshot.navState?.localized ? "Localized" : "Not localized"}
                  color={snapshot.navState?.localized ? "secondary" : "default"}
                  variant={snapshot.navState?.localized ? "filled" : "outlined"}
                />
                <Chip
                  label={snapshot.connectionState || "connection"}
                  color={statusTone(snapshot.connectionState)}
                  variant="outlined"
                />
                <Chip
                  label={snapshot.dockingState || "docking"}
                  color={statusTone(snapshot.dockingState)}
                  variant="outlined"
                />
                <Chip
                  label={`motors ${snapshot.motorPowerState || "unknown"}`}
                  color={statusTone(snapshot.motorPowerState)}
                  variant="outlined"
                />
                <Chip
                  label={snapshot.behaviorState || "behavior"}
                  color={statusTone(snapshot.behaviorState)}
                  variant="outlined"
                />
                {snapshot.navState?.active && snapshot.navState.status_name ? (
                  <Chip label={snapshot.navState.status_name} color="warning" />
                ) : null}
              </Stack>
            </Box>

            <Box
              sx={{
                width: { xs: "100%", md: 360 },
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
                    disabled={!undockCommandAvailable || snapshot.dockingState !== "docked" || pendingCommand === config.undockCommandName}
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
                    disabled={!returnAndDockCommandAvailable || pendingCommand === config.returnAndDockCommandName}
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
                    disabled={!cancelCommandAvailable || !snapshot.navState?.active || pendingCommand === config.cancelNavCommandName}
                    onClick={() =>
                      invokeAction(
                        config.cancelNavCommandName,
                        "Sending navigation cancel command...",
                        "Navigation cancel command accepted."
                      )
                    }
                  >
                    Cancel Nav
                  </Button>
                </Stack>

                {movementReadinessIssues.length ? (
                  <Alert severity="warning">
                    <Stack spacing={0.5}>
                      <Typography variant="subtitle2">Movement blocked</Typography>
                      {movementReadinessIssues.map((issue) => (
                        <Typography key={issue} variant="body2">
                          {issue}
                        </Typography>
                      ))}
                    </Stack>
                  </Alert>
                ) : (
                  <Alert severity="success">Robot is ready for waypoint or point navigation.</Alert>
                )}

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
                              <ListItemText
                                primary={waypoint.name}
                                secondary={secondary || waypoint.id}
                              />
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
                          {mapUuid
                            ? formatWaypointGotoPayload(mapUuid, selection.waypoint.id)
                            : "No command payload"}
                        </Typography>
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
                          {mapUuid
                            ? formatGotoPosePayload(
                                mapUuid,
                                selection.target.x,
                                selection.target.y,
                                selection.target.yawDeg
                              )
                            : "No command payload"}
                        </Typography>
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
                    Map metadata: {isFresh(snapshot.mapImageMetadataTime, 8000) ? "live" : "stale"} ·
                    Nav state: {isFresh(snapshot.navStateTime, 4000) ? "live" : "stale"} ·
                    Overlay: {isFresh(snapshot.overlayTime, 15000) ? "live" : "stale"}
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
