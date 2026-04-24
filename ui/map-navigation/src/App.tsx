import { type ReactNode, useEffect, useMemo, useRef, useState } from "react";
import {
  Alert,
  Box,
  Button,
  Chip,
  CircularProgress,
  Collapse,
  CssBaseline,
  Dialog,
  DialogActions,
  DialogContent,
  DialogTitle,
  IconButton,
  List,
  ListItemButton,
  ListItemText,
  Stack,
  TextField,
  Typography
} from "@mui/material";
import { alpha, createTheme, ThemeProvider } from "@mui/material/styles";
import { App as FormantApp } from "@formant/data-sdk";
import {
  DEFAULT_MODULE_CONFIG,
  authenticateAndGetDevice,
  createCommandRequestId,
  formatGotoPosePayload,
  formatMapIdPayload,
  formatRequestIdPayload,
  formatWaypointDeletePayload,
  formatWaypointGotoPayload,
  formatWaypointGotoByNamePayload,
  formatWaypointSavePayload,
  getInitialModuleConfig,
  loadTelemetrySnapshot,
  parseModuleConfig,
  subscribeRealtimeSnapshot,
  TELEMETRY_POLL_INTERVAL_MS
} from "./formant";
import { getCanvasSize, pixelToSeed, seedToPixel } from "./mapMath";
import {
  AsyncHealthState,
  ConnectionScreenState,
  INITIAL_CONNECT_TIMEOUT_MS,
  NAV_STATE_MAX_AGE_MS,
  ROBOT_STATE_MAX_AGE_MS,
  buildConnectionScreenState,
  deriveConnectionHealth,
  deriveMapState,
  deriveNavigationSummary,
  deriveRobotMetrics,
  isBehaviorNavigationReady,
  isFresh
} from "./statusModel";
import {
  GraphNavOverlayWaypoint,
  ModuleConfig,
  NavState,
  StreamSnapshot,
  TargetPose
} from "./types";

const LOG_PREFIX = "[spot-nav]";
const MAP_SURFACE_BACKGROUND = "#2a180f";
const MAP_STAGE_BACKGROUND = "#341f15";

const theme = createTheme({
  palette: {
    mode: "dark",
    primary: { main: "#2bb6ff" },
    secondary: { main: "#89f3c7" },
    warning: { main: "#ffb44d" },
    error: { main: "#ff6c7b" },
    background: {
      default: "#09131b",
      paper: "#101821"
    }
  },
  shape: {
    borderRadius: 6
  },
  typography: {
    fontFamily: `"IBM Plex Sans", "Inter", sans-serif`,
    h6: { fontWeight: 600 },
    subtitle2: { fontWeight: 600 }
  },
  components: {
    MuiButton: {
      styleOverrides: {
        root: {
          borderRadius: 8,
          textTransform: "none",
          boxShadow: "none"
        }
      }
    },
    MuiChip: {
      styleOverrides: {
        root: {
          borderRadius: 8
        }
      }
    },
    MuiPaper: {
      styleOverrides: {
        rounded: {
          borderRadius: 10
        }
      }
    }
  }
});

type SelectionMode = "waypoints" | "point";
type PointHeadingMode = "current" | "path" | "custom";
type Selection =
  | { kind: "waypoint"; mapUuid: string; waypoint: GraphNavOverlayWaypoint }
  | { kind: "point"; mapUuid: string; target: TargetPose; headingMode: PointHeadingMode };
type ModalState =
  | { kind: "waypoint-goto"; waypoint: GraphNavOverlayWaypoint; mapUuid: string }
  | { kind: "waypoint-delete"; waypoint: GraphNavOverlayWaypoint; mapUuid: string }
  | {
      kind: "point-goto";
      target: TargetPose;
      headingMode: PointHeadingMode;
      mapUuid: string;
    };

const panelSurfaceSx = {
  backdropFilter: "blur(18px)",
  backgroundColor: "rgba(10, 19, 27, 0.88)",
  border: "1px solid rgba(255,255,255,0.06)",
  boxShadow: "0 18px 54px rgba(0,0,0,0.28)"
} as const;

const cardSx = {
  p: 1.5,
  borderRadius: "10px",
  backgroundColor: "rgba(255,255,255,0.03)",
  border: "1px solid rgba(255,255,255,0.06)"
} as const;

function TrashIcon() {
  return (
    <svg width="14" height="14" viewBox="0 0 24 24" fill="none" aria-hidden="true">
      <path
        d="M9 3.75h6m-9 3h12m-1.5 0-.6 10.2a1.5 1.5 0 0 1-1.5 1.41H9.6a1.5 1.5 0 0 1-1.5-1.41L7.5 6.75m3 3.75v5.25m3-5.25v5.25"
        stroke="currentColor"
        strokeWidth="1.8"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  );
}

function roundCoord(value: number): number {
  return Math.round(value * 100);
}

function getWaypointSignature(waypoint: GraphNavOverlayWaypoint): string {
  return [
    waypoint.id || "",
    waypoint.name,
    roundCoord(waypoint.x),
    roundCoord(waypoint.y),
    waypoint.is_dock ? 1 : 0
  ].join("|");
}

function getCurrentWaypointLabel(snapshot: StreamSnapshot): string {
  const currentWaypointName = snapshot.overlay?.current_waypoint_name || "";
  if (currentWaypointName) return currentWaypointName;
  if (!snapshot.navState?.localized) return "Not localized";
  return "No saved waypoint nearby";
}

function formatPose(target?: TargetPose): string {
  if (!target) return "No target selected";
  return `x=${target.x.toFixed(2)} m, y=${target.y.toFixed(2)} m, yaw=${target.yawDeg.toFixed(1)} deg`;
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

function uniqueIssues(issues: string[]): string[] {
  return Array.from(new Set(issues.filter(Boolean)));
}

function dedupeWaypoints(waypoints: GraphNavOverlayWaypoint[]): GraphNavOverlayWaypoint[] {
  const seen = new Set<string>();
  const deduped: GraphNavOverlayWaypoint[] = [];
  for (const waypoint of waypoints) {
    const signature = getWaypointSignature(waypoint);
    if (seen.has(signature)) continue;
    seen.add(signature);
    deduped.push(waypoint);
  }
  return deduped;
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

function fitRect(
  containerWidth: number,
  containerHeight: number,
  aspectRatio: number,
  padding = 28
): { width: number; height: number } {
  const availableWidth = Math.max(containerWidth - (padding * 2), 0);
  const availableHeight = Math.max(containerHeight - (padding * 2), 0);
  if (!availableWidth || !availableHeight || !Number.isFinite(aspectRatio) || aspectRatio <= 0) {
    return { width: 0, height: 0 };
  }

  let width = availableWidth;
  let height = width / aspectRatio;
  if (height > availableHeight) {
    height = availableHeight;
    width = height * aspectRatio;
  }

  return {
    width: Math.max(width, 0),
    height: Math.max(height, 0)
  };
}

function useElementSize<T extends HTMLElement>() {
  const ref = useRef<T | null>(null);
  const [size, setSize] = useState({ width: 0, height: 0 });

  useEffect(() => {
    const element = ref.current;
    if (!element) return undefined;

    const observer = new ResizeObserver((entries) => {
      const entry = entries[0];
      if (!entry) return;
      setSize({
        width: entry.contentRect.width,
        height: entry.contentRect.height
      });
    });

    observer.observe(element);
    return () => observer.disconnect();
  }, []);

  return [ref, size] as const;
}

function navLifecycleTone(
  result: string | undefined
): "default" | "secondary" | "warning" | "error" {
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

function buildBaseNavigationIssues(
  snapshot: StreamSnapshot,
  currentMapUuid: string,
  preciseMapReady: boolean,
  requirePreciseMap: boolean,
  now: number
): string[] {
  const issues: string[] = [];
  const navFresh = isFresh(snapshot.navStateTime, NAV_STATE_MAX_AGE_MS, now);

  if (!currentMapUuid) {
    issues.push("No live map is available for navigation.");
  }

  if (!isFresh(snapshot.connectionStateTime, ROBOT_STATE_MAX_AGE_MS, now)) {
    issues.push("Waiting for the robot connection state.");
  } else if (snapshot.connectionState !== "connected") {
    issues.push("Robot is not connected.");
  }

  if (!navFresh) {
    issues.push("Waiting for live navigation state.");
  } else if (!snapshot.navState?.localized) {
    issues.push("Robot is not localized.");
  }

  if (!isFresh(snapshot.dockingStateTime, ROBOT_STATE_MAX_AGE_MS, now)) {
    issues.push("Waiting for docking state.");
  } else if (snapshot.dockingState === "docked") {
    issues.push("Robot is docked.");
  }

  if (!isFresh(snapshot.motorPowerStateTime, ROBOT_STATE_MAX_AGE_MS, now)) {
    issues.push("Waiting for motor power state.");
  } else if (snapshot.motorPowerState !== "on") {
    issues.push("Motor power is off.");
  }

  if (!isFresh(snapshot.behaviorStateTime, ROBOT_STATE_MAX_AGE_MS, now)) {
    issues.push("Waiting for behavior state.");
  } else if (!isBehaviorNavigationReady(snapshot.behaviorState)) {
    issues.push("Robot is not in a navigation-ready stance.");
  }

  if (requirePreciseMap && !preciseMapReady) {
    issues.push("Precise map metadata is still syncing.");
  }

  return issues;
}

function buildWaypointNavigationIssues(
  snapshot: StreamSnapshot,
  selection: Extract<Selection, { kind: "waypoint" }>,
  currentMapUuid: string,
  overlayWaypoints: GraphNavOverlayWaypoint[],
  now: number
): string[] {
  const issues = buildBaseNavigationIssues(snapshot, currentMapUuid, true, false, now);

  if (!isFresh(snapshot.overlayTime, ROBOT_STATE_MAX_AGE_MS, now)) {
    issues.push("Saved waypoint data is stale.");
  }

  const selectedSignature = getWaypointSignature(selection.waypoint);
  const waypointStillPresent = overlayWaypoints.some(
    (waypoint) => getWaypointSignature(waypoint) === selectedSignature
  );
  if (!waypointStillPresent) {
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
  currentMapUuid: string,
  preciseMapReady: boolean,
  now: number
): string[] {
  const issues = buildBaseNavigationIssues(
    snapshot,
    currentMapUuid,
    preciseMapReady,
    true,
    now
  );

  if (selection.mapUuid && currentMapUuid && selection.mapUuid !== currentMapUuid) {
    issues.push("Selected point no longer matches the displayed map.");
  }

  return issues;
}

function buildWaypointSaveIssues(
  snapshot: StreamSnapshot,
  currentMapId: string,
  now: number
): string[] {
  const issues: string[] = [];
  if (!currentMapId) {
    issues.push("Load a map before saving a waypoint.");
  }
  if (!isFresh(snapshot.connectionStateTime, ROBOT_STATE_MAX_AGE_MS, now)) {
    issues.push("Waiting for the robot connection.");
  } else if (snapshot.connectionState !== "connected") {
    issues.push("Robot is not connected.");
  }
  if (snapshot.navState?.active) {
    issues.push("Finish the active navigation before saving a waypoint.");
  }
  return issues;
}

interface SectionCardProps {
  title: string;
  subtitle?: string;
  actions?: ReactNode;
  children: ReactNode;
}

function SectionCard({ title, subtitle, actions, children }: SectionCardProps) {
  return (
    <Box sx={cardSx}>
      <Stack spacing={1.25}>
        <Stack direction="row" justifyContent="space-between" alignItems="flex-start" spacing={1}>
          <Stack spacing={0.35} sx={{ minWidth: 0 }}>
            <Typography variant="subtitle2">{title}</Typography>
            {subtitle ? (
              <Typography variant="caption" color="text.secondary">
                {subtitle}
              </Typography>
            ) : null}
          </Stack>
          {actions}
        </Stack>
        {children}
      </Stack>
    </Box>
  );
}

function StatusTile({
  label,
  value,
  tone
}: {
  label: string;
  value: string;
  tone: "good" | "neutral" | "caution" | "bad";
}) {
  const lightColor =
    tone === "good"
      ? "#89f3c7"
      : tone === "caution"
        ? "#ffb44d"
        : tone === "bad"
          ? "#ff6c7b"
          : "rgba(255,255,255,0.34)";
  const glowColor =
    tone === "good"
      ? alpha("#89f3c7", 0.22)
      : tone === "caution"
        ? alpha("#ffb44d", 0.2)
        : tone === "bad"
          ? alpha("#ff6c7b", 0.2)
          : "rgba(255,255,255,0.06)";

  return (
    <Box
      sx={{
        px: 0.85,
        py: 0.7,
        minHeight: 48,
        borderRadius: "6px",
        border: "1px solid rgba(255,255,255,0.06)",
        backgroundColor: "rgba(255,255,255,0.025)"
      }}
    >
      <Stack spacing={0.25}>
        <Stack direction="row" spacing={0.6} alignItems="center">
          <Box
            sx={{
              width: 6.5,
              height: 6.5,
              borderRadius: "50%",
              flexShrink: 0,
              backgroundColor: lightColor,
              boxShadow: `0 0 0 3px ${glowColor}`
            }}
          />
          <Typography
            variant="caption"
            color="text.secondary"
            sx={{
              lineHeight: 1,
              whiteSpace: "nowrap",
              letterSpacing: 0.2,
              fontSize: "0.68rem"
            }}
          >
            {label}
          </Typography>
        </Stack>
        <Typography
          variant="body2"
          sx={{
            fontWeight: 600,
            lineHeight: 1.1,
            fontSize: "0.77rem",
            pl: 1.55,
            whiteSpace: "nowrap",
            overflow: "hidden",
            textOverflow: "ellipsis"
          }}
        >
          {value}
        </Typography>
      </Stack>
    </Box>
  );
}

function ModeSelector({
  value,
  onChange
}: {
  value: SelectionMode;
  onChange: (value: SelectionMode) => void;
}) {
  return (
    <Stack
      direction="row"
      spacing={0.75}
      sx={{
        p: 0.5,
        borderRadius: "8px",
        bgcolor: "rgba(255,255,255,0.03)",
        border: "1px solid rgba(255,255,255,0.06)"
      }}
    >
      {([
        ["waypoints", "Waypoints"],
        ["point", "Point"]
      ] as const).map(([mode, label]) => {
        const selected = value === mode;
        return (
          <Button
            key={mode}
            fullWidth
            variant={selected ? "contained" : "text"}
            color={selected ? "primary" : "inherit"}
            onClick={() => onChange(mode)}
            sx={{
              minHeight: 40,
              color: selected ? undefined : "text.secondary",
              bgcolor: selected ? undefined : "transparent"
            }}
          >
            {label}
          </Button>
        );
      })}
    </Stack>
  );
}

function ConnectionScreen({
  state,
  onRetry
}: {
  state: ConnectionScreenState;
  onRetry: () => void;
}) {
  return (
    <Box
      sx={{
        width: "100%",
        height: "100%",
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
        px: 3
      }}
    >
      <Stack spacing={2.5} sx={{ width: "min(460px, 100%)", textAlign: "center" }}>
        <Box sx={{ height: 48, display: "flex", alignItems: "center", justifyContent: "center" }}>
          {state.phase === "connecting" ? (
            <CircularProgress size={34} color="primary" />
          ) : (
            <Box
              sx={{
                width: 38,
                height: 38,
                borderRadius: "50%",
                border: "2px solid rgba(255,255,255,0.22)",
                display: "flex",
                alignItems: "center",
                justifyContent: "center",
                fontSize: 22,
                fontWeight: 700,
                color: "warning.main"
              }}
            >
              !
            </Box>
          )}
        </Box>

        <Stack spacing={1}>
          <Typography variant="h6">{state.title}</Typography>
          <Typography variant="body2" color="text.secondary" sx={{ minHeight: 40 }}>
            {state.detail}
          </Typography>
        </Stack>

        <Stack spacing={1}>
          {state.steps.map((step) => (
            <Box
              key={step.label}
              sx={{
                p: 1.15,
                borderRadius: "8px",
                textAlign: "left",
                border: "1px solid rgba(255,255,255,0.06)",
                backgroundColor: "rgba(255,255,255,0.03)"
              }}
            >
              <Stack direction="row" justifyContent="space-between" alignItems="center" spacing={1}>
                <Typography variant="subtitle2">{step.label}</Typography>
                <Chip
                  size="small"
                  label={
                    step.status === "complete"
                      ? "Ready"
                      : step.status === "failed"
                        ? "Failed"
                        : "Waiting"
                  }
                  color={
                    step.status === "complete"
                      ? "secondary"
                      : step.status === "failed"
                        ? "warning"
                        : "default"
                  }
                  variant={step.status === "complete" ? "filled" : "outlined"}
                />
              </Stack>
              <Typography variant="caption" color="text.secondary">
                {step.detail}
              </Typography>
            </Box>
          ))}
        </Stack>

        {state.phase === "failed" ? (
          <Button variant="contained" onClick={onRetry} sx={{ alignSelf: "center", minWidth: 120 }}>
            Retry
          </Button>
        ) : null}
      </Stack>
    </Box>
  );
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

interface MapStageProps {
  snapshot: StreamSnapshot;
  selectionMode: SelectionMode;
  selection?: Selection;
  waypoints: GraphNavOverlayWaypoint[];
  labeledWaypointKeys: Set<string>;
  onSelectWaypoint: (waypoint: GraphNavOverlayWaypoint) => void;
  onSelectPoint: (target: TargetPose) => void;
}

function MapStage({
  snapshot,
  selectionMode,
  selection,
  waypoints,
  labeledWaypointKeys,
  onSelectWaypoint,
  onSelectPoint
}: MapStageProps) {
  const [containerRef, containerSize] = useElementSize<HTMLDivElement>();
  const overlayRef = useRef<SVGSVGElement | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const canvasSize = getCanvasSize(snapshot.mapImageMetadata);
  const aspectRatio = canvasSize.x / canvasSize.y;
  const stageSize = fitRect(containerSize.width, containerSize.height, aspectRatio, 32);

  useEffect(() => {
    if (!snapshot.mapImageCanvas || !canvasRef.current) return;
    const target = canvasRef.current;
    target.width = snapshot.mapImageCanvas.width;
    target.height = snapshot.mapImageCanvas.height;
    const context = target.getContext("2d");
    context?.drawImage(snapshot.mapImageCanvas, 0, 0);
  }, [snapshot.mapImageCanvas, snapshot.mapImageFrameVersion]);

  const waypointPixels = waypoints
    .map((waypoint) => {
      const point = seedToPixel(snapshot.mapImageMetadata, waypoint.x, waypoint.y);
      return point ? { waypoint, point } : null;
    })
    .filter(
      (entry): entry is { waypoint: GraphNavOverlayWaypoint; point: { x: number; y: number } } =>
        Boolean(entry)
    );
  const waypointPixelsById = new Map(
    waypointPixels.flatMap(({ waypoint, point }) =>
      waypoint.id ? [[waypoint.id, point] as const] : []
    )
  );
  const edgeLines = (snapshot.overlay?.edges || []).flatMap((edge) => {
    const from = waypointPixelsById.get(edge.from_waypoint_id);
    const to = waypointPixelsById.get(edge.to_waypoint_id);
    return from && to ? [{ from, to }] : [];
  });

  const handleMapClick = (event: React.MouseEvent<SVGSVGElement>) => {
    if (selectionMode !== "point") return;
    if (!snapshot.mapImageMetadata) return;
    const svg = overlayRef.current;
    if (!svg) return;
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
      ref={containerRef}
      sx={{
        position: "relative",
        flex: 1,
        minHeight: 0,
        overflow: "hidden",
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
        background: `linear-gradient(180deg, ${MAP_STAGE_BACKGROUND}, ${MAP_SURFACE_BACKGROUND})`
      }}
    >
      <Box
        sx={{
          position: "relative",
          width: `${stageSize.width}px`,
          height: `${stageSize.height}px`,
          maxWidth: "100%",
          maxHeight: "100%",
          borderRadius: "10px",
          overflow: "hidden",
          backgroundColor: MAP_SURFACE_BACKGROUND,
          boxShadow: "0 28px 72px rgba(0,0,0,0.28)"
        }}
      >
        {snapshot.mapImageCanvas ? (
          <canvas
            ref={canvasRef}
            style={{ width: "100%", height: "100%", display: "block" }}
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
          viewBox={`0 0 ${canvasSize.x} ${canvasSize.y}`}
          onClick={handleMapClick}
          style={{
            position: "absolute",
            inset: 0,
            width: "100%",
            height: "100%",
            cursor: selectionMode === "point" ? "crosshair" : "default"
          }}
        >
          {edgeLines.map(({ from, to }, index) => (
            <line
              key={`${from.x},${from.y}-${to.x},${to.y}-${index}`}
              x1={from.x}
              y1={from.y}
              x2={to.x}
              y2={to.y}
              stroke="rgba(255,255,255,0.24)"
              strokeWidth="2"
              strokeLinecap="round"
            />
          ))}

          {waypointPixels.map(({ waypoint, point }) => {
            const waypointSignature = getWaypointSignature(waypoint);
            const currentWaypointMatches =
              (waypoint.id && snapshot.overlay?.current_waypoint_id === waypoint.id) ||
              snapshot.overlay?.current_waypoint_name === waypoint.name;
            const isSelected =
              selection?.kind === "waypoint" &&
              getWaypointSignature(selection.waypoint) === waypointSignature;
            const showLabel = labeledWaypointKeys.has(waypointSignature);

            return (
              <g
                key={waypointSignature}
                onClick={(event) => {
                  event.stopPropagation();
                  onSelectWaypoint(waypoint);
                }}
                style={{ cursor: "pointer" }}
              >
                {currentWaypointMatches ? (
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
                {showLabel && waypoint.name.trim() ? (
                  <text
                    x={point.x + 10}
                    y={point.y - 8}
                    fill="white"
                    fontSize="13"
                    fontFamily="IBM Plex Sans, sans-serif"
                    stroke="rgba(9,19,27,0.94)"
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
    </Box>
  );
}

export default function App() {
  const connectionReadyRef = useRef(false);
  const phaseLogRef = useRef("");
  const [device, setDevice] = useState<{ id: string; name: string }>();
  const [config, setConfig] = useState<ModuleConfig>(DEFAULT_MODULE_CONFIG);
  const [snapshot, setSnapshot] = useState<StreamSnapshot>({});
  const [now, setNow] = useState(() => Date.now());
  const [connectionAttempt, setConnectionAttempt] = useState(0);
  const [connectionAttemptStartedAt, setConnectionAttemptStartedAt] = useState(() => Date.now());
  const [hasReachedReady, setHasReachedReady] = useState(false);
  const [bootstrapState, setBootstrapState] = useState<AsyncHealthState>({ status: "idle" });
  const [telemetryState, setTelemetryState] = useState<AsyncHealthState>({ status: "idle" });
  const [selectionMode, setSelectionMode] = useState<SelectionMode>("waypoints");
  const [selection, setSelection] = useState<Selection>();
  const [modalState, setModalState] = useState<ModalState>();
  const [availableCommands, setAvailableCommands] = useState<string[]>([]);
  const [waypointSearch, setWaypointSearch] = useState("");
  const [saveWaypointName, setSaveWaypointName] = useState("");
  const [saveWaypointExpanded, setSaveWaypointExpanded] = useState(false);
  const [mapSearch, setMapSearch] = useState("");
  const [mapsExpanded, setMapsExpanded] = useState(false);
  const [pendingCommand, setPendingCommand] = useState<string>();
  const [pendingCommandRequestId, setPendingCommandRequestId] = useState<string>();
  const [commandError, setCommandError] = useState<string>();
  const [commandNotice, setCommandNotice] = useState<string>();

  useEffect(() => {
    const intervalId = window.setInterval(() => setNow(Date.now()), 1000);
    return () => window.clearInterval(intervalId);
  }, []);

  useEffect(() => {
    let cancelled = false;
    let configCleanup: (() => void) | undefined;

    const bootstrap = async () => {
      setBootstrapState({ status: "pending" });
      try {
        const currentDevice = await authenticateAndGetDevice(INITIAL_CONNECT_TIMEOUT_MS);
        const initialConfig = await getInitialModuleConfig();
        const commands = await currentDevice.getAvailableCommands().catch(() => []);
        if (cancelled) return;
        setDevice({ id: currentDevice.id, name: currentDevice.name });
        setConfig(initialConfig);
        setAvailableCommands(commands.map((command: { name: string }) => command.name));
        configCleanup = FormantApp.addModuleConfigurationListener((event: { configuration: string }) => {
          setConfig(parseModuleConfig(event.configuration));
        });
        setBootstrapState({ status: "ready", lastSuccessAt: Date.now() });
      } catch (bootstrapError) {
        if (cancelled) return;
        setBootstrapState({
          status: "failed",
          error:
            bootstrapError instanceof Error
              ? bootstrapError.message
              : "Failed to initialize the module."
        });
      }
    };

    void bootstrap();

    return () => {
      cancelled = true;
      configCleanup?.();
    };
  }, [connectionAttempt]);

  useEffect(() => {
    if (!device) return undefined;

    let cancelled = false;
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
  }, [device, config, connectionAttempt]);

  useEffect(() => {
    if (!device) return undefined;

    let cancelled = false;
    setTelemetryState((previous) => ({
      status: previous.lastSuccessAt ? previous.status : "pending",
      lastSuccessAt: previous.lastSuccessAt,
      error: undefined
    }));

    const refreshTelemetry = async () => {
      try {
        const patch = await loadTelemetrySnapshot(device.id, config);
        if (cancelled) return;
        setSnapshot((previous) => ({
          ...previous,
          ...patch
        }));
        setTelemetryState({
          status: "ready",
          lastSuccessAt: Date.now()
        });
      } catch (telemetryError) {
        console.warn(`${LOG_PREFIX} telemetry refresh failed`, telemetryError);
        if (cancelled) return;
        setTelemetryState((previous) => ({
          status: previous.lastSuccessAt ? "ready" : "failed",
          lastSuccessAt: previous.lastSuccessAt,
          error:
            telemetryError instanceof Error
              ? telemetryError.message
              : "Failed to refresh telemetry snapshot."
        }));
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
  }, [device, config, connectionAttempt]);

  useEffect(() => {
    if (snapshot.navState?.active) {
      setSelection(undefined);
      setModalState(undefined);
      setSelectionMode("waypoints");
      setSaveWaypointExpanded(false);
    }
  }, [snapshot.navState?.active, snapshot.navState?.command_id]);

  useEffect(() => {
    const phase = snapshot.navState?.phase;
    if (phase !== "starting" && phase !== "active" && phase !== "terminal") return;
    const navRequestId =
      snapshot.navState?.request_id || snapshot.navState?.command_request_id || "";
    if (pendingCommandRequestId && navRequestId !== pendingCommandRequestId) return;
    setCommandNotice(undefined);
    if (pendingCommandRequestId && navRequestId === pendingCommandRequestId) {
      setPendingCommandRequestId(undefined);
    }
  }, [
    pendingCommandRequestId,
    snapshot.navState?.phase,
    snapshot.navState?.request_id,
    snapshot.navState?.command_request_id,
    snapshot.navState?.last_completed_command_id,
    snapshot.navState?.command_id
  ]);

  const overlayWaypoints = useMemo(
    () => dedupeWaypoints(snapshot.overlay?.waypoints || []),
    [snapshot.overlay?.waypoints]
  );

  const duplicateWaypointNames = useMemo(() => {
    const counts = new Map<string, number>();
    overlayWaypoints.forEach((waypoint) => {
      counts.set(waypoint.name, (counts.get(waypoint.name) || 0) + 1);
    });
    return counts;
  }, [overlayWaypoints]);

  const mapState = useMemo(() => deriveMapState(snapshot, now), [snapshot, now]);
  const robotMetrics = useMemo(() => deriveRobotMetrics(snapshot, now), [snapshot, now]);
  const navigationSummary = useMemo(
    () => deriveNavigationSummary(snapshot, mapState, now),
    [snapshot, mapState, now]
  );
  const connectionHealth = useMemo(
    () => deriveConnectionHealth(now, snapshot, bootstrapState, telemetryState),
    [now, snapshot, bootstrapState, telemetryState]
  );
  const connectionScreenState = useMemo(
    () =>
      buildConnectionScreenState(
        now,
        connectionHealth,
        snapshot,
        bootstrapState,
        telemetryState,
        connectionAttemptStartedAt,
        hasReachedReady
      ),
    [
      now,
      connectionHealth,
      snapshot,
      bootstrapState,
      telemetryState,
      connectionAttemptStartedAt,
      hasReachedReady
    ]
  );

  useEffect(() => {
    if (connectionHealth.interfaceReady === connectionReadyRef.current) return;
    connectionReadyRef.current = connectionHealth.interfaceReady;
    setConnectionAttemptStartedAt(Date.now());
    if (connectionHealth.interfaceReady) {
      setHasReachedReady(true);
    }
  }, [connectionHealth.interfaceReady]);

  useEffect(() => {
    const signature = JSON.stringify({
      phase: connectionScreenState.phase,
      title: connectionScreenState.title,
      bootstrapStatus: bootstrapState.status,
      telemetryStatus: telemetryState.status,
      interfaceReady: connectionHealth.interfaceReady,
      liveSessionReady: connectionHealth.liveSessionReady,
      liveMapHealthy: connectionHealth.liveMapHealthy,
      catalogHealthy: connectionHealth.catalogHealthy
    });
    if (phaseLogRef.current === signature) return;
    phaseLogRef.current = signature;
    console.info(`${LOG_PREFIX} connection state`, {
      phase: connectionScreenState.phase,
      title: connectionScreenState.title,
      detail: connectionScreenState.detail,
      bootstrap: bootstrapState,
      telemetry: telemetryState,
      health: connectionHealth,
      snapshotTimes: {
        realtimeActivityTime: snapshot.realtimeActivityTime,
        realtimeConnectionStateTime: snapshot.realtimeConnectionStateTime,
        navStateRealtimeTime: snapshot.navStateRealtimeTime,
        mapImageRealtimeTime: snapshot.mapImageRealtimeTime,
        mapImageMetadataRealtimeTime: snapshot.mapImageMetadataRealtimeTime,
        overlayRealtimeTime: snapshot.overlayRealtimeTime,
        mapsTime: snapshot.mapsTime,
        currentMapTime: snapshot.currentMapTime,
        defaultMapTime: snapshot.defaultMapTime
      }
    });
  }, [
    bootstrapState,
    connectionHealth,
    connectionScreenState.detail,
    connectionScreenState.phase,
    connectionScreenState.title,
    snapshot.currentMapTime,
    snapshot.defaultMapTime,
    snapshot.mapImageMetadataRealtimeTime,
    snapshot.mapImageRealtimeTime,
    snapshot.mapsTime,
    snapshot.navStateRealtimeTime,
    snapshot.overlayRealtimeTime,
    snapshot.realtimeActivityTime,
    snapshot.realtimeConnectionStateTime,
    telemetryState
  ]);

  const currentMapId = mapState.currentMapId;
  const displayedMapUuid = mapState.displayedMapUuid;
  const defaultMapId = mapState.defaultMapId;

  const sortedWaypoints = useMemo(() => {
    const currentWaypointName = snapshot.overlay?.current_waypoint_name || "";
    const filter = waypointSearch.trim().toLowerCase();
    return [...overlayWaypoints]
      .filter((waypoint) => {
        if (!filter) return true;
        return waypoint.name.toLowerCase().includes(filter);
      })
      .sort((left, right) => {
        if (left.is_dock !== right.is_dock) return left.is_dock ? -1 : 1;
        if ((left.name === currentWaypointName) !== (right.name === currentWaypointName)) {
          return left.name === currentWaypointName ? -1 : 1;
        }
        return left.name.localeCompare(right.name);
      });
  }, [overlayWaypoints, snapshot.overlay?.current_waypoint_name, waypointSearch]);

  const allMapIds = useMemo(() => {
    const ids = new Set<string>();
    (snapshot.maps || []).forEach((mapId) => {
      if (mapId) ids.add(mapId);
    });
    if (mapState.currentMapId) ids.add(mapState.currentMapId);
    if (mapState.displayedMapId) ids.add(mapState.displayedMapId);
    if (mapState.defaultMapId) ids.add(mapState.defaultMapId);
    return [...ids];
  }, [mapState.currentMapId, mapState.defaultMapId, mapState.displayedMapId, snapshot.maps]);

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

  const labeledWaypointKeys = useMemo(() => {
    const keys = new Set<string>();
    overlayWaypoints.forEach((waypoint) => {
      if (waypoint.name.trim()) {
        keys.add(getWaypointSignature(waypoint));
      }
    });

    return keys;
  }, [overlayWaypoints]);

  const selectionReadinessIssues = useMemo(() => {
    if (!selection) return [];
    if (selection.kind === "waypoint") {
      const issues = buildWaypointNavigationIssues(
        snapshot,
        selection,
        displayedMapUuid,
        overlayWaypoints,
        now
      );
      if (!availableCommands.includes(config.waypointGotoCommandName)) {
        issues.push("Waypoint goto command is unavailable.");
      }
      if (snapshot.navState?.active) {
        issues.push("Hold or complete the active navigation before sending a new destination.");
      }
      return uniqueIssues(issues);
    }

    const issues = buildPointNavigationIssues(
      snapshot,
      selection,
      displayedMapUuid,
      mapState.precisionFresh,
      now
    );
    if (!availableCommands.includes(config.gotoPoseCommandName)) {
      issues.push("Point goto command is unavailable.");
    }
    if (snapshot.navState?.active) {
      issues.push("Hold or complete the active navigation before sending a new destination.");
    }
    return uniqueIssues(issues);
  }, [
    availableCommands,
    config.gotoPoseCommandName,
    config.waypointGotoCommandName,
    displayedMapUuid,
    mapState.precisionFresh,
    now,
    overlayWaypoints,
    selection,
    snapshot
  ]);

  const waypointSaveIssues = useMemo(
    () => buildWaypointSaveIssues(snapshot, currentMapId, now),
    [snapshot, currentMapId, now]
  );
  const mapLoadCommandAvailable = availableCommands.includes(config.mapLoadCommandName);
  const mapSetDefaultCommandAvailable = availableCommands.includes(config.mapSetDefaultCommandName);
  const waypointSaveCommandAvailable = availableCommands.includes(config.waypointSaveCommandName);
  const waypointDeleteCommandAvailable = availableCommands.includes(
    config.waypointDeleteCommandName
  );
  const cancelCommandAvailable = availableCommands.includes(config.cancelNavCommandName);
  const returnAndDockCommandAvailable = availableCommands.includes(config.returnAndDockCommandName);
  const undockCommandAvailable = availableCommands.includes(config.undockCommandName);

  const waypointDeleteIssues = useMemo(() => {
    if (selection?.kind !== "waypoint") return [];
    const issues: string[] = [];
    if (!currentMapId) {
      issues.push("No active map is loaded.");
    }
    if (!waypointDeleteCommandAvailable) {
      issues.push("Waypoint delete command is unavailable.");
    }
    if (
      isFresh(snapshot.overlayTime, ROBOT_STATE_MAX_AGE_MS, now) &&
      !overlayWaypoints.some(
        (waypoint) =>
          getWaypointSignature(waypoint) === getWaypointSignature(selection.waypoint)
      )
    ) {
      issues.push("Selected waypoint is no longer available.");
    }
    return uniqueIssues(issues);
  }, [
    currentMapId,
    now,
    overlayWaypoints,
    selection,
    snapshot.overlayTime,
    waypointDeleteCommandAvailable
  ]);

  const selectionReady = Boolean(selection) && selectionReadinessIssues.length === 0;
  const currentPoseLabel =
    snapshot.navState?.has_current_seed_pose &&
    typeof snapshot.navState.current_seed_x === "number" &&
    typeof snapshot.navState.current_seed_y === "number"
      ? `x=${snapshot.navState.current_seed_x.toFixed(2)} m, y=${snapshot.navState.current_seed_y.toFixed(2)} m`
      : "No live pose";

  const activeNavLabel =
    snapshot.navState?.target_name ||
    (snapshot.navState?.target_waypoint_id
      ? `Waypoint ${snapshot.navState.target_waypoint_id}`
      : snapshot.navState?.has_seed_goal &&
          typeof snapshot.navState.target_seed_x === "number" &&
          typeof snapshot.navState.target_seed_y === "number"
        ? `x=${snapshot.navState.target_seed_x.toFixed(1)} m, y=${snapshot.navState.target_seed_y.toFixed(1)} m`
        : "No active destination");

  const connectionReady =
    isFresh(snapshot.connectionStateTime, ROBOT_STATE_MAX_AGE_MS, now) &&
    snapshot.connectionState === "connected";
  const dockingReady =
    isFresh(snapshot.dockingStateTime, ROBOT_STATE_MAX_AGE_MS, now);
  const motorReady =
    isFresh(snapshot.motorPowerStateTime, ROBOT_STATE_MAX_AGE_MS, now) &&
    snapshot.motorPowerState === "on";
  const localizedReady =
    isFresh(snapshot.navStateTime, NAV_STATE_MAX_AGE_MS, now) &&
    Boolean(snapshot.navState?.localized);
  const mapsReady = isFresh(snapshot.mapsTime, ROBOT_STATE_MAX_AGE_MS, now);

  const canLoadMaps = mapLoadCommandAvailable && connectionReady && !snapshot.navState?.active;
  const canSetDefaultMaps = mapSetDefaultCommandAvailable;
  const canSaveWaypoint =
    waypointSaveCommandAvailable &&
    waypointSaveIssues.length === 0 &&
    pendingCommand !== config.waypointSaveCommandName;

  const mapsPanelVisible = mapsExpanded || !currentMapId || sortedMaps.length === 0;
  const navTerminalVisible =
    snapshot.navState?.phase === "terminal" && Boolean(snapshot.navState?.terminal_result);

  useEffect(() => {
    setSelection((current) => {
      if (!current) return current;
      if (current.mapUuid && displayedMapUuid && current.mapUuid !== displayedMapUuid) {
        return undefined;
      }
      if (current.kind === "waypoint") {
        const stillPresent = overlayWaypoints.some(
          (waypoint) =>
            getWaypointSignature(waypoint) === getWaypointSignature(current.waypoint)
        );
        if (!stillPresent) return undefined;
      }
      return current;
    });
  }, [displayedMapUuid, overlayWaypoints]);

  useEffect(() => {
    if (selection) return;
    setModalState(undefined);
  }, [selection]);

  const handleRetryConnection = () => {
    connectionReadyRef.current = false;
    setDevice(undefined);
    setConfig(DEFAULT_MODULE_CONFIG);
    setSnapshot({});
    setAvailableCommands([]);
    setSelection(undefined);
    setModalState(undefined);
    setSelectionMode("waypoints");
    setWaypointSearch("");
    setSaveWaypointName("");
    setSaveWaypointExpanded(false);
    setMapSearch("");
    setMapsExpanded(false);
    setPendingCommand(undefined);
    setPendingCommandRequestId(undefined);
    setCommandError(undefined);
    setCommandNotice(undefined);
    setBootstrapState({ status: "idle" });
    setTelemetryState({ status: "idle" });
    setHasReachedReady(false);
    setConnectionAttemptStartedAt(Date.now());
    setConnectionAttempt((current) => current + 1);
  };

  const sendCommand = async (
    name: string,
    payload: string | undefined,
    optimisticNotice: string,
    successNotice: string,
    requestId?: string
  ) => {
    setPendingCommand(name);
    setPendingCommandRequestId(requestId);
    setCommandError(undefined);
    setCommandNotice(optimisticNotice);

    try {
      const currentDevice = await authenticateAndGetDevice();
      await currentDevice.sendCommand(name, payload);
      setCommandNotice(successNotice);
    } catch (deviceError) {
      setPendingCommandRequestId(undefined);
      setCommandError(
        deviceError instanceof Error ? deviceError.message : `Command ${name} failed.`
      );
    } finally {
      setPendingCommand((current) => (current === name ? undefined : current));
      if (!requestId) setPendingCommandRequestId(undefined);
    }
  };

  const handleLoadMap = (mapId: string) => {
    if (!mapId) return;
    setSelection(undefined);
    setSelectionMode("waypoints");
    setSaveWaypointExpanded(false);
    setSaveWaypointName("");
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
      "Default map command sent. Waiting for the adapter state to update..."
    );
  };

  const handleSelectWaypoint = (waypoint: GraphNavOverlayWaypoint) => {
    const mapUuid = snapshot.overlay?.map_uuid || snapshot.mapImageMetadata?.map_uuid || "";
    setSaveWaypointExpanded(false);
    setSelection({ kind: "waypoint", mapUuid, waypoint });
    setModalState({ kind: "waypoint-goto", waypoint, mapUuid });
    setSelectionMode("waypoints");
    setCommandError(undefined);
    setCommandNotice(undefined);
  };

  const handleRequestWaypointDelete = (waypoint: GraphNavOverlayWaypoint) => {
    const mapUuid = snapshot.overlay?.map_uuid || snapshot.mapImageMetadata?.map_uuid || "";
    setSelection({ kind: "waypoint", mapUuid, waypoint });
    setModalState({ kind: "waypoint-delete", waypoint, mapUuid });
    setSelectionMode("waypoints");
    setCommandError(undefined);
    setCommandNotice(undefined);
  };

  const handleSelectPoint = (target: TargetPose) => {
    const mapUuid = snapshot.mapImageMetadata?.map_uuid || "";
    const nextTarget = { ...target, yawDeg: getCurrentYawDeg(config, snapshot.navState) };
    setSaveWaypointExpanded(false);
    const headingMode = config.defaultYawMode === "fixed" ? "custom" : "current";
    setSelection({ kind: "point", mapUuid, target: nextTarget, headingMode });
    setModalState({ kind: "point-goto", mapUuid, target: nextTarget, headingMode });
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
    const nextSelection: Selection = {
      kind: "point",
      mapUuid: selection.mapUuid,
      headingMode,
      target: {
        ...selection.target,
        yawDeg: nextYawDeg
      }
    };
    setSelection(nextSelection);
    setModalState((current) =>
      current?.kind === "point-goto"
        ? {
            kind: "point-goto",
            mapUuid: nextSelection.mapUuid,
            headingMode,
            target: nextSelection.target
          }
        : current
    );
  };

  const updatePointYawDeg = (value: number) => {
    if (selection?.kind !== "point" || !Number.isFinite(value)) return;
    const nextSelection: Selection = {
      ...selection,
      headingMode: "custom",
      target: {
        ...selection.target,
        yawDeg: value
      }
    };
    setSelection(nextSelection);
    setModalState((current) =>
      current?.kind === "point-goto"
        ? {
            kind: "point-goto",
            mapUuid: nextSelection.mapUuid,
            headingMode: "custom",
            target: nextSelection.target
          }
        : current
    );
  };

  const handleSaveWaypoint = () => {
    const trimmedName = saveWaypointName.trim();
    setSelection(undefined);
    setSelectionMode("waypoints");
    setCommandError(undefined);
    setCommandNotice(undefined);
    setSaveWaypointExpanded(false);
    setSaveWaypointName("");
    void sendCommand(
      config.waypointSaveCommandName,
      formatWaypointSavePayload(trimmedName),
      trimmedName ? `Saving ${trimmedName}...` : "Saving current pose as a waypoint...",
      trimmedName
        ? `${trimmedName} save command sent. Waiting for the waypoint list to update...`
        : "Waypoint save command sent. Waiting for the waypoint list to update..."
    );
  };

  const confirmWaypointNavigation = () => {
    if (selection?.kind !== "waypoint" || !displayedMapUuid) return;
    const requestId = createCommandRequestId("waypoint");
    const payload = selection.waypoint.id
      ? formatWaypointGotoPayload(displayedMapUuid, selection.waypoint.id, requestId)
      : formatWaypointGotoByNamePayload(displayedMapUuid, selection.waypoint.name, requestId);
    setModalState(undefined);
    setSelection(undefined);
    void sendCommand(
      config.waypointGotoCommandName,
      payload,
      `Sending ${selection.waypoint.name}...`,
      `${selection.waypoint.name} command sent. Waiting for navigation state...`,
      requestId
    );
  };

  const confirmPointNavigation = () => {
    if (selection?.kind !== "point" || !displayedMapUuid) return;
    const requestId = createCommandRequestId("pose");
    const payload = formatGotoPosePayload(
      displayedMapUuid,
      selection.target.x,
      selection.target.y,
      selection.target.yawDeg,
      requestId
    );
    setModalState(undefined);
    setSelection(undefined);
    void sendCommand(
      config.gotoPoseCommandName,
      payload,
      "Sending point target...",
      "Point target command sent. Waiting for navigation state...",
      requestId
    );
  };

  const confirmWaypointDelete = () => {
    if (selection?.kind !== "waypoint") return;
    setModalState(undefined);
    setSelection(undefined);
    void sendCommand(
      config.waypointDeleteCommandName,
      formatWaypointDeletePayload(selection.waypoint.name),
      `Deleting ${selection.waypoint.name}...`,
      `${selection.waypoint.name} delete command sent. Waiting for the waypoint list to update...`
    );
  };

  const invokeAction = (name: string, optimisticNotice: string, successNotice: string) => {
    const shouldCorrelateNav =
      name === config.cancelNavCommandName || name === config.returnAndDockCommandName;
    const requestId = shouldCorrelateNav ? createCommandRequestId("nav-action") : undefined;
    void sendCommand(
      name,
      requestId ? formatRequestIdPayload(requestId) : undefined,
      optimisticNotice,
      successNotice,
      requestId
    );
  };

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
        {connectionScreenState.phase !== "ready" ? (
          <ConnectionScreen state={connectionScreenState} onRetry={handleRetryConnection} />
        ) : (
          <Box
            sx={{
              display: "flex",
              flexDirection: { xs: "column", md: "row" },
              width: "100%",
              height: "100%"
            }}
          >
            <Box
              sx={{
                flex: 1,
                minHeight: { xs: "55%", md: 0 },
                display: "flex",
                backgroundColor: MAP_SURFACE_BACKGROUND
              }}
            >
              <MapStage
                snapshot={snapshot}
                selectionMode={selectionMode}
                selection={selection}
                waypoints={overlayWaypoints}
                labeledWaypointKeys={labeledWaypointKeys}
                onSelectWaypoint={handleSelectWaypoint}
                onSelectPoint={handleSelectPoint}
              />
            </Box>

            <Box
              sx={{
                width: { xs: "100%", md: 388 },
                borderLeft: { md: "1px solid rgba(255,255,255,0.06)" },
                borderTop: { xs: "1px solid rgba(255,255,255,0.06)", md: "none" },
                ...panelSurfaceSx
              }}
            >
              <Stack spacing={1.5} sx={{ height: "100%", p: 1.5 }}>
                <Stack spacing={0.5}>
                  <Typography variant="h6" noWrap>
                    {device?.name || "Current device"}
                  </Typography>
                  <Typography variant="body2" color="text.secondary" noWrap>
                    Current waypoint: {getCurrentWaypointLabel(snapshot)}
                  </Typography>
                  <Typography variant="body2" color="text.secondary" noWrap>
                    Pose: {currentPoseLabel}
                  </Typography>
                </Stack>

                <SectionCard
                  title="Map"
                  subtitle={undefined}
                  actions={
                    allMapIds.length > 1 ? (
                      <Button
                        size="small"
                        variant={mapsExpanded ? "contained" : "outlined"}
                        onClick={() => setMapsExpanded((current) => !current)}
                        sx={{ minWidth: 0, px: 1.1 }}
                      >
                        {mapsExpanded ? "Hide" : "Catalog"}
                      </Button>
                    ) : undefined
                  }
                >
                  <Stack spacing={0.7}>
                    <Stack spacing={0.35}>
                      <Stack
                        direction="row"
                        spacing={0.75}
                        alignItems="center"
                        flexWrap="wrap"
                        useFlexGap
                      >
                        <Typography
                          variant="body1"
                          sx={{
                            fontWeight: 600,
                            minWidth: 0,
                            maxWidth: "100%",
                            whiteSpace: "nowrap",
                            overflow: "hidden",
                            textOverflow: "ellipsis"
                          }}
                        >
                          {currentMapId || "No active map"}
                        </Typography>
                        {currentMapId ? <Chip size="small" label="Active" color="primary" /> : null}
                        {defaultMapId === currentMapId && currentMapId ? (
                          <Chip size="small" label="Default" color="secondary" />
                        ) : null}
                      </Stack>
                      <Typography variant="caption" color="text.secondary" noWrap>
                        {defaultMapId
                          ? defaultMapId === currentMapId
                            ? "Default map"
                            : `Default: ${defaultMapId}`
                          : allMapIds.length
                            ? "No default map"
                            : "No saved maps"}
                        {mapState.savedCount ? ` · ${mapState.savedCount} saved` : ""}
                        {mapState.headline ? ` · ${mapState.headline}` : ""}
                      </Typography>
                      {!mapsExpanded ? (
                        <Typography variant="caption" color="text.secondary" noWrap>
                          {mapState.detail}
                        </Typography>
                      ) : null}
                    </Stack>

                    <Collapse in={mapsPanelVisible} unmountOnExit>
                      <Stack spacing={0.85}>
                        {allMapIds.length > 6 ? (
                          <TextField
                            value={mapSearch}
                            onChange={(event) => setMapSearch(event.target.value)}
                            size="small"
                            label="Search maps"
                          />
                        ) : null}

                        <Box sx={{ maxHeight: 232, overflow: "auto" }}>
                          <Stack spacing={0.75}>
                            {sortedMaps.map((mapId) => {
                              const isCurrent = mapId === currentMapId;
                              const isDefault = mapId === defaultMapId;
                              return (
                                <Box
                                  key={mapId}
                                  sx={{
                                    p: 0.95,
                                    borderRadius: 1.6,
                                    border: "1px solid rgba(255,255,255,0.06)",
                                    backgroundColor: isCurrent
                                      ? alpha("#2bb6ff", 0.1)
                                      : "rgba(255,255,255,0.02)"
                                  }}
                                >
                                  <Stack spacing={0.8}>
                                    <Stack
                                      direction="row"
                                      justifyContent="space-between"
                                      alignItems="center"
                                      spacing={1}
                                    >
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
                                      <Button
                                        fullWidth
                                        size="small"
                                        variant={isCurrent ? "outlined" : "contained"}
                                        disabled={
                                          isCurrent ||
                                          !canLoadMaps ||
                                          pendingCommand === config.mapLoadCommandName
                                        }
                                        onClick={() => handleLoadMap(mapId)}
                                        sx={{ minWidth: 0 }}
                                      >
                                        {isCurrent ? "Loaded" : "Load"}
                                      </Button>
                                      <Button
                                        fullWidth
                                        size="small"
                                        variant={isDefault ? "outlined" : "text"}
                                        disabled={
                                          isDefault ||
                                          !canSetDefaultMaps ||
                                          pendingCommand === config.mapSetDefaultCommandName
                                        }
                                        onClick={() => handleSetDefaultMap(mapId)}
                                        sx={{ minWidth: 0 }}
                                      >
                                        {isDefault ? "Default" : "Default"}
                                      </Button>
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
                        {!mapsReady ? (
                          <Typography variant="caption" color="text.secondary">
                            Map catalog is refreshing.
                          </Typography>
                        ) : null}
                      </Stack>
                    </Collapse>
                  </Stack>
                </SectionCard>

                <SectionCard title="Robot State">
                  <Box
                    sx={{
                      display: "grid",
                      gridTemplateColumns: "repeat(3, minmax(0, 1fr))",
                      gap: 0.6
                    }}
                  >
                    {robotMetrics.map((metric) => (
                      <StatusTile
                        key={metric.label}
                        label={metric.label}
                        value={metric.value}
                        tone={metric.tone}
                      />
                    ))}
                  </Box>
                </SectionCard>

                <SectionCard title="Robot Actions">
                  <Stack direction="row" spacing={0.75}>
                    <Button
                      sx={{ flex: 1, minWidth: 0 }}
                      size="small"
                      variant={snapshot.dockingState === "docked" ? "contained" : "outlined"}
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
                      sx={{ flex: 1, minWidth: 0 }}
                      size="small"
                      variant={
                        snapshot.dockingState === "undocked" && localizedReady
                          ? "contained"
                          : "outlined"
                      }
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
                      Return
                    </Button>
                    <Button
                      sx={{ flex: 1, minWidth: 0 }}
                      size="small"
                      color="warning"
                      variant={snapshot.navState?.active ? "contained" : "outlined"}
                      disabled={
                        !cancelCommandAvailable ||
                        !isFresh(snapshot.navStateTime, NAV_STATE_MAX_AGE_MS, now) ||
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
                      Hold
                    </Button>
                  </Stack>
                </SectionCard>

                <Box sx={{ flex: 1, minHeight: 0, display: "flex" }}>
                  <Box sx={{ ...cardSx, flex: 1, minHeight: 0, display: "flex" }}>
                    <Stack spacing={1} sx={{ flex: 1, minHeight: 0 }}>
                      <Stack spacing={0.45}>
                        <Stack
                          direction="row"
                          spacing={1}
                          justifyContent="space-between"
                          alignItems="center"
                        >
                          <Typography variant="subtitle2">Navigate</Typography>
                          <Typography variant="caption" color="text.secondary" noWrap>
                            {navigationSummary.title}
                          </Typography>
                        </Stack>
                        <Typography variant="caption" color="text.secondary" noWrap>
                          {selectionMode === "waypoints"
                            ? "Click a saved waypoint to send the robot."
                            : "Click the map to place a point target."}
                        </Typography>
                      </Stack>

                      <ModeSelector
                        value={selectionMode}
                        onChange={(value) => {
                          setSelectionMode(value);
                          setSelection(undefined);
                          setSaveWaypointExpanded(false);
                        }}
                      />

                      {selectionMode === "waypoints" ? (
                        <>
                          <Stack
                            direction="row"
                            spacing={0.75}
                            justifyContent="space-between"
                            alignItems="center"
                          >
                            <Typography variant="subtitle2">Saved Waypoints</Typography>
                            <Button
                              size="small"
                              variant={saveWaypointExpanded ? "contained" : "outlined"}
                              disabled={
                                !waypointSaveCommandAvailable ||
                                pendingCommand === config.waypointSaveCommandName
                              }
                              onClick={() => {
                                setSaveWaypointExpanded((current) => !current);
                                setSelection(undefined);
                                setModalState(undefined);
                              }}
                            >
                              Save Current
                            </Button>
                          </Stack>

                          <Collapse in={saveWaypointExpanded} unmountOnExit>
                            <Box
                              sx={{
                                p: 1.15,
                                borderRadius: 1.8,
                                border: "1px solid rgba(255,255,255,0.06)",
                                backgroundColor: "rgba(255,255,255,0.02)"
                              }}
                            >
                              <Stack spacing={0.85}>
                                <Typography variant="caption" color="text.secondary">
                                  Save the current pose on {currentMapId || "the active map"}.
                                </Typography>
                                <TextField
                                  label="Waypoint name"
                                  placeholder="Leave blank to auto-generate"
                                  size="small"
                                  value={saveWaypointName}
                                  onChange={(event) => setSaveWaypointName(event.target.value)}
                                />
                                {waypointSaveIssues.length ? (
                                  <Alert severity="warning">
                                    <Stack spacing={0.5}>
                                      {waypointSaveIssues.map((issue) => (
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
                                    onClick={handleSaveWaypoint}
                                    disabled={!canSaveWaypoint}
                                  >
                                    Save
                                  </Button>
                                  <Button
                                    variant="outlined"
                                    onClick={() => {
                                      setSaveWaypointExpanded(false);
                                      setSaveWaypointName("");
                                    }}
                                  >
                                    Cancel
                                  </Button>
                                </Stack>
                              </Stack>
                            </Box>
                          </Collapse>

                          <TextField
                            value={waypointSearch}
                            onChange={(event) => setWaypointSearch(event.target.value)}
                            size="small"
                            label="Search waypoints"
                          />

                          <Box
                            sx={{
                              flex: 1,
                              minHeight: 0,
                              overflow: "auto",
                              borderRadius: 2,
                              border: "1px solid rgba(255,255,255,0.06)",
                              backgroundColor: "rgba(255,255,255,0.02)"
                            }}
                          >
                            <List dense disablePadding sx={{ p: 0.65 }}>
                              {sortedWaypoints.map((waypoint) => {
                                const isSelected =
                                  selection?.kind === "waypoint" &&
                                  getWaypointSignature(selection.waypoint) ===
                                    getWaypointSignature(waypoint);
                                const duplicateNameCount =
                                  duplicateWaypointNames.get(waypoint.name) || 0;
                                const secondaryParts = [
                                  waypoint.is_dock ? "Dock" : "",
                                  snapshot.overlay?.current_waypoint_name === waypoint.name
                                    ? "Current"
                                    : "",
                                  duplicateNameCount > 1
                                    ? `${waypoint.x.toFixed(1)}, ${waypoint.y.toFixed(1)}`
                                    : ""
                                ].filter(Boolean);

                                return (
                                  <ListItemButton
                                    key={getWaypointSignature(waypoint)}
                                    selected={isSelected}
                                    onClick={() => handleSelectWaypoint(waypoint)}
                                    sx={{
                                      borderRadius: 1.4,
                                      mb: 0.4,
                                      pr: 0.75,
                                      alignItems: "stretch"
                                    }}
                                  >
                                    <ListItemText
                                      primary={waypoint.name}
                                      secondary={secondaryParts.join(" · ") || undefined}
                                      primaryTypographyProps={{ noWrap: true }}
                                      secondaryTypographyProps={{ noWrap: true }}
                                    />
                                    <IconButton
                                      size="small"
                                      edge="end"
                                      disabled={
                                        !waypointDeleteCommandAvailable ||
                                        pendingCommand === config.waypointDeleteCommandName
                                      }
                                      onClick={(event) => {
                                        event.stopPropagation();
                                        handleRequestWaypointDelete(waypoint);
                                      }}
                                      sx={{
                                        ml: 0.25,
                                        color: "text.secondary"
                                      }}
                                    >
                                      <TrashIcon />
                                    </IconButton>
                                  </ListItemButton>
                                );
                              })}
                              {!sortedWaypoints.length ? (
                                <Typography variant="body2" color="text.secondary" sx={{ p: 1 }}>
                                  {currentMapId
                                    ? "No saved waypoints yet for this map."
                                    : "Load a map to browse or save waypoints."}
                                </Typography>
                              ) : null}
                            </List>
                          </Box>

                          {!waypointSaveCommandAvailable ? (
                            <Typography variant="caption" color="warning.main">
                              Waypoint save command is unavailable.
                            </Typography>
                          ) : null}
                          {!waypointDeleteCommandAvailable ? (
                            <Typography variant="caption" color="warning.main">
                              Waypoint delete command is unavailable.
                            </Typography>
                          ) : null}
                        </>
                      ) : (
                        <Box
                          sx={{
                            p: 1.15,
                            borderRadius: 1.8,
                            border: "1px solid rgba(255,255,255,0.06)",
                            backgroundColor: "rgba(255,255,255,0.02)"
                          }}
                        >
                          <Typography variant="body2" color="text.secondary">
                            Point mode is advanced. Click the map to review a target before sending it.
                          </Typography>
                        </Box>
                      )}
                    </Stack>
                  </Box>
                </Box>

                <Dialog
                  open={Boolean(modalState)}
                  onClose={() => {
                    setModalState(undefined);
                    setSelection(undefined);
                  }}
                  fullWidth
                  maxWidth="xs"
                >
                  {modalState?.kind === "waypoint-goto" && selection?.kind === "waypoint" ? (
                    <>
                      <DialogTitle>Send Robot To Waypoint</DialogTitle>
                      <DialogContent>
                        <Stack spacing={1.2} sx={{ pt: 0.5 }}>
                          <Typography variant="body1" sx={{ fontWeight: 600 }}>
                            {selection.waypoint.name}
                          </Typography>
                          <Typography variant="body2" color="text.secondary">
                            {selection.waypoint.is_dock ? "Dock waypoint" : "Saved waypoint"}
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
                          ) : (
                            <Typography variant="body2" color="text.secondary">
                              Send the robot to this saved waypoint?
                            </Typography>
                          )}
                        </Stack>
                      </DialogContent>
                      <DialogActions sx={{ px: 3, pb: 2.25 }}>
                        <Button
                          variant="outlined"
                          onClick={() => {
                            setModalState(undefined);
                            setSelection(undefined);
                          }}
                        >
                          Cancel
                        </Button>
                        <Button
                          variant="contained"
                          onClick={confirmWaypointNavigation}
                          disabled={
                            !selectionReady ||
                            pendingCommand === config.waypointGotoCommandName
                          }
                        >
                          Send
                        </Button>
                      </DialogActions>
                    </>
                  ) : null}

                  {modalState?.kind === "waypoint-delete" && selection?.kind === "waypoint" ? (
                    <>
                      <DialogTitle>Delete Saved Waypoint</DialogTitle>
                      <DialogContent>
                        <Stack spacing={1.2} sx={{ pt: 0.5 }}>
                          <Typography variant="body1" sx={{ fontWeight: 600 }}>
                            {selection.waypoint.name}
                          </Typography>
                          <Typography variant="body2" color="text.secondary">
                            This removes the saved waypoint alias from the active map.
                          </Typography>
                          {waypointDeleteIssues.length ? (
                            <Alert severity="warning">
                              <Stack spacing={0.5}>
                                {waypointDeleteIssues.map((issue) => (
                                  <Typography key={issue} variant="body2">
                                    {issue}
                                  </Typography>
                                ))}
                              </Stack>
                            </Alert>
                          ) : null}
                        </Stack>
                      </DialogContent>
                      <DialogActions sx={{ px: 3, pb: 2.25 }}>
                        <Button
                          variant="outlined"
                          onClick={() => {
                            setModalState(undefined);
                            setSelection(undefined);
                          }}
                        >
                          Cancel
                        </Button>
                        <Button
                          color="warning"
                          variant="contained"
                          onClick={confirmWaypointDelete}
                          disabled={
                            waypointDeleteIssues.length > 0 ||
                            pendingCommand === config.waypointDeleteCommandName
                          }
                        >
                          Delete
                        </Button>
                      </DialogActions>
                    </>
                  ) : null}

                  {modalState?.kind === "point-goto" && selection?.kind === "point" ? (
                    <>
                      <DialogTitle>Send Robot To Point</DialogTitle>
                      <DialogContent>
                        <Stack spacing={1.2} sx={{ pt: 0.5 }}>
                          <Typography variant="body2" color="text.secondary">
                            {formatPose(selection.target)}
                          </Typography>

                          <Stack direction="row" spacing={0.75}>
                            {([
                              ["current", "Keep"],
                              ["path", "Face"],
                              ["custom", "Custom"]
                            ] as const).map(([mode, label]) => {
                              const selected = selection.headingMode === mode;
                              return (
                                <Button
                                  key={mode}
                                  fullWidth
                                  variant={selected ? "contained" : "outlined"}
                                  onClick={() => updatePointHeadingMode(mode)}
                                >
                                  {label}
                                </Button>
                              );
                            })}
                          </Stack>

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
                        </Stack>
                      </DialogContent>
                      <DialogActions sx={{ px: 3, pb: 2.25 }}>
                        <Button
                          variant="outlined"
                          onClick={() => {
                            setModalState(undefined);
                            setSelection(undefined);
                          }}
                        >
                          Cancel
                        </Button>
                        <Button
                          variant="contained"
                          onClick={confirmPointNavigation}
                          disabled={
                            !selectionReady || pendingCommand === config.gotoPoseCommandName
                          }
                        >
                          Send
                        </Button>
                      </DialogActions>
                    </>
                  ) : null}
                </Dialog>

                {snapshot.navState?.active ? (
                  <Alert severity="info">
                    <Stack spacing={0.5}>
                      <Typography variant="subtitle2">Active Navigation</Typography>
                      <Typography variant="body2">{activeNavLabel}</Typography>
                      <Typography variant="body2" color="text.secondary">
                        Status: {snapshot.navState.status_name || "Unknown"}
                      </Typography>
                      <Typography variant="body2" color="text.secondary">
                        Remaining route: {snapshot.navState.remaining_route_length_m.toFixed(2)} m
                      </Typography>
                    </Stack>
                  </Alert>
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

                {commandError ? <Alert severity="error">{commandError}</Alert> : null}
                {commandNotice ? <Alert severity="info">{commandNotice}</Alert> : null}
              </Stack>
            </Box>
          </Box>
        )}
      </Box>
    </ThemeProvider>
  );
}
