import { useEffect, useRef, useState } from "react";
import {
  Alert,
  Box,
  Button,
  Card,
  CardContent,
  Chip,
  CircularProgress,
  CssBaseline,
  Divider,
  Link,
  Stack,
  TextField,
  ThemeProvider,
  Typography,
  createTheme
} from "@mui/material";
import { App as FormantApp } from "@formant/data-sdk";
import {
  authenticateAndGetDevice,
  fetchStreamSnapshot,
  formatGotoPosePayload,
  getInitialModuleConfig,
  parseModuleConfig
} from "./formant";
import { getCanvasSize, pixelToSeed, seedToPixel } from "./mapMath";
import {
  ModuleConfig,
  NavState,
  StreamSnapshot,
  TargetPose
} from "./types";

const theme = createTheme({
  palette: {
    mode: "dark",
    primary: { main: "#23b0ff" },
    secondary: { main: "#97ffcd" },
    background: {
      default: "#08131d",
      paper: "#0d1b27"
    }
  },
  shape: {
    borderRadius: 16
  },
  typography: {
    fontFamily: `"IBM Plex Sans", "Inter", sans-serif`,
    h4: {
      fontWeight: 600
    },
    overline: {
      letterSpacing: "0.12em"
    }
  }
});

const defaultConfig: ModuleConfig = {
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

function formatClock(timestamp?: number): string {
  if (!timestamp) return "No data yet";
  return new Date(timestamp).toLocaleTimeString();
}

function formatPose(target?: TargetPose): string {
  if (!target) return "No target selected";
  return `x=${target.x.toFixed(2)} m, y=${target.y.toFixed(2)} m, yaw=${target.yawDeg.toFixed(1)} deg`;
}

function getMapUuid(snapshot: StreamSnapshot): string {
  return (
    snapshot.mapImageMetadata?.map_uuid ||
    snapshot.navState?.map_uuid ||
    snapshot.graphnavMetadata?.map_uuid ||
    ""
  );
}

function getMapLabel(snapshot: StreamSnapshot): string {
  return (
    snapshot.mapImageMetadata?.map_id ||
    snapshot.navState?.map_id ||
    snapshot.graphnavMetadata?.map_id ||
    "Unknown map"
  );
}

function getDefaultYawDeg(config: ModuleConfig, navState?: NavState): number {
  if (config.defaultYawMode === "current" && typeof navState?.current_seed_yaw_rad === "number") {
    return (navState.current_seed_yaw_rad * 180) / Math.PI;
  }
  return config.defaultYawDeg;
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
  config: ModuleConfig;
  selectedTarget?: TargetPose;
  onSelectTarget: (target: TargetPose) => void;
}

function MapView({ snapshot, config, selectedTarget, onSelectTarget }: MapViewProps) {
  const overlayRef = useRef<SVGSVGElement | null>(null);
  const canvas = getCanvasSize(snapshot.mapImageMetadata);
  const waypointsById = new Map<string, { x: number; y: number; dock: boolean; label: string }>();
  if (snapshot.graphnavMetadata) {
    for (const waypoint of snapshot.graphnavMetadata.waypoints) {
      const point = seedToPixel(
        snapshot.mapImageMetadata,
        waypoint.seed_tform_waypoint.x,
        waypoint.seed_tform_waypoint.y
      );
      if (!point) continue;
      waypointsById.set(waypoint.id, {
        x: point.x,
        y: point.y,
        dock: Boolean(waypoint.is_dock),
        label: waypoint.display_name || waypoint.label || waypoint.id
      });
    }
  }

  const handleMapClick = (event: React.MouseEvent<SVGSVGElement>) => {
    const svg = overlayRef.current;
    if (!svg || !snapshot.mapImageMetadata) return;
    const rect = svg.getBoundingClientRect();
    const scaleX = canvas.x / rect.width;
    const scaleY = canvas.y / rect.height;
    const pixel = {
      x: (event.clientX - rect.left) * scaleX,
      y: (event.clientY - rect.top) * scaleY
    };
    const seed = pixelToSeed(snapshot.mapImageMetadata, pixel.x, pixel.y);
    if (!seed) return;
    onSelectTarget({
      x: seed.x,
      y: seed.y,
      yawDeg: getDefaultYawDeg(config, snapshot.navState)
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
  const targetPixel =
    snapshot.navState?.has_seed_goal &&
    typeof snapshot.navState.target_seed_x === "number" &&
    typeof snapshot.navState.target_seed_y === "number"
      ? seedToPixel(
          snapshot.mapImageMetadata,
          snapshot.navState.target_seed_x,
          snapshot.navState.target_seed_y
        )
      : null;
  const selectedPixel = selectedTarget
    ? seedToPixel(snapshot.mapImageMetadata, selectedTarget.x, selectedTarget.y)
    : null;

  return (
    <Box
      sx={{
        position: "relative",
        aspectRatio: `${canvas.x} / ${canvas.y}`,
        width: "100%",
        overflow: "hidden",
        borderRadius: 3,
        border: "1px solid rgba(151, 255, 205, 0.15)",
        background:
          "linear-gradient(180deg, rgba(35,176,255,0.08), rgba(151,255,205,0.03))"
      }}
    >
      {snapshot.mapImageUrl ? (
        <img
          alt="Spot GraphNav map"
          src={snapshot.mapImageUrl}
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
        style={{ position: "absolute", inset: 0, width: "100%", height: "100%", cursor: "crosshair" }}
      >
        {snapshot.mapImageMetadata?.draw_rect ? (
          <rect
            x={snapshot.mapImageMetadata.draw_rect.x}
            y={snapshot.mapImageMetadata.draw_rect.y}
            width={snapshot.mapImageMetadata.draw_rect.width}
            height={snapshot.mapImageMetadata.draw_rect.height}
            fill="none"
            stroke="rgba(255,255,255,0.2)"
            strokeDasharray="6 6"
          />
        ) : null}

        {snapshot.graphnavMetadata?.edges.map((edge, index) => {
          const from = waypointsById.get(edge.from_waypoint_id);
          const to = waypointsById.get(edge.to_waypoint_id);
          if (!from || !to) return null;
          return (
            <line
              key={`${edge.from_waypoint_id}-${edge.to_waypoint_id}-${index}`}
              x1={from.x}
              y1={from.y}
              x2={to.x}
              y2={to.y}
              stroke="rgba(35,176,255,0.55)"
              strokeWidth="2"
            />
          );
        })}

        {Array.from(waypointsById.entries()).map(([id, waypoint]) => (
          <g key={id}>
            <circle
              cx={waypoint.x}
              cy={waypoint.y}
              r={waypoint.dock ? 7 : 5}
              fill={waypoint.dock ? "#ffb000" : "#23b0ff"}
              stroke="white"
              strokeWidth="1.5"
            />
            {config.showWaypointLabels ? (
              <text
                x={waypoint.x + 10}
                y={waypoint.y - 8}
                fill="white"
                fontSize="14"
                fontFamily="IBM Plex Sans, sans-serif"
                stroke="rgba(8,19,29,0.9)"
                strokeWidth="3"
                paintOrder="stroke"
              >
                {waypoint.label}
              </text>
            ) : null}
          </g>
        ))}

        {robotPixel && typeof snapshot.navState?.current_seed_yaw_rad === "number" ? (
          <PoseMarker
            x={robotPixel.x}
            y={robotPixel.y}
            yawRad={snapshot.navState.current_seed_yaw_rad}
            fill="#97ffcd"
            outline="#03131c"
          />
        ) : null}

        {targetPixel && typeof snapshot.navState?.target_seed_yaw_rad === "number" ? (
          <PoseMarker
            x={targetPixel.x}
            y={targetPixel.y}
            yawRad={snapshot.navState.target_seed_yaw_rad}
            fill="#ff8c42"
            outline="#2a1205"
            size={12}
          />
        ) : null}

        {selectedPixel ? (
          <g>
            <circle cx={selectedPixel.x} cy={selectedPixel.y} r="10" fill="none" stroke="#ffffff" strokeWidth="2" />
            <line x1={selectedPixel.x - 12} y1={selectedPixel.y} x2={selectedPixel.x + 12} y2={selectedPixel.y} stroke="#ffffff" strokeWidth="2" />
            <line x1={selectedPixel.x} y1={selectedPixel.y - 12} x2={selectedPixel.x} y2={selectedPixel.y + 12} stroke="#ffffff" strokeWidth="2" />
          </g>
        ) : null}
      </svg>
    </Box>
  );
}

function TelemetryTimestamp({
  label,
  time
}: {
  label: string;
  time?: number;
}) {
  return (
    <Stack direction="row" justifyContent="space-between">
      <Typography variant="body2" color="text.secondary">
        {label}
      </Typography>
      <Typography variant="body2">{formatClock(time)}</Typography>
    </Stack>
  );
}

export default function App() {
  const [device, setDevice] = useState<{ id: string; name: string }>();
  const [config, setConfig] = useState<ModuleConfig>(defaultConfig);
  const [snapshot, setSnapshot] = useState<StreamSnapshot>({});
  const [selectedTarget, setSelectedTarget] = useState<TargetPose>();
  const [availableCommands, setAvailableCommands] = useState<string[]>([]);
  const [loading, setLoading] = useState(true);
  const [refreshing, setRefreshing] = useState(false);
  const [sending, setSending] = useState(false);
  const [error, setError] = useState<string>();
  const [sendError, setSendError] = useState<string>();
  const [statusMessage, setStatusMessage] = useState<string>();

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
    let intervalId: number | undefined;

    const refresh = async () => {
      try {
        if (!cancelled) {
          setRefreshing(true);
          setError(undefined);
        }
        const nextSnapshot = await fetchStreamSnapshot(device.id, config);
        if (cancelled) return;
        setSnapshot(nextSnapshot);
      } catch (refreshError) {
        if (cancelled) return;
        setError(
          refreshError instanceof Error ? refreshError.message : "Failed loading GraphNav streams."
        );
      } finally {
        if (!cancelled) setRefreshing(false);
      }
    };

    void refresh();
    intervalId = window.setInterval(() => {
      void refresh();
    }, config.pollIntervalMs);

    return () => {
      cancelled = true;
      if (intervalId !== undefined) {
        window.clearInterval(intervalId);
      }
    };
  }, [device, config]);

  const mapUuid = getMapUuid(snapshot);
  const commandConfigured = availableCommands.includes(config.gotoPoseCommandName);
  const commandPayload =
    selectedTarget && mapUuid
      ? formatGotoPosePayload(mapUuid, selectedTarget.x, selectedTarget.y, selectedTarget.yawDeg)
      : "";

  const handleSend = async () => {
    if (!device || !selectedTarget || !mapUuid) return;
    setSending(true);
    setSendError(undefined);
    setStatusMessage(undefined);
    try {
      const currentDevice = await authenticateAndGetDevice();
      await currentDevice.sendCommand(config.gotoPoseCommandName, commandPayload);
      setStatusMessage(`Sent ${config.gotoPoseCommandName} to ${device.name}`);
      FormantApp.showMessage?.(`Sent ${config.gotoPoseCommandName}`);
    } catch (commandError) {
      setSendError(
        commandError instanceof Error ? commandError.message : "Failed sending goto pose command."
      );
    } finally {
      setSending(false);
    }
  };

  const handleYawChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    if (!selectedTarget) return;
    const next = Number(event.target.value);
    if (!Number.isFinite(next)) return;
    setSelectedTarget({ ...selectedTarget, yawDeg: next });
  };

  const currentPoseLabel =
    snapshot.navState?.has_current_seed_pose &&
    typeof snapshot.navState.current_seed_x === "number" &&
    typeof snapshot.navState.current_seed_y === "number"
      ? `x=${snapshot.navState.current_seed_x.toFixed(2)} m, y=${snapshot.navState.current_seed_y.toFixed(
          2
        )} m, yaw=${(((snapshot.navState.current_seed_yaw_rad ?? 0) * 180) / Math.PI).toFixed(1)} deg`
      : "No live pose";

  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <Box
        sx={{
          minHeight: "100vh",
          background:
            "radial-gradient(circle at top left, rgba(35,176,255,0.18), transparent 36%), radial-gradient(circle at bottom right, rgba(151,255,205,0.12), transparent 30%), #08131d",
          color: "text.primary",
          p: { xs: 2, md: 3 }
        }}
      >
        <Stack spacing={3}>
          <Stack
            direction={{ xs: "column", lg: "row" }}
            spacing={2}
            justifyContent="space-between"
            alignItems={{ xs: "flex-start", lg: "center" }}
          >
            <Box>
              <Typography variant="overline" color="primary.light">
                Spot Map Navigation
              </Typography>
              <Typography variant="h4">
                {device ? device.name : "Connecting to Formant"}
              </Typography>
              <Typography variant="body1" color="text.secondary">
                Hosted custom module for GraphNav map inspection and click-to-go targets.
              </Typography>
            </Box>
            <Stack direction="row" spacing={1} flexWrap="wrap" useFlexGap>
              <Chip label={device ? `Device ${device.id}` : "No device"} />
              <Chip label={`Map ${getMapLabel(snapshot)}`} color="primary" variant="outlined" />
              <Chip
                label={snapshot.navState?.localized ? "Localized" : "Not localized"}
                color={snapshot.navState?.localized ? "secondary" : "default"}
                variant={snapshot.navState?.localized ? "filled" : "outlined"}
              />
              <Chip
                label={snapshot.navState?.active ? snapshot.navState.status_name : "Idle"}
                color={snapshot.navState?.active ? "warning" : "default"}
                variant="outlined"
              />
            </Stack>
          </Stack>

          {loading ? (
            <Card>
              <CardContent>
                <Stack direction="row" spacing={2} alignItems="center">
                  <CircularProgress size={24} color="primary" />
                  <Typography>Authenticating with Formant and loading module context.</Typography>
                </Stack>
              </CardContent>
            </Card>
          ) : null}

          {error ? <Alert severity="error">{error}</Alert> : null}
          {sendError ? <Alert severity="error">{sendError}</Alert> : null}
          {statusMessage ? <Alert severity="success">{statusMessage}</Alert> : null}
          {!commandConfigured && device ? (
            <Alert severity="warning">
              The configured goto command <code>{config.gotoPoseCommandName}</code> is not in the device’s available command set.
            </Alert>
          ) : null}

          <Box
            sx={{
              display: "grid",
              gridTemplateColumns: { xs: "1fr", xl: "360px 1fr" },
              gap: 3
            }}
          >
            <Stack spacing={3}>
              <Card>
                <CardContent>
                  <Stack spacing={1.5}>
                    <Typography variant="h6">Navigation State</Typography>
                    <TelemetryTimestamp label="Map image" time={snapshot.mapImageTime} />
                    <TelemetryTimestamp label="Image metadata" time={snapshot.mapImageMetadataTime} />
                    <TelemetryTimestamp label="GraphNav metadata" time={snapshot.graphnavMetadataTime} />
                    <TelemetryTimestamp label="Nav state" time={snapshot.navStateTime} />
                    <Divider />
                    <Stack direction="row" justifyContent="space-between">
                      <Typography variant="body2" color="text.secondary">
                        Pose
                      </Typography>
                      <Typography variant="body2">{currentPoseLabel}</Typography>
                    </Stack>
                    <Stack direction="row" justifyContent="space-between">
                      <Typography variant="body2" color="text.secondary">
                        Status
                      </Typography>
                      <Typography variant="body2">
                        {snapshot.navState?.status_name || "Unknown"}
                      </Typography>
                    </Stack>
                    <Stack direction="row" justifyContent="space-between">
                      <Typography variant="body2" color="text.secondary">
                        Remaining route
                      </Typography>
                      <Typography variant="body2">
                        {snapshot.navState?.remaining_route_length_m?.toFixed(2) || "0.00"} m
                      </Typography>
                    </Stack>
                  </Stack>
                </CardContent>
              </Card>

              <Card>
                <CardContent>
                  <Stack spacing={2}>
                    <Typography variant="h6">Selected Target</Typography>
                    <Typography variant="body2" color="text.secondary">
                      Click inside the rendered map bounds to choose a target. The module uses the adapter’s image metadata stream to convert pixels back into seed-frame coordinates.
                    </Typography>
                    <Typography variant="body2">{formatPose(selectedTarget)}</Typography>
                    <TextField
                      label="Target heading (deg)"
                      type="number"
                      value={selectedTarget?.yawDeg ?? ""}
                      onChange={handleYawChange}
                      disabled={!selectedTarget}
                      inputProps={{ step: 1 }}
                    />
                    <TextField
                      label="Command payload preview"
                      value={commandPayload}
                      disabled
                      multiline
                      minRows={3}
                    />
                    <Stack direction="row" spacing={1}>
                      <Button
                        variant="contained"
                        onClick={() => void handleSend()}
                        disabled={!selectedTarget || !mapUuid || sending || !commandConfigured}
                      >
                        {sending ? "Sending..." : "Send goto pose"}
                      </Button>
                      <Button variant="outlined" onClick={() => setSelectedTarget(undefined)}>
                        Clear
                      </Button>
                    </Stack>
                  </Stack>
                </CardContent>
              </Card>

              <Card>
                <CardContent>
                  <Stack spacing={1.5}>
                    <Typography variant="h6">Module Wiring</Typography>
                    <Typography variant="body2" color="text.secondary">
                      The app is designed for GitHub Pages hosting and Formant iframe embedding. Authentication comes from Formant via query context and the toolkit’s auth helpers.
                    </Typography>
                    <Typography variant="body2">
                      Streams: <code>{config.mapImageStreamName}</code>, <code>{config.mapImageMetadataStreamName}</code>, <code>{config.graphnavMetadataStreamName}</code>, <code>{config.navStateStreamName}</code>
                    </Typography>
                    <Typography variant="body2">
                      Command: <code>{config.gotoPoseCommandName}</code>
                    </Typography>
                    <Typography variant="body2">
                      Refresh: {config.pollIntervalMs} ms
                    </Typography>
                    <Typography variant="body2">
                      Config schema:{" "}
                      <Link href="./config.schema.json" target="_blank" rel="noreferrer">
                        ./config.schema.json
                      </Link>
                    </Typography>
                  </Stack>
                </CardContent>
              </Card>
            </Stack>

            <Card sx={{ minHeight: 600 }}>
              <CardContent sx={{ height: "100%" }}>
                <Stack spacing={2} sx={{ height: "100%" }}>
                  <Stack
                    direction={{ xs: "column", md: "row" }}
                    justifyContent="space-between"
                    spacing={1}
                    alignItems={{ md: "center" }}
                  >
                    <Box>
                      <Typography variant="h6">GraphNav Map</Typography>
                      <Typography variant="body2" color="text.secondary">
                        Waypoints and edges come from <code>{config.graphnavMetadataStreamName}</code>; live pose and active target come from <code>{config.navStateStreamName}</code>.
                      </Typography>
                    </Box>
                    <Stack direction="row" spacing={1}>
                      <Chip label={refreshing ? "Refreshing" : "Up to date"} color={refreshing ? "warning" : "secondary"} variant="outlined" />
                      <Chip label={snapshot.graphnavMetadata?.waypoints.length ? `${snapshot.graphnavMetadata.waypoints.length} waypoints` : "No waypoints"} />
                    </Stack>
                  </Stack>

                  <MapView
                    snapshot={snapshot}
                    config={config}
                    selectedTarget={selectedTarget}
                    onSelectTarget={setSelectedTarget}
                  />
                </Stack>
              </CardContent>
            </Card>
          </Box>
        </Stack>
      </Box>
    </ThemeProvider>
  );
}
