import { useEffect, useRef, useState } from "react";
import {
  Alert,
  Box,
  Button,
  Chip,
  CircularProgress,
  CssBaseline,
  IconButton,
  Stack,
  TextField,
  ThemeProvider,
  Typography,
  createTheme
} from "@mui/material";
import { App as FormantApp } from "@formant/data-sdk";
import {
  authenticateAndGetDevice,
  formatGotoPosePayload,
  getInitialModuleConfig,
  parseModuleConfig,
  subscribeRealtimeSnapshot
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
  graphnavMetadataStreamName: "",
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
  const realtimeCanvasRef = useRef<HTMLCanvasElement | null>(null);
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

  useEffect(() => {
    if (!snapshot.mapImageCanvas || !realtimeCanvasRef.current) return;
    const target = realtimeCanvasRef.current;
    target.width = snapshot.mapImageCanvas.width;
    target.height = snapshot.mapImageCanvas.height;
    const context = target.getContext("2d");
    context?.drawImage(snapshot.mapImageCanvas, 0, 0);
  }, [snapshot.mapImageCanvas, snapshot.mapImageFrameVersion]);

  return (
    <Box
      sx={{
        position: "relative",
        width: "100%",
        height: "100%",
        overflow: "hidden",
        background:
          "linear-gradient(180deg, rgba(35,176,255,0.08), rgba(151,255,205,0.03))"
      }}
    >
      {snapshot.mapImageCanvas ? (
        <canvas
          ref={realtimeCanvasRef}
          style={{ width: "100%", height: "100%", objectFit: "contain", display: "block" }}
        />
      ) : snapshot.mapImageUrl ? (
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

const floatingPanelSx = {
  backdropFilter: "blur(18px)",
  backgroundColor: "rgba(5, 16, 24, 0.78)",
  border: "1px solid rgba(255,255,255,0.08)",
  borderRadius: 3,
  boxShadow: "0 18px 48px rgba(0,0,0,0.28)"
} as const;

export default function App() {
  const [device, setDevice] = useState<{ id: string; name: string }>();
  const [config, setConfig] = useState<ModuleConfig>(defaultConfig);
  const [snapshot, setSnapshot] = useState<StreamSnapshot>({});
  const [selectedTarget, setSelectedTarget] = useState<TargetPose>();
  const [availableCommands, setAvailableCommands] = useState<string[]>([]);
  const [loading, setLoading] = useState(true);
  const [refreshing, setRefreshing] = useState(false);
  const [sending, setSending] = useState(false);
  const [infoOpen, setInfoOpen] = useState(false);
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
    setRefreshing(false);
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
          width: "100vw",
          height: "100vh",
          backgroundColor: "#08131d",
          color: "text.primary",
          overflow: "hidden"
        }}
      >
        {loading ? (
          <Stack
            alignItems="center"
            justifyContent="center"
            spacing={2}
            sx={{ width: "100%", height: "100%" }}
          >
            <CircularProgress size={28} color="primary" />
            <Typography variant="body2" color="text.secondary">
              Connecting
            </Typography>
          </Stack>
        ) : (
          <Box sx={{ position: "relative", width: "100%", height: "100%" }}>
            <MapView
              snapshot={snapshot}
              config={config}
              selectedTarget={selectedTarget}
              onSelectTarget={setSelectedTarget}
            />

            <Stack
              direction="row"
              spacing={1}
              flexWrap="wrap"
              useFlexGap
              sx={{ position: "absolute", top: 12, left: 12, maxWidth: "calc(100% - 80px)" }}
            >
              <Chip
                label={getMapLabel(snapshot)}
                color="primary"
                sx={{ backgroundColor: "rgba(35,176,255,0.18)" }}
              />
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
              <Chip
                label={refreshing ? "Refreshing" : "Live"}
                variant="outlined"
                sx={{ borderColor: "rgba(255,255,255,0.18)" }}
              />
            </Stack>

            <IconButton
              onClick={() => setInfoOpen((open) => !open)}
              sx={{
                position: "absolute",
                top: 12,
                right: 12,
                width: 42,
                height: 42,
                color: "white",
                ...floatingPanelSx
              }}
            >
              <Typography variant="button">i</Typography>
            </IconButton>

            {infoOpen ? (
              <Box
                sx={{
                  position: "absolute",
                  top: 64,
                  right: 12,
                  width: { xs: "min(320px, calc(100vw - 24px))", sm: 320 },
                  p: 1.75,
                  ...floatingPanelSx
                }}
              >
                <Stack spacing={1.25}>
                  <Typography variant="subtitle2">Map Details</Typography>
                  <Typography variant="body2" color="text.secondary">
                    {device?.name || "Current device"}
                  </Typography>
                  <Typography variant="body2">Pose: {currentPoseLabel}</Typography>
                  <Typography variant="body2">
                    Status: {snapshot.navState?.status_name || "Unknown"}
                  </Typography>
                  <Typography variant="body2">
                    Route: {snapshot.navState?.remaining_route_length_m?.toFixed(2) || "0.00"} m
                  </Typography>
                  <Typography variant="body2">
                    Map image: {formatClock(snapshot.mapImageTime)}
                  </Typography>
                  <Typography variant="body2">
                    Image metadata: {formatClock(snapshot.mapImageMetadataTime)}
                  </Typography>
                  <Typography variant="body2">
                    Nav state: {formatClock(snapshot.navStateTime)}
                  </Typography>
                  <Typography variant="body2">
                    Overlay stream: {config.graphnavMetadataStreamName || "Disabled"}
                  </Typography>
                  {snapshot.warnings?.map((warning) => (
                    <Alert key={warning} severity="warning" sx={{ py: 0 }}>
                      {warning}
                    </Alert>
                  ))}
                  {!commandConfigured ? (
                    <Alert severity="warning" sx={{ py: 0 }}>
                      Missing command <code>{config.gotoPoseCommandName}</code>
                    </Alert>
                  ) : null}
                </Stack>
              </Box>
            ) : null}

            {selectedTarget ? (
              <Box
                sx={{
                  position: "absolute",
                  left: 12,
                  right: 12,
                  bottom: 12,
                  p: 1.5,
                  ...floatingPanelSx
                }}
              >
                <Stack
                  direction={{ xs: "column", md: "row" }}
                  spacing={1.5}
                  alignItems={{ md: "center" }}
                  justifyContent="space-between"
                >
                  <Stack spacing={0.5} sx={{ minWidth: 0 }}>
                    <Typography variant="subtitle2">Selected Target</Typography>
                    <Typography variant="body2" color="text.secondary">
                      {formatPose(selectedTarget)}
                    </Typography>
                    <Typography
                      variant="caption"
                      color="text.secondary"
                      sx={{ wordBreak: "break-word" }}
                    >
                      {commandPayload || "No command payload"}
                    </Typography>
                  </Stack>

                  <Stack
                    direction={{ xs: "column", sm: "row" }}
                    spacing={1}
                    alignItems={{ sm: "center" }}
                  >
                    <TextField
                      label="Yaw"
                      type="number"
                      size="small"
                      value={selectedTarget.yawDeg}
                      onChange={handleYawChange}
                      inputProps={{ step: 1 }}
                      sx={{
                        minWidth: 120,
                        "& .MuiOutlinedInput-root": {
                          backgroundColor: "rgba(255,255,255,0.04)"
                        }
                      }}
                    />
                    <Button
                      variant="contained"
                      onClick={() => void handleSend()}
                      disabled={!mapUuid || sending || !commandConfigured}
                    >
                      {sending ? "Sending..." : "Go to target"}
                    </Button>
                    <Button variant="outlined" onClick={() => setSelectedTarget(undefined)}>
                      Clear
                    </Button>
                  </Stack>
                </Stack>
              </Box>
            ) : (
              <Box
                sx={{
                  position: "absolute",
                  left: "50%",
                  bottom: 12,
                  transform: "translateX(-50%)",
                  px: 1.5,
                  py: 0.75,
                  ...floatingPanelSx
                }}
              >
                <Typography variant="body2" color="text.secondary">
                  Click the map to place a target
                </Typography>
              </Box>
            )}

            {error ? (
              <Alert
                severity="error"
                sx={{ position: "absolute", left: 12, right: 12, top: 64 }}
              >
                {error}
              </Alert>
            ) : null}

            {sendError ? (
              <Alert
                severity="error"
                sx={{ position: "absolute", left: 12, right: 12, bottom: selectedTarget ? 116 : 64 }}
              >
                {sendError}
              </Alert>
            ) : null}

            {statusMessage ? (
              <Alert
                severity="success"
                sx={{ position: "absolute", left: 12, right: 12, bottom: selectedTarget ? 116 : 64 }}
              >
                {statusMessage}
              </Alert>
            ) : null}
          </Box>
        )}
      </Box>
    </ThemeProvider>
  );
}
