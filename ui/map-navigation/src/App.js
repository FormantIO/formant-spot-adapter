import { jsx as _jsx, jsxs as _jsxs } from "react/jsx-runtime";
import { useEffect, useRef, useState } from "react";
import { Alert, Box, Button, Card, CardContent, Chip, CircularProgress, CssBaseline, Divider, Link, Stack, TextField, ThemeProvider, Typography, createTheme } from "@mui/material";
import { App as FormantApp } from "@formant/data-sdk";
import { authenticateAndGetDevice, fetchStreamSnapshot, formatGotoPosePayload, getInitialModuleConfig, parseModuleConfig } from "./formant";
import { getCanvasSize, pixelToSeed, seedToPixel } from "./mapMath";
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
const defaultConfig = {
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
function formatClock(timestamp) {
    if (!timestamp)
        return "No data yet";
    return new Date(timestamp).toLocaleTimeString();
}
function formatPose(target) {
    if (!target)
        return "No target selected";
    return `x=${target.x.toFixed(2)} m, y=${target.y.toFixed(2)} m, yaw=${target.yawDeg.toFixed(1)} deg`;
}
function getMapUuid(snapshot) {
    return (snapshot.mapImageMetadata?.map_uuid ||
        snapshot.navState?.map_uuid ||
        snapshot.graphnavMetadata?.map_uuid ||
        "");
}
function getMapLabel(snapshot) {
    return (snapshot.mapImageMetadata?.map_id ||
        snapshot.navState?.map_id ||
        snapshot.graphnavMetadata?.map_id ||
        "Unknown map");
}
function getDefaultYawDeg(config, navState) {
    if (config.defaultYawMode === "current" && typeof navState?.current_seed_yaw_rad === "number") {
        return (navState.current_seed_yaw_rad * 180) / Math.PI;
    }
    return config.defaultYawDeg;
}
function PoseMarker({ x, y, yawRad, fill, outline, size = 14 }) {
    const half = size / 2;
    const points = `${x},${y - size} ${x + half},${y + half} ${x},${y + (half / 2)} ${x - half},${y + half}`;
    return (_jsx("g", { transform: `rotate(${(yawRad * 180) / Math.PI} ${x} ${y})`, children: _jsx("polygon", { points: points, fill: fill, stroke: outline, strokeWidth: "2" }) }));
}
function MapView({ snapshot, config, selectedTarget, onSelectTarget }) {
    const overlayRef = useRef(null);
    const canvas = getCanvasSize(snapshot.mapImageMetadata);
    const waypointsById = new Map();
    if (snapshot.graphnavMetadata) {
        for (const waypoint of snapshot.graphnavMetadata.waypoints) {
            const point = seedToPixel(snapshot.mapImageMetadata, waypoint.seed_tform_waypoint.x, waypoint.seed_tform_waypoint.y);
            if (!point)
                continue;
            waypointsById.set(waypoint.id, {
                x: point.x,
                y: point.y,
                dock: Boolean(waypoint.is_dock),
                label: waypoint.display_name || waypoint.label || waypoint.id
            });
        }
    }
    const handleMapClick = (event) => {
        const svg = overlayRef.current;
        if (!svg || !snapshot.mapImageMetadata)
            return;
        const rect = svg.getBoundingClientRect();
        const scaleX = canvas.x / rect.width;
        const scaleY = canvas.y / rect.height;
        const pixel = {
            x: (event.clientX - rect.left) * scaleX,
            y: (event.clientY - rect.top) * scaleY
        };
        const seed = pixelToSeed(snapshot.mapImageMetadata, pixel.x, pixel.y);
        if (!seed)
            return;
        onSelectTarget({
            x: seed.x,
            y: seed.y,
            yawDeg: getDefaultYawDeg(config, snapshot.navState)
        });
    };
    const robotPixel = snapshot.navState?.has_current_seed_pose &&
        typeof snapshot.navState.current_seed_x === "number" &&
        typeof snapshot.navState.current_seed_y === "number"
        ? seedToPixel(snapshot.mapImageMetadata, snapshot.navState.current_seed_x, snapshot.navState.current_seed_y)
        : null;
    const targetPixel = snapshot.navState?.has_seed_goal &&
        typeof snapshot.navState.target_seed_x === "number" &&
        typeof snapshot.navState.target_seed_y === "number"
        ? seedToPixel(snapshot.mapImageMetadata, snapshot.navState.target_seed_x, snapshot.navState.target_seed_y)
        : null;
    const selectedPixel = selectedTarget
        ? seedToPixel(snapshot.mapImageMetadata, selectedTarget.x, selectedTarget.y)
        : null;
    return (_jsxs(Box, { sx: {
            position: "relative",
            aspectRatio: `${canvas.x} / ${canvas.y}`,
            width: "100%",
            overflow: "hidden",
            borderRadius: 3,
            border: "1px solid rgba(151, 255, 205, 0.15)",
            background: "linear-gradient(180deg, rgba(35,176,255,0.08), rgba(151,255,205,0.03))"
        }, children: [snapshot.mapImageUrl ? (_jsx("img", { alt: "Spot GraphNav map", src: snapshot.mapImageUrl, style: { width: "100%", height: "100%", objectFit: "contain", display: "block" } })) : (_jsxs(Stack, { alignItems: "center", justifyContent: "center", sx: { position: "absolute", inset: 0, textAlign: "center", p: 3 }, spacing: 1, children: [_jsx(Typography, { variant: "h6", children: snapshot.mapImageMetadata?.status_title || "Waiting for GraphNav map" }), _jsx(Typography, { variant: "body2", color: "text.secondary", children: snapshot.mapImageMetadata?.status_detail ||
                            "The adapter has not published the global map image yet." })] })), _jsxs("svg", { ref: overlayRef, viewBox: `0 0 ${canvas.x} ${canvas.y}`, onClick: handleMapClick, style: { position: "absolute", inset: 0, width: "100%", height: "100%", cursor: "crosshair" }, children: [snapshot.mapImageMetadata?.draw_rect ? (_jsx("rect", { x: snapshot.mapImageMetadata.draw_rect.x, y: snapshot.mapImageMetadata.draw_rect.y, width: snapshot.mapImageMetadata.draw_rect.width, height: snapshot.mapImageMetadata.draw_rect.height, fill: "none", stroke: "rgba(255,255,255,0.2)", strokeDasharray: "6 6" })) : null, snapshot.graphnavMetadata?.edges.map((edge, index) => {
                        const from = waypointsById.get(edge.from_waypoint_id);
                        const to = waypointsById.get(edge.to_waypoint_id);
                        if (!from || !to)
                            return null;
                        return (_jsx("line", { x1: from.x, y1: from.y, x2: to.x, y2: to.y, stroke: "rgba(35,176,255,0.55)", strokeWidth: "2" }, `${edge.from_waypoint_id}-${edge.to_waypoint_id}-${index}`));
                    }), Array.from(waypointsById.entries()).map(([id, waypoint]) => (_jsxs("g", { children: [_jsx("circle", { cx: waypoint.x, cy: waypoint.y, r: waypoint.dock ? 7 : 5, fill: waypoint.dock ? "#ffb000" : "#23b0ff", stroke: "white", strokeWidth: "1.5" }), config.showWaypointLabels ? (_jsx("text", { x: waypoint.x + 10, y: waypoint.y - 8, fill: "white", fontSize: "14", fontFamily: "IBM Plex Sans, sans-serif", stroke: "rgba(8,19,29,0.9)", strokeWidth: "3", paintOrder: "stroke", children: waypoint.label })) : null] }, id))), robotPixel && typeof snapshot.navState?.current_seed_yaw_rad === "number" ? (_jsx(PoseMarker, { x: robotPixel.x, y: robotPixel.y, yawRad: snapshot.navState.current_seed_yaw_rad, fill: "#97ffcd", outline: "#03131c" })) : null, targetPixel && typeof snapshot.navState?.target_seed_yaw_rad === "number" ? (_jsx(PoseMarker, { x: targetPixel.x, y: targetPixel.y, yawRad: snapshot.navState.target_seed_yaw_rad, fill: "#ff8c42", outline: "#2a1205", size: 12 })) : null, selectedPixel ? (_jsxs("g", { children: [_jsx("circle", { cx: selectedPixel.x, cy: selectedPixel.y, r: "10", fill: "none", stroke: "#ffffff", strokeWidth: "2" }), _jsx("line", { x1: selectedPixel.x - 12, y1: selectedPixel.y, x2: selectedPixel.x + 12, y2: selectedPixel.y, stroke: "#ffffff", strokeWidth: "2" }), _jsx("line", { x1: selectedPixel.x, y1: selectedPixel.y - 12, x2: selectedPixel.x, y2: selectedPixel.y + 12, stroke: "#ffffff", strokeWidth: "2" })] })) : null] })] }));
}
function TelemetryTimestamp({ label, time }) {
    return (_jsxs(Stack, { direction: "row", justifyContent: "space-between", children: [_jsx(Typography, { variant: "body2", color: "text.secondary", children: label }), _jsx(Typography, { variant: "body2", children: formatClock(time) })] }));
}
export default function App() {
    const [device, setDevice] = useState();
    const [config, setConfig] = useState(defaultConfig);
    const [snapshot, setSnapshot] = useState({});
    const [selectedTarget, setSelectedTarget] = useState();
    const [availableCommands, setAvailableCommands] = useState([]);
    const [loading, setLoading] = useState(true);
    const [refreshing, setRefreshing] = useState(false);
    const [sending, setSending] = useState(false);
    const [error, setError] = useState();
    const [sendError, setSendError] = useState();
    const [statusMessage, setStatusMessage] = useState();
    useEffect(() => {
        let cancelled = false;
        let configCleanup;
        const bootstrap = async () => {
            try {
                const currentDevice = await authenticateAndGetDevice();
                const initialConfig = await getInitialModuleConfig();
                const commands = await currentDevice.getAvailableCommands().catch(() => []);
                if (cancelled)
                    return;
                setDevice({ id: currentDevice.id, name: currentDevice.name });
                setConfig(initialConfig);
                setAvailableCommands(commands.map((command) => command.name));
                configCleanup = FormantApp.addModuleConfigurationListener((event) => {
                    setConfig(parseModuleConfig(event.configuration));
                });
            }
            catch (bootstrapError) {
                if (cancelled)
                    return;
                setError(bootstrapError instanceof Error ? bootstrapError.message : "Failed to initialize the module.");
            }
            finally {
                if (!cancelled)
                    setLoading(false);
            }
        };
        void bootstrap();
        return () => {
            cancelled = true;
            configCleanup?.();
        };
    }, []);
    useEffect(() => {
        if (!device)
            return undefined;
        let cancelled = false;
        let intervalId;
        const refresh = async () => {
            try {
                if (!cancelled) {
                    setRefreshing(true);
                    setError(undefined);
                }
                const nextSnapshot = await fetchStreamSnapshot(device.id, config);
                if (cancelled)
                    return;
                setSnapshot(nextSnapshot);
            }
            catch (refreshError) {
                if (cancelled)
                    return;
                setError(refreshError instanceof Error ? refreshError.message : "Failed loading GraphNav streams.");
            }
            finally {
                if (!cancelled)
                    setRefreshing(false);
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
    const commandPayload = selectedTarget && mapUuid
        ? formatGotoPosePayload(mapUuid, selectedTarget.x, selectedTarget.y, selectedTarget.yawDeg)
        : "";
    const handleSend = async () => {
        if (!device || !selectedTarget || !mapUuid)
            return;
        setSending(true);
        setSendError(undefined);
        setStatusMessage(undefined);
        try {
            const currentDevice = await authenticateAndGetDevice();
            await currentDevice.sendCommand(config.gotoPoseCommandName, commandPayload);
            setStatusMessage(`Sent ${config.gotoPoseCommandName} to ${device.name}`);
            FormantApp.showMessage?.(`Sent ${config.gotoPoseCommandName}`);
        }
        catch (commandError) {
            setSendError(commandError instanceof Error ? commandError.message : "Failed sending goto pose command.");
        }
        finally {
            setSending(false);
        }
    };
    const handleYawChange = (event) => {
        if (!selectedTarget)
            return;
        const next = Number(event.target.value);
        if (!Number.isFinite(next))
            return;
        setSelectedTarget({ ...selectedTarget, yawDeg: next });
    };
    const currentPoseLabel = snapshot.navState?.has_current_seed_pose &&
        typeof snapshot.navState.current_seed_x === "number" &&
        typeof snapshot.navState.current_seed_y === "number"
        ? `x=${snapshot.navState.current_seed_x.toFixed(2)} m, y=${snapshot.navState.current_seed_y.toFixed(2)} m, yaw=${(((snapshot.navState.current_seed_yaw_rad ?? 0) * 180) / Math.PI).toFixed(1)} deg`
        : "No live pose";
    return (_jsxs(ThemeProvider, { theme: theme, children: [_jsx(CssBaseline, {}), _jsx(Box, { sx: {
                    minHeight: "100vh",
                    background: "radial-gradient(circle at top left, rgba(35,176,255,0.18), transparent 36%), radial-gradient(circle at bottom right, rgba(151,255,205,0.12), transparent 30%), #08131d",
                    color: "text.primary",
                    p: { xs: 2, md: 3 }
                }, children: _jsxs(Stack, { spacing: 3, children: [_jsxs(Stack, { direction: { xs: "column", lg: "row" }, spacing: 2, justifyContent: "space-between", alignItems: { xs: "flex-start", lg: "center" }, children: [_jsxs(Box, { children: [_jsx(Typography, { variant: "overline", color: "primary.light", children: "Spot Map Navigation" }), _jsx(Typography, { variant: "h4", children: device ? device.name : "Connecting to Formant" }), _jsx(Typography, { variant: "body1", color: "text.secondary", children: "Hosted custom module for GraphNav map inspection and click-to-go targets." })] }), _jsxs(Stack, { direction: "row", spacing: 1, flexWrap: "wrap", useFlexGap: true, children: [_jsx(Chip, { label: device ? `Device ${device.id}` : "No device" }), _jsx(Chip, { label: `Map ${getMapLabel(snapshot)}`, color: "primary", variant: "outlined" }), _jsx(Chip, { label: snapshot.navState?.localized ? "Localized" : "Not localized", color: snapshot.navState?.localized ? "secondary" : "default", variant: snapshot.navState?.localized ? "filled" : "outlined" }), _jsx(Chip, { label: snapshot.navState?.active ? snapshot.navState.status_name : "Idle", color: snapshot.navState?.active ? "warning" : "default", variant: "outlined" })] })] }), loading ? (_jsx(Card, { children: _jsx(CardContent, { children: _jsxs(Stack, { direction: "row", spacing: 2, alignItems: "center", children: [_jsx(CircularProgress, { size: 24, color: "primary" }), _jsx(Typography, { children: "Authenticating with Formant and loading module context." })] }) }) })) : null, error ? _jsx(Alert, { severity: "error", children: error }) : null, sendError ? _jsx(Alert, { severity: "error", children: sendError }) : null, statusMessage ? _jsx(Alert, { severity: "success", children: statusMessage }) : null, !commandConfigured && device ? (_jsxs(Alert, { severity: "warning", children: ["The configured goto command ", _jsx("code", { children: config.gotoPoseCommandName }), " is not in the device\u2019s available command set."] })) : null, _jsxs(Box, { sx: {
                                display: "grid",
                                gridTemplateColumns: { xs: "1fr", xl: "360px 1fr" },
                                gap: 3
                            }, children: [_jsxs(Stack, { spacing: 3, children: [_jsx(Card, { children: _jsx(CardContent, { children: _jsxs(Stack, { spacing: 1.5, children: [_jsx(Typography, { variant: "h6", children: "Navigation State" }), _jsx(TelemetryTimestamp, { label: "Map image", time: snapshot.mapImageTime }), _jsx(TelemetryTimestamp, { label: "Image metadata", time: snapshot.mapImageMetadataTime }), _jsx(TelemetryTimestamp, { label: "GraphNav metadata", time: snapshot.graphnavMetadataTime }), _jsx(TelemetryTimestamp, { label: "Nav state", time: snapshot.navStateTime }), _jsx(Divider, {}), _jsxs(Stack, { direction: "row", justifyContent: "space-between", children: [_jsx(Typography, { variant: "body2", color: "text.secondary", children: "Pose" }), _jsx(Typography, { variant: "body2", children: currentPoseLabel })] }), _jsxs(Stack, { direction: "row", justifyContent: "space-between", children: [_jsx(Typography, { variant: "body2", color: "text.secondary", children: "Status" }), _jsx(Typography, { variant: "body2", children: snapshot.navState?.status_name || "Unknown" })] }), _jsxs(Stack, { direction: "row", justifyContent: "space-between", children: [_jsx(Typography, { variant: "body2", color: "text.secondary", children: "Remaining route" }), _jsxs(Typography, { variant: "body2", children: [snapshot.navState?.remaining_route_length_m?.toFixed(2) || "0.00", " m"] })] })] }) }) }), _jsx(Card, { children: _jsx(CardContent, { children: _jsxs(Stack, { spacing: 2, children: [_jsx(Typography, { variant: "h6", children: "Selected Target" }), _jsx(Typography, { variant: "body2", color: "text.secondary", children: "Click inside the rendered map bounds to choose a target. The module uses the adapter\u2019s image metadata stream to convert pixels back into seed-frame coordinates." }), _jsx(Typography, { variant: "body2", children: formatPose(selectedTarget) }), _jsx(TextField, { label: "Target heading (deg)", type: "number", value: selectedTarget?.yawDeg ?? "", onChange: handleYawChange, disabled: !selectedTarget, inputProps: { step: 1 } }), _jsx(TextField, { label: "Command payload preview", value: commandPayload, disabled: true, multiline: true, minRows: 3 }), _jsxs(Stack, { direction: "row", spacing: 1, children: [_jsx(Button, { variant: "contained", onClick: () => void handleSend(), disabled: !selectedTarget || !mapUuid || sending || !commandConfigured, children: sending ? "Sending..." : "Send goto pose" }), _jsx(Button, { variant: "outlined", onClick: () => setSelectedTarget(undefined), children: "Clear" })] })] }) }) }), _jsx(Card, { children: _jsx(CardContent, { children: _jsxs(Stack, { spacing: 1.5, children: [_jsx(Typography, { variant: "h6", children: "Module Wiring" }), _jsx(Typography, { variant: "body2", color: "text.secondary", children: "The app is designed for GitHub Pages hosting and Formant iframe embedding. Authentication comes from Formant via query context and the toolkit\u2019s auth helpers." }), _jsxs(Typography, { variant: "body2", children: ["Streams: ", _jsx("code", { children: config.mapImageStreamName }), ", ", _jsx("code", { children: config.mapImageMetadataStreamName }), ", ", _jsx("code", { children: config.graphnavMetadataStreamName }), ", ", _jsx("code", { children: config.navStateStreamName })] }), _jsxs(Typography, { variant: "body2", children: ["Command: ", _jsx("code", { children: config.gotoPoseCommandName })] }), _jsxs(Typography, { variant: "body2", children: ["Refresh: ", config.pollIntervalMs, " ms"] }), _jsxs(Typography, { variant: "body2", children: ["Config schema:", " ", _jsx(Link, { href: "./config.schema.json", target: "_blank", rel: "noreferrer", children: "./config.schema.json" })] })] }) }) })] }), _jsx(Card, { sx: { minHeight: 600 }, children: _jsx(CardContent, { sx: { height: "100%" }, children: _jsxs(Stack, { spacing: 2, sx: { height: "100%" }, children: [_jsxs(Stack, { direction: { xs: "column", md: "row" }, justifyContent: "space-between", spacing: 1, alignItems: { md: "center" }, children: [_jsxs(Box, { children: [_jsx(Typography, { variant: "h6", children: "GraphNav Map" }), _jsxs(Typography, { variant: "body2", color: "text.secondary", children: ["Waypoints and edges come from ", _jsx("code", { children: config.graphnavMetadataStreamName }), "; live pose and active target come from ", _jsx("code", { children: config.navStateStreamName }), "."] })] }), _jsxs(Stack, { direction: "row", spacing: 1, children: [_jsx(Chip, { label: refreshing ? "Refreshing" : "Up to date", color: refreshing ? "warning" : "secondary", variant: "outlined" }), _jsx(Chip, { label: snapshot.graphnavMetadata?.waypoints.length ? `${snapshot.graphnavMetadata.waypoints.length} waypoints` : "No waypoints" })] })] }), _jsx(MapView, { snapshot: snapshot, config: config, selectedTarget: selectedTarget, onSelectTarget: setSelectedTarget })] }) }) })] })] }) })] }));
}
