export interface ModuleConfig {
  mapImageStreamName: string;
  mapImageMetadataStreamName: string;
  overlayStreamName: string;
  navStateStreamName: string;
  mapsStreamName: string;
  currentMapStreamName: string;
  defaultMapStreamName: string;
  connectionStateStreamName: string;
  dockingStateStreamName: string;
  motorPowerStateStreamName: string;
  behaviorStateStreamName: string;
  batteryStreamName: string;
  mapLoadCommandName: string;
  mapSetDefaultCommandName: string;
  waypointGotoCommandName: string;
  gotoPoseCommandName: string;
  cancelNavCommandName: string;
  returnAndDockCommandName: string;
  undockCommandName: string;
  showWaypointLabels: boolean;
  defaultYawMode: "current" | "fixed";
  defaultYawDeg: number;
}

export interface Pose3DJson {
  x: number;
  y: number;
  z: number;
  qw: number;
  qx: number;
  qy: number;
  qz: number;
}

export interface DrawRectJson {
  x: number;
  y: number;
  width: number;
  height: number;
}

export interface CanvasSizeJson {
  width: number;
  height: number;
}

export interface GraphNavMapJson {
  width: number;
  height: number;
  resolution_m: number;
  seed_tform_grid: Pose3DJson;
}

export interface GraphNavMapImageMetadata {
  image_stream: string;
  metadata_stream: string;
  has_map: boolean;
  status_title: string;
  status_detail: string;
  map_id: string;
  map_uuid: string;
  localized: boolean;
  current_waypoint_id: string;
  has_live_pose_overlay: boolean;
  current_seed_x?: number;
  current_seed_y?: number;
  current_seed_z?: number;
  current_seed_yaw_rad?: number;
  canvas?: CanvasSizeJson;
  draw_rect?: DrawRectJson;
  render_scale?: number;
  map?: GraphNavMapJson;
}

export interface GraphNavOverlayWaypoint {
  id: string;
  name: string;
  label: string;
  x: number;
  y: number;
  is_dock: boolean;
}

export interface GraphNavOverlay {
  map_id: string;
  map_uuid: string;
  current_waypoint_id: string;
  dock_waypoint_id: string;
  waypoints: GraphNavOverlayWaypoint[];
  waypoint_count?: number;
}

export interface NavState {
  connected: boolean;
  active: boolean;
  command_id: number;
  status: number;
  status_name: string;
  remaining_route_length_m: number;
  auto_recovered: boolean;
  phase: string;
  terminal_result: string;
  terminal_reason: string;
  last_completed_command_id: number;
  updated_at_ms: number;
  mode: string;
  target_waypoint_id: string;
  target_name: string;
  map_id: string;
  map_uuid: string;
  localized: boolean;
  current_waypoint_id: string;
  has_current_seed_pose: boolean;
  current_seed_x?: number;
  current_seed_y?: number;
  current_seed_z?: number;
  current_seed_yaw_rad?: number;
  has_seed_goal: boolean;
  has_waypoint_goal?: boolean;
  target_seed_x?: number;
  target_seed_y?: number;
  target_seed_yaw_rad?: number;
  target_waypoint_goal_x?: number;
  target_waypoint_goal_y?: number;
  target_waypoint_goal_yaw_rad?: number;
}

export interface TargetPose {
  x: number;
  y: number;
  yawDeg: number;
}

export interface StreamSnapshot {
  mapImageCanvas?: HTMLCanvasElement;
  mapImageFrameVersion?: number;
  mapImageTime?: number;
  mapImageMetadata?: GraphNavMapImageMetadata;
  mapImageMetadataTime?: number;
  overlay?: GraphNavOverlay;
  overlayTime?: number;
  navState?: NavState;
  navStateTime?: number;
  maps?: string[];
  mapsTime?: number;
  currentMapId?: string;
  currentMapTime?: number;
  defaultMapId?: string;
  defaultMapTime?: number;
  connectionState?: string;
  connectionStateTime?: number;
  dockingState?: string;
  dockingStateTime?: number;
  motorPowerState?: string;
  motorPowerStateTime?: number;
  behaviorState?: string;
  behaviorStateTime?: number;
  batteryPct?: number;
  batteryTime?: number;
  warnings?: string[];
}
