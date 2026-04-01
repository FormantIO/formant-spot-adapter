export interface ModuleConfig {
  mapImageStreamName: string;
  mapImageMetadataStreamName: string;
  graphnavMetadataStreamName: string;
  navStateStreamName: string;
  gotoPoseCommandName: string;
  pollIntervalMs: number;
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

export interface GraphNavWaypoint {
  id: string;
  label?: string;
  display_name?: string;
  is_dock?: boolean;
  seed_tform_waypoint: Pose3DJson;
}

export interface GraphNavEdge {
  from_waypoint_id: string;
  to_waypoint_id: string;
}

export interface GraphNavMetadata {
  map_id: string;
  map_uuid: string;
  waypoints: GraphNavWaypoint[];
  edges: GraphNavEdge[];
}

export interface NavState {
  connected: boolean;
  active: boolean;
  command_id: number;
  status: number;
  status_name: string;
  remaining_route_length_m: number;
  auto_recovered: boolean;
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
  target_seed_x?: number;
  target_seed_y?: number;
  target_seed_yaw_rad?: number;
}

export interface TargetPose {
  x: number;
  y: number;
  yawDeg: number;
}

export interface StreamSnapshot {
  mapImageUrl?: string;
  mapImageCanvas?: HTMLCanvasElement;
  mapImageFrameVersion?: number;
  mapImageTime?: number;
  mapImageMetadata?: GraphNavMapImageMetadata;
  mapImageMetadataTime?: number;
  graphnavMetadata?: GraphNavMetadata;
  graphnavMetadataTime?: number;
  navState?: NavState;
  navStateTime?: number;
  warnings?: string[];
}
