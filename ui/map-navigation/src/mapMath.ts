import {
  GraphNavMapImageMetadata,
  Pose3DJson
} from "./types";

export interface Point2D {
  x: number;
  y: number;
}

function yawFromQuaternion(pose: Pose3DJson): number {
  const sinyCosp = 2 * ((pose.qw * pose.qz) + (pose.qx * pose.qy));
  const cosyCosp = 1 - 2 * ((pose.qy * pose.qy) + (pose.qz * pose.qz));
  return Math.atan2(sinyCosp, cosyCosp);
}

function hasRenderableMap(meta?: GraphNavMapImageMetadata): meta is GraphNavMapImageMetadata &
  Required<Pick<GraphNavMapImageMetadata, "canvas" | "draw_rect" | "map" | "render_scale">> {
  return Boolean(
    meta &&
      meta.has_map &&
      meta.canvas &&
      meta.draw_rect &&
      meta.map &&
      typeof meta.render_scale === "number" &&
      meta.render_scale > 0
  );
}

export function getCanvasSize(meta?: GraphNavMapImageMetadata): Point2D {
  return {
    x: meta?.canvas?.width ?? 1280,
    y: meta?.canvas?.height ?? 720
  };
}

export function poseYawRad(pose: Pose3DJson): number {
  return yawFromQuaternion(pose);
}

export function seedToPixel(
  meta: GraphNavMapImageMetadata | undefined,
  seedX: number,
  seedY: number
): Point2D | null {
  if (!hasRenderableMap(meta)) return null;

  const gridYaw = poseYawRad(meta.map.seed_tform_grid);
  const dx = seedX - meta.map.seed_tform_grid.x;
  const dy = seedY - meta.map.seed_tform_grid.y;
  const c = Math.cos(gridYaw);
  const s = Math.sin(gridYaw);
  const localX = (c * dx) + (s * dy);
  const localY = (-s * dx) + (c * dy);
  const gridX = localX / meta.map.resolution_m;
  const gridY = localY / meta.map.resolution_m;

  return {
    x: meta.draw_rect.x + (gridX * meta.render_scale),
    y: meta.draw_rect.y + meta.draw_rect.height - (gridY * meta.render_scale)
  };
}

export function pixelToSeed(
  meta: GraphNavMapImageMetadata | undefined,
  pixelX: number,
  pixelY: number
): Point2D | null {
  if (!hasRenderableMap(meta)) return null;

  const inside =
    pixelX >= meta.draw_rect.x &&
    pixelX < meta.draw_rect.x + meta.draw_rect.width &&
    pixelY >= meta.draw_rect.y &&
    pixelY < meta.draw_rect.y + meta.draw_rect.height;
  if (!inside) return null;

  const gridX = (pixelX - meta.draw_rect.x) / meta.render_scale;
  const gridY =
    (meta.draw_rect.y + meta.draw_rect.height - pixelY) / meta.render_scale;
  const localX = gridX * meta.map.resolution_m;
  const localY = gridY * meta.map.resolution_m;
  const gridYaw = poseYawRad(meta.map.seed_tform_grid);
  const c = Math.cos(gridYaw);
  const s = Math.sin(gridYaw);

  return {
    x: meta.map.seed_tform_grid.x + (c * localX) - (s * localY),
    y: meta.map.seed_tform_grid.y + (s * localX) + (c * localY)
  };
}
