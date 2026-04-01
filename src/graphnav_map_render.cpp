#include "formant_spot_adapter/graphnav_map_render.hpp"

#include <algorithm>
#include <cmath>

namespace fsa {

namespace {

bool layout_has_map(const GraphNavMapImageLayout& layout) {
  return layout.map.width > 0 && layout.map.height > 0 && layout.map.resolution_m > 0.0 &&
         layout.scale > 0.0 && layout.draw_width > 0 && layout.draw_height > 0;
}

}  // namespace

bool BuildGraphNavMapImageLayout(const GraphNavMapGeometry& map,
                                 int canvas_width,
                                 int canvas_height,
                                 int pad_x,
                                 int pad_top,
                                 int pad_bottom,
                                 GraphNavMapImageLayout* out_layout) {
  if (!out_layout) return false;
  if (canvas_width <= 0 || canvas_height <= 0 || map.width <= 0 || map.height <= 0 ||
      map.resolution_m <= 0.0) {
    return false;
  }

  GraphNavMapImageLayout layout;
  layout.canvas_width = canvas_width;
  layout.canvas_height = canvas_height;
  layout.pad_x = std::max(0, pad_x);
  layout.pad_top = std::max(0, pad_top);
  layout.pad_bottom = std::max(0, pad_bottom);
  layout.map = map;

  const int available_width = std::max(1, canvas_width - (layout.pad_x * 2));
  const int available_height = std::max(1, canvas_height - layout.pad_top - layout.pad_bottom);
  layout.scale = std::min(static_cast<double>(available_width) / std::max(1, map.width),
                          static_cast<double>(available_height) / std::max(1, map.height));
  layout.draw_width = std::max(1, static_cast<int>(std::lround(map.width * layout.scale)));
  layout.draw_height = std::max(1, static_cast<int>(std::lround(map.height * layout.scale)));
  layout.draw_x = (canvas_width - layout.draw_width) / 2;
  layout.draw_y = layout.pad_top + ((available_height - layout.draw_height) / 2);

  *out_layout = layout;
  return true;
}

bool GraphNavMapImageContainsPixel(const GraphNavMapImageLayout& layout,
                                   double pixel_x,
                                   double pixel_y) {
  if (!layout_has_map(layout)) return false;
  return pixel_x >= static_cast<double>(layout.draw_x) &&
         pixel_x < static_cast<double>(layout.draw_x + layout.draw_width) &&
         pixel_y >= static_cast<double>(layout.draw_y) &&
         pixel_y < static_cast<double>(layout.draw_y + layout.draw_height);
}

bool GraphNavMapImageLocalToPixel(const GraphNavMapImageLayout& layout,
                                  double local_x_m,
                                  double local_y_m,
                                  double* out_pixel_x,
                                  double* out_pixel_y) {
  if (!out_pixel_x || !out_pixel_y || !layout_has_map(layout)) return false;
  const double grid_x = local_x_m / layout.map.resolution_m;
  const double grid_y = local_y_m / layout.map.resolution_m;
  *out_pixel_x = static_cast<double>(layout.draw_x) + (grid_x * layout.scale);
  *out_pixel_y =
      static_cast<double>(layout.draw_y + layout.draw_height) - (grid_y * layout.scale);
  return true;
}

bool GraphNavMapImagePixelToLocal(const GraphNavMapImageLayout& layout,
                                  double pixel_x,
                                  double pixel_y,
                                  double* out_local_x_m,
                                  double* out_local_y_m) {
  if (!out_local_x_m || !out_local_y_m || !layout_has_map(layout) ||
      !GraphNavMapImageContainsPixel(layout, pixel_x, pixel_y)) {
    return false;
  }
  const double grid_x = (pixel_x - static_cast<double>(layout.draw_x)) / layout.scale;
  const double grid_y =
      (static_cast<double>(layout.draw_y + layout.draw_height) - pixel_y) / layout.scale;
  *out_local_x_m = grid_x * layout.map.resolution_m;
  *out_local_y_m = grid_y * layout.map.resolution_m;
  return true;
}

bool GraphNavMapImageSeedToPixel(const GraphNavMapImageLayout& layout,
                                 double seed_x,
                                 double seed_y,
                                 double* out_pixel_x,
                                 double* out_pixel_y) {
  if (!out_pixel_x || !out_pixel_y || !layout_has_map(layout)) return false;
  const double dx = seed_x - layout.map.seed_tform_grid.x;
  const double dy = seed_y - layout.map.seed_tform_grid.y;
  const double c = std::cos(layout.map.seed_tform_grid.yaw_rad);
  const double s = std::sin(layout.map.seed_tform_grid.yaw_rad);
  const double local_x_m = (c * dx) + (s * dy);
  const double local_y_m = (-s * dx) + (c * dy);
  return GraphNavMapImageLocalToPixel(layout, local_x_m, local_y_m,
                                      out_pixel_x, out_pixel_y);
}

bool GraphNavMapImagePixelToSeed(const GraphNavMapImageLayout& layout,
                                 double pixel_x,
                                 double pixel_y,
                                 double* out_seed_x,
                                 double* out_seed_y) {
  if (!out_seed_x || !out_seed_y || !layout_has_map(layout)) return false;
  double local_x_m = 0.0;
  double local_y_m = 0.0;
  if (!GraphNavMapImagePixelToLocal(layout, pixel_x, pixel_y, &local_x_m, &local_y_m)) {
    return false;
  }
  const double c = std::cos(layout.map.seed_tform_grid.yaw_rad);
  const double s = std::sin(layout.map.seed_tform_grid.yaw_rad);
  *out_seed_x = layout.map.seed_tform_grid.x + (c * local_x_m) - (s * local_y_m);
  *out_seed_y = layout.map.seed_tform_grid.y + (s * local_x_m) + (c * local_y_m);
  return true;
}

}  // namespace fsa
