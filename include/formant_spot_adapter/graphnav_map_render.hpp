#pragma once

namespace fsa {

struct GraphNavMapPose2D {
  double x{0.0};
  double y{0.0};
  double yaw_rad{0.0};
};

struct GraphNavMapGeometry {
  int width{0};
  int height{0};
  double resolution_m{0.0};
  GraphNavMapPose2D seed_tform_grid;
};

struct GraphNavMapImageLayout {
  int canvas_width{0};
  int canvas_height{0};
  int pad_x{0};
  int pad_top{0};
  int pad_bottom{0};
  int draw_x{0};
  int draw_y{0};
  int draw_width{0};
  int draw_height{0};
  double scale{0.0};
  GraphNavMapGeometry map;
};

bool BuildGraphNavMapImageLayout(const GraphNavMapGeometry& map,
                                 int canvas_width,
                                 int canvas_height,
                                 int pad_x,
                                 int pad_top,
                                 int pad_bottom,
                                 GraphNavMapImageLayout* out_layout);

bool GraphNavMapImageContainsPixel(const GraphNavMapImageLayout& layout,
                                   double pixel_x,
                                   double pixel_y);

bool GraphNavMapImageLocalToPixel(const GraphNavMapImageLayout& layout,
                                  double local_x_m,
                                  double local_y_m,
                                  double* out_pixel_x,
                                  double* out_pixel_y);

bool GraphNavMapImagePixelToLocal(const GraphNavMapImageLayout& layout,
                                  double pixel_x,
                                  double pixel_y,
                                  double* out_local_x_m,
                                  double* out_local_y_m);

bool GraphNavMapImageSeedToPixel(const GraphNavMapImageLayout& layout,
                                 double seed_x,
                                 double seed_y,
                                 double* out_pixel_x,
                                 double* out_pixel_y);

bool GraphNavMapImagePixelToSeed(const GraphNavMapImageLayout& layout,
                                 double pixel_x,
                                 double pixel_y,
                                 double* out_seed_x,
                                 double* out_seed_y);

}  // namespace fsa
