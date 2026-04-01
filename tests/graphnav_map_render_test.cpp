#include <cmath>
#include <iostream>

#include "formant_spot_adapter/graphnav_map_render.hpp"

namespace {

bool nearly_equal(double lhs, double rhs, double tol) {
  return std::abs(lhs - rhs) <= tol;
}

int require(bool condition, const char* message) {
  if (condition) return 0;
  std::cerr << "graphnav_map_render_test failed: " << message << std::endl;
  return 1;
}

}  // namespace

int main() {
  fsa::GraphNavMapGeometry map;
  map.width = 160;
  map.height = 80;
  map.resolution_m = 0.25;
  map.seed_tform_grid.x = 12.5;
  map.seed_tform_grid.y = -8.0;
  map.seed_tform_grid.yaw_rad = 0.0;

  fsa::GraphNavMapImageLayout layout;
  if (int rc = require(fsa::BuildGraphNavMapImageLayout(map, 1280, 720, 12, 12, 12, &layout),
                       "failed building identity layout")) {
    return rc;
  }

  const double seed_x = 20.75;
  const double seed_y = -1.25;
  double pixel_x = 0.0;
  double pixel_y = 0.0;
  if (int rc = require(fsa::GraphNavMapImageSeedToPixel(layout, seed_x, seed_y, &pixel_x, &pixel_y),
                       "failed converting seed to pixel")) {
    return rc;
  }
  if (int rc = require(fsa::GraphNavMapImageContainsPixel(layout, pixel_x, pixel_y),
                       "seed-to-pixel result not inside draw rect")) {
    return rc;
  }

  double roundtrip_seed_x = 0.0;
  double roundtrip_seed_y = 0.0;
  if (int rc = require(fsa::GraphNavMapImagePixelToSeed(
                           layout, pixel_x, pixel_y, &roundtrip_seed_x, &roundtrip_seed_y),
                       "failed converting pixel back to seed")) {
    return rc;
  }
  const double tol_m = map.resolution_m / std::max(1.0, layout.scale);
  if (int rc = require(nearly_equal(roundtrip_seed_x, seed_x, tol_m) &&
                           nearly_equal(roundtrip_seed_y, seed_y, tol_m),
                       "identity roundtrip seed mismatch")) {
    return rc;
  }

  fsa::GraphNavMapGeometry rotated_map = map;
  rotated_map.seed_tform_grid.x = -3.0;
  rotated_map.seed_tform_grid.y = 7.5;
  rotated_map.seed_tform_grid.yaw_rad = 3.14159265358979323846 / 2.0;
  if (int rc = require(
          fsa::BuildGraphNavMapImageLayout(rotated_map, 1280, 720, 12, 12, 12, &layout),
          "failed building rotated layout")) {
    return rc;
  }

  const double rotated_seed_x = -5.0;
  const double rotated_seed_y = 13.0;
  if (int rc = require(fsa::GraphNavMapImageSeedToPixel(
                           layout, rotated_seed_x, rotated_seed_y, &pixel_x, &pixel_y),
                       "failed converting rotated seed to pixel")) {
    return rc;
  }
  if (int rc = require(fsa::GraphNavMapImageContainsPixel(layout, pixel_x, pixel_y),
                       "rotated seed-to-pixel result not inside draw rect")) {
    return rc;
  }
  if (int rc = require(fsa::GraphNavMapImagePixelToSeed(
                           layout, pixel_x, pixel_y, &roundtrip_seed_x, &roundtrip_seed_y),
                       "failed converting rotated pixel back to seed")) {
    return rc;
  }
  if (int rc = require(nearly_equal(roundtrip_seed_x, rotated_seed_x, tol_m) &&
                           nearly_equal(roundtrip_seed_y, rotated_seed_y, tol_m),
                       "rotated roundtrip seed mismatch")) {
    return rc;
  }

  if (int rc = require(!fsa::GraphNavMapImagePixelToSeed(layout, 0.0, 0.0,
                                                         &roundtrip_seed_x, &roundtrip_seed_y),
                       "outside pixel unexpectedly converted to seed")) {
    return rc;
  }

  return 0;
}
