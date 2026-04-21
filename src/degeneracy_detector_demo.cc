// Copyright 2026 The DGMapping Authors.  Apache License 2.0.
//
// ---------------------------------------------------------------------------
//
// Standalone demonstration of the DegeneracyDetector module.  Synthesises
// three scenes that exercise different failure modes of 2D LiDAR SLAM and
// prints the resulting descriptors:
//
//     L-shaped room  : nominal,  expected  low  R_deg / G_deg
//     Long corridor  : geometric degeneracy along the corridor axis
//                      -> high G_deg, phi aligned with walls
//     Sparse arc     : range  degeneracy   (only ~25 % of returns)
//                      -> high R_deg, L_deg, F_deg
//
// Build (preferred): cmake -S . -B build && cmake --build build
// Run:               ./build/degeneracy_detector_demo

#include <Eigen/Core>

#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "dgmapping/degeneracy_detector.h"

namespace {

struct Point {
  Eigen::Vector3f position;
};

Point MakePoint(double x, double y) {
  Point p;
  p.position = Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y),
                               0.0f);
  return p;
}

// L-shaped room: outer 8x8 m square with a 4x4 m corner cut out.
std::vector<Point> MakeLShapedRoom(double step) {
  std::vector<Point> pts;
  for (double y = 0.0; y <= 8.0 + 1e-9; y += step) pts.push_back(MakePoint(0.0, y));
  for (double x = 0.0; x <= 8.0 + 1e-9; x += step) pts.push_back(MakePoint(x, 0.0));
  for (double y = 0.0; y <= 4.0 + 1e-9; y += step) pts.push_back(MakePoint(8.0, y));
  for (double x = 8.0; x >= 4.0 - 1e-9; x -= step) pts.push_back(MakePoint(x, 4.0));
  for (double y = 4.0; y <= 8.0 + 1e-9; y += step) pts.push_back(MakePoint(4.0, y));
  for (double x = 4.0; x >= 0.0 - 1e-9; x -= step) pts.push_back(MakePoint(x, 8.0));
  return pts;
}

// 30 m x 2 m straight corridor: two parallel walls => geometric degeneracy.
std::vector<Point> MakeCorridor(double step) {
  std::vector<Point> pts;
  for (double x = 0.0; x <= 30.0 + 1e-9; x += step) pts.push_back(MakePoint(x, 0.0));
  for (double x = 0.0; x <= 30.0 + 1e-9; x += step) pts.push_back(MakePoint(x, 2.0));
  return pts;
}

// Sparse 90 deg arc at 5 m -- few returns => range degeneracy.
std::vector<Point> MakeSparseArc(double angular_step_rad) {
  std::vector<Point> pts;
  const double radius = 5.0;
  for (double a = -M_PI_4; a <= M_PI_4 + 1e-9; a += angular_step_rad) {
    pts.push_back(MakePoint(radius * std::cos(a), radius * std::sin(a)));
  }
  return pts;
}

void AddNoise(std::vector<Point>* pts, double sigma, std::mt19937* rng) {
  std::normal_distribution<double> n(0.0, sigma);
  for (auto& p : *pts) {
    p.position.x() += static_cast<float>(n(*rng));
    p.position.y() += static_cast<float>(n(*rng));
  }
}

void Report(const std::string& name, const std::vector<Point>& cloud,
            const cartographer::mapping::scan_matching::DegeneracyDetectorOptions&
                opts) {
  cartographer::mapping::scan_matching::DegeneracyDetector det(opts);
  const auto r = det.DetectDegeneracy(cloud, /*fusion=*/cloud);
  std::printf("--- %s ---\n", name.c_str());
  std::printf("  points (raw / valid normals) : %d / %d\n",
              r.lidar_features.total_points, r.lidar_features.valid_normal_count);
  std::printf("  R_deg = %.3f   L_deg = %.3f   F_deg = %.3f\n",
              r.R_deg(), r.L_deg(), r.F_deg());
  std::printf("  G_deg = %.3f   phi   = %+.2f deg\n\n",
              r.G_deg, r.phi() * 180.0 / M_PI);
}

}  // namespace

int main() {
  using cartographer::mapping::scan_matching::DegeneracyDetectorOptions;

  // The synthetic clouds below are noisy 2D LiDAR scans; we configure the
  // expected sample count S_theta / R_theta to match a 360 deg / 1 deg scan
  // (= 360 returns) so the R_deg coefficients are interpretable.
  DegeneracyDetectorOptions opts;
  opts.voxel_size                = 0.10;
  opts.min_points_per_voxel      = 1;
  opts.k_neighbors               = 8;
  opts.max_neighbor_distance     = 0.40;
  opts.lidar_fov                 = 2.0 * M_PI;
  opts.lidar_angular_resolution  = 1.0 * M_PI / 180.0;   // expected ~360
  opts.sigma_squared             = 0.5;
  opts.curvature_threshold       = 0.10;

  std::mt19937 rng(0xDA2026u);

  auto room = MakeLShapedRoom(0.05);
  AddNoise(&room, 0.01, &rng);

  auto corridor = MakeCorridor(0.05);
  AddNoise(&corridor, 0.01, &rng);

  auto arc = MakeSparseArc(2.0 * M_PI / 180.0);
  AddNoise(&arc, 0.01, &rng);

  std::printf(
      "Degeneracy descriptors are normalised to [0,1].  Higher = more degenerate.\n"
      "Expected sample count (Eqs. 5-6):  S_theta / R_theta = %d returns.\n\n",
      static_cast<int>(opts.lidar_fov / opts.lidar_angular_resolution));

  Report("L-shaped room (nominal scene)", room, opts);
  Report("Long corridor (geometric degeneracy)", corridor, opts);
  Report("Sparse 90 deg arc (range degeneracy)", arc, opts);

  std::printf(
      "Reading guide:\n"
      "  * L-shaped room : low  G_deg (mixed normal directions),  low R_deg\n"
      "  * Corridor      : high G_deg (parallel walls -> phi ~ 0), low R_deg\n"
      "  * Sparse arc    : high R_deg / L_deg / F_deg (few returns vs expected)\n");

  return 0;
}
