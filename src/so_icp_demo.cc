// Copyright 2026 The DGMapping Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// ---------------------------------------------------------------------------
//
// Standalone demonstration of the SO-ICP module (`dgmapping/so_icp.h`).
// Synthesises an L-shaped indoor scene scanned by a 2D LiDAR (P^L) and a
// misaligned pseudo-LiDAR cloud from an RGB-D camera (P^D) with ~30% of
// camera-only outliers, then recovers the SE(2) offset with SO-ICP and
// prints the residual error.
//
// Build:
//   g++ -std=c++17 -O2 -I include -I /usr/include/eigen3 src/so_icp_demo.cc -o so_icp_demo
//
// Run:
//   ./so_icp_demo

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "dgmapping/so_icp.h"

namespace {

// Minimal point type compatible with the dgmapping convention
// (T::position must be convertible to Eigen::Vector3f).
struct Point {
  Eigen::Vector3f position;
};

Point MakePoint(double x, double y, double z = 0.0) {
  Point p;
  p.position = Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y),
                               static_cast<float>(z));
  return p;
}

// L-shaped room: outer walls of an 8×8 m square with a 4×4 m corner cut
// out of the top-right quadrant.  `step` controls the sampling density.
std::vector<Point> MakeLShapedRoom(double step) {
  std::vector<Point> points;
  for (double y = 0.0; y <= 8.0 + 1e-9; y += step) {
    points.push_back(MakePoint(0.0, y));
  }
  for (double x = 0.0; x <= 8.0 + 1e-9; x += step) {
    points.push_back(MakePoint(x, 0.0));
  }
  for (double y = 0.0; y <= 4.0 + 1e-9; y += step) {
    points.push_back(MakePoint(8.0, y));
  }
  for (double x = 8.0; x >= 4.0 - 1e-9; x -= step) {
    points.push_back(MakePoint(x, 4.0));
  }
  for (double y = 4.0; y <= 8.0 + 1e-9; y += step) {
    points.push_back(MakePoint(4.0, y));
  }
  for (double x = 4.0; x >= 0.0 - 1e-9; x -= step) {
    points.push_back(MakePoint(x, 8.0));
  }
  return points;
}

std::vector<Point> Transform(const std::vector<Point>& points, double yaw,
                             double tx, double ty) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  std::vector<Point> out;
  out.reserve(points.size());
  for (const auto& p : points) {
    const double x = p.position.x();
    const double y = p.position.y();
    out.push_back(MakePoint(c * x - s * y + tx, s * x + c * y + ty,
                            p.position.z()));
  }
  return out;
}

void AddNoise(std::vector<Point>* points, double sigma, std::mt19937* rng) {
  std::normal_distribution<double> n(0.0, sigma);
  for (auto& p : *points) {
    p.position.x() += static_cast<float>(n(*rng));
    p.position.y() += static_cast<float>(n(*rng));
  }
}

// Sprinkle "camera-only" outliers uniformly inside the room interior
// (cf. the red 3D points highlighted in Fig. 4 of the paper).
void AddOutliers(std::vector<Point>* points, int count, std::mt19937* rng) {
  std::uniform_real_distribution<double> u(0.5, 7.5);
  for (int i = 0; i < count; ++i) {
    points->push_back(MakePoint(u(*rng), u(*rng), 0.0));
  }
}

}  // namespace

int main() {
  using cartographer::mapping::scan_matching::SoIcp;
  using cartographer::mapping::scan_matching::SoIcpOptions;
  using cartographer::mapping::scan_matching::SoIcpResult;
  using cartographer::mapping::scan_matching::BuildFusedObservation;

  std::mt19937 rng(0xDA2026u);
  const auto layout = MakeLShapedRoom(/*step=*/0.05);

  // 2D LiDAR scan P^L: noise σ ≈ 1 cm, no outliers.
  auto lidar = layout;
  AddNoise(&lidar, 0.01, &rng);

  // Pseudo-LiDAR cloud P^D: same scene seen from a misaligned RGB-D camera,
  // with heavier depth noise and ~30 % camera-only outliers.  We want
  // SO-ICP to recover T such that T * P^D ≈ P^L, i.e. the inverse of the
  // forward camera-pose offset (gt_yaw, gt_tx, gt_ty).
  const double gt_yaw = 8.0 * M_PI / 180.0;
  const double gt_tx = 0.30;
  const double gt_ty = -0.20;
  auto pseudo = Transform(layout, -gt_yaw, -(std::cos(-gt_yaw) * gt_tx -
                                              std::sin(-gt_yaw) * gt_ty),
                           -(std::sin(-gt_yaw) * gt_tx +
                              std::cos(-gt_yaw) * gt_ty));
  AddNoise(&pseudo, 0.025, &rng);
  AddOutliers(&pseudo, static_cast<int>(0.30 * pseudo.size()), &rng);

  std::printf("Cloud sizes:  P^L = %zu  P^D = %zu (with ~30%% outliers)\n",
              lidar.size(), pseudo.size());
  std::printf("Ground-truth offset to recover:  yaw=%.3f deg  tx=%.3f  ty=%.3f\n\n",
              gt_yaw * 180.0 / M_PI, gt_tx, gt_ty);

  SoIcpOptions opt;
  opt.max_iterations = 40;
  opt.max_correspondence_distance = 0.40;
  opt.fov_overlap_ratio = 0.0;  // 0 ⇒ auto-estimate η at the initial pose.
  opt.grid_cell_size = 0.20;
  opt.convergence_eps = 1e-6;
  opt.min_samples = 50;

  SoIcp solver(opt);
  const SoIcpResult r = solver.Align(pseudo, lidar);

  const Eigen::Matrix2d R = r.transform.linear();
  const Eigen::Vector2d t = r.transform.translation();
  const double yaw_rec = std::atan2(R(1, 0), R(0, 0));

  std::printf("SO-ICP result:\n");
  std::printf("  iterations  : %d  (converged=%s)\n", r.iterations,
              r.converged ? "true" : "false");
  std::printf("  sample size m: %d / %zu\n", r.sample_size, pseudo.size());
  std::printf("  inliers kept : %zu\n", r.inlier_indices.size());
  std::printf("  RMSE        : %.5f m\n", r.rmse);
  std::printf("  recovered   : yaw=%.3f deg  tx=%.4f  ty=%.4f\n",
              yaw_rec * 180.0 / M_PI, t.x(), t.y());
  std::printf("  abs error   : dyaw=%.3f deg  dtx=%.4f  dty=%.4f\n",
              (yaw_rec - gt_yaw) * 180.0 / M_PI, t.x() - gt_tx,
              t.y() - gt_ty);

  // Fused observation cloud z^F_t (input to Eq. 14).
  const auto fused = BuildFusedObservation(lidar, pseudo, r);
  std::printf("\nFused observation cloud z^F_t built: %zu points\n",
              fused.size());
  std::printf("  = %zu LiDAR + %zu inliers from P^D\n", lidar.size(),
              fused.size() - lidar.size());
  return 0;
}
