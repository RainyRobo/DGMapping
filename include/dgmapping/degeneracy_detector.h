// Copyright 2026 The DGMapping Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// ---------------------------------------------------------------------------
//
// Degeneracy detector for 2D LiDAR / RGB-D fusion.
//
// Header-only implementation of the descriptors defined in Sec. III-C of the
// DGMapping paper.  Given the raw 2D LiDAR cloud P^L and the (optional) fused
// cloud P^F, the detector returns five scalars consumed by the scan matcher:
//
//     R_deg  (Eq. 7)   range degeneration
//     L_deg  (Eq. 8)   LiDAR point-cloud degeneration
//     F_deg  (Eq. 9)   fused point-cloud degeneration
//     phi    (Eq. 11)  principal degeneration direction
//     G_deg  (Eq. 12)  geometric degeneration
//
// The intermediate range-degeneration coefficients R^L_deg (Eq. 5) and
// R^F_deg (Eq. 6) are also exposed for diagnostics.

#ifndef DGMAPPING_DEGENERACY_DETECTOR_H_
#define DGMAPPING_DEGENERACY_DETECTOR_H_

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cartographer {
namespace mapping {
namespace scan_matching {

// ---------------------------------------------------------------------------
// Public result types.
// ---------------------------------------------------------------------------

struct NormalFeatureResult {
  int total_points = 0;
  int valid_normal_count = 0;
  double normal_feature_ratio = 0.0;

  // Eigenvalues of the symmetrized normal covariance Σ (Eq. 10), sorted in
  // descending order.  lambda1 > lambda2 means the normals concentrate along
  // a single direction -> high geometric degeneration.
  double lambda1 = 0.0;
  double lambda2 = 0.0;

  // Principal degeneration direction phi (Eq. 11), in radians.
  double phi = 0.0;

  // Per-point unit normals after voxel-downsampling and curvature gating.
  // Stored as (point_xyz, normal_xyz) pairs in the global frame; only the xy
  // components of `second` are used by Eq. (12) but z is preserved so the
  // caller can repurpose the cache for visualisation or diagnostics.
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> points_with_normals;
};

struct RangeDegeneracyResult {
  double R_deg_L = 0.0;  // Eq. (5)
  double R_deg_F = 0.0;  // Eq. (6)
  double R_deg = 0.0;    // Eq. (7) = R_deg_L * R_deg_F
};

struct PointCloudDegeneracyResult {
  double L_deg = 0.0;    // Eq. (8)
  double F_deg = 0.0;    // Eq. (9)
  double phi = 0.0;      // Eq. (11), copied from the LiDAR features
  double lambda1 = 0.0;
  double lambda2 = 0.0;
};

struct DegeneracyResult {
  NormalFeatureResult lidar_features;
  NormalFeatureResult fusion_features;
  RangeDegeneracyResult range_deg;
  PointCloudDegeneracyResult pc_deg;

  // G_deg (Eq. 12), evaluated on the rotated normal cloud of P^L.
  double G_deg = 0.0;

  // Convenience accessors mirroring the symbols used in the paper / scan
  // matcher signature (R_deg, G_deg, L_deg, F_deg, phi).
  double R_deg() const { return range_deg.R_deg; }
  double L_deg() const { return pc_deg.L_deg; }
  double F_deg() const { return pc_deg.F_deg; }
  double phi() const { return pc_deg.phi; }
};

struct DegeneracyDetectorOptions {
  // Voxel downsampling for normal estimation.
  double voxel_size = 0.1;
  int min_points_per_voxel = 3;

  // K-NN neighbourhood for per-point normal estimation.
  int k_neighbors = 10;
  double max_neighbor_distance = 0.5;

  // LiDAR FOV and angular resolution define the expected sample count
  // S_theta / R_theta used in Eqs. (5)-(6).  Defaults match a 360° 2D LiDAR
  // with 0.25° resolution.
  double lidar_fov = 2 * M_PI;
  double lidar_angular_resolution = 0.25 * M_PI / 180.0;

  // sigma^2 in Eq. (9).  Smaller values make F_deg more conservative.
  double sigma_squared = 0.5;

  // Points whose surface curvature exceeds this value are excluded from the
  // PCA in Eq. (10) (they correspond to corners / edges and would bias the
  // principal-direction estimate).
  double curvature_threshold = 0.1;
};

// ---------------------------------------------------------------------------
// Internal building blocks (kept inline to preserve header-only usage).
// ---------------------------------------------------------------------------

namespace detector_internal {

struct VoxelKey {
  int x, y, z;
  bool operator==(const VoxelKey& o) const {
    return x == o.x && y == o.y && z == o.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& k) const noexcept {
    return (static_cast<std::size_t>(k.x) * 73856093u) ^
           (static_cast<std::size_t>(k.y) * 19349663u) ^
           (static_cast<std::size_t>(k.z) * 83492791u);
  }
};

template <typename T>
struct PointWithNormal {
  T point;
  Eigen::Vector3f normal;
  float curvature;
  bool valid_normal;
};

// 3D hash-grid for k-NN queries.  The point type T must expose
// `T::position` convertible to Eigen::Vector3f.
template <typename T>
class SpatialIndex {
 public:
  explicit SpatialIndex(double cell_size) : cell_size_(cell_size) {}

  void Build(const std::vector<T>& points) {
    cells_.clear();
    cells_.reserve(points.size() * 2);
    for (std::size_t i = 0; i < points.size(); ++i) {
      cells_[GetKey(points[i])].push_back(i);
    }
  }

  std::vector<std::size_t> FindKNearest(const T& query,
                                        const std::vector<T>& points, int k,
                                        double max_distance) const {
    const VoxelKey center = GetKey(query);
    const int radius =
        std::max(1, static_cast<int>(std::ceil(max_distance / cell_size_)));
    std::vector<std::pair<double, std::size_t>> candidates;
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dz = -radius; dz <= radius; ++dz) {
          const auto it = cells_.find({center.x + dx, center.y + dy,
                                       center.z + dz});
          if (it == cells_.end()) continue;
          for (std::size_t idx : it->second) {
            const double d = (points[idx].position - query.position).norm();
            if (d <= max_distance && d > 1e-10) {
              candidates.emplace_back(d, idx);
            }
          }
        }
      }
    }
    std::sort(candidates.begin(), candidates.end());
    std::vector<std::size_t> out;
    out.reserve(std::min<std::size_t>(k, candidates.size()));
    for (std::size_t i = 0; i < std::min<std::size_t>(k, candidates.size());
         ++i) {
      out.push_back(candidates[i].second);
    }
    return out;
  }

 private:
  VoxelKey GetKey(const T& p) const {
    return {static_cast<int>(std::floor(p.position.x() / cell_size_)),
            static_cast<int>(std::floor(p.position.y() / cell_size_)),
            static_cast<int>(std::floor(p.position.z() / cell_size_))};
  }

  double cell_size_;
  std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelKeyHash> cells_;
};

}  // namespace detector_internal

// ---------------------------------------------------------------------------
// DegeneracyDetector
// ---------------------------------------------------------------------------

class DegeneracyDetector {
 public:
  explicit DegeneracyDetector(const DegeneracyDetectorOptions& options)
      : options_(options) {}

  // End-to-end entry point producing every descriptor needed by the
  // scan matcher.  `fusion_points` may be empty, in which case fusion-side
  // descriptors degenerate to their LiDAR-only counterparts.
  template <typename T>
  DegeneracyResult DetectDegeneracy(
      const std::vector<T>& lidar_points,
      const std::vector<T>& fusion_points) {
    DegeneracyResult result;

    result.lidar_features = ExtractNormalFeatures(lidar_points);
    result.fusion_features = fusion_points.empty()
                                 ? result.lidar_features
                                 : ExtractNormalFeatures(fusion_points);

    result.range_deg = CalculateRangeDegeneracy(result.lidar_features,
                                                result.fusion_features);

    result.pc_deg = CalculatePointCloudDegeneracy(result.range_deg,
                                                  result.lidar_features);

    // G_deg (Eq. 12) is evaluated on the *normal-vector features* of P^L
    // rotated into the principal-direction frame {n'}, NOT on the raw
    // scan points (cf. paper Sec. III-C, paragraph after Eq. 11).
    result.G_deg = CalculateGeometricDegeneracy(
        result.lidar_features.points_with_normals, result.pc_deg.phi);

    return result;
  }

  // Step 1 (Sec. III-C, Eqs. 10-11):
  //   1. Voxel-downsample the input cloud.
  //   2. Estimate per-point unit normals via PCA on the K-NN neighbourhood.
  //   3. Build the symmetrized planar normal cloud P_s^n = {+n_i, -n_i};
  //      since |P_s^n| = 2 N_n is mean-free, Σ reduces to (1/N_n) Σ n_i n_i^T.
  //   4. The eigenvector of Σ with the largest eigenvalue is the principal
  //      degeneration direction e_n.  phi = atan2(e_{n,y}, e_{n,x}).
  //
  // 2D LiDAR scans live in the scanning plane, so only the xy components of
  // each normal participate in the PCA.
  template <typename T>
  NormalFeatureResult ExtractNormalFeatures(const std::vector<T>& points) {
    NormalFeatureResult result;
    result.total_points = static_cast<int>(points.size());
    if (points.size() < 10) return result;

    const auto downsampled = VoxelDownsample(points);
    const auto with_normals = EstimateNormals(downsampled);

    int valid_count = 0;
    for (const auto& pn : with_normals) {
      if (pn.valid_normal && pn.curvature < options_.curvature_threshold) {
        ++valid_count;
        result.points_with_normals.emplace_back(pn.point.position, pn.normal);
      }
    }
    result.valid_normal_count = valid_count;
    result.normal_feature_ratio =
        static_cast<double>(valid_count) /
        std::max<std::size_t>(downsampled.size(), 1);

    Eigen::Matrix2d sigma = Eigen::Matrix2d::Zero();
    int n_valid = 0;
    for (const auto& pn : with_normals) {
      if (!pn.valid_normal || pn.curvature >= options_.curvature_threshold) {
        continue;
      }
      Eigen::Vector2d n2(static_cast<double>(pn.normal.x()),
                         static_cast<double>(pn.normal.y()));
      const double norm = n2.norm();
      if (norm < 1e-9) continue;
      n2 /= norm;
      sigma += n2 * n2.transpose();
      ++n_valid;
    }

    if (n_valid >= 2) {
      sigma /= static_cast<double>(n_valid);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(sigma);
      // Eigen returns eigenvalues in ascending order.
      const Eigen::Vector2d eigvals = solver.eigenvalues();
      const Eigen::Matrix2d eigvecs = solver.eigenvectors();
      result.lambda1 = eigvals(1);
      result.lambda2 = eigvals(0);
      const Eigen::Vector2d e_n = eigvecs.col(1);
      result.phi = std::atan2(e_n.y(), e_n.x());
    }

    return result;
  }

  // Step 2 (Sec. III-C, Eqs. 5-7):
  //   R^L_deg = 1 - (N^L_n  * R_theta / S_theta)
  //   R^F_deg = 1 - (N^F_n  * R_theta / S_theta)
  //   R_deg   = R^L_deg * R^F_deg
  RangeDegeneracyResult CalculateRangeDegeneracy(
      const NormalFeatureResult& lidar_features,
      const NormalFeatureResult& fusion_features) const {
    RangeDegeneracyResult result;
    const double expected_count = std::max(
        options_.lidar_fov / options_.lidar_angular_resolution, 1.0);

    result.R_deg_L = std::clamp(
        1.0 - static_cast<double>(lidar_features.valid_normal_count) /
                  expected_count,
        0.0, 1.0);
    result.R_deg_F = std::clamp(
        1.0 - static_cast<double>(fusion_features.valid_normal_count) /
                  expected_count,
        0.0, 1.0);
    result.R_deg = result.R_deg_L * result.R_deg_F;
    return result;
  }

  // Step 3 (Sec. III-C, Eqs. 8-9):
  //   F_deg = R^F_deg * exp( - R^L_deg * R^F_deg / sigma^2 )
  //   L_deg = R^L_deg * (1 - F_deg)
  PointCloudDegeneracyResult CalculatePointCloudDegeneracy(
      const RangeDegeneracyResult& range_deg,
      const NormalFeatureResult& lidar_features) const {
    PointCloudDegeneracyResult result;
    const double exponent = -range_deg.R_deg_L * range_deg.R_deg_F /
                            std::max(options_.sigma_squared, 1e-6);
    result.F_deg = std::clamp(range_deg.R_deg_F * std::exp(exponent), 0.0, 1.0);
    result.L_deg =
        std::clamp(range_deg.R_deg_L * (1.0 - result.F_deg), 0.0, 1.0);
    result.phi = lidar_features.phi;
    result.lambda1 = lidar_features.lambda1;
    result.lambda2 = lidar_features.lambda2;
    return result;
  }

  // Step 4 (Sec. III-C, Eq. 12): the planar normal cloud is rotated into
  // {n'} so that e_n becomes the x-axis, and
  //
  //     G_deg = 1 - exp( - Σ_i |x_i| / Σ_i |y_i| )
  //
  // where (x_i, y_i) are the rotated coordinates of the i-th unit normal.
  // High concentration along the principal direction -> ratio diverges
  // -> G_deg -> 1 (highly degenerate environment, e.g. corridor).
  double CalculateGeometricDegeneracy(
      const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>&
          points_with_normals,
      double phi) const {
    if (points_with_normals.size() < 3) return 0.0;

    const double cos_phi = std::cos(-phi);
    const double sin_phi = std::sin(-phi);

    double sum_abs_x = 0.0;
    double sum_abs_y = 0.0;
    int count = 0;
    for (const auto& pn : points_with_normals) {
      Eigen::Vector2d n2(static_cast<double>(pn.second.x()),
                         static_cast<double>(pn.second.y()));
      const double norm = n2.norm();
      if (norm < 1e-9) continue;
      n2 /= norm;
      sum_abs_x += std::abs(cos_phi * n2.x() - sin_phi * n2.y());
      sum_abs_y += std::abs(sin_phi * n2.x() + cos_phi * n2.y());
      ++count;
    }
    if (count < 3 || sum_abs_y < 1e-10) return 1.0;
    return std::clamp(1.0 - std::exp(-sum_abs_x / sum_abs_y), 0.0, 1.0);
  }

 private:
  template <typename T>
  std::vector<T> VoxelDownsample(const std::vector<T>& points) const {
    using detector_internal::VoxelKey;
    using detector_internal::VoxelKeyHash;
    std::unordered_map<VoxelKey, std::vector<const T*>, VoxelKeyHash> grid;
    grid.reserve(points.size());
    for (const auto& p : points) {
      grid[{static_cast<int>(std::floor(p.position.x() / options_.voxel_size)),
            static_cast<int>(std::floor(p.position.y() / options_.voxel_size)),
            static_cast<int>(std::floor(p.position.z() / options_.voxel_size))}]
          .push_back(&p);
    }

    std::vector<T> out;
    out.reserve(grid.size());
    for (auto& [key, cell] : grid) {
      if (cell.size() <
          static_cast<std::size_t>(options_.min_points_per_voxel)) {
        continue;
      }
      Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
      for (const T* p : cell) centroid += p->position;
      centroid /= static_cast<float>(cell.size());
      T rep;
      rep.position = centroid;
      out.push_back(rep);
    }
    return out;
  }

  template <typename T>
  std::vector<detector_internal::PointWithNormal<T>> EstimateNormals(
      const std::vector<T>& points) const {
    using detector_internal::PointWithNormal;
    using detector_internal::SpatialIndex;
    std::vector<PointWithNormal<T>> out(points.size());

    if (points.size() < static_cast<std::size_t>(options_.k_neighbors)) {
      for (std::size_t i = 0; i < points.size(); ++i) {
        out[i].point = points[i];
        out[i].normal = Eigen::Vector3f::UnitZ();
        out[i].curvature = 0.0f;
        out[i].valid_normal = false;
      }
      return out;
    }

    SpatialIndex<T> index(options_.max_neighbor_distance);
    index.Build(points);

    for (std::size_t i = 0; i < points.size(); ++i) {
      out[i].point = points[i];
      const auto neighbors = index.FindKNearest(
          points[i], points, options_.k_neighbors,
          options_.max_neighbor_distance);
      if (neighbors.size() < 3) {
        out[i].normal = Eigen::Vector3f::UnitZ();
        out[i].curvature = 0.0f;
        out[i].valid_normal = false;
        continue;
      }

      Eigen::Vector3f centroid = points[i].position;
      for (std::size_t idx : neighbors) centroid += points[idx].position;
      centroid /= static_cast<float>(neighbors.size() + 1);

      Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
      const Eigen::Vector3f diff_self = points[i].position - centroid;
      cov += diff_self * diff_self.transpose();
      for (std::size_t idx : neighbors) {
        const Eigen::Vector3f d = points[idx].position - centroid;
        cov += d * d.transpose();
      }
      cov /= static_cast<float>(neighbors.size() + 1);

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
      const Eigen::Vector3f eigvals = solver.eigenvalues();
      const Eigen::Matrix3f eigvecs = solver.eigenvectors();

      // Smallest eigenvalue -> normal direction.  Eigen returns sorted
      // eigenvalues in ascending order so column 0 is the answer.
      out[i].normal = eigvecs.col(0);
      if (out[i].normal.z() < 0) out[i].normal = -out[i].normal;
      const float sum_eig = eigvals.sum();
      out[i].curvature = (sum_eig > 1e-10f) ? eigvals(0) / sum_eig : 0.0f;
      out[i].valid_normal = true;
    }
    return out;
  }

  DegeneracyDetectorOptions options_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // DGMAPPING_DEGENERACY_DETECTOR_H_
