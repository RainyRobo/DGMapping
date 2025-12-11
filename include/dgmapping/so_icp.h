// Copyright 2026 The DGMapping Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//
// ---------------------------------------------------------------------------
//
// Sampling-Optimized Iterative Closest Point (SO-ICP).
//
// Header-only implementation of Algorithm 1 (Sec. III-B) of the DGMapping
// paper.  SO-ICP aligns the pseudo-LiDAR cloud P^D extracted from an RGB-D
// camera to the real 2D LiDAR cloud P^L.  Compared with vanilla ICP it
// restricts the inner least-squares step to the m = floor(N * eta) points
// of P^D with the smallest residual to P^L under the current transform,
// where eta is the FOV overlap ratio of the RGB-D camera and the 2D LiDAR.
// This focuses the fit on structurally consistent points and rejects the
// camera-only 3D content that would otherwise bias the alignment.
//
// Equation references map to the manuscript:
//   Eq. (3)   d_i = min_{j} || T P^D_i - P^L_j ||
//   Eq. (4)   T*  = argmin_{R, t} Σ_i || (R P^D_i + t) - P^L_i ||²
//   Algorithm 1: SAMPLING -> ESTIMATION -> COMPUTEDISTANCE -> RMSE -> accept

#ifndef DGMAPPING_SO_ICP_H_
#define DGMAPPING_SO_ICP_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cartographer {
namespace mapping {
namespace scan_matching {

// Tunable parameters for SO-ICP.
struct SoIcpOptions {
  // Maximum outer iterations of the Algorithm 1 while-loop.
  int max_iterations = 30;

  // Tolerance on the per-iteration RMSE decrease.  When two successive
  // accepted iterations differ by less than this value the algorithm is
  // declared to have converged (this is in addition to the paper's
  // "RMSE no longer decreases" termination criterion).
  double convergence_eps = 1e-5;

  // Maximum allowed point-to-point distance for an Eq. (3) correspondence.
  // Pairs whose distance exceeds this value are considered outliers and
  // are not used in the inner least-squares ESTIMATION step.  The same
  // threshold gates the residual computation for the smallest-m sampling.
  double max_correspondence_distance = 0.5;  // metres

  // FOV-overlap ratio  eta  used to derive the sample size  m = N * eta
  // (Sec. III-B).  Set to a non-positive value to estimate it from the
  // initial pose by counting how many P^D points have a P^L neighbour
  // within max_correspondence_distance.  A typical RGB-D + 360° 2D LiDAR
  // setup yields eta ≈ 0.5 - 0.8 once redundant points are pruned.
  double fov_overlap_ratio = 0.0;

  // Lower bound on the sample size m.  If N * eta falls below this value
  // (e.g. eta is severely underestimated) we still keep at least this
  // many points so that the 2D rigid-body fit is well determined.
  int min_samples = 10;

  // 2D grid cell size for the spatial nearest-neighbour index.  A value
  // close to (but no smaller than) the typical inter-point spacing of P^L
  // gives the best lookup performance.
  double grid_cell_size = 0.2;  // metres
};

// Result returned by SoIcp::Align.
struct SoIcpResult {
  // Final estimated SE(2) transformation T_θ taking source points (P^D)
  // into the target frame (P^L).  Transformed source = T * src.
  Eigen::Isometry2d transform = Eigen::Isometry2d::Identity();

  // RMSE evaluated on the smallest-m residuals at the accepted iteration.
  double rmse = std::numeric_limits<double>::infinity();

  // Number of completed outer iterations.
  int iterations = 0;

  // Sample size  m = floor(N * eta)  used during the run.
  int sample_size = 0;

  // True if the convergence-eps test fired before max_iterations elapsed
  // or before the RMSE-monotonic-decrease test terminated the loop.
  bool converged = false;

  // Indices into the source cloud of the smallest-residual subset that
  // SO-ICP considered structurally consistent at termination.  These are
  // the rows that should be kept when assembling the fused observation
  // cloud z^F_t in Eq. (14).
  std::vector<std::size_t> inlier_indices;
};

namespace internal {

// O(1)-average 2D nearest-neighbour query backed by a hash-grid.
class Grid2DIndex {
 public:
  explicit Grid2DIndex(double cell_size)
      : cell_size_(cell_size > 1e-9 ? cell_size : 0.1) {}

  void Build(const std::vector<Eigen::Vector2d>& points) {
    cells_.clear();
    points_ = &points;
    cells_.reserve(points.size() * 2);
    for (std::size_t i = 0; i < points.size(); ++i) {
      cells_[Key(points[i])].push_back(i);
    }
  }

  // Returns (index, squared_distance).  index == npos when no neighbour
  // sits within max_distance of the query.
  std::pair<std::size_t, double> FindNearest(const Eigen::Vector2d& q,
                                             double max_distance) const {
    constexpr std::size_t kNoMatch = static_cast<std::size_t>(-1);
    std::pair<std::size_t, double> best{kNoMatch, max_distance * max_distance};
    if (!points_ || points_->empty()) return best;

    const int radius =
        std::max(1, static_cast<int>(std::ceil(max_distance / cell_size_)));
    const auto k = Key(q);
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        const auto it = cells_.find({k.first + dx, k.second + dy});
        if (it == cells_.end()) continue;
        for (std::size_t idx : it->second) {
          const double d2 = ((*points_)[idx] - q).squaredNorm();
          if (d2 < best.second) {
            best = {idx, d2};
          }
        }
      }
    }
    return best;
  }

 private:
  using KeyT = std::pair<int, int>;
  struct KeyHash {
    std::size_t operator()(const KeyT& k) const noexcept {
      return (static_cast<std::size_t>(k.first) * 73856093u) ^
             (static_cast<std::size_t>(k.second) * 19349663u);
    }
  };

  KeyT Key(const Eigen::Vector2d& p) const {
    return {static_cast<int>(std::floor(p.x() / cell_size_)),
            static_cast<int>(std::floor(p.y() / cell_size_))};
  }

  double cell_size_;
  const std::vector<Eigen::Vector2d>* points_ = nullptr;
  std::unordered_map<KeyT, std::vector<std::size_t>, KeyHash> cells_;
};

// Closed-form 2D rigid-body Procrustes alignment minimising
//     Σ_i || (R src_i + t) - dst_i ||²       (paper Eq. 4)
// via the Kabsch-Umeyama SVD construction.  Reflections are removed so
// the returned linear part is always a proper rotation (det = +1).
inline Eigen::Isometry2d ProcrustesRigid2D(
    const std::vector<Eigen::Vector2d>& src,
    const std::vector<Eigen::Vector2d>& dst) {
  Eigen::Isometry2d T = Eigen::Isometry2d::Identity();
  if (src.empty() || src.size() != dst.size()) return T;

  Eigen::Vector2d mu_src = Eigen::Vector2d::Zero();
  Eigen::Vector2d mu_dst = Eigen::Vector2d::Zero();
  for (std::size_t i = 0; i < src.size(); ++i) {
    mu_src += src[i];
    mu_dst += dst[i];
  }
  const double inv_n = 1.0 / static_cast<double>(src.size());
  mu_src *= inv_n;
  mu_dst *= inv_n;

  Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
  for (std::size_t i = 0; i < src.size(); ++i) {
    H += (src[i] - mu_src) * (dst[i] - mu_dst).transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix2d> svd(
      H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix2d& U = svd.matrixU();
  const Eigen::Matrix2d& V = svd.matrixV();

  Eigen::Matrix2d D = Eigen::Matrix2d::Identity();
  if ((V * U.transpose()).determinant() < 0.0) {
    D(1, 1) = -1.0;
  }
  const Eigen::Matrix2d R = V * D * U.transpose();
  const Eigen::Vector2d t = mu_dst - R * mu_src;

  T.linear() = R;
  T.translation() = t;
  return T;
}

}  // namespace internal

// SO-ICP aligner.
//
// Usage:
//
//   struct MyPoint { Eigen::Vector3f position; };
//   std::vector<MyPoint> pseudo_lidar = ...;     // P^D
//   std::vector<MyPoint> real_lidar   = ...;     // P^L
//
//   SoIcpOptions opt;
//   opt.fov_overlap_ratio = 0.7;                 // η
//   SoIcp solver(opt);
//   SoIcpResult r = solver.Align(pseudo_lidar, real_lidar);
//
//   const Eigen::Matrix2d R = r.transform.linear();
//   const Eigen::Vector2d t = r.transform.translation();
//   const double yaw = std::atan2(R(1, 0), R(0, 0));
//   // Optional: convert to cartographer::transform::Rigid2d
//   //   transform::Rigid2d({t.x(), t.y()}, yaw);
class SoIcp {
 public:
  explicit SoIcp(const SoIcpOptions& options = SoIcpOptions{})
      : options_(options) {}

  // Aligns the pseudo-LiDAR cloud (source, P^D) to the 2D LiDAR cloud
  // (target, P^L) starting from the initial transformation T_0.  The
  // input points may carry arbitrary z-coordinates: SO-ICP, like the
  // paper, projects them onto the 2D scanning plane before alignment.
  //
  // The point type T must expose `T::position` convertible to
  // Eigen::Vector3f (i.e. the same convention used by the rest of the
  // dgmapping codebase, including DegeneracyDetector).
  template <typename T>
  SoIcpResult Align(
      const std::vector<T>& source, const std::vector<T>& target,
      const Eigen::Isometry2d& initial_transform = Eigen::Isometry2d::Identity()) const {
    SoIcpResult result;
    result.transform = initial_transform;

    if (source.empty() || target.empty()) {
      return result;
    }

    // Project both clouds onto the 2D scanning plane.
    std::vector<Eigen::Vector2d> src;
    src.reserve(source.size());
    for (const auto& p : source) {
      src.emplace_back(static_cast<double>(p.position.x()),
                       static_cast<double>(p.position.y()));
    }
    std::vector<Eigen::Vector2d> dst;
    dst.reserve(target.size());
    for (const auto& p : target) {
      dst.emplace_back(static_cast<double>(p.position.x()),
                       static_cast<double>(p.position.y()));
    }

    internal::Grid2DIndex target_index(options_.grid_cell_size);
    target_index.Build(dst);

    // -------- m = floor(N * eta)  (Sec. III-B, paragraph after Eq. 4) --------
    double eta = options_.fov_overlap_ratio;
    if (!(eta > 0.0)) {
      eta = EstimateOverlapRatio(src, target_index, initial_transform);
    }
    eta = std::clamp(eta, 0.0, 1.0);
    int m = static_cast<int>(
        std::floor(static_cast<double>(src.size()) * eta));
    m = std::max(m, options_.min_samples);
    m = std::min(m, static_cast<int>(src.size()));
    result.sample_size = m;

    // ------------------------------ Algorithm 1 ----------------------------
    Eigen::Isometry2d T_theta = initial_transform;
    double best_e = std::numeric_limits<double>::infinity();

    // The paper writes "D ← I" as the initialisation of the residual set;
    // to make the very first SAMPLING step meaningful we instead seed D
    // with the residuals under T_0 (equivalent when T_0 = identity).
    std::vector<double> residuals;
    std::vector<std::size_t> nn_indices;
    ComputeDistances(src, target_index, T_theta, &residuals, &nn_indices);

    for (int it = 0; it < options_.max_iterations; ++it) {
      // 3: S ← SAMPLING(P^D, m, D)
      auto sample_idx = SmallestM(residuals, m);
      sample_idx.erase(
          std::remove_if(
              sample_idx.begin(), sample_idx.end(),
              [&](std::size_t i) { return !std::isfinite(residuals[i]); }),
          sample_idx.end());

      std::vector<Eigen::Vector2d> src_pts;
      std::vector<Eigen::Vector2d> dst_pts;
      src_pts.reserve(sample_idx.size());
      dst_pts.reserve(sample_idx.size());
      for (std::size_t idx : sample_idx) {
        const Eigen::Vector2d transformed = T_theta * src[idx];
        const auto nn = target_index.FindNearest(
            transformed, options_.max_correspondence_distance);
        if (nn.first == static_cast<std::size_t>(-1)) continue;
        // The rigid fit recovers the absolute pose (not an increment), so
        // we feed the un-transformed src_i and the target picked under T_θ.
        src_pts.push_back(src[idx]);
        dst_pts.push_back(dst[nn.first]);
      }

      if (src_pts.size() < 3) break;  // Need ≥3 pairs for 2D rigid fit.

      // 4: T*_θ ← ESTIMATION(S, P^L)               (Eq. 4)
      const Eigen::Isometry2d T_star =
          internal::ProcrustesRigid2D(src_pts, dst_pts);

      // 5: D ← COMPUTEDISTANCE(P^D, P^L, T*_θ)    (Eq. 3, full P^D)
      ComputeDistances(src, target_index, T_star, &residuals, &nn_indices);

      // 6: e* ← RMSE over the m smallest residuals.
      auto smallest = SmallestM(residuals, m);
      double sum_sq = 0.0;
      int valid = 0;
      for (std::size_t i : smallest) {
        if (std::isfinite(residuals[i])) {
          sum_sq += residuals[i] * residuals[i];
          ++valid;
        }
      }
      const double e_star = (valid > 0)
                                ? std::sqrt(sum_sq / static_cast<double>(valid))
                                : std::numeric_limits<double>::infinity();

      // 7-10: monotone-decrease acceptance test.
      if (e_star <= best_e) {
        const double improvement = best_e - e_star;
        T_theta = T_star;
        best_e = e_star;
        result.iterations = it + 1;
        result.transform = T_theta;
        result.rmse = best_e;
        result.inlier_indices = smallest;
        if (improvement < options_.convergence_eps) {
          result.converged = true;
          return result;
        }
      } else {
        break;
      }
    }

    return result;
  }

 private:
  // Returns the indices of the m smallest entries of `residuals`.
  // Behaves as a partial sort and is O(N) on average.
  static std::vector<std::size_t> SmallestM(
      const std::vector<double>& residuals, int m) {
    std::vector<std::size_t> indices(residuals.size());
    for (std::size_t i = 0; i < residuals.size(); ++i) indices[i] = i;
    if (static_cast<int>(indices.size()) <= m) return indices;
    std::nth_element(
        indices.begin(), indices.begin() + m, indices.end(),
        [&](std::size_t a, std::size_t b) { return residuals[a] < residuals[b]; });
    indices.resize(m);
    return indices;
  }

  // Eq. (3): for each P^D_i populate residuals[i] = || T * P^D_i - NN(P^L) ||.
  // Points whose nearest neighbour falls outside max_correspondence_distance
  // receive an infinite residual and are therefore demoted to the bottom of
  // the next SAMPLING step.
  void ComputeDistances(const std::vector<Eigen::Vector2d>& src,
                        const internal::Grid2DIndex& target_index,
                        const Eigen::Isometry2d& T,
                        std::vector<double>* residuals,
                        std::vector<std::size_t>* nn_indices) const {
    residuals->assign(src.size(), std::numeric_limits<double>::infinity());
    nn_indices->assign(src.size(), static_cast<std::size_t>(-1));
    for (std::size_t i = 0; i < src.size(); ++i) {
      const Eigen::Vector2d transformed = T * src[i];
      const auto nn = target_index.FindNearest(
          transformed, options_.max_correspondence_distance);
      if (nn.first != static_cast<std::size_t>(-1)) {
        (*residuals)[i] = std::sqrt(nn.second);
        (*nn_indices)[i] = nn.first;
      }
    }
  }

  // Estimate η at the initial pose: fraction of P^D points whose nearest
  // neighbour in P^L lies within max_correspondence_distance after T_0.
  double EstimateOverlapRatio(const std::vector<Eigen::Vector2d>& src,
                              const internal::Grid2DIndex& target_index,
                              const Eigen::Isometry2d& T_init) const {
    if (src.empty()) return 0.0;
    int hits = 0;
    for (const auto& p : src) {
      const auto nn = target_index.FindNearest(
          T_init * p, options_.max_correspondence_distance);
      if (nn.first != static_cast<std::size_t>(-1)) ++hits;
    }
    return static_cast<double>(hits) / static_cast<double>(src.size());
  }

  SoIcpOptions options_;
};

// Builds the fused observation cloud z^F_t (input to Eq. 14) from the 2D
// LiDAR scan and the pseudo-LiDAR cloud after SO-ICP alignment.  The
// LiDAR cloud is preserved verbatim and the structurally consistent
// subset of P^D (so_icp_result.inlier_indices) is appended after being
// transformed by the recovered SE(2) pose.  The original z of every
// point is preserved so callers can still inspect 3D structure.
template <typename T>
std::vector<T> BuildFusedObservation(
    const std::vector<T>& lidar_cloud,
    const std::vector<T>& pseudo_lidar_cloud,
    const SoIcpResult& so_icp_result) {
  std::vector<T> fused;
  fused.reserve(lidar_cloud.size() + so_icp_result.inlier_indices.size());

  for (const auto& p : lidar_cloud) fused.push_back(p);

  const Eigen::Matrix2d R = so_icp_result.transform.linear();
  const Eigen::Vector2d t = so_icp_result.transform.translation();
  for (std::size_t idx : so_icp_result.inlier_indices) {
    if (idx >= pseudo_lidar_cloud.size()) continue;
    T p = pseudo_lidar_cloud[idx];
    const Eigen::Vector2d xy(static_cast<double>(p.position.x()),
                             static_cast<double>(p.position.y()));
    const Eigen::Vector2d xy_t = R * xy + t;
    p.position.x() = static_cast<float>(xy_t.x());
    p.position.y() = static_cast<float>(xy_t.y());
    // z is preserved.
    fused.push_back(p);
  }
  return fused;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // DGMAPPING_SO_ICP_H_
