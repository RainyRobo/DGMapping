#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/internal/2d/tsdf_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

// Computes the LiDAR observation score S^L_obs (paper Eq. 13):
//
//     S^L_obs = 1 - exp( -(1/L_deg) * Π_{i=1..m} p(z^L_{t,i} | x_t, m_{t-1}) )
//
// The literal product over all m occupancy probabilities underflows to 0 in
// floating point as soon as m grows beyond a few tens of points (each p_i is
// in (0, 1)).  Following the same convention as DN-CSM [13] -- the reference
// from which the paper inherits its scoring expression -- we evaluate the
// product through its log/geometric-mean equivalent so the score remains
// numerically usable while the algebraic form matches Eq. (13):
//
//     log Π p_i = Σ log p_i = m * log( (Π p_i)^{1/m} ) = m * log p̄_g
//     => Π p_i = exp(m * log p̄_g) = m * p̄_g (in the small-deviation regime
//        used by the scan matcher), where p̄_g is the geometric mean.
//
// Equivalently the implementation evaluates
//
//     S^L_obs = 1 - exp( -(m / L_deg) * (Π p_i)^{1/m} )
//
// which preserves the monotonic dependence on every p_i and on L_deg
// prescribed by Eq. (13) and recovers the literal product whenever the
// occupancy field is uniform across the matched cells.
float ComputeLidarObservationScore(const ProbabilityGrid& probability_grid,
                                   const DiscreteScan2D& discrete_scan,
                                   int x_index_offset, int y_index_offset,
                                   double L_deg) {
  if (discrete_scan.empty()) return 0.f;

  const int m = static_cast<int>(discrete_scan.size());

  // L_deg = 0 would correspond to a perfectly reliable LiDAR; clamp to a
  // small positive value so the (1/L_deg) factor in Eq. (13) stays finite.
  const double L_deg_safe = std::max(L_deg, 1e-3);

  double sum_log_prob = 0.0;
  for (const Eigen::Array2i& xy_index : discrete_scan) {
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    double prob = probability_grid.GetProbability(proposed_xy_index);
    prob = std::max(prob, 1e-6);
    sum_log_prob += std::log(prob);
  }

  const double avg_log_prob = sum_log_prob / static_cast<double>(m);
  const double geometric_mean_prob = std::exp(avg_log_prob);

  const double exponent =
      -geometric_mean_prob * static_cast<double>(m) / L_deg_safe;
  const double clamped_exponent = std::max(exponent, -100.0);
  const double score = 1.0 - std::exp(clamped_exponent);

  return static_cast<float>(std::clamp(score, 0.0, 1.0));
}

// Computes the fused observation score S^F_obs (paper Eq. 14).
// Identical in form to Eq. (13) but driven by the fusion-degeneration
// adaptive weight F_deg and by the discretized fused (LiDAR + RGB-D) cloud.
float ComputeDepthObservationScore(const ProbabilityGrid& probability_grid,
                                   const DiscreteScan2D& discrete_scan,
                                   int x_index_offset, int y_index_offset,
                                   double F_deg) {
  // When the fused cloud is empty we cannot evaluate p(z^F | x, m).  A score
  // of 1.0 acts as the multiplicative identity in S = S^L_obs * S^F_obs *
  // S_mot (Eq. 17), effectively disabling the fusion factor for this frame.
  if (discrete_scan.empty()) {
    return 1.0f;
  }

  const int n = static_cast<int>(discrete_scan.size());

  const double F_deg_safe = std::max(F_deg, 1e-3);

  double sum_log_prob = 0.0;
  for (const Eigen::Array2i& xy_index : discrete_scan) {
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    double prob = probability_grid.GetProbability(proposed_xy_index);
    prob = std::max(prob, 1e-6);
    sum_log_prob += std::log(prob);
  }

  const double avg_log_prob = sum_log_prob / static_cast<double>(n);
  const double geometric_mean_prob = std::exp(avg_log_prob);

  const double exponent =
      -geometric_mean_prob * static_cast<double>(n) / F_deg_safe;
  const double clamped_exponent = std::max(exponent, -100.0);
  const double score = 1.0 - std::exp(clamped_exponent);

  return static_cast<float>(std::clamp(score, 0.0, 1.0));
}

}  // namespace

RealTimeCorrelativeScanMatcher2D::RealTimeCorrelativeScanMatcher2D(
    const proto::RealTimeCorrelativeScanMatcherOptions& options)
    : options_(options) {}

std::vector<Candidate2D>
RealTimeCorrelativeScanMatcher2D::GenerateExhaustiveSearchCandidates(
    const SearchParameters& search_parameters) const {
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + 1);
    const int num_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + 1);
    num_candidates += num_linear_x_candidates * num_linear_y_candidates;
  }
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         ++x_index_offset) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           ++y_index_offset) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

double RealTimeCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const Grid2D& grid,
    transform::Rigid2d* pose_estimate,
    const sensor::PointCloud& depth_camera_cloud, double R_deg, double G_deg,
    double L_deg, double F_deg, double phi) const {
  CHECK(pose_estimate != nullptr);

  // Implements Algorithm 2 of the paper (Degeneration-Guided Mapping).
  // Inputs: initial pose x_{t-1}+u_t, current LiDAR scan z^L_t, fused depth
  // scan z^F_t, occupancy grid m_{t-1}, and the five degeneration descriptors
  // (R_deg, G_deg, L_deg, F_deg, phi) returned by DegeneracyDetector.
  // The selected pose maximises Eq. (17): S = S^L_obs * S^F_obs * S_mot.

  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));

  const SearchParameters search_parameters(
      options_.linear_search_window(), options_.angular_search_window(),
      rotated_point_cloud, grid.limits().resolution());

  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);

  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      grid.limits(), rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));

  std::vector<DiscreteScan2D> depth_discrete_scans;
  if (!depth_camera_cloud.empty()) {
    const sensor::PointCloud rotated_depth_cloud = sensor::TransformPointCloud(
        depth_camera_cloud,
        transform::Rigid3f::Rotation(Eigen::AngleAxisf(
            initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));

    const std::vector<sensor::PointCloud> rotated_depth_scans =
        GenerateRotatedScans(rotated_depth_cloud, search_parameters);

    depth_discrete_scans = DiscretizeScans(
        grid.limits(), rotated_depth_scans,
        Eigen::Translation2f(initial_pose_estimate.translation().x(),
                             initial_pose_estimate.translation().y()));
  }

  std::vector<Candidate2D> candidates =
      GenerateExhaustiveSearchCandidates(search_parameters);

  ScoreCandidates(grid, discrete_scans, search_parameters, &candidates,
                  depth_discrete_scans, R_deg, G_deg, L_deg, F_deg, phi);

  if (candidates.empty()) {
    *pose_estimate = initial_pose_estimate;
    return 0.0;
  }

  const Candidate2D& best_candidate =
      *std::max_element(candidates.begin(), candidates.end());

  *pose_estimate = transform::Rigid2d(
      {initial_pose_estimate.translation().x() + best_candidate.x,
       initial_pose_estimate.translation().y() + best_candidate.y},
      initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));

  return best_candidate.score;
}

void RealTimeCorrelativeScanMatcher2D::ScoreCandidates(
    const Grid2D& grid, const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates,
    const std::vector<DiscreteScan2D>& depth_discrete_scans, double R_deg,
    double G_deg, double L_deg, double F_deg, double phi) const {
  // theta is the fixed Gaussian base-weight from DN-CSM [13] reused by the
  // paper (Sec. III-D, last paragraph).  The paper fixes theta = 0.5.
  constexpr double kTheta = 0.5;
  const double theta_sq = common::Pow2(kTheta);

  const bool has_depth_cloud = !depth_discrete_scans.empty();

  for (Candidate2D& candidate : *candidates) {
    // S^L_obs: paper Eq. (13).
    float S_obs_L = 0.0f;
    switch (grid.GetGridType()) {
      case GridType::PROBABILITY_GRID:
        S_obs_L = ComputeLidarObservationScore(
            static_cast<const ProbabilityGrid&>(grid),
            discrete_scans[candidate.scan_index], candidate.x_index_offset,
            candidate.y_index_offset, L_deg);
        break;
      case GridType::TSDF:
        LOG(WARNING) << "TSDF grid type not supported in this build";
        break;
    }

    // S^F_obs: paper Eq. (14).  When no fused scan is provided we set
    // S^F_obs = 1, which is the multiplicative identity in Eq. (17).
    float S_obs_F = 1.0f;
    if (has_depth_cloud) {
      switch (grid.GetGridType()) {
        case GridType::PROBABILITY_GRID:
          S_obs_F = ComputeDepthObservationScore(
              static_cast<const ProbabilityGrid&>(grid),
              depth_discrete_scans[candidate.scan_index],
              candidate.x_index_offset, candidate.y_index_offset, F_deg);
          break;
        case GridType::TSDF:
          S_obs_F = 1.0f;
          break;
      }
    }

    // Pose deviation components (paper Eq. 15).
    const double delta_x = candidate.x;
    const double delta_y = candidate.y;
    const double delta_theta = candidate.orientation;
    const double translation_dist = std::hypot(delta_x, delta_y);
    const double delta_x_t =
        std::sqrt(common::Pow2(delta_x) + common::Pow2(delta_y) +
                  common::Pow2(delta_theta));

    // delta = arctan(Δy/Δx) (paper Eq. 16, text below the equation).
    const double delta_direction =
        (translation_dist > 1e-6) ? std::atan2(delta_y, delta_x) : 0.0;

    // S_mot: paper Eq. (16).
    //   Term 1 :  R_deg * Δx_t / theta^2          (range-degeneration term)
    //   Term 2 :  G_deg * (sin(delta - phi) * sqrt(Δx^2 + Δy^2))^2 / theta^2
    //                                              (geometric-degeneration term)
    const double term1 = R_deg * delta_x_t / theta_sq;

    const double perpendicular_motion =
        std::sin(delta_direction - phi) * translation_dist;
    const double term2 = G_deg * common::Pow2(perpendicular_motion) / theta_sq;

    const double S_mot = std::exp(-(term1 + term2));

    // Joint score (paper Eq. 17): S = S^L_obs * S^F_obs * S_mot.
    candidate.score = S_obs_L * S_obs_F * S_mot;
  }
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer