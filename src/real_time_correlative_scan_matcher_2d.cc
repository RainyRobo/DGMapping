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

float ComputeLidarObservationScore(const ProbabilityGrid& probability_grid,
                                   const DiscreteScan2D& discrete_scan,
                                   int x_index_offset, int y_index_offset,
                                   double L_deg) {
  if (discrete_scan.empty()) return 0.f;

  const int m = static_cast<int>(discrete_scan.size());

  const double L_deg_safe = std::max(L_deg, 0.01);

  double sum_log_prob = 0.0;
  for (const Eigen::Array2i& xy_index : discrete_scan) {
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    double prob = probability_grid.GetProbability(proposed_xy_index);
    prob = std::max(prob, 1e-6);
    sum_log_prob += std::log(prob);
  }

  const double avg_log_prob = sum_log_prob / static_cast<double>(m);
  const double avg_prob = std::exp(avg_log_prob);

  const double exponent = -avg_prob * static_cast<double>(m) / L_deg_safe;
  const double clamped_exponent = std::max(exponent, -100.0);
  const double score = 1.0 - std::exp(clamped_exponent);

  return static_cast<float>(std::clamp(score, 0.0, 1.0));
}

float ComputeDepthObservationScore(const ProbabilityGrid& probability_grid,
                                   const DiscreteScan2D& discrete_scan,
                                   int x_index_offset, int y_index_offset,
                                   double F_deg) {
  if (discrete_scan.empty()) {
    return 1.0f;
  }

  const int n = static_cast<int>(discrete_scan.size());

  const double F_deg_safe = std::max(F_deg, 0.01);

  double sum_log_prob = 0.0;
  for (const Eigen::Array2i& xy_index : discrete_scan) {
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    double prob = probability_grid.GetProbability(proposed_xy_index);
    prob = std::max(prob, 1e-6);
    sum_log_prob += std::log(prob);
  }

  const double avg_log_prob = sum_log_prob / static_cast<double>(n);
  const double avg_prob = std::exp(avg_log_prob);

  const double exponent = -avg_prob * static_cast<double>(n) / F_deg_safe;
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

  if (R_deg >= 0.99 && F_deg < 0.5) {
    *pose_estimate = initial_pose_estimate;
    return 0.1;
  }

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
  const double theta_sq = common::Pow2(0.5);

  const bool has_depth_cloud = !depth_discrete_scans.empty();

  for (Candidate2D& candidate : *candidates) {
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

    const double delta_x = candidate.x;
    const double delta_y = candidate.y;
    const double delta_theta = candidate.orientation;
    const double translation_dist = std::hypot(delta_x, delta_y);
    const double delta_x_t =
        std::sqrt(common::Pow2(delta_x) + common::Pow2(delta_y) +
                  common::Pow2(delta_theta));

    const double delta_direction =
        (translation_dist > 1e-6) ? std::atan2(delta_y, delta_x) : 0.0;

    const double term1 = R_deg * delta_x_t / theta_sq;

    const double sin_delta_minus_phi = std::sin(delta_direction - phi);
    const double perpendicular_motion = sin_delta_minus_phi * translation_dist;
    const double term2 = G_deg * common::Pow2(perpendicular_motion) / theta_sq;

    const double S_mot = std::exp(-(term1 + term2));

    candidate.score = S_obs_L * S_obs_F * S_mot;
  }
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer