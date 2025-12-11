#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_DEGENERACY_DETECTOR_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_DEGENERACY_DETECTOR_H_

#include <sys/stat.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace cartographer {
namespace mapping {
namespace scan_matching {

struct NormalFeatureResult {
  int total_points = 0;
  int valid_normal_count = 0;
  double normal_feature_ratio = 0.0;

  // Covariance analysis metrics
  double lambda1 = 0.0;
  double lambda2 = 0.0;
  double lambda3 = 0.0;
  double phi = 0.0;
  double normal_consistency = 1.0;

  std::vector<Eigen::Vector3f> downsampled_points;
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> points_with_normals;
};

struct RangeDegeneracyResult {
  double R_deg_L = 0.0;
  double R_deg_F = 0.0;
  double R_deg = 0.0;
};

struct PointCloudDegeneracyResult {
  double L_deg = 0.0;
  double F_deg = 0.0;

  double phi = 0.0;
  double lambda1 = 0.0;
  double lambda2 = 0.0;
};

struct DegeneracyResult {
  NormalFeatureResult lidar_features;
  NormalFeatureResult fusion_features;

  RangeDegeneracyResult range_deg;
  PointCloudDegeneracyResult pc_deg;

  // Legacy compatibility fields
  double p_alpha = 0.0;
  double p_beta = 0.0;
  double phi = 0.0;
  double lambda1 = 0.0;
  double lambda2 = 0.0;
  double lambda3 = 0.0;
  double normal_consistency = 1.0;

  double GetLdeg() const { return pc_deg.L_deg; }
  double GetFdeg() const { return pc_deg.F_deg; }
  double GetRdeg() const { return range_deg.R_deg; }
  double GetGdeg() const { return p_beta; }
};

struct VoxelKey {
  int x, y, z;

  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const {
    return ((std::size_t)key.x * 73856093) ^ ((std::size_t)key.y * 19349663) ^
           ((std::size_t)key.z * 83492791);
  }
};

template <typename T>
struct PointWithNormal {
  T point;
  Eigen::Vector3f normal;
  float curvature;
  bool valid_normal;
};

template <typename T>
struct Voxel {
  std::vector<T> points;
  Eigen::Vector3f centroid;
  Eigen::Vector3f normal;
  float curvature;
  bool valid;

  Voxel()
      : centroid(Eigen::Vector3f::Zero()),
        normal(Eigen::Vector3f::Zero()),
        curvature(0.0f),
        valid(false) {}
};

struct DegeneracyDetectorOptions {
  double voxel_size = 0.1;
  int min_points_per_voxel = 3;

  int k_neighbors = 10;
  double max_neighbor_distance = 0.5;

  double lidar_fov = 2 * M_PI;
  double lidar_angular_resolution = 0.25 * M_PI / 180.0;

  // Calculation parameters
  double sigma_squared = 0.5;

  // Thresholds
  double sparse_threshold = 20.0;     // Minimum density to consider non-sparse
  double corridor_threshold = 0.1;    // Eigenvalue ratio for corridor detection
  // planar_threshold removed — planar (p_gamma) handling disabled

  double normal_consistency_threshold = 0.8;
  double curvature_threshold = 0.1;
  double min_normal_quality = 0.3;

  // Debugging
  bool enable_debug = false;
  std::string debug_output_dir = "./debug_output";

  int GetExpectedPointCount() const {
    return static_cast<int>(lidar_fov / lidar_angular_resolution);
  }
};

struct DebugVisualization {
  std::vector<Eigen::Vector3f> original_points;
  std::vector<Eigen::Vector3f> downsampled_points;
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> points_with_normals;
  Eigen::Vector3f eigenvalues;
  Eigen::Matrix3f eigenvectors;
  Eigen::Vector3f centroid;
  DegeneracyResult result;
  std::string source_name;
};

// Simple PPM image writer for debugging (no external dependencies)
class DebugImageWriter {
 public:
  explicit DebugImageWriter(const std::string& output_dir)
      : output_dir_(output_dir), frame_id_(0) {
    mkdir(output_dir.c_str(), 0755);
  }

  void SaveVisualization(const DebugVisualization& viz,
                         const std::string& suffix = "") {
    std::string filename = GenerateFilename(viz.source_name, suffix);

    const int width = 800;
    const int height = 800;
    const int margin = 50;

    std::vector<unsigned char> image(width * height * 3, 255);

    // Compute bounds
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();

    for (const auto& p : viz.original_points) {
      min_x = std::min(min_x, p.x());
      max_x = std::max(max_x, p.x());
      min_y = std::min(min_y, p.y());
      max_y = std::max(max_y, p.y());
    }

    float range_x = max_x - min_x + 1e-6f;
    float range_y = max_y - min_y + 1e-6f;
    float scale = std::min((width - 2 * margin) / range_x,
                           (height - 2 * margin) / range_y);

    auto toPixel = [&](const Eigen::Vector3f& p) -> std::pair<int, int> {
      int px = margin + static_cast<int>((p.x() - min_x) * scale);
      int py = height - margin - static_cast<int>((p.y() - min_y) * scale);
      return {std::clamp(px, 0, width - 1), std::clamp(py, 0, height - 1)};
    };

    // Draw original points (light gray)
    for (const auto& p : viz.original_points) {
      auto [px, py] = toPixel(p);
      DrawCircle(image, width, height, px, py, 2, 200, 200, 200);
    }

    // Draw downsampled points (blue)
    for (const auto& p : viz.downsampled_points) {
      auto [px, py] = toPixel(p);
      DrawCircle(image, width, height, px, py, 4, 0, 100, 255);
    }

    // Draw normals (green arrows)
    for (const auto& [point, normal] : viz.points_with_normals) {
      auto [px, py] = toPixel(point);
      float normal_scale = 0.3f * scale;
      int nx = px + static_cast<int>(normal.x() * normal_scale);
      int ny = py - static_cast<int>(normal.y() * normal_scale);
      DrawLine(image, width, height, px, py, nx, ny, 0, 200, 0);
    }

    // Draw centroid (red)
    auto [cx, cy] = toPixel(viz.centroid);
    DrawCircle(image, width, height, cx, cy, 6, 255, 0, 0);

    // Draw principal directions (eigenvectors)
    float eigen_scale = 50.0f;
    for (int i = 0; i < 2; ++i) {
      Eigen::Vector3f dir = viz.eigenvectors.col(i) *
                            std::sqrt(viz.eigenvalues(i)) * eigen_scale / scale;
      int ex1 = cx + static_cast<int>(dir.x() * scale);
      int ey1 = cy - static_cast<int>(dir.y() * scale);
      int ex2 = cx - static_cast<int>(dir.x() * scale);
      int ey2 = cy + static_cast<int>(dir.y() * scale);

      if (i == 0) {
        DrawLine(image, width, height, cx, cy, ex1, ey1, 255, 0,
                 255);  // Magenta
        DrawLine(image, width, height, cx, cy, ex2, ey2, 255, 0, 255);
      } else {
        DrawLine(image, width, height, cx, cy, ex1, ey1, 0, 255, 255);  // Cyan
        DrawLine(image, width, height, cx, cy, ex2, ey2, 0, 255, 255);
      }
    }

    // Draw text info
    DrawText(image, width, height, 10, 20, viz.source_name);

        std::ostringstream info;
        info << std::fixed << std::setprecision(3);
        info << "p_alpha=" << viz.result.p_alpha << " p_beta=" << viz.result.p_beta;
        DrawText(image, width, height, 10, 40, info.str());

    std::ostringstream phi_info;
    phi_info << std::fixed << std::setprecision(1);
    phi_info << "phi=" << (viz.result.phi * 180.0 / M_PI) << " deg";
    DrawText(image, width, height, 10, 60, phi_info.str());

    std::ostringstream points_info;
    points_info << "Original: " << viz.original_points.size()
                << " Downsampled: " << viz.downsampled_points.size();
    DrawText(image, width, height, 10, 80, points_info.str());

    DrawLegend(image, width, height);
    WritePPM(filename, image, width, height);
    SaveCSV(viz, filename);
  }

  void IncrementFrame() { ++frame_id_; }
  void SetFrameId(int frame_id) { frame_id_ = frame_id; }
  int GetFrameId() const { return frame_id_; }

 private:
  std::string GenerateFilename(const std::string& source,
                               const std::string& suffix) {
    std::ostringstream ss;
    ss << output_dir_ << "/frame_" << std::setfill('0') << std::setw(6)
       << frame_id_ << "_" << source;
    if (!suffix.empty()) {
      ss << "_" << suffix;
    }
    return ss.str();
  }

  void DrawCircle(std::vector<unsigned char>& img, int w, int h, int cx, int cy,
                  int r, unsigned char R, unsigned char G, unsigned char B) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (dx * dx + dy * dy <= r * r) {
          int x = cx + dx;
          int y = cy + dy;
          if (x >= 0 && x < w && y >= 0 && y < h) {
            int idx = (y * w + x) * 3;
            img[idx] = R;
            img[idx + 1] = G;
            img[idx + 2] = B;
          }
        }
      }
    }
  }

  void DrawLine(std::vector<unsigned char>& img, int w, int h, int x0, int y0,
                int x1, int y1, unsigned char R, unsigned char G,
                unsigned char B) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
      if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h) {
        int idx = (y0 * w + x0) * 3;
        img[idx] = R;
        img[idx + 1] = G;
        img[idx + 2] = B;
      }
      if (x0 == x1 && y0 == y1) break;
      int e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x0 += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y0 += sy;
      }
    }
  }

  void DrawText(std::vector<unsigned char>& img, int w, int h, int x, int y,
                const std::string& text) {
    for (size_t i = 0; i < text.size() && i < 50; ++i) {
      DrawCircle(img, w, h, x + i * 8, y, 1, 0, 0, 0);
    }
  }

  void DrawLegend(std::vector<unsigned char>& img, int w, int h) {
    int lx = w - 180;
    int ly = h - 120;

    // Background
    for (int dy = 0; dy < 110; ++dy) {
      for (int dx = 0; dx < 170; ++dx) {
        int idx = ((ly + dy) * w + lx + dx) * 3;
        if (idx >= 0 && idx < (int)img.size() - 2) {
          img[idx] = 240;
          img[idx + 1] = 240;
          img[idx + 2] = 240;
        }
      }
    }

    DrawCircle(img, w, h, lx + 10, ly + 15, 2, 200, 200, 200);
    DrawCircle(img, w, h, lx + 10, ly + 35, 4, 0, 100, 255);
    DrawLine(img, w, h, lx + 5, ly + 55, lx + 15, ly + 55, 0, 200, 0);
    DrawCircle(img, w, h, lx + 10, ly + 75, 6, 255, 0, 0);
    DrawLine(img, w, h, lx + 5, ly + 95, lx + 15, ly + 95, 255, 0, 255);
  }

  void WritePPM(const std::string& filename,
                const std::vector<unsigned char>& img, int w, int h) {
    std::string ppm_file = filename + ".ppm";
    std::ofstream out(ppm_file, std::ios::binary);
    if (!out) return;

    out << "P6\n" << w << " " << h << "\n255\n";
    out.write(reinterpret_cast<const char*>(img.data()), img.size());
  }

  void SaveCSV(const DebugVisualization& viz, const std::string& filename) {
    std::string csv_orig = filename + "_original.csv";
    std::ofstream out_orig(csv_orig);
    if (out_orig) {
      out_orig << "x,y,z\n";
      for (const auto& p : viz.original_points) {
        out_orig << p.x() << "," << p.y() << "," << p.z() << "\n";
      }
    }

    std::string csv_down = filename + "_downsampled.csv";
    std::ofstream out_down(csv_down);
    if (out_down) {
      out_down << "x,y,z,nx,ny,nz\n";
      for (const auto& [point, normal] : viz.points_with_normals) {
        out_down << point.x() << "," << point.y() << "," << point.z() << ","
                 << normal.x() << "," << normal.y() << "," << normal.z()
                 << "\n";
      }
    }

    std::string csv_result = filename + "_result.csv";
    std::ofstream out_result(csv_result);
    if (out_result) {
      out_result << "parameter,value\n";
      out_result << "p_alpha," << viz.result.p_alpha << "\n";
      out_result << "p_beta," << viz.result.p_beta << "\n";
      out_result << "phi," << viz.result.phi << "\n";
      out_result << "phi_deg," << (viz.result.phi * 180.0 / M_PI) << "\n";
      out_result << "lambda1," << viz.result.lambda1 << "\n";
      out_result << "lambda2," << viz.result.lambda2 << "\n";
      out_result << "lambda3," << viz.result.lambda3 << "\n";
      out_result << "normal_consistency," << viz.result.normal_consistency
                 << "\n";
      out_result << "centroid_x," << viz.centroid.x() << "\n";
      out_result << "centroid_y," << viz.centroid.y() << "\n";
      out_result << "centroid_z," << viz.centroid.z() << "\n";
    }
  }

  std::string output_dir_;
  int frame_id_;
};

// Grid-based spatial index for fast nearest neighbor search
template <typename T>
class SpatialIndex {
 public:
  explicit SpatialIndex(double cell_size) : cell_size_(cell_size) {}

  void AddPoint(const T& point, size_t index) {
    VoxelKey key = GetKey(point);
    cells_[key].push_back(index);
  }

  void Build(const std::vector<T>& points) {
    cells_.clear();
    for (size_t i = 0; i < points.size(); ++i) {
      AddPoint(points[i], i);
    }
  }

  std::vector<size_t> FindKNearest(const T& query, const std::vector<T>& points,
                                   int k, double max_distance) const {
    VoxelKey center = GetKey(query);
    int search_radius = static_cast<int>(std::ceil(max_distance / cell_size_));

    std::vector<std::pair<double, size_t>> candidates;

    for (int dx = -search_radius; dx <= search_radius; ++dx) {
      for (int dy = -search_radius; dy <= search_radius; ++dy) {
        for (int dz = -search_radius; dz <= search_radius; ++dz) {
          VoxelKey key{center.x + dx, center.y + dy, center.z + dz};
          auto it = cells_.find(key);
          if (it != cells_.end()) {
            for (size_t idx : it->second) {
              double dist = Distance(query, points[idx]);
              if (dist <= max_distance && dist > 1e-10) {
                candidates.emplace_back(dist, idx);
              }
            }
          }
        }
      }
    }

    std::sort(candidates.begin(), candidates.end());
    std::vector<size_t> result;
    for (size_t i = 0; i < std::min((size_t)k, candidates.size()); ++i) {
      result.push_back(candidates[i].second);
    }
    return result;
  }

 private:
  VoxelKey GetKey(const T& point) const {
    return VoxelKey{
        static_cast<int>(std::floor(point.position.x() / cell_size_)),
        static_cast<int>(std::floor(point.position.y() / cell_size_)),
        static_cast<int>(std::floor(point.position.z() / cell_size_))};
  }

  double Distance(const T& a, const T& b) const {
    return (a.position - b.position).norm();
  }

  double cell_size_;
  std::unordered_map<VoxelKey, std::vector<size_t>, VoxelKeyHash> cells_;
};

/**
 * @brief Degeneracy detector using normal vector feature extraction.
 * Implements normal feature extraction, range degeneracy, and point cloud
 * degeneracy calculations based on multi-sensor input.
 */
class DegeneracyDetector {
 public:
  explicit DegeneracyDetector(const DegeneracyDetectorOptions& options)
      : options_(options) {
    if (options_.enable_debug) {
      debug_writer_ =
          std::make_unique<DebugImageWriter>(options_.debug_output_dir);
    }
  }

  // Extracts normal vector features from point cloud (Downsampling -> K-NN ->
  // PCA)
  template <typename T>
  NormalFeatureResult ExtractNormalFeatures(
      const std::vector<T>& points, const std::string& source_name = "lidar") {
    NormalFeatureResult result;
    result.total_points = static_cast<int>(points.size());

    if (points.size() < 10) {
      return result;
    }

    auto downsampled = VoxelDownsample(points);
    auto points_with_normals = EstimateNormals(downsampled);

    int valid_count = 0;
    for (const auto& pn : points_with_normals) {
      if (pn.valid_normal && pn.curvature < options_.curvature_threshold) {
        valid_count++;
        result.points_with_normals.emplace_back(pn.point.position, pn.normal);
      }
      result.downsampled_points.push_back(pn.point.position);
    }
    result.valid_normal_count = valid_count;
    result.normal_feature_ratio =
        static_cast<double>(valid_count) /
        std::max(1, static_cast<int>(downsampled.size()));

    auto [centroid, covariance] = ComputeCovarianceMatrix(downsampled);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    Eigen::Vector3f eigenvalues = solver.eigenvalues();
    Eigen::Matrix3f eigenvectors = solver.eigenvectors();

    // Sort eigenvalues (descending)
    std::vector<std::pair<float, int>> eigen_pairs;
    for (int i = 0; i < 3; ++i) {
      eigen_pairs.emplace_back(eigenvalues(i), i);
    }
    std::sort(eigen_pairs.rbegin(), eigen_pairs.rend());

    result.lambda1 = eigen_pairs[0].first;
    result.lambda2 = eigen_pairs[1].first;
    result.lambda3 = eigen_pairs[2].first;

    Eigen::Vector3f principal_direction =
        eigenvectors.col(eigen_pairs[0].second);
    result.phi = std::atan2(principal_direction.y(), principal_direction.x());
    result.normal_consistency = ComputeNormalConsistency(points_with_normals);

    return result;
  }

  // Calculate range degeneracy based on valid normal ratios
  RangeDegeneracyResult CalculateRangeDegeneracy(
      const NormalFeatureResult& lidar_features,
      const NormalFeatureResult& fusion_features) const {
    RangeDegeneracyResult result;

    // Expected point count based on LiDAR parameters
    double expected_count =
        options_.lidar_fov / options_.lidar_angular_resolution;

    double ratio_L =
        static_cast<double>(lidar_features.valid_normal_count) / expected_count;
    result.R_deg_L = std::clamp(1.0 - ratio_L, 0.0, 1.0);

    double ratio_F = static_cast<double>(fusion_features.valid_normal_count) /
                     expected_count;
    result.R_deg_F = std::clamp(1.0 - ratio_F, 0.0, 1.0);

    result.R_deg = result.R_deg_L * result.R_deg_F;

    return result;
  }

  // Calculate combined point cloud degeneracy (L_deg, F_deg)
  PointCloudDegeneracyResult CalculatePointCloudDegeneracy(
      const RangeDegeneracyResult& range_deg,
      const NormalFeatureResult& lidar_features) const {
    PointCloudDegeneracyResult result;

    // Compute fusion degeneracy factor
    double exponent = 0.0;
    if (range_deg.R_deg_F > 1e-6) {
      exponent =
          -range_deg.R_deg_L / (options_.sigma_squared * range_deg.R_deg_F);
    } else {
      exponent = -1e6;  // Force exp to approx 0
    }
    result.F_deg = range_deg.R_deg_F * std::exp(exponent);
    result.F_deg = std::clamp(result.F_deg, 0.0, 1.0);

    result.L_deg = range_deg.R_deg_L * (1.0 - result.F_deg);
    result.L_deg = std::clamp(result.L_deg, 0.0, 1.0);

    result.phi = lidar_features.phi;
    result.lambda1 = lidar_features.lambda1;
    result.lambda2 = lidar_features.lambda2;

    return result;
  }

  /**
   * @brief Calculate geometric degeneracy (G_deg)
   * Evaluates spread of points along and perpendicular to the principal
   * direction.
   */
  template <typename T>
  double CalculateGeometricDegeneracy(const std::vector<T>& points,
                                      double phi) const {
    if (points.size() < 3) {
      return 0.0;
    }

    // Rotate coordinate system to align principal direction with x-axis
    double cos_phi = std::cos(-phi);
    double sin_phi = std::sin(-phi);

    Eigen::Vector2d centroid(0.0, 0.0);
    for (const auto& point : points) {
      centroid.x() += static_cast<double>(point.position.x());
      centroid.y() += static_cast<double>(point.position.y());
    }
    centroid /= static_cast<double>(points.size());

    double sum_abs_x = 0.0;  // Along principal direction
    double sum_abs_y = 0.0;  // Perpendicular

    for (const auto& point : points) {
      double px = static_cast<double>(point.position.x()) - centroid.x();
      double py = static_cast<double>(point.position.y()) - centroid.y();

      double x_rot = cos_phi * px - sin_phi * py;
      double y_rot = sin_phi * px + cos_phi * py;

      sum_abs_x += std::abs(x_rot);
      sum_abs_y += std::abs(y_rot);
    }

    if (sum_abs_y < 1e-10) {
      return 1.0;
    }

    double ratio = sum_abs_x / sum_abs_y;
    double G_deg = 1.0 - std::exp(-ratio);

    return std::clamp(G_deg, 0.0, 1.0);
  }

  // Main detection workflow
  template <typename T>
  DegeneracyResult DetectDegeneracy(const std::vector<T>& lidar_points,
                                    const std::vector<T>& fusion_points) {
    DegeneracyResult result;

    result.lidar_features = ExtractNormalFeatures(lidar_points, "lidar");

    if (!fusion_points.empty()) {
      result.fusion_features = ExtractNormalFeatures(fusion_points, "fusion");
    } else {
      result.fusion_features = result.lidar_features;
    }

    result.range_deg =
        CalculateRangeDegeneracy(result.lidar_features, result.fusion_features);

    result.pc_deg =
        CalculatePointCloudDegeneracy(result.range_deg, result.lidar_features);

    // Fill legacy fields
    result.p_alpha = result.range_deg.R_deg;
    result.phi = result.pc_deg.phi;
    result.lambda1 = result.lidar_features.lambda1;
    result.lambda2 = result.lidar_features.lambda2;
    result.lambda3 = result.lidar_features.lambda3;
    result.normal_consistency = result.lidar_features.normal_consistency;


    // ---------------------------------------------------

    // Calculate geometric degeneracy (p_beta)
    result.p_beta = CalculateGeometricDegeneracy(lidar_points, result.phi);

    // planar degeneracy (p_gamma) removed — not used for 2D workflow

    if (options_.enable_debug) {
      SaveDebugVisualization(lidar_points, result.lidar_features, "lidar",
                             result);
      if (!fusion_points.empty()) {
        SaveDebugVisualization(fusion_points, result.fusion_features, "fusion",
                               result);
      }
      debug_writer_->IncrementFrame();
    }

    return result;
  }

  // Legacy 2D detection interface
  template <typename T>
  DegeneracyResult Detect2D(const std::vector<T>& points,
                            const std::string& source_name = "lidar") {
    std::vector<T> empty_fusion;
    auto result = DetectDegeneracy(points, empty_fusion);

    // Adjust for single source
    result.range_deg.R_deg_F = result.range_deg.R_deg_L;
    result.range_deg.R_deg = result.range_deg.R_deg_L;

    result.pc_deg.F_deg = 0.0;
    result.pc_deg.L_deg = result.range_deg.R_deg_L;

    result.p_alpha = result.range_deg.R_deg_L;

    return result;
  }

  // Legacy 3D detection interface
  template <typename T>
  DegeneracyResult Detect3D(const std::vector<T>& points,
                            const std::string& source_name = "fused") {
    return Detect2D(points, source_name);
  }

  template <typename T>
  std::pair<DegeneracyResult, DegeneracyResult> DetectBoth(
      const std::vector<T>& lidar_points, const std::vector<T>& fused_points) {
    auto combined_result = DetectDegeneracy(lidar_points, fused_points);

    DegeneracyResult lidar_result = combined_result;
    DegeneracyResult fused_result = combined_result;

    lidar_result.p_alpha = combined_result.range_deg.R_deg_L;
    fused_result.p_alpha = combined_result.range_deg.R_deg_F;

    return {lidar_result, fused_result};
  }

  // Fuse results from multiple sensors (Legacy)
  DegeneracyResult Fuse(const DegeneracyResult& lidar_result,
                        const DegeneracyResult& depth_result,
                        double lidar_weight = 0.5) const {
    DegeneracyResult result;
    double depth_weight = 1.0 - lidar_weight;

    result.p_alpha = lidar_weight * lidar_result.p_alpha +
                     depth_weight * depth_result.p_alpha;
    result.p_beta =
      lidar_weight * lidar_result.p_beta + depth_weight * depth_result.p_beta;

    if (lidar_result.p_beta < depth_result.p_beta) {
      result.phi = lidar_result.phi;
    } else {
      result.phi = depth_result.phi;
    }

    result.lambda1 = lidar_weight * lidar_result.lambda1 +
                     depth_weight * depth_result.lambda1;
    result.lambda2 = lidar_weight * lidar_result.lambda2 +
                     depth_weight * depth_result.lambda2;
    result.lambda3 = lidar_weight * lidar_result.lambda3 +
                     depth_weight * depth_result.lambda3;

    result.normal_consistency = lidar_weight * lidar_result.normal_consistency +
                                depth_weight * depth_result.normal_consistency;

    return result;
  }

  void SetDebugEnabled(bool enabled) {
    options_.enable_debug = enabled;
    if (enabled && !debug_writer_) {
      debug_writer_ =
          std::make_unique<DebugImageWriter>(options_.debug_output_dir);
    }
  }

  void SetDebugOutputDir(const std::string& dir) {
    options_.debug_output_dir = dir;
    if (options_.enable_debug) {
      debug_writer_ = std::make_unique<DebugImageWriter>(dir);
    }
  }

  void SetDebugFrameId(int frame_id) {
    if (debug_writer_) {
      debug_writer_->SetFrameId(frame_id);
    }
  }

 private:
  template <typename T>
  void SaveDebugVisualization(const std::vector<T>& original_points,
                              const NormalFeatureResult& features,
                              const std::string& source_name,
                              const DegeneracyResult& result) {
    DebugVisualization viz;
    viz.source_name = source_name;

    for (const auto& p : original_points) {
      viz.original_points.push_back(p.position);
    }

    viz.downsampled_points = features.downsampled_points;
    viz.points_with_normals = features.points_with_normals;

    viz.eigenvalues =
        Eigen::Vector3f(features.lambda1, features.lambda2, features.lambda3);
    viz.centroid = Eigen::Vector3f::Zero();
    if (!features.downsampled_points.empty()) {
      for (const auto& p : features.downsampled_points) {
        viz.centroid += p;
      }
      viz.centroid /= features.downsampled_points.size();
    }

    if (!features.downsampled_points.empty()) {
      Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
      for (const auto& p : features.downsampled_points) {
        Eigen::Vector3f centered = p - viz.centroid;
        cov += centered * centered.transpose();
      }
      cov /= features.downsampled_points.size();

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
      viz.eigenvectors = solver.eigenvectors();
    }

    viz.result = result;
    debug_writer_->SaveVisualization(viz, source_name == "lidar" ? "2d" : "3d");
  }

  template <typename T>
  std::vector<T> VoxelDownsample(const std::vector<T>& points) const {
    std::unordered_map<VoxelKey, Voxel<T>, VoxelKeyHash> voxel_grid;

    for (const auto& point : points) {
      VoxelKey key{static_cast<int>(
                       std::floor(point.position.x() / options_.voxel_size)),
                   static_cast<int>(
                       std::floor(point.position.y() / options_.voxel_size)),
                   static_cast<int>(
                       std::floor(point.position.z() / options_.voxel_size))};
      voxel_grid[key].points.push_back(point);
    }

    std::vector<T> downsampled;
    for (auto& [key, voxel] : voxel_grid) {
      if (voxel.points.size() >=
          static_cast<size_t>(options_.min_points_per_voxel)) {
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (const auto& p : voxel.points) {
          centroid += p.position;
        }
        centroid /= voxel.points.size();

        T rep_point;
        rep_point.position = centroid;
        downsampled.push_back(rep_point);
      }
    }

    return downsampled;
  }

  template <typename T>
  std::vector<PointWithNormal<T>> EstimateNormals(
      const std::vector<T>& points) const {
    std::vector<PointWithNormal<T>> result(points.size());

    if (points.size() < static_cast<size_t>(options_.k_neighbors)) {
      for (size_t i = 0; i < points.size(); ++i) {
        result[i].point = points[i];
        result[i].normal = Eigen::Vector3f::UnitZ();
        result[i].curvature = 0.0f;
        result[i].valid_normal = false;
      }
      return result;
    }

    SpatialIndex<T> index(options_.max_neighbor_distance);
    index.Build(points);

    for (size_t i = 0; i < points.size(); ++i) {
      result[i].point = points[i];

      auto neighbors =
          index.FindKNearest(points[i], points, options_.k_neighbors,
                             options_.max_neighbor_distance);

      if (neighbors.size() < 3) {
        result[i].normal = Eigen::Vector3f::UnitZ();
        result[i].curvature = 0.0f;
        result[i].valid_normal = false;
        continue;
      }

      Eigen::Vector3f centroid = points[i].position;
      for (size_t idx : neighbors) {
        centroid += points[idx].position;
      }
      centroid /= (neighbors.size() + 1);

      Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
      cov += (points[i].position - centroid) *
             (points[i].position - centroid).transpose();
      for (size_t idx : neighbors) {
        Eigen::Vector3f diff = points[idx].position - centroid;
        cov += diff * diff.transpose();
      }
      cov /= (neighbors.size() + 1);

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
      Eigen::Vector3f eigenvalues = solver.eigenvalues();
      Eigen::Matrix3f eigenvectors = solver.eigenvectors();

      int min_idx = 0;
      float min_val = eigenvalues(0);
      for (int j = 1; j < 3; ++j) {
        if (eigenvalues(j) < min_val) {
          min_val = eigenvalues(j);
          min_idx = j;
        }
      }

      result[i].normal = eigenvectors.col(min_idx);

      if (result[i].normal.z() < 0) {
        result[i].normal = -result[i].normal;
      }

      float sum_eigen = eigenvalues.sum();
      result[i].curvature = (sum_eigen > 1e-10f) ? min_val / sum_eigen : 0.0f;
      result[i].valid_normal = true;
    }

    return result;
  }

  template <typename T>
  std::pair<Eigen::Vector3f, Eigen::Matrix3f> ComputeCovarianceMatrix(
      const std::vector<T>& points) const {
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const auto& p : points) {
      centroid += p.position;
    }
    centroid /= points.size();

    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const auto& p : points) {
      Eigen::Vector3f diff = p.position - centroid;
      covariance += diff * diff.transpose();
    }
    covariance /= points.size();

    return {centroid, covariance};
  }

  template <typename T>
  double ComputeNormalConsistency(
      const std::vector<PointWithNormal<T>>& points) const {
    if (points.empty()) return 1.0;

    Eigen::Vector3f mean_normal = Eigen::Vector3f::Zero();
    int valid_count = 0;
    for (const auto& p : points) {
      if (p.valid_normal) {
        mean_normal += p.normal;
        ++valid_count;
      }
    }

    if (valid_count < 2) return 1.0;

    mean_normal.normalize();

    double consistency_sum = 0.0;
    for (const auto& p : points) {
      if (p.valid_normal) {
        consistency_sum += std::abs(p.normal.dot(mean_normal));
      }
    }

    return consistency_sum / valid_count;
  }

  template <typename T>
  double ComputeConvexHullArea2D(const std::vector<T>& points) const {
    if (points.empty()) return 0.0;

    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();

    for (const auto& p : points) {
      min_x = std::min(min_x, p.position.x());
      max_x = std::max(max_x, p.position.x());
      min_y = std::min(min_y, p.position.y());
      max_y = std::max(max_y, p.position.y());
    }

    return (max_x - min_x) * (max_y - min_y);
  }

  template <typename T>
  double ComputeBoundingBoxVolume(const std::vector<T>& points) const {
    if (points.empty()) return 0.0;

    Eigen::Vector3f min_pt =
        Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    Eigen::Vector3f max_pt =
        Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());

    for (const auto& p : points) {
      min_pt = min_pt.cwiseMin(p.position);
      max_pt = max_pt.cwiseMax(p.position);
    }

    Eigen::Vector3f extent = max_pt - min_pt;
    return extent.x() * extent.y() * std::max(extent.z(), 0.01f);
  }

  DegeneracyDetectorOptions options_;
  std::unique_ptr<DebugImageWriter> debug_writer_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_DEGENERACY_DETECTOR_H_