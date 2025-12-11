# DGMapping: Degeneration-Guided Adaptive Multi-Sensor Fusion

[![Project Page](https://img.shields.io/badge/Project-Page-green.svg)](https://rainyrobo.github.io/DGMapping/)
[![Hugging Face](https://img.shields.io/badge/%F0%9F%A4%97-Hugging%20Face-yellow.svg)](https://huggingface.co/datasets/RainyBot/DGMapping)
[![Code License](https://img.shields.io/badge/Code%20License-Apache%202.0-green.svg)](LICENSE)
[![Data License](https://img.shields.io/badge/Data%20License-CC%20By%20NC%204.0-red.svg)](LICENSE)



## 📁 1. Directory Structure

The repository is organized to enable modular integration with existing SLAM frameworks (notably Google Cartographer).

```text
DGMapping/
├── cartographer/                                  # Upstream base framework
├── include/dgmapping/                             # [Core]: Degeneracy-Aware Fusion Modules
│   ├── degeneracy_detector.h                      # Degeneracy detection 
│   └── real_time_correlative_scan_matcher_2d.h    # Adaptive fusion interface for degeneracy handling
└── src/                                           
    └── real_time_correlative_scan_matcher_2d.cc   # Implementation of degeneration-aware CSM
```

## 📐 2. Core API & Mathematical Formulation

The core functionality is provided by the `DegeneracyDetector` class, which quantifies environmental reliability using descriptors derived from spectral analysis of normals.

### 2.1 Degeneracy descriptors

The system computes four primary descriptors that correspond to the equations in the paper:

| Code variable | Symbol | Equation | Physical interpretation |
|---------------:|:------:|:--------:|:-----------------------|
| `R_deg`  | $R_{deg}$ | Eq. (7)  | Range degeneration |
| `p_beta` | $G_{deg}$ | Eq. (12) | Geometric degeneration |
| `L_deg`  | $L_{deg}$ | Eq. (8)  | LiDAR degeneration |
| `F_deg`  | $F_{deg}$ | Eq. (9)  | Fusion degeneration |
| `phi`    | $\\varphi$ | Eq. (11) | Principal direction |

### 2.2 Integration workflow

The snippet below demonstrates a typical integration pattern for Scan-to-Map matching using DGMapping.

```cpp
#include "dgmapping/degeneracy_detector.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"

void RunDegeneracyGuidedMatching() {
    // I. Data acquisition
    std::vector<RangefinderPoint> raw_scan = GetLidarScan();
    std::vector<RangefinderPoint> depth_cloud = GetCameraPseudoLidar();

    // II. Degeneracy analysis (model construction)
    DegeneracyDetector detector(options);
    auto result = detector.DetectDegeneracy(raw_scan, depth_cloud);

    double R_deg = result.range_deg.R_deg;
    double G_deg = result.p_beta;
    double L_deg = result.pc_deg.L_deg;
    double F_deg = result.pc_deg.F_deg;
    double phi   = result.phi;

    // III. State estimation (optimization)
    transform::Rigid2d pose_estimate;
    transform::Rigid2d initial_pose = transform::Rigid2d::Translation({2.5, 2.5});

    double score = real_time_correlative_scan_matcher_->Match(
            initial_pose,
            raw_scan,
            *probability_grid,
            &pose_estimate,
            depth_cloud, // visual constraints P_D
            R_deg,       // range degeneration weight
            G_deg,       // geometric degeneration weight
            L_deg,       // LiDAR model penalty
            F_deg,       // fusion model penalty
            phi          // principal direction
    );
}
```

## 🚀 3. Deployment & Testing

To facilitate rapid algorithm verification, a standalone test suite is provided.

> **⚠️ Note:** The current test harness is designed for core algorithmic validation. A comprehensive engineering deployment (including full ROS integration, Docker support, and launch files) will be made publicly available **upon the paper's formal acceptance**.

### Prerequisites

- Eigen3
- C++17-compatible compiler (GCC/Clang)
- Cartographer (optional integration)

```bash
cd cartographer
git clone https://github.com/cartographer-project/cartographer.git
```

## 📅 4. Roadmap

We are committed to maintaining this repository. The current release covers the core algorithmic modules.

- [x] **Initial Release**: Core `DegeneracyDetector` and adaptive `ScanMatcher` source code.
- [ ] **Full Engineering Deployment**




## 👏 Acknowledgments

We deeply appreciate the open-source contribution of **Google Cartographer**, which serves as the robust backbone for our implementation. Our **DGMapping** framework is built upon the [cartographer](https://github.com/cartographer-project/cartographer) repository. 

We strictly adhere to the **Apache 2.0 License** of the original project. If you use our code, please also credit the original Cartographer authors:

> W. Hess, D. Kohler, H. Rapp, and D. Andor, "Real-time loop closure in 2D LIDAR SLAM," in *IEEE International Conference on Robotics and Automation (ICRA)*, 2016.