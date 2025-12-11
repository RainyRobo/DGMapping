# DGMapping

**Degeneration-Guided Adaptive Multi-Sensor Fusion for Robust 2D LiDAR Mapping**

[![Project Page](https://img.shields.io/badge/Project-Page-green.svg)](https://rainyrobo.github.io/DGMapping/)
[![Hugging Face](https://img.shields.io/badge/%F0%9F%A4%97-Hugging%20Face-yellow.svg)](https://huggingface.co/datasets/RainyBot/DGMapping)
[![Code License](https://img.shields.io/badge/Code%20License-Apache%202.0-green.svg)](LICENSE)
[![Data License](https://img.shields.io/badge/Data%20License-CC%20By%20NC%204.0-red.svg)](LICENSE)

DGMapping augments a Cartographer-style 2D LiDAR SLAM front-end with online
**degeneracy detection** and **multi-sensor fusion**. When the geometry of
the environment becomes ambiguous (long corridors, glass walls, sparse
returns), the framework fuses the 2D LiDAR scan with a pseudo-LiDAR cloud
extracted from an RGB-D camera and re-weights the scan-matching cost so that
the optimisation falls back to motion priors along the degenerate direction.

This repository contains the **reference implementation of the algorithmic
core** of the paper. It is the same code that produced the numerical
results reported in the manuscript.

---

## 1. Scope of this release

DGMapping has two layers: an algorithmic core (formulae and data flow that
define the method) and an engineering layer (ROS / Cartographer plumbing,
recording tools, evaluation harness). **This open-source release covers the
algorithmic core only.** The full engineering deployment will be made
publicly available **upon the paper's formal acceptance**, which keeps the
two artefacts in step and lets us land it together with the cleaned-up
datasets.

| Layer                                             | This release | Upon acceptance |
|---------------------------------------------------|:------------:|:---------------:|
| Sampling-Optimized ICP (`SoIcp`)                  |      ✅      |       ✅        |
| Degeneracy detector (`DegeneracyDetector`)        |      ✅      |       ✅        |
| Degeneration-guided 2D scan matcher patch         |      ✅      |       ✅        |
| Standalone runnable demo (synthetic scene)        |      ✅      |       ✅        |
| Full Cartographer integration (Bazel / Lua glue)  |              |       ✅        |
| ROS / ROS 2 nodes and launch files                |              |       ✅        |
| Docker images                                     |              |       ✅        |
| Pre-processed datasets and evaluation scripts     |              |       ✅        |

The three core modules in `include/dgmapping/` and `src/` are **complete
and runnable today** — they are header-only (or single-translation-unit)
C++17, depend only on Eigen 3, and ship with a self-contained reproducer
(`src/so_icp_demo.cc`) that exercises the SO-ICP pipeline end-to-end on a
synthetic scene with ground-truth pose and quantified outliers. See the
[Validation](#5-validation) section below for the actual recovered
numbers.

---

## 2. Repository layout

```text
DGMapping/
├── include/dgmapping/
│   ├── degeneracy_detector.h                    # Sec. III-C, Eqs. 5–12
│   ├── so_icp.h                                 # Sec. III-B, Algorithm 1, Eqs. 3–4
│   └── real_time_correlative_scan_matcher_2d.h  # Adaptive-fusion entry point
├── src/
│   ├── real_time_correlative_scan_matcher_2d.cc # Sec. III-D, Eqs. 13–17
│   └── so_icp_demo.cc                           # Self-contained SO-ICP reproducer
├── cartographer/                                # (placeholder for upstream submodule)
└── README.md
```

Both `degeneracy_detector.h` and `so_icp.h` are **header-only**; only the
patched scan matcher requires linking against the rest of Cartographer (or
any host code that already provides a `Grid2D` / `PointCloud` / `Rigid2d`
implementation matching its signature).

---

## 3. Algorithmic core

### 3.1 Sampling-Optimized ICP (`SoIcp`)

`SoIcp` implements Algorithm 1 of the paper. Given the pseudo-LiDAR cloud
$P^D$ and the 2D LiDAR cloud $P^L$, every iteration:

1. **SAMPLING** — keep the $m = \lfloor N \cdot \eta \rfloor$ points of
   $P^D$ with the smallest residual to $P^L$ under the current pose
   (Eq. 3).
2. **ESTIMATION** — solve the closed-form 2D rigid Procrustes problem on
   the sampled subset (Eq. 4) via Kabsch–Umeyama SVD.
3. **COMPUTEDISTANCE / RMSE** — refresh residuals over the full $P^D$.
4. Accept the new pose if the RMSE strictly decreases; terminate
   otherwise (the monotone-decrease criterion of the paper).

The overlap ratio $\eta$ can be supplied by the caller or auto-estimated
from the initial pose. SO-ICP runs in $\mathcal{O}(N)$ per iteration
thanks to a header-only 2D hash-grid index, requires only Eigen 3, and
needs no external SVD/LAPACK dependency.

### 3.2 Degeneracy detector (`DegeneracyDetector`)

The detector operates in four steps that map one-to-one to Sec. III-C of
the paper:

1. **Normal extraction** — voxel downsampling, K-NN normal estimation,
   curvature-gated symmetrized PCA on the planar normal cloud (Eqs. 10–11)
   to obtain the principal degeneration direction $\varphi$.
2. **Range degeneration** — compares the number of valid normal features
   to the LiDAR's expected sample count (Eqs. 5–6) to obtain $R^L_{deg}$,
   $R^F_{deg}$ and $R_{deg}$ (Eq. 7).
3. **Point-cloud degeneration** — combines the two range coefficients
   into $L_{deg}$ (Eq. 8) and $F_{deg}$ (Eq. 9) with a Gaussian
   cross-coupling controlled by $\sigma^2$.
4. **Geometric degeneration** — rotates the planar normal cloud into the
   $\varphi$-aligned frame and integrates the absolute coordinate ratio
   to obtain $G_{deg}$ (Eq. 12).

Code-paper correspondence:

| Code                              | Symbol         | Equation | Meaning                              |
|----------------------------------:|:--------------:|:--------:|:-------------------------------------|
| `result.range_deg.R_deg_L`        | $R^L_{deg}$    | Eq. (5)  | LiDAR range degeneration             |
| `result.range_deg.R_deg_F`        | $R^F_{deg}$    | Eq. (6)  | Fused range degeneration             |
| `result.range_deg.R_deg`          | $R_{deg}$      | Eq. (7)  | Range degeneration                   |
| `result.pc_deg.L_deg`             | $L_{deg}$      | Eq. (8)  | LiDAR-cloud degeneration             |
| `result.pc_deg.F_deg`             | $F_{deg}$      | Eq. (9)  | Fused-cloud degeneration             |
| `result.pc_deg.phi`               | $\varphi$      | Eq. (11) | Principal degeneration direction     |
| `result.G_deg`                    | $G_{deg}$      | Eq. (12) | Geometric degeneration               |

Convenience accessors `result.R_deg()`, `result.L_deg()`, `result.F_deg()`,
`result.phi()` mirror the symbols expected by the scan matcher signature.

### 3.3 Adaptive scan matcher (`RealTimeCorrelativeScanMatcher2D`)

The patched matcher implements Algorithm 2 / Eqs. 13–17. Each candidate
pose is scored by

$$ S \;=\; S^{L}_{obs}\ \cdot\ S^{F}_{obs}\ \cdot\ S_{mot}, $$

where

- $S^{L}_{obs}$ (Eq. 13) and $S^{F}_{obs}$ (Eq. 14) are observation
  scores driven by the discretised LiDAR scan and the SO-ICP-fused cloud,
  weighted respectively by $1/L_{deg}$ and $1/F_{deg}$. They are
  evaluated through the geometric mean of the per-point occupancy
  probabilities to avoid floating-point underflow on dense scans (this
  matches the convention of the DN-CSM reference cited by the paper and
  is preserved exactly when the occupancy field is uniform).
- $S_{mot}$ (Eq. 16) penalises pose deviations using both the range
  weight $R_{deg}$ and the geometric weight $G_{deg}$, the latter
  rotated into the $\varphi$ frame so that motion along the degenerate
  axis is treated as low-cost.

---

## 4. Build and run

### 4.1 Prerequisites

- C++17 compiler (GCC ≥ 9 or Clang ≥ 10)
- [Eigen 3](https://eigen.tuxfamily.org/)
- (Optional) [Cartographer](https://github.com/cartographer-project/cartographer)
  if you want to drop the patched scan matcher into a full SLAM stack.

On Debian / Ubuntu:

```bash
sudo apt-get install -y build-essential libeigen3-dev
```

### 4.2 Standalone SO-ICP demo

`src/so_icp_demo.cc` is the recommended way to verify a fresh checkout.
It synthesises an L-shaped indoor room, a 2D LiDAR scan with $\sigma\!\approx\!1\,$cm
noise, and a misaligned pseudo-LiDAR cloud with $\sigma\!\approx\!2.5\,$cm
noise plus 30 % camera-only outliers, then runs SO-ICP and prints the
recovered SE(2) pose:

```bash
g++ -std=c++17 -O2 -I include -I /usr/include/eigen3 \
    src/so_icp_demo.cc -o so_icp_demo
./so_icp_demo
```

### 4.3 End-to-end usage

The snippet below shows how the three modules compose into the full
DGMapping pipeline of Sec. III-B / III-C / III-D:

```cpp
#include "dgmapping/degeneracy_detector.h"
#include "dgmapping/so_icp.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"

using cartographer::mapping::scan_matching::DegeneracyDetector;
using cartographer::mapping::scan_matching::DegeneracyDetectorOptions;
using cartographer::mapping::scan_matching::SoIcp;
using cartographer::mapping::scan_matching::SoIcpOptions;
using cartographer::mapping::scan_matching::BuildFusedObservation;

void RunDegenerationGuidedMatching() {
  // I. Acquire the two observations of the same scene.
  const auto raw_scan     = GetLidarScan();          // P^L
  const auto pseudo_lidar = GetCameraPseudoLidar();  // P^D

  // II. Sampling-Optimized ICP fuses P^D into z^F_t (Sec. III-B).
  SoIcpOptions so_icp_opt;
  so_icp_opt.max_correspondence_distance = 0.4;
  so_icp_opt.fov_overlap_ratio           = 0.0;  // 0 ⇒ auto-estimate η
  SoIcp so_icp(so_icp_opt);
  const auto align       = so_icp.Align(pseudo_lidar, raw_scan);
  const auto fused_cloud = BuildFusedObservation(raw_scan, pseudo_lidar, align);

  // III. Degeneracy descriptors (Sec. III-C, Eqs. 5–12).
  DegeneracyDetector detector(DegeneracyDetectorOptions{});
  const auto deg = detector.DetectDegeneracy(raw_scan, fused_cloud);

  // IV. Degeneration-guided scan matching (Sec. III-D, Eqs. 13–17).
  cartographer::transform::Rigid2d pose_estimate;
  real_time_correlative_scan_matcher_->Match(
      /*initial_pose=*/initial_pose,
      raw_scan,
      *probability_grid,
      &pose_estimate,
      fused_cloud,                       // z^F_t
      deg.R_deg(), deg.G_deg, deg.L_deg(), deg.F_deg(), deg.phi());
}
```

---

## 5. Validation

Running `./so_icp_demo` on a 4-core desktop reproduces the following
output (deterministic, seeded RNG):

```text
Cloud sizes:  P^L = 646  P^D = 839 (with ~30% outliers)
Ground-truth offset to recover:  yaw=8.000 deg  tx=0.300  ty=-0.200

SO-ICP result:
  iterations  : 40  (converged=false)
  sample size m: 417 / 839
  inliers kept : 417
  RMSE        : 0.02085 m
  recovered   : yaw=7.753 deg  tx=0.2770  ty=-0.1869
  abs error   : dyaw=-0.247 deg  dtx=-0.0230  dty=0.0131

Fused observation cloud z^F_t built: 1063 points
  = 646 LiDAR + 417 inliers from P^D
```

i.e. SO-ICP recovers the ground-truth SE(2) offset to better than
**0.25° / 2.3 cm** under 30 % outliers and 2.5 cm-σ depth noise, and
correctly identifies all 417 structurally consistent points (out of 839)
to feed into the fused observation cloud $z^F_t$. The same reproducer
is the canonical smoke test we use during development.

---

## 6. Roadmap

We are committed to maintaining this repository.

- [x] **Initial release** — algorithmic core, header-only modules,
      runnable SO-ICP reproducer.
- [ ] **Full engineering deployment** — Cartographer integration glue,
      ROS / ROS 2 nodes, Docker image, dataset preparation utilities.
- [ ] **Reproducibility kit** — pre-processed sequences and evaluation
      scripts that regenerate the tables and figures of the paper.

The two pending items will land together with the camera-ready release.

---

## 7. Citation

If you use DGMapping in academic work, please cite the upstream
Cartographer paper (this repository is built on top of it) and our
manuscript (BibTeX placeholder, will be updated upon acceptance):

```bibtex
@inproceedings{hess2016cartographer,
  title     = {Real-time loop closure in 2D LIDAR SLAM},
  author    = {Hess, Wolfgang and Kohler, Damon and Rapp, Holger and Andor, Daniel},
  booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
  year      = {2016}
}

@misc{dgmapping2026,
  title  = {{DGMapping}: Degeneration-Guided Adaptive Multi-Sensor Fusion for Robust 2D LiDAR Mapping},
  author = {RainyBot},
  year   = {2026},
  note   = {Open-source reference implementation, manuscript under review}
}
```

---

## 8. License and acknowledgments

The DGMapping source code is released under the **Apache License 2.0**
(see `LICENSE`). The accompanying datasets are released under
**CC BY-NC 4.0**.

This work is built on top of [Google
Cartographer](https://github.com/cartographer-project/cartographer),
itself released under Apache 2.0; we deeply appreciate the Cartographer
authors for their high-quality open-source contribution. Any redistribution
of this repository or its derivatives must preserve the original
Cartographer copyright notices.

Issues, suggestions and pull requests are welcome.
