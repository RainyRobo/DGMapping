# `cartographer/` (placeholder)

This directory is intentionally empty in the algorithmic-core release of
DGMapping.

The patched scan matcher
[`src/real_time_correlative_scan_matcher_2d.cc`](../src/real_time_correlative_scan_matcher_2d.cc)
is a drop-in replacement for the file of the same name in
[Google Cartographer](https://github.com/cartographer-project/cartographer):

```
cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.cc
```

To rebuild Cartographer with DGMapping's degeneracy-aware matching:

1. Clone Cartographer into this directory:
   ```bash
   git clone https://github.com/cartographer-project/cartographer.git ./src
   ```
2. Overwrite the corresponding file with our patched version.
3. Build Cartographer as usual (Bazel or CMake).
4. To enable the patched-matcher target inside this repo's CMake build,
   pass `-DDGMAPPING_BUILD_SCAN_MATCHER=ON` once Cartographer is
   discoverable via `find_package(cartographer)`.

The full pipeline integration (RGB-D producer, `LocalTrajectoryBuilder2D`
wiring, ROS bringup) is **not** part of this open-source release; only the
algorithmic core needed to reproduce the paper's contributions is.
