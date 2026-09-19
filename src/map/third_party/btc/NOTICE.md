# BTC Descriptor Core Notice

This directory contains a local extraction of the BTC descriptor core from:

- Repository: https://github.com/hku-mars/btc_descriptor
- Upstream commit: `742af157036144edad9a8350330c0158ea1a40d5`
- Upstream files used: `include/btc.h`, `src/btc.cpp`, `config/config_indoor.yaml`, `config/config_outdoor.yaml`

The upstream repository does not declare a concrete license at this commit:

- No `LICENSE` file was present in the cloned upstream tree.
- `package.xml` contains `<license>TODO</license>`.

This source is included locally at the user's request for BTC descriptor
integration. Do not assume MIT, GPL, BSD, or another license unless upstream
publishes one and the project updates this notice.

## Local extraction

Kept:

- `ConfigSetting`, `BinaryDescriptor`, `BTC`, `Plane`, `BTCMatchList`,
  `VOXEL_LOC`, `BTC_LOC`, `OctoTree`
- `BtcDescManager::GenerateBtcDescs`
- `BtcDescManager::AddBtcDescs`
- Voxel plane extraction, binary descriptor extraction, non-maximum
  suppression, BTC triangle generation, and descriptor database insertion

Removed:

- ROS 1 publishers and visualization helpers
- YAML loading through OpenCV
- Ceres plane ICP
- Loop-search/candidate-verification path
- `<execution>` parallel loops, to avoid introducing a TBB dependency
- Raw projection-grid arrays; local code uses RAII `std::vector` containers
  instead.

Local robustness fixes:

- Initialize previously uninitialized config thresholds and descriptor fields.
- Store `BTC::frame_number_` as `uint64_t` so submap IDs are not constrained to
  `unsigned short`.
- Keep one `history_binary_list_` entry and one `plane_cloud_vec_` entry for
  every `GenerateBtcDescs` call, including empty inputs.
- Return empty descriptors for null/empty clouds, empty plane lists, invalid
  projection settings, too few binary vertices, and short KNN results.
- Release voxel-map `OctoTree` allocations through scope cleanup if descriptor
  extraction throws.
- Avoid reverse-iterator underflow when there are fewer than two planes.
- Replace the generic Eigen solver with `SelfAdjointEigenSolver` for symmetric
  covariance matrices and reject failed/non-finite solves.
- Clamp projection grid indices and binary summary byte values.
- Throw a clear error instead of silently returning when projected coordinates
  would overflow integer grid dimensions or exceed the local 4,000,000-cell
  projection limit.
- Validate voxel-grid coordinates before converting them to `int64_t`.
- Initialize `BTC::angle_` to zero because upstream triangle generation used
  uninitialized normal vectors for angle computation.
- Canonically order input points and break plane-ranking ties so point ordering
  does not change covariance accumulation or selected projection planes.
- Unite already-labeled compatible plane groups instead of leaving connected
  planes split by traversal order.
- Use an orthonormal projection basis with a stable reference axis; orient plane
  normals toward the sensor origin to avoid signed-bin flips on vertical walls.
- Keep a deterministic representative of equal-score neighboring corners during
  non-maximum suppression instead of discarding all of them.

These extraction changes use `btc.extraction_revision: 2` in map metadata.
Legacy descriptors must be rebuilt from their saved submap PCDs before matching
new queries; the relocalization loader does this in memory without changing the
snapshot on disk. The `MAP_BTC 1` serialization format is unchanged.
