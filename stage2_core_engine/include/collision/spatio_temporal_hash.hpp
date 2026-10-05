#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

// 4D Spatio-Temporal Collision Engine: reduces
// the O(N^2) all-pairs collision check to O(N log N) by hashing each drone's
// per-window bounding box into a voxel grid of size
// (d_min x d_min x d_min x delta_T) and only generating collision candidates
// for drones that share a voxel or one of its 26 spatial neighbors within
// the same time window.
//
// FIXED SCALABILITY BUG (found 2026-09-17, fixed 2026-09-18): this O(N log N)
// bound assumes drones are reasonably spread out relative to voxel_size_xyz;
// it degrades when a large fraction of N drones are genuinely packed within
// a few voxels of each other (e.g., a real 300-drone holding-area launch pad
// too small for its fleet size, forced into dense vertical stacking), each
// such drone's padded bbox spanning many voxel cells (see scp_solver.cpp's
// margin = safety_radius_m + enforced_min_distance). Confirmed on a real
// 300-drone Phase 1 export (`C:\Users\brake\Local\Temp\intermediate_export-
// 1.json`, holding area 40x10m with 2m grid spacing -- geometrically too
// small for 300 drones at 1.5m minimum separation without heavy Z-layer
// stacking): the very first SCP iteration of the holding-area departure
// transition drove process memory from ~2.7 GB to 7.5+ GB in under 10
// seconds. Root-caused to find_candidate_pairs()'s old per-*occupied-cell*
// outer loop: because insert() adds one grid_ entry per voxel a padded bbox
// spans (V cells for a drone whose bbox is V voxels across), the old
// implementation re-ran a fresh 27-neighbor-cell scan from EACH of those V
// occupied cells separately, an O(V) redundant-rescan multiplier per drone-
// window on top of the genuine candidate-pair cost -- see
// find_candidate_pairs()'s comment for the fix (iterate once per drone-
// window over a single dilated bounding-box cell range instead of once per
// occupied cell). This removes the redundant multiplier; it does NOT change
// the fact that a holding area genuinely too small for its fleet (i.e. where
// true candidate pairs approach O(N) per drone) is inherently O(N^2)-ish
// work -- that is a property of the geometry, not a fixable algorithmic
// inefficiency. See stage2_nway_conflict_limitation memory for the full
// investigation and re-test results.

namespace drone_core::collision {

struct VoxelKey {
    int x = 0;
    int y = 0;
    int z = 0;
    int t = 0;

    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z && t == other.t;
    }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const noexcept {
        // 64-bit mix of the 4 signed grid coordinates.
        uint64_t h = 0xcbf29ce484222325ULL;
        auto mix = [&h](int v) {
            h ^= static_cast<uint64_t>(static_cast<int64_t>(v));
            h *= 0x100000001b3ULL;
        };
        mix(k.x);
        mix(k.y);
        mix(k.z);
        mix(k.t);
        return static_cast<size_t>(h);
    }
};

struct DroneWindow {
    int drone_id = 0;
    int window_index = 0;
    Eigen::Vector3d bbox_min = Eigen::Vector3d::Zero();
    Eigen::Vector3d bbox_max = Eigen::Vector3d::Zero();
};

struct CandidatePair {
    int drone_i = 0;
    int drone_j = 0;
    int window_index = 0;
};

class SpatioTemporalHash {
public:
    SpatioTemporalHash(double voxel_size_xyz, double voxel_size_t);

    void insert(const DroneWindow& window);

    // Returns deduplicated (drone_i < drone_j, window_index) candidate pairs
    // whose bounding boxes occupy the same or a spatially-neighboring voxel
    // within the same time window.
    std::vector<CandidatePair> find_candidate_pairs() const;

private:
    double voxel_size_xyz_;
    double voxel_size_t_;
    std::unordered_multimap<VoxelKey, DroneWindow, VoxelKeyHash> grid_;
    // Canonical one-entry-per-insert() list, mirroring `grid_`'s multimap
    // entries but without the per-voxel duplication -- find_candidate_pairs()
    // iterates this (one pass per drone-window) instead of over `grid_`'s
    // occupied cells (which would revisit the same drone-window once per
    // voxel its own padded bbox spans; see this header's class comment).
    std::vector<DroneWindow> windows_;

    std::vector<VoxelKey> voxel_keys_for_bbox(const DroneWindow& window) const;
};

// A drone-pair conflict-graph edge, distinct from CandidatePair only in that
// it's deduplicated to one entry per unique drone pair (keeping the minimum
// sampled distance across every window that pair shares) and carries that
// distance. The caller (scp_solver, which owns the actual splines) currently
// builds one ConflictEdge for *every* unique pair appearing in its
// CandidatePair list, with no distance-based filtering: this struct's
// `distance` field once doubled as a filter threshold (an earlier
// cluster_distance_threshold_m, dropping edges for pairs
// whose bounding boxes merely brushed past each other at long range), but
// that filtering was found to be unsafe in this codebase's graph-coloring
// setup — see build_conflict_edges()'s comment in scp_solver.cpp for the
// real 300-drone incident (two drones landed 0.029 m apart) this caused, and
// why the edge set here must stay a 1:1 mirror of every collision-QP-row
// pair rather than a filtered subset. `distance` is kept for potential
// future coloring-order heuristics, not currently read by
// connected_components() or greedy_graph_coloring() below.
struct ConflictEdge {
    int drone_i = 0;
    int drone_j = 0;
    double distance = 0.0;
};

// Conflict Graph clustering via Disjoint-Set Union: builds G(V, E) from
// `edges` and returns its connected components. `max_cluster_size`, when
// > 0, caps a component's size by processing edges in ascending distance
// order and skipping a union that would exceed the cap (Kruskal-style weak-
// edge pruning: the closest/most urgent pairs are honored, the longest-
// distance edges in an oversized component are the ones left uncut). Pass
// <= 0 to disable the cap — which is what scp_solver.cpp currently always
// does, because capping cuts edges this same graph's caller also depends on
// for QP-row generation (see ConflictEdge's comment above); the parameter
// still exists for a caller whose edge set and dependency graph are
// verified to stay in lockstep under capping. `drone_ids` seeds every drone
// as its own singleton component first, so drones with zero conflict edges
// still get a (size == 1) cluster of their own.
std::vector<std::vector<int>> connected_components(const std::vector<int>& drone_ids,
                                                     const std::vector<ConflictEdge>& edges, int max_cluster_size);

// Greedy graph coloring (Welsh-Powell: descending degree order, each vertex
// takes the smallest color not already used by an already-colored neighbor)
// over one cluster's internal edges,
// used to replace a pure ascending-drone-id Gauss-Seidel sweep with
// same-color batches that have no edge between any two members and can
// therefore be solved in parallel via OpenMP. Returns each color's member
// list, indexed by color (batch 0 first).
std::vector<std::vector<int>> greedy_graph_coloring(const std::vector<int>& cluster_members,
                                                     const std::vector<ConflictEdge>& edges);

}  // namespace drone_core::collision
