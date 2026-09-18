#include "collision/spatio_temporal_hash.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <tuple>
#include <unordered_map>

namespace drone_core::collision {

SpatioTemporalHash::SpatioTemporalHash(double voxel_size_xyz, double voxel_size_t)
    : voxel_size_xyz_(voxel_size_xyz), voxel_size_t_(voxel_size_t) {}

std::vector<VoxelKey> SpatioTemporalHash::voxel_keys_for_bbox(const DroneWindow& window) const {
    const int x0 = static_cast<int>(std::floor(window.bbox_min.x() / voxel_size_xyz_));
    const int x1 = static_cast<int>(std::floor(window.bbox_max.x() / voxel_size_xyz_));
    const int y0 = static_cast<int>(std::floor(window.bbox_min.y() / voxel_size_xyz_));
    const int y1 = static_cast<int>(std::floor(window.bbox_max.y() / voxel_size_xyz_));
    const int z0 = static_cast<int>(std::floor(window.bbox_min.z() / voxel_size_xyz_));
    const int z1 = static_cast<int>(std::floor(window.bbox_max.z() / voxel_size_xyz_));

    std::vector<VoxelKey> keys;
    keys.reserve(static_cast<size_t>((x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1)));
    for (int x = x0; x <= x1; ++x) {
        for (int y = y0; y <= y1; ++y) {
            for (int z = z0; z <= z1; ++z) {
                keys.push_back(VoxelKey{x, y, z, window.window_index});
            }
        }
    }
    return keys;
}

void SpatioTemporalHash::insert(const DroneWindow& window) {
    for (const auto& key : voxel_keys_for_bbox(window)) {
        grid_.emplace(key, window);
    }
    windows_.push_back(window);
}

// FIXED (see spatio_temporal_hash.hpp's class-level comment for the full
// incident writeup): the old implementation iterated per *occupied cell*
// (of which a single drone-window contributes one per voxel its padded bbox
// spans -- V of them) and, for each, rebuilt a fresh 27-neighbor-cell scan
// from scratch. Two cells belonging to the same drone-window's own bbox
// produce nearly-identical 27-neighborhoods (offset by one cell), so that
// redundant rescanning multiplied the real candidate-pair cost by a factor
// of roughly V -- the dominant driver of the multi-GB memory spike on a
// real densely-packed 300-drone holding area.
//
// Fixed by iterating once per drone-window (`windows_`, one entry per
// insert() call) instead of once per occupied cell. For each drone-window,
// its own bbox's voxel range [x0,x1]x[y0,y1]x[z0,z1] is dilated by exactly
// one cell on every side in a single range computation -- not by unioning
// 27-cell neighborhoods per individual cell -- which is the exact set of
// voxels that could contain a spatially-adjacent neighbor of ANY cell the
// bbox itself occupies (standard Minkowski dilation of an axis-aligned box
// by one cell). `partner_drone_ids` deduplicates a neighbor drone that
// shares more than one of those dilated cells with this window, so it is
// still only emitted once per drone-window pair.
//
// This does not change the fact that a cluster genuinely too dense for its
// voxel size (true candidate pairs approaching O(N) per drone) is
// inherently expensive -- only the redundant multiplicative rescanning on
// top of that genuine cost is removed.
std::vector<CandidatePair> SpatioTemporalHash::find_candidate_pairs() const {
    std::vector<CandidatePair> pairs;
    std::set<std::tuple<int, int, int>> seen;

    for (const auto& self_window : windows_) {
        const int x0 = static_cast<int>(std::floor(self_window.bbox_min.x() / voxel_size_xyz_)) - 1;
        const int x1 = static_cast<int>(std::floor(self_window.bbox_max.x() / voxel_size_xyz_)) + 1;
        const int y0 = static_cast<int>(std::floor(self_window.bbox_min.y() / voxel_size_xyz_)) - 1;
        const int y1 = static_cast<int>(std::floor(self_window.bbox_max.y() / voxel_size_xyz_)) + 1;
        const int z0 = static_cast<int>(std::floor(self_window.bbox_min.z() / voxel_size_xyz_)) - 1;
        const int z1 = static_cast<int>(std::floor(self_window.bbox_max.z() / voxel_size_xyz_)) + 1;

        std::set<int> partner_drone_ids;
        for (int x = x0; x <= x1; ++x) {
            for (int y = y0; y <= y1; ++y) {
                for (int z = z0; z <= z1; ++z) {
                    const auto range = grid_.equal_range(VoxelKey{x, y, z, self_window.window_index});
                    for (auto it = range.first; it != range.second; ++it) {
                        if (it->second.drone_id != self_window.drone_id) {
                            partner_drone_ids.insert(it->second.drone_id);
                        }
                    }
                }
            }
        }

        for (int other_id : partner_drone_ids) {
            int a = self_window.drone_id;
            int b = other_id;
            if (a > b) std::swap(a, b);
            if (seen.insert({a, b, self_window.window_index}).second) {
                pairs.push_back(CandidatePair{a, b, self_window.window_index});
            }
        }
    }

    return pairs;
}

namespace {

// Union-by-size DSU: size tracking (not just union-by-rank) is load-bearing
// here, not just a perf nicety -- connected_components()'s max_cluster_size
// cap needs each root's live member count to decide whether admitting an
// edge would grow a component past the limit.
struct DisjointSetUnion {
    std::vector<int> parent;
    std::vector<int> size;
    explicit DisjointSetUnion(int n) : parent(n), size(n, 1) {
        for (int i = 0; i < n; ++i) parent[i] = i;
    }
    int find(int x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }
    void unite(int a, int b) {
        a = find(a);
        b = find(b);
        if (a == b) return;
        if (size[a] < size[b]) std::swap(a, b);
        parent[b] = a;
        size[a] += size[b];
    }
};

}  // namespace

std::vector<std::vector<int>> connected_components(const std::vector<int>& drone_ids,
                                                     const std::vector<ConflictEdge>& edges, int max_cluster_size) {
    std::unordered_map<int, int> index_of;
    index_of.reserve(drone_ids.size());
    for (int i = 0; i < static_cast<int>(drone_ids.size()); ++i) {
        index_of[drone_ids[i]] = i;
    }

    std::vector<ConflictEdge> sorted_edges = edges;
    std::sort(sorted_edges.begin(), sorted_edges.end(),
              [](const ConflictEdge& a, const ConflictEdge& b) { return a.distance < b.distance; });

    DisjointSetUnion dsu(static_cast<int>(drone_ids.size()));
    const bool cap_enabled = max_cluster_size > 0;
    for (const auto& edge : sorted_edges) {
        auto it_i = index_of.find(edge.drone_i);
        auto it_j = index_of.find(edge.drone_j);
        if (it_i == index_of.end() || it_j == index_of.end()) continue;
        const int root_i = dsu.find(it_i->second);
        const int root_j = dsu.find(it_j->second);
        if (root_i == root_j) continue;
        if (cap_enabled && dsu.size[root_i] + dsu.size[root_j] > max_cluster_size) {
            // Weak-edge cluster cut: this (and every longer-distance edge
            // between these two roots we haven't seen yet, by construction
            // of the ascending sort) is left uncut/unmerged.
            continue;
        }
        dsu.unite(root_i, root_j);
    }

    std::unordered_map<int, std::vector<int>> clusters_by_root;
    for (int i = 0; i < static_cast<int>(drone_ids.size()); ++i) {
        clusters_by_root[dsu.find(i)].push_back(drone_ids[i]);
    }

    std::vector<std::vector<int>> clusters;
    clusters.reserve(clusters_by_root.size());
    for (auto& [root, members] : clusters_by_root) {
        clusters.push_back(std::move(members));
    }
    return clusters;
}

std::vector<std::vector<int>> greedy_graph_coloring(const std::vector<int>& cluster_members,
                                                     const std::vector<ConflictEdge>& edges) {
    const int n = static_cast<int>(cluster_members.size());
    std::unordered_map<int, int> local_index;
    local_index.reserve(n);
    for (int i = 0; i < n; ++i) local_index[cluster_members[i]] = i;

    std::vector<std::vector<int>> adjacency(n);
    for (const auto& edge : edges) {
        auto it_i = local_index.find(edge.drone_i);
        auto it_j = local_index.find(edge.drone_j);
        if (it_i == local_index.end() || it_j == local_index.end()) continue;
        adjacency[it_i->second].push_back(it_j->second);
        adjacency[it_j->second].push_back(it_i->second);
    }

    // Welsh-Powell: coloring highest-degree vertices first tends to use
    // fewer colors (more parallelism per batch) than a plain index order.
    std::vector<int> order(n);
    for (int i = 0; i < n; ++i) order[i] = i;
    std::sort(order.begin(), order.end(),
              [&](int a, int b) { return adjacency[a].size() > adjacency[b].size(); });

    std::vector<int> color(n, -1);
    int num_colors = 0;
    for (int v : order) {
        std::vector<bool> used_by_neighbor(num_colors, false);
        for (int nb : adjacency[v]) {
            if (color[nb] >= 0) used_by_neighbor[color[nb]] = true;
        }
        int chosen = 0;
        while (chosen < num_colors && used_by_neighbor[chosen]) ++chosen;
        color[v] = chosen;
        if (chosen == num_colors) ++num_colors;
    }

    std::vector<std::vector<int>> batches(num_colors);
    for (int i = 0; i < n; ++i) {
        batches[color[i]].push_back(cluster_members[i]);
    }
    return batches;
}

}  // namespace drone_core::collision
