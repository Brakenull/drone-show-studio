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
}

// KNOWN SCALABILITY RISK (see spatio_temporal_hash.hpp's class-level
// comment for the full incident writeup): for a densely-packed cluster
// where many drones' padded bboxes span several voxels each, `grid_`
// contains many entries per drone-window, so this function's outer loop
// revisits that cluster from many different cells, each time rebuilding a
// `neighborhood` that can itself contain a large fraction of the same
// cluster -- observed to drive memory into the multiple-GB range on a real
// 300-drone dense holding-area departure. `seen` deduplicates the *output*
// pairs but not this redundant re-scanning work itself.
std::vector<CandidatePair> SpatioTemporalHash::find_candidate_pairs() const {
    std::vector<CandidatePair> pairs;
    std::set<std::tuple<int, int, int>> seen;
    std::unordered_map<VoxelKey, bool, VoxelKeyHash> visited_cells;

    for (const auto& [key, self_window] : grid_) {
        if (visited_cells.find(key) != visited_cells.end()) {
            continue;
        }
        visited_cells[key] = true;

        std::vector<const DroneWindow*> neighborhood;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    const VoxelKey neighbor_key{key.x + dx, key.y + dy, key.z + dz, key.t};
                    auto range = grid_.equal_range(neighbor_key);
                    for (auto it = range.first; it != range.second; ++it) {
                        neighborhood.push_back(&it->second);
                    }
                }
            }
        }

        auto self_range = grid_.equal_range(key);
        for (auto sit = self_range.first; sit != self_range.second; ++sit) {
            for (const DroneWindow* other : neighborhood) {
                if (other->drone_id == sit->second.drone_id) {
                    continue;
                }
                int a = sit->second.drone_id;
                int b = other->drone_id;
                if (a > b) std::swap(a, b);
                if (seen.insert({a, b, key.t}).second) {
                    pairs.push_back(CandidatePair{a, b, key.t});
                }
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
