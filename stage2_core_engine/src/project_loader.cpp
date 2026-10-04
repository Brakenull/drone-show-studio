#include "io/project_loader.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_core::io {

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

}  // namespace

ProjectData parse_project(const nlohmann::json& root) {
    require(root.contains("project_metadata"), "phase1 JSON missing project_metadata");
    require(root.contains("keyframes"), "phase1 JSON missing keyframes");

    const nlohmann::json& meta_json = root.at("project_metadata");
    ProjectData data;
    data.metadata.raw = meta_json;
    data.metadata.fleet_size = meta_json.at("fleet_size").get<int>();
    data.metadata.sampling_mode = meta_json.at("sampling_mode").get<std::string>();
    data.metadata.total_duration_sec = meta_json.at("total_duration_sec").get<double>();
    data.metadata.safety_radius_m = meta_json.at("safety_radius_m").get<double>();
    data.metadata.min_distance_m = meta_json.at("min_distance_m").get<double>();
    data.metadata.heading_offset_deg = meta_json.at("heading_offset_deg").get<double>();

    const nlohmann::json& holding_json = meta_json.at("holding_area");
    const auto center = holding_json.at("center").get<std::vector<double>>();
    const auto size = holding_json.at("size").get<std::vector<double>>();
    require(center.size() == 3, "holding_area.center must have 3 elements");
    require(size.size() == 2, "holding_area.size must have 2 elements");
    data.metadata.holding_area.center = Eigen::Vector3d(center[0], center[1], center[2]);
    data.metadata.holding_area.size = Eigen::Vector2d(size[0], size[1]);
    data.metadata.holding_area.max_height = holding_json.at("max_height").get<double>();
    data.metadata.holding_area.layer_spacing_m = holding_json.at("layer_spacing_m").get<double>();
    // grid_spacing_m is new in Phase 1 Rev 1.5; older exports don't have it,
    // so fall back to layer_spacing_m (which was itself just min_distance_m
    // before that revision separated the two).
    data.metadata.holding_area.grid_spacing_m = holding_json.contains("grid_spacing_m")
                                                     ? holding_json.at("grid_spacing_m").get<double>()
                                                     : data.metadata.holding_area.layer_spacing_m;
    if (holding_json.contains("staggered_layers") && !holding_json.at("staggered_layers").is_null()) {
        data.metadata.holding_area.staggered_layers = holding_json.at("staggered_layers").get<bool>();
    }
    if (holding_json.contains("show_clearance_m") && !holding_json.at("show_clearance_m").is_null()) {
        const double clearance = holding_json.at("show_clearance_m").get<double>();
        require(clearance >= 0.0, "holding_area.show_clearance_m must be >= 0");
        data.metadata.holding_area.show_clearance_m = clearance;
    }

    // Phase 1 schema 1.7.0 (optional): waiting areas.
    if (meta_json.contains("waiting_areas") && !meta_json.at("waiting_areas").is_null()) {
        for (const auto& area_json : meta_json.at("waiting_areas")) {
            WaitingArea area;
            const auto c = area_json.at("center").get<std::vector<double>>();
            const auto s = area_json.at("size").get<std::vector<double>>();
            require(c.size() == 3, "waiting_areas[].center must have 3 elements");
            require(s.size() == 2, "waiting_areas[].size must have 2 elements");
            area.center = Eigen::Vector3d(c[0], c[1], c[2]);
            area.size = Eigen::Vector2d(s[0], s[1]);
            area.grid_spacing_m = area_json.at("grid_spacing_m").get<double>();
            area.show_clearance_m = area_json.value("show_clearance_m", 0.0);
            area.slot_count = area_json.at("slot_count").get<int>();
            require(area.grid_spacing_m > 0.0, "waiting_areas[].grid_spacing_m must be > 0");
            require(area.slot_count >= 1, "waiting_areas[].slot_count must be >= 1");
            data.metadata.waiting_areas.push_back(area);
        }
    }

    // Phase 1 schema 1.6.0 (optional): the ground level.
    if (meta_json.contains("ground_z_m") && !meta_json.at("ground_z_m").is_null()) {
        data.metadata.ground_z_m = meta_json.at("ground_z_m").get<double>();
    }

    // Phase 1 schema 1.6.0 (optional): takeoff / return leg targets.
    if (meta_json.contains("legs") && !meta_json.at("legs").is_null()) {
        const nlohmann::json& legs_json = meta_json.at("legs");
        auto read_leg = [&](const char* name) -> std::optional<double> {
            require(legs_json.contains(name), "project_metadata.legs must have 'takeoff' and 'return'");
            const nlohmann::json& duration = legs_json.at(name).at("duration_sec");
            if (duration.is_null()) return std::nullopt;
            const double value = duration.get<double>();
            require(value > 0.0, "project_metadata.legs.*.duration_sec must be > 0 or null");
            return value;
        };
        data.metadata.legs.present = true;
        data.metadata.legs.takeoff_duration_sec = read_leg("takeoff");
        data.metadata.legs.return_duration_sec = read_leg("return");
    }

    for (const auto& kf_json : root.at("keyframes")) {
        Keyframe kf;
        kf.time_sec = kf_json.at("time_sec").get<double>();
        kf.shape_name = kf_json.at("shape_name").get<std::string>();
        for (const auto& pt_json : kf_json.at("points")) {
            Point pt;
            pt.index = pt_json.at("index").get<int>();
            const auto pos = pt_json.at("pos").get<std::vector<double>>();
            require(pos.size() == 3, "point.pos must have 3 elements");
            pt.position = Eigen::Vector3d(pos[0], pos[1], pos[2]);
            const auto color = pt_json.at("color").get<std::vector<int>>();
            require(color.size() == 3, "point.color must have 3 elements");
            pt.color = Eigen::Vector3i(color[0], color[1], color[2]);
            kf.points.push_back(pt);
        }
        data.keyframes.push_back(std::move(kf));
    }
    require(!data.keyframes.empty(), "phase1 JSON must contain at least 1 keyframe");
    std::sort(data.keyframes.begin(), data.keyframes.end(),
              [](const Keyframe& a, const Keyframe& b) { return a.time_sec < b.time_sec; });

    return data;
}

namespace {

// One layer's slot grid: cols x rows slots, the first one at (x0, y0).
struct LayerGrid {
    int cols = 0;
    int rows = 0;
    double x0 = 0.0;
    double y0 = 0.0;
    double z = 0.0;
    int capacity() const { return cols * rows; }
};

// Shared by compute_holding_positions(), compute_holding_row_indices() and
// compute_holding_region() so they can never disagree on the layer grids /
// how many layers / whether the footprint had to widen. A port of Phase 1's
// HoldingLayout (stage1_designer/core/holding_area.py, spec section 3.2).
struct HoldingLayout {
    int cols = 0;  // columns / rows of the even (unshifted) layers
    int rows = 0;
    int layers_needed = 0;
    double width = 0.0;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double grid_spacing_m = 0.0;
    double layer_spacing_m = 0.0;
    bool staggered = false;

    // Odd layers of a staggered layout are shifted half a slot along each
    // axis that has more than one slot (and lose that axis's last slot).
    LayerGrid layer(int m) const {
        const double d = grid_spacing_m;
        LayerGrid grid{cols, rows, center.x() - ((cols - 1) * d) / 2.0, center.y() - ((rows - 1) * d) / 2.0,
                       center.z() + m * layer_spacing_m};
        if (staggered && m % 2 == 1) {
            if (grid.cols > 1) {
                grid.cols -= 1;
                grid.x0 += d / 2.0;
            }
            if (grid.rows > 1) {
                grid.rows -= 1;
                grid.y0 += d / 2.0;
            }
        }
        return grid;
    }

    int total_capacity(int layers) const {
        int total = 0;
        for (int m = 0; m < layers; ++m) total += layer(m).capacity();
        return total;
    }

    // Fewest bottom layers that hold `count` drones.
    int layers_for(int count) const {
        int layers = 0;
        for (int placed = 0; placed < count; ++layers) placed += layer(layers).capacity();
        return layers;
    }
};

HoldingLayout compute_holding_layout(int fleet_size, const HoldingArea& holding_area, double grid_spacing_m) {
    const double zc = holding_area.center.z();
    const double gap = holding_area.layer_spacing_m > 0.0 ? holding_area.layer_spacing_m : grid_spacing_m;
    double width = holding_area.size.x();
    const double length = holding_area.size.y();

    auto make = [&](double w) {
        HoldingLayout layout;
        layout.cols = static_cast<int>(std::floor(w / grid_spacing_m)) + 1;
        layout.rows = static_cast<int>(std::floor(length / grid_spacing_m)) + 1;
        layout.width = w;
        layout.center = holding_area.center;
        layout.grid_spacing_m = grid_spacing_m;
        layout.layer_spacing_m = gap;
        layout.staggered = holding_area.staggered_layers;
        return layout;
    };

    const int max_layers = std::max(static_cast<int>(std::floor((holding_area.max_height - zc) / gap)) + 1, 1);
    HoldingLayout layout = make(width);
    if (layout.layers_for(fleet_size) > max_layers) {
        // Expand width along X (adding grid columns) until the layers that
        // fit under max_height hold everyone.
        while (layout.total_capacity(max_layers) < fleet_size) {
            width += grid_spacing_m;
            layout = make(width);
        }
    }
    layout.layers_needed = layout.layers_for(fleet_size);
    return layout;
}

// Calls visit(slot, position, row within its layer) for the first
// `fleet_size` slots, layer by layer from the bottom, row by row.
template <typename Visit>
void for_each_holding_slot(int fleet_size, const HoldingLayout& layout, Visit&& visit) {
    int placed = 0;
    for (int m = 0; m < layout.layers_needed && placed < fleet_size; ++m) {
        const LayerGrid grid = layout.layer(m);
        const int n_this_layer = std::min(grid.capacity(), fleet_size - placed);
        for (int i = 0; i < n_this_layer; ++i) {
            const int row = i / grid.cols;
            const int col = i % grid.cols;
            visit(placed, Eigen::Vector3d(grid.x0 + col * layout.grid_spacing_m, grid.y0 + row * layout.grid_spacing_m,
                                          grid.z),
                  row);
            ++placed;
        }
    }
}

}  // namespace

Eigen::MatrixXd compute_holding_positions(int fleet_size, const HoldingArea& holding_area, double grid_spacing_m) {
    if (fleet_size <= 0) {
        return Eigen::MatrixXd(0, 3);
    }
    Eigen::MatrixXd positions(fleet_size, 3);
    for_each_holding_slot(fleet_size, compute_holding_layout(fleet_size, holding_area, grid_spacing_m),
                          [&](int slot, const Eigen::Vector3d& p, int) { positions.row(slot) = p; });
    return positions;
}

std::vector<int> compute_holding_row_indices(int fleet_size, const HoldingArea& holding_area, double grid_spacing_m) {
    if (fleet_size <= 0) {
        return {};
    }
    std::vector<int> row_indices(fleet_size, 0);
    for_each_holding_slot(fleet_size, compute_holding_layout(fleet_size, holding_area, grid_spacing_m),
                          [&](int slot, const Eigen::Vector3d&, int row) { row_indices[slot] = row; });
    return row_indices;
}

HoldingRegion compute_holding_region(int fleet_size, const HoldingArea& holding_area, double grid_spacing_m) {
    const Eigen::Vector3d& c = holding_area.center;
    const HoldingLayout layout = compute_holding_layout(std::max(fleet_size, 0), holding_area, grid_spacing_m);
    HoldingRegion region;
    region.lo = Eigen::Vector3d(c.x() - layout.width / 2.0, c.y() - holding_area.size.y() / 2.0, c.z());
    region.hi = Eigen::Vector3d(c.x() + layout.width / 2.0, c.y() + holding_area.size.y() / 2.0,
                                std::max(holding_area.max_height, c.z()));
    const Eigen::MatrixXd slots = compute_holding_positions(fleet_size, holding_area, grid_spacing_m);
    if (slots.rows() > 0) {
        const Eigen::Vector3d pad = Eigen::Vector3d::Constant(grid_spacing_m / 2.0);
        region.lo = region.lo.cwiseMin(Eigen::Vector3d(slots.colwise().minCoeff().transpose()) - pad);
        region.hi = region.hi.cwiseMax(Eigen::Vector3d(slots.colwise().maxCoeff().transpose()) + pad);
    }
    return region;
}

namespace {

// cols x rows of one waiting area's single layer, and its effective
// footprint. Grows one grid step at a time on its shorter side (X on a tie),
// centred: Phase 1's compute_waiting_layout().
struct WaitingLayout {
    int cols = 0;
    int rows = 0;
    double width = 0.0;
    double length = 0.0;
};

WaitingLayout compute_waiting_layout(const WaitingArea& area) {
    const double d = area.grid_spacing_m;
    WaitingLayout layout;
    layout.width = area.size.x();
    layout.length = area.size.y();
    const auto dims = [&]() {
        layout.cols = static_cast<int>(std::floor(layout.width / d)) + 1;
        layout.rows = static_cast<int>(std::floor(layout.length / d)) + 1;
    };
    dims();
    while (layout.cols * layout.rows < area.slot_count) {
        if (layout.width <= layout.length) {
            layout.width += d;
        } else {
            layout.length += d;
        }
        dims();
    }
    return layout;
}

}  // namespace

Eigen::MatrixXd compute_waiting_positions(const WaitingArea& area) {
    const int n = std::max(area.slot_count, 0);
    Eigen::MatrixXd slots(n, 3);
    if (n == 0) return slots;
    const WaitingLayout layout = compute_waiting_layout(area);
    const double d = area.grid_spacing_m;
    const double x0 = area.center.x() - ((layout.cols - 1) * d) / 2.0;
    const double y0 = area.center.y() - ((layout.rows - 1) * d) / 2.0;
    for (int i = 0; i < n; ++i) {
        slots.row(i) = Eigen::Vector3d(x0 + (i % layout.cols) * d, y0 + (i / layout.cols) * d, area.center.z());
    }
    return slots;
}

HoldingRegion compute_waiting_region(const WaitingArea& area) {
    const WaitingLayout layout = compute_waiting_layout(area);
    const Eigen::Vector3d half(layout.width / 2.0, layout.length / 2.0, 0.0);
    HoldingRegion region{area.center - half, area.center + half};
    const Eigen::MatrixXd slots = compute_waiting_positions(area);
    if (slots.rows() > 0) {
        const Eigen::Vector3d pad = Eigen::Vector3d::Constant(area.grid_spacing_m / 2.0);
        region.lo = region.lo.cwiseMin(Eigen::Vector3d(slots.colwise().minCoeff().transpose()) - pad);
        region.hi = region.hi.cwiseMax(Eigen::Vector3d(slots.colwise().maxCoeff().transpose()) + pad);
    }
    return region;
}

Eigen::MatrixXd compute_all_waiting_slots(const std::vector<WaitingArea>& areas) {
    int total = 0;
    for (const WaitingArea& a : areas) total += std::max(a.slot_count, 0);
    Eigen::MatrixXd all(total, 3);
    int row = 0;
    for (const WaitingArea& a : areas) {
        const Eigen::MatrixXd slots = compute_waiting_positions(a);
        all.middleRows(row, slots.rows()) = slots;
        row += static_cast<int>(slots.rows());
    }
    return all;
}

}  // namespace drone_core::io
