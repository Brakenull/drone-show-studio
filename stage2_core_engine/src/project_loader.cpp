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

// Shared by compute_holding_positions() and compute_holding_row_indices() so
// the two can never disagree on layer capacity / how many layers / whether
// the footprint had to widen — both need the *identical* grid to report
// consistent positions vs. row indices for the same slot.
struct HoldingLayout {
    int cols = 0;
    int rows = 0;
    int capacity = 0;
    int layers_needed = 0;
    double width = 0.0;
};

HoldingLayout compute_holding_layout(int fleet_size, const HoldingArea& holding_area, double grid_spacing_m) {
    const double zc = holding_area.center.z();
    double width = holding_area.size.x();
    const double length = holding_area.size.y();

    auto layer_grid_dims = [&](double w, double l) {
        const int cols = static_cast<int>(std::floor(w / grid_spacing_m)) + 1;
        const int rows = static_cast<int>(std::floor(l / grid_spacing_m)) + 1;
        return std::make_pair(cols, rows);
    };

    int max_layers = static_cast<int>(std::floor((holding_area.max_height - zc) / grid_spacing_m)) + 1;
    max_layers = std::max(max_layers, 1);

    auto [cols, rows] = layer_grid_dims(width, length);
    int capacity = cols * rows;
    int layers_needed = static_cast<int>(std::ceil(static_cast<double>(fleet_size) / capacity));

    if (layers_needed > max_layers) {
        const int required_capacity = static_cast<int>(std::ceil(static_cast<double>(fleet_size) / max_layers));
        while (capacity < required_capacity) {
            width += grid_spacing_m;
            std::tie(cols, rows) = layer_grid_dims(width, length);
            capacity = cols * rows;
        }
        layers_needed = static_cast<int>(std::ceil(static_cast<double>(fleet_size) / capacity));
    }

    return HoldingLayout{cols, rows, capacity, layers_needed, width};
}

}  // namespace

Eigen::MatrixXd compute_holding_positions(int fleet_size, const HoldingArea& holding_area, double grid_spacing_m) {
    if (fleet_size <= 0) {
        return Eigen::MatrixXd(0, 3);
    }

    const double xc = holding_area.center.x();
    const double yc = holding_area.center.y();
    const double zc = holding_area.center.z();
    const HoldingLayout layout = compute_holding_layout(fleet_size, holding_area, grid_spacing_m);

    Eigen::MatrixXd positions(fleet_size, 3);
    const double x0 = xc - ((layout.cols - 1) * grid_spacing_m) / 2.0;
    const double y0 = yc - ((layout.rows - 1) * grid_spacing_m) / 2.0;

    int placed = 0;
    for (int m = 0; m < layout.layers_needed && placed < fleet_size; ++m) {
        const double z = zc + m * grid_spacing_m;
        const int n_this_layer = std::min(layout.capacity, fleet_size - placed);
        for (int i = 0; i < n_this_layer; ++i) {
            const int row = i / layout.cols;
            const int col = i % layout.cols;
            const double x = x0 + col * grid_spacing_m;
            const double y = y0 + row * grid_spacing_m;
            positions.row(placed) = Eigen::Vector3d(x, y, z);
            ++placed;
        }
    }

    return positions;
}

std::vector<int> compute_holding_row_indices(int fleet_size, const HoldingArea& holding_area, double grid_spacing_m) {
    if (fleet_size <= 0) {
        return {};
    }

    const HoldingLayout layout = compute_holding_layout(fleet_size, holding_area, grid_spacing_m);

    std::vector<int> row_indices(fleet_size, 0);
    int placed = 0;
    for (int m = 0; m < layout.layers_needed && placed < fleet_size; ++m) {
        const int n_this_layer = std::min(layout.capacity, fleet_size - placed);
        for (int i = 0; i < n_this_layer; ++i) {
            row_indices[placed] = i / layout.cols;
            ++placed;
        }
    }
    return row_indices;
}

}  // namespace drone_core::io
