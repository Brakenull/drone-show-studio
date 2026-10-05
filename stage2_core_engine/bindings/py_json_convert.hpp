#pragma once

#include <pybind11/pybind11.h>

#include <nlohmann/json.hpp>

// Manual py::object <-> nlohmann::json bridge. Written by hand instead of
// pulling in the third-party "pybind11_json" header, to avoid adding another
// dependency beyond the 4 already required (Eigen3,
// OSQP, OpenMP, pybind11) plus nlohmann-json for core_config.json parsing.

namespace drone_core::bindings {

namespace py = pybind11;

nlohmann::json py_to_json(const py::handle& obj);
py::object json_to_py(const nlohmann::json& value);

}  // namespace drone_core::bindings
