#include "py_json_convert.hpp"

#include <pybind11/stl.h>

#include <stdexcept>

namespace drone_core::bindings {

nlohmann::json py_to_json(const py::handle& obj) {
    if (obj.is_none()) {
        return nullptr;
    }
    if (py::isinstance<py::bool_>(obj)) {
        return obj.cast<bool>();
    }
    if (py::isinstance<py::int_>(obj)) {
        return obj.cast<long long>();
    }
    if (py::isinstance<py::float_>(obj)) {
        return obj.cast<double>();
    }
    if (py::isinstance<py::str>(obj)) {
        return obj.cast<std::string>();
    }
    if (py::isinstance<py::dict>(obj)) {
        nlohmann::json out = nlohmann::json::object();
        for (const auto& item : obj.cast<py::dict>()) {
            out[py::str(item.first).cast<std::string>()] = py_to_json(item.second);
        }
        return out;
    }
    if (py::isinstance<py::list>(obj) || py::isinstance<py::tuple>(obj)) {
        nlohmann::json out = nlohmann::json::array();
        for (const auto& item : obj) {
            out.push_back(py_to_json(item));
        }
        return out;
    }
    throw std::runtime_error("py_to_json: unsupported Python type " +
                              py::str(obj.get_type()).cast<std::string>());
}

py::object json_to_py(const nlohmann::json& value) {
    switch (value.type()) {
        case nlohmann::json::value_t::null:
            return py::none();
        case nlohmann::json::value_t::boolean:
            return py::bool_(value.get<bool>());
        case nlohmann::json::value_t::number_integer:
        case nlohmann::json::value_t::number_unsigned:
            return py::int_(value.get<long long>());
        case nlohmann::json::value_t::number_float:
            return py::float_(value.get<double>());
        case nlohmann::json::value_t::string:
            return py::str(value.get<std::string>());
        case nlohmann::json::value_t::array: {
            py::list out;
            for (const auto& item : value) {
                out.append(json_to_py(item));
            }
            return out;
        }
        case nlohmann::json::value_t::object: {
            py::dict out;
            for (auto it = value.begin(); it != value.end(); ++it) {
                out[py::str(it.key())] = json_to_py(it.value());
            }
            return out;
        }
        default:
            return py::none();
    }
}

}  // namespace drone_core::bindings
