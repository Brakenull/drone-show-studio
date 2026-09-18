cmake_minimum_required(VERSION 3.21)
project(drone_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Requires a vcpkg toolchain (or otherwise discoverable installs) providing:
#   eigen3, osqp, nlohmann-json, pybind11
# e.g. configure with:
#   cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenMP REQUIRED)
find_package(osqp CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

add_library(drone_core_lib STATIC
    src/lap_auction.cpp
    src/quintic_bspline.cpp
    src/apf_seeder.cpp
    src/spatio_temporal_hash.cpp
    src/scp_solver.cpp
    src/project_loader.cpp
    src/pipeline.cpp
)

target_include_directories(drone_core_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(drone_core_lib PUBLIC
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
    osqp::osqp
    nlohmann_json::nlohmann_json
)

target_compile_definitions(drone_core_lib PUBLIC
    DRONE_CORE_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
)

pybind11_add_module(drone_core_py
    bindings/py_bindings.cpp
    bindings/py_json_convert.cpp
)
set_target_properties(drone_core_py PROPERTIES OUTPUT_NAME "drone_core")
target_link_libraries(drone_core_py PRIVATE drone_core_lib)

option(DRONE_CORE_BUILD_TESTS "Build stage2_core_engine unit tests" ON)
if(DRONE_CORE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
