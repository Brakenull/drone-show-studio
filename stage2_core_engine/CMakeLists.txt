cmake_minimum_required(VERSION 3.21)
project(drone_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Requires a vcpkg toolchain (or otherwise discoverable installs) providing:
#   eigen3, osqp, nlohmann-json
# and a Python with pybind11 installed (pip install pybind11), by default the
# repo's .venv. e.g. configure with:
#   cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenMP REQUIRED)
find_package(osqp CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

# drone_core must be built for the Python that imports it (its ABI tag, e.g.
# cp314, has to match), so both the interpreter and pybind11 come from the
# repo's .venv unless -DPython_EXECUTABLE / -Dpybind11_DIR say otherwise.
if(NOT Python_EXECUTABLE)
  if(WIN32)
    set(_venv_python "${CMAKE_CURRENT_SOURCE_DIR}/../.venv/Scripts/python.exe")
  else()
    set(_venv_python "${CMAKE_CURRENT_SOURCE_DIR}/../.venv/bin/python")
  endif()
  if(NOT EXISTS "${_venv_python}")
    message(FATAL_ERROR "No Python found at ${_venv_python}. Create the repo's .venv, "
                        "or pass -DPython_EXECUTABLE=<python with pybind11 installed>.")
  endif()
  get_filename_component(_venv_python "${_venv_python}" ABSOLUTE)
  set(Python_EXECUTABLE "${_venv_python}" CACHE FILEPATH "Python that drone_core is built for and imported by")
endif()

if(NOT pybind11_DIR)
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -m pybind11 --cmakedir
    OUTPUT_VARIABLE _pybind11_cmakedir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _pybind11_result
    ERROR_QUIET)
  if(NOT _pybind11_result EQUAL 0)
    message(FATAL_ERROR "pybind11 is not installed for ${Python_EXECUTABLE}. "
                        "Run: \"${Python_EXECUTABLE}\" -m pip install pybind11")
  endif()
  set(pybind11_DIR "${_pybind11_cmakedir}" CACHE PATH "pybind11 CMake package directory")
endif()

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)
message(STATUS "drone_core: Python ${Python_VERSION} at ${Python_EXECUTABLE}, pybind11 ${pybind11_VERSION}")

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
