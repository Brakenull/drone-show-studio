cmake_minimum_required(VERSION 3.21)
project(flight_packer LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Only the binary packer is C++; the digital twin (twin_sim/) is Python + OpenCL (pyopencl).
# Requires nlohmann-json, e.g. via the same vcpkg toolchain as stage2_core_engine:
#   cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
find_package(nlohmann_json CONFIG REQUIRED)

add_library(flight_packer_lib STATIC
    packer/src/quantizer.cpp
    packer/src/trajectory_sampler.cpp
    packer/src/flight_file.cpp
)
target_include_directories(flight_packer_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/packer/include)
target_link_libraries(flight_packer_lib PUBLIC nlohmann_json::nlohmann_json)
if(MSVC)
  target_compile_options(flight_packer_lib PRIVATE /W4)
else()
  target_compile_options(flight_packer_lib PRIVATE -Wall -Wextra)
endif()

add_executable(pack_to_binary packer/src/pack_to_binary.cpp)
target_link_libraries(pack_to_binary PRIVATE flight_packer_lib)

option(FLIGHT_PACKER_BUILD_TESTS "Build stage3 packer unit tests" ON)
if(FLIGHT_PACKER_BUILD_TESTS)
    enable_testing()
    add_subdirectory(packer/tests)
endif()
