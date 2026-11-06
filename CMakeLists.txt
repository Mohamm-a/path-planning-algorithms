cmake_minimum_required(VERSION 3.16)

project(core_skeleton LANGUAGES CXX)

include(GNUInstallDirs)

# Header-only core. No external dependencies [FROM: AGENTS.md section 4].
# Build-tree and install-tree include paths are separated so the exported
# package points at the installed headers [FROM: Phase A plan, approved].
add_library(core INTERFACE)
target_include_directories(core INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(core INTERFACE cxx_std_17)

# Verification-only double build for the float regression described in
# docs/hardware_integration.md. Do not mix it with the float build: `real`
# changes every struct layout, so all translation units in a target must agree
# [FROM: user approval, Phase A stop condition 1].
option(ROBOT_CORE_REAL_DOUBLE "Build the core with real = double (regression only)" OFF)
if(ROBOT_CORE_REAL_DOUBLE)
    target_compile_definitions(core INTERFACE CORE_REAL_DOUBLE)
endif()

# Odometry implementation. Kept out of the header-only core target so the
# core interface stays include-only [FROM: Phase A plan approval].
add_library(core_odometry STATIC src/odometry.cpp)
target_link_libraries(core_odometry PUBLIC core)
target_compile_options(core_odometry PRIVATE -Wall -Wextra -Wpedantic -Werror)

# Map manager implementation (grid storage is caller-owned). Kept out of the
# header-only core target, matching core_odometry [FROM: Phase A plan approval].
add_library(core_map STATIC src/map_manager.cpp)
target_link_libraries(core_map PUBLIC core)
target_compile_options(core_map PRIVATE -Wall -Wextra -Wpedantic -Werror)

# Global planner implementation (A*, stubs, factory, replan trigger). Kept out
# of the header-only core target, matching core_map [FROM: Phase A plan approval].
add_library(core_global_planner STATIC src/global_planner_manager.cpp)
target_link_libraries(core_global_planner PUBLIC core)
target_compile_options(core_global_planner PRIVATE -Wall -Wextra -Wpedantic -Werror)

# TEB local planner implementation (bicycle rollout, projected descent). Kept
# out of the header-only core target, matching core_global_planner
# [FROM: Phase A plan approval].
add_library(core_teb_local_planner STATIC src/teb_local_planner.cpp)
target_link_libraries(core_teb_local_planner PUBLIC core)
target_compile_options(core_teb_local_planner PRIVATE -Wall -Wextra -Wpedantic -Werror)

# Output converter implementation. Matches the per-stage libraries above.
add_library(core_output_converter STATIC src/output_converter.cpp)
target_link_libraries(core_output_converter PUBLIC core)
target_compile_options(core_output_converter PRIVATE -Wall -Wextra -Wpedantic -Werror)

# Core facade: owns the stages and runs the pipeline in step() [FROM: Phase A
# plan approval, decision D1]. Links every stage library.
add_library(core_facade STATIC src/core.cpp)
target_link_libraries(core_facade PUBLIC core core_odometry core_map core_global_planner core_teb_local_planner core_output_converter)
target_compile_options(core_facade PRIVATE -Wall -Wextra -Wpedantic -Werror)

# Compile-only check: includes every public and stage header. Not a test. Not
# installed or exported.
add_library(core_compile_check OBJECT src/compile_check.cpp)
target_link_libraries(core_compile_check PRIVATE core)
target_compile_options(core_compile_check PRIVATE -Wall -Wextra -Wpedantic -Werror)

# Opt-in end-to-end wall simulation. Off by default and not installed. Build
# it with the placeholder values as -D flags in CMAKE_CXX_FLAGS so the core
# libraries and the simulation are compiled with the same values.
option(ROBOT_CORE_BUILD_SIM "Build the end-to-end wall simulation" OFF)
if(ROBOT_CORE_BUILD_SIM)
    add_executable(wall_sim sim/wall_sim.cpp)
    target_link_libraries(wall_sim PRIVATE core_facade)
    target_compile_options(wall_sim PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

# ---------------------------------------------------------------------------
# Installation and package export
# ---------------------------------------------------------------------------
# Consumers: find_package(robot_core REQUIRED) then
# target_link_libraries(my_robot PRIVATE robot_core::core_facade).

set(ROBOT_CORE_TARGETS
    core
    core_odometry
    core_map
    core_global_planner
    core_teb_local_planner
    core_output_converter
    core_facade)

install(TARGETS ${ROBOT_CORE_TARGETS}
    EXPORT robot_core_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Public and stage headers, including placeholders.hpp. Installed as-is.
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT robot_core_targets
    FILE robot_core-targets.cmake
    NAMESPACE robot_core::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robot_core)

include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/robot_core-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/robot_core-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robot_core)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/robot_core-config.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robot_core)
