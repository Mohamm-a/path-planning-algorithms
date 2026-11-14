// Map manager stage interface. Declarations only; implementation is in
// src/map_manager.cpp. Fixed borders, SLAM overlay and inflation are
// replaceable layers, not branches inside planners [FROM: AGENTS.md section 2].

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "core/core.hpp"
#include "core/placeholders.hpp"

namespace core::map {

// Occupancy state of one grid cell (user decision: enum, not a cost byte).
enum class CellState : std::uint8_t {
    Free,
    Occupied,
    Unknown,
};

// Edge length of one grid cell in m [UNKNOWN: PH-0008, see placeholders.hpp].
inline constexpr real kCellResolution_m = MAP_RESOLUTION_M;

// Non-owning row-major occupancy grid (pointer plus extents).
// The caller owns the storage. No allocation occurs inside the core.
struct GridMap {
    Point2D origin;            // m; map-frame position of cell (0, 0)
    std::size_t width;         // cells, along map x
    std::size_t height;        // cells, along map y
    const CellState* cells;    // row-major, width * height entries; may be
                               // null only when width * height is 0
};

// Non-owning mutable grid. Used for the SLAM overlay (updated in place each
// cycle, persists across cycles through the caller) and for output buffers.
struct MutableGridMap {
    Point2D origin;            // m; map-frame position of cell (0, 0)
    std::size_t width;         // cells, along map x
    std::size_t height;        // cells, along map y
    CellState* cells;          // row-major, width * height entries; may be
                               // null only when width * height is 0
};

// Signed clearance from p to the nearest blocked cell centre (Occupied or
// Unknown) or to the grid boundary. The cell-centre distance is reduced by
// half a cell diagonal, so the value is the distance to the conservative
// cell footprint rather than to a point. The result is negative outside the
// grid. This is the single clearance definition used by A*, D*-Lite, RRT,
// smoothing and TEB [FROM: user approval, clearance unification].
inline real clearance(const GridMap& g, Point2D p, real search_radius) {
    if (!(kCellResolution_m > 0.0) || g.width == 0 || g.height == 0 ||
        g.cells == nullptr || !std::isfinite(p.x) || !std::isfinite(p.y) ||
        !std::isfinite(search_radius) || search_radius < 0.0) {
        return -std::numeric_limits<real>::infinity();
    }

    const real res = kCellResolution_m;
    const real x_min = g.origin.x;
    const real y_min = g.origin.y;
    const real x_max = x_min + static_cast<real>(g.width) * res;
    const real y_max = y_min + static_cast<real>(g.height) * res;
    const real half_diag = real(0.5) * res * std::sqrt(real(2));

    const real dx_out = std::max({x_min - p.x, real(0), p.x - x_max});
    const real dy_out = std::max({y_min - p.y, real(0), p.y - y_max});
    if (dx_out > real(0) || dy_out > real(0)) {
        return -std::hypot(dx_out, dy_out);
    }

    real best = std::min({p.x - x_min, x_max - p.x, p.y - y_min, y_max - p.y});

    const real search = search_radius + half_diag;
    const real max_ix = static_cast<real>(g.width - 1);
    const real max_iy = static_cast<real>(g.height - 1);
    const auto ix0 = static_cast<std::size_t>(
        std::clamp(std::floor((p.x - search - x_min) / res), real(0), max_ix));
    const auto ix1 = static_cast<std::size_t>(
        std::clamp(std::floor((p.x + search - x_min) / res), real(0), max_ix));
    const auto iy0 = static_cast<std::size_t>(
        std::clamp(std::floor((p.y - search - y_min) / res), real(0), max_iy));
    const auto iy1 = static_cast<std::size_t>(
        std::clamp(std::floor((p.y + search - y_min) / res), real(0), max_iy));

    for (std::size_t iy = iy0; iy <= iy1; ++iy) {
        for (std::size_t ix = ix0; ix <= ix1; ++ix) {
            if (g.cells[iy * g.width + ix] == CellState::Free) {
                continue;
            }
            const Point2D c{
                x_min + (static_cast<real>(ix) + real(0.5)) * res,
                y_min + (static_cast<real>(iy) + real(0.5)) * res};
            const real d = std::hypot(c.x - p.x, c.y - p.y) - half_diag;
            best = std::min(best, d);
        }
    }
    return best;
}

// Read-only view of a mutable grid.
inline GridMap as_const(const MutableGridMap& grid) {
    return GridMap{grid.origin, grid.width, grid.height, grid.cells};
}

// Replaceable map layer kinds [FROM: AGENTS.md section 2].
enum class LayerKind {
    FixedBorders,
    SlamOverlay,
    Inflation,
};

// One replaceable map layer.
class IMapLayer {
public:
    virtual ~IMapLayer() = default;

    virtual LayerKind kind() const = 0;

    // Grid this layer contributes. Returns an empty grid if the layer has none.
    virtual GridMap grid() const = 0;
};

// Layer that wraps a caller-owned grid view.
class GridLayer final : public IMapLayer {
public:
    GridLayer(LayerKind kind, GridMap grid) : kind_(kind), grid_(grid) {}

    LayerKind kind() const override { return kind_; }
    GridMap grid() const override { return grid_; }

private:
    LayerKind kind_;
    GridMap grid_;
};

// Map manager interface.
class IMapManager {
public:
    virtual ~IMapManager() = default;

    // Return the layer of the given kind, or nullptr if none is present.
    virtual const IMapLayer* layer(LayerKind kind) const = 0;
};

// Concrete map manager. Storage for every grid is owned by the caller.
//
// Per cycle, update():
//   1. Validates placeholders, pose, LiDAR beams and grid geometry. Any
//      failure returns Status::Fault and changes no grid.
//   2. Ray-casts each beam into the SLAM overlay (in place). Cells the beam
//      passes through are Free; the endpoint is Occupied when the return is
//      within LIDAR_MAX_RANGE_M and inside the grid. Parts of a beam
//      outside the grid are clipped silently.
//   3. Rebuilds the inflation layer from occupied cells of the fixed borders
//      and SLAM overlay. Inflation cells are Occupied; all others Unknown.
//   4. Composes the layers into the output grid: Occupied if any layer is
//      Occupied, else Free if any layer is Free, else Unknown.
//   5. Bumps map_generation() on the first successful update, and afterwards
//      only when a composed cell changed value.
//
// All four grids must share the same origin, width and height. A mismatch
// returns Status::Fault. The writable grids (SLAM overlay, inflation,
// composed) must not share storage; overlap returns Status::Fault.
class GridMapManager final : public IMapManager {
public:
    GridMapManager(GridMap fixed_borders,
                   MutableGridMap slam_overlay,
                   MutableGridMap inflation,
                   MutableGridMap composed);

    const IMapLayer* layer(LayerKind kind) const override;

    // Output of the most recent successful update().
    GridMap composed() const { return as_const(composed_); }

    // Increments by one on the first successful update(), and afterwards only
    // when the composed grid content changes. Starts at 0, which means no
    // update has succeeded yet. Consumers compare values to detect a map
    // change [FROM: Global planner Phase A plan, approved; change-only rule
    // FROM: map-generation Phase A plan, approved].
    std::uint64_t map_generation() const { return map_generation_; }

    // Runs one map update for the given pose and LiDAR scan.
    // Pose is the base_link pose in the map frame [FROM: AGENTS.md section 2].
    Status update(const Pose2D& pose, const LidarView& lidar);

private:
    GridLayer fixed_layer_;
    GridLayer slam_layer_;
    GridLayer inflation_layer_;
    MutableGridMap slam_;
    MutableGridMap inflation_;
    MutableGridMap composed_;
    std::uint64_t map_generation_ = 0;
};

}  // namespace core::map
