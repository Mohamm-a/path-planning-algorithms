// Map manager stage interface. Declarations only; implementation is in
// src/map_manager.cpp. Fixed borders, SLAM overlay and inflation are
// replaceable layers, not branches inside planners [FROM: AGENTS.md section 2].

#pragma once

#include <cstddef>
#include <cstdint>

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
inline constexpr double kCellResolution_m = CORE_PH_MAP_RESOLUTION_M;

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
//      within CORE_PH_LIDAR_MAX_RANGE_M and inside the grid. Parts of a beam
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
