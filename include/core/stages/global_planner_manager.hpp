// Global planner manager stage interface. Declarations only; implementation
// is in src/global_planner_manager.cpp. Selectable via strategy/factory.
// Produces a topological guide only, and runs on demand (goal change, map
// change, timeout), not every cycle [FROM: AGENTS.md section 2].

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "core/core.hpp"
#include "core/placeholders.hpp"
#include "core/stages/map_manager.hpp"

namespace core::global_planner {

// Non-owning view over ordered map-frame waypoints (pointer plus count).
// The caller owns the storage (see WaypointBuffer). No allocation occurs
// inside the core.
struct PointView {
    const Point2D* data;  // may be null only when count is 0
    std::size_t count;    // number of valid entries at data
};

// Topological guide produced by a global planner.
// waypoints is ordered from start to goal, in the map frame (m).
// It is empty unless status is Status::Valid.
struct GlobalGuide {
    Status status;
    PointView waypoints;
};

// Runtime configuration. No defaults: the caller must supply every field.
// plan() returns Status::Fault if max_expansions is 0.
struct GlobalPlannerConfig {
    std::size_t max_expansions;  // nodes expanded per plan. Exceeding it
                                 // returns Status::PlannerTimeout. Stands in
                                 // for wall-clock time, which the core may not
                                 // read [FROM: Phase A plan, approved].
};

// Caller-owned output storage for waypoints (non-owning pointer plus capacity).
// plan() writes into data[0 .. count). If count exceeds capacity, plan()
// returns Status::Fault and writes nothing. It never truncates.
struct WaypointBuffer {
    Point2D* data;            // may be null only when capacity is 0
    std::size_t capacity;     // number of Point2D entries at data
};

// Caller-owned scratch memory for one search (non-owning pointers). Every
// array covers at least `capacity` cells, and capacity must be >= width *
// height of the grid being planned. A smaller capacity or a null pointer
// returns Status::Fault.
struct PlannerWorkspace {
    std::size_t capacity;      // cells covered by each array below
    double* g_cost;            // cost from start, in cell units
    std::size_t* parent;       // predecessor cell index
    bool* closed;              // expanded flag
    std::size_t* heap;         // open list: binary heap of cell indices
    std::size_t* heap_slot;    // heap position of each cell, or not-in-heap
};

// Caller-owned record of the last plan. The caller keeps one per planner
// instance and passes it in. The core keeps no hidden state [FROM: AGENTS.md
// section 5].
struct PlanCache {
    bool valid;                    // false until record_plan() is called
    Point2D goal;                  // m, m; map frame
    std::uint64_t map_generation;  // GridMapManager::map_generation() at plan time
    double planned_at;             // s; caller-defined epoch
};

// Global planner strategy interface (A*, D*-Lite, RRT).
class IGlobalPlanner {
public:
    virtual ~IGlobalPlanner() = default;

    // Plans from start to goal over the given composed grid. Map-frame
    // positions are converted to cells with kCellResolution_m and the grid
    // origin. Cells that are not Free are not traversable (see
    // Status::NoPath). Start or goal Occupied, or outside the grid, returns
    // Status::Fault.
    virtual GlobalGuide plan(Point2D start, Point2D goal, map::GridMap map,
                             const GlobalPlannerConfig& config,
                             PlannerWorkspace& workspace,
                             WaypointBuffer& out) const = 0;
};

// Factory. AStar returns the A* planner. DStarLite and RRT return a planner
// whose plan() always returns Status::Fault (not implemented yet; no silent
// fallback to another algorithm).
std::unique_ptr<IGlobalPlanner> make_global_planner(GlobalPlannerKind kind);

// True when the global planner must run again: no valid cache, goal changed,
// map generation changed, the period has elapsed, or now is before planned_at
// (clock went backwards or is NaN; replanning is the safe choice). The period
// is GLOBAL_PLANNER_REPLAN_PERIOD_S (PH-0017). initialize() refuses to
// operate while it is unreplaced, so this function is not reached in that state.
bool needs_replan(const PlanCache& cache, Point2D goal,
                  std::uint64_t map_generation, double now);

// Stores the goal, map generation and time of a plan run. Call after every
// plan() attempt, so that NoPath does not trigger a replan every cycle.
void record_plan(PlanCache& cache, Point2D goal,
                 std::uint64_t map_generation, double now);

}  // namespace core::global_planner
