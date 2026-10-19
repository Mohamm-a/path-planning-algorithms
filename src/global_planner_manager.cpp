// Global planner manager implementation. Interface: core/stages/global_planner_manager.hpp.
// A* over the composed grid. D*-Lite and RRT are not implemented and return
// Status::Fault. No allocation, no clock reads, no hidden state.

#include "core/stages/global_planner_manager.hpp"

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>

namespace core::global_planner {
namespace {

using map::CellState;

constexpr std::size_t kNoCell = std::numeric_limits<std::size_t>::max();

// Cost of a diagonal step, in cell units. Orthogonal steps cost 1.
// [FROM: standard 8-connected grid geometry]
const double kDiagonalCost = std::sqrt(2.0);

// Returns true when the grid can be searched: non-empty, storage present,
// and width * height representable.
bool grid_is_usable(const map::GridMap& grid) {
    if (grid.width == 0 || grid.height == 0 || grid.cells == nullptr) {
        return false;
    }
    return grid.width <= std::numeric_limits<std::size_t>::max() / grid.height;
}

// Returns true when every scratch array covers cell_count cells.
bool workspace_is_usable(const PlannerWorkspace& ws, std::size_t cell_count) {
    return ws.capacity >= cell_count && ws.g_cost != nullptr &&
           ws.parent != nullptr && ws.closed != nullptr &&
           ws.heap != nullptr && ws.heap_slot != nullptr;
}

// Maps a map-frame point to a cell index. Returns false if the point is
// non-finite or outside the grid.
bool point_to_cell(const map::GridMap& grid, Point2D p, std::size_t& ix,
                   std::size_t& iy) {
    const double fx = (p.x - grid.origin.x) / map::kCellResolution_m;
    const double fy = (p.y - grid.origin.y) / map::kCellResolution_m;
    if (!std::isfinite(fx) || !std::isfinite(fy) || fx < 0.0 || fy < 0.0) {
        return false;
    }
    const double cx = std::floor(fx);
    const double cy = std::floor(fy);
    if (cx >= static_cast<double>(grid.width) ||
        cy >= static_cast<double>(grid.height)) {
        return false;
    }
    ix = static_cast<std::size_t>(cx);
    iy = static_cast<std::size_t>(cy);
    return true;
}

// Map-frame centre of cell (ix, iy). Waypoints are cell centres
// [ASSUMED: cell-centre convention, Phase A assumption 2].
Point2D cell_centre(const map::GridMap& grid, std::size_t ix, std::size_t iy) {
    return Point2D{
        grid.origin.x + (static_cast<double>(ix) + 0.5) * map::kCellResolution_m,
        grid.origin.y + (static_cast<double>(iy) + 0.5) * map::kCellResolution_m,
    };
}

// Footprint half-width in cells: ceil((R + margin) / resolution), at least 1.
// Returns false unless CORE_PH_ROBOT_RADIUS_M > 0 (PH-0018) and
// CORE_PH_PATH_SMOOTH_MARGIN_M >= 0 (PH-0021). [ASSUMED: minimum of 1 cell,
// Phase A assumption 3]
bool footprint_radius_cells(long long& radius_cells) {
    const double r = CORE_PH_ROBOT_RADIUS_M;
    const double margin = CORE_PH_PATH_SMOOTH_MARGIN_M;
    if (!std::isfinite(r) || !(r > 0.0) || !std::isfinite(margin) ||
        margin < 0.0) {
        return false;
    }
    const double cells = std::ceil((r + margin) / map::kCellResolution_m);
    if (!std::isfinite(cells)) {
        return false;
    }
    radius_cells = cells < 1.0 ? 1 : static_cast<long long>(cells);
    return true;
}

// Cells around each planned node that must be Free, so the guide keeps the
// clearance TEB enforces: radius + half cell diagonal, rounded up to cells.
// TEB measures clearance to blocked cell centres minus a half diagonal
// (teb_local_planner.cpp clearance()). Returns false unless
// CORE_PH_ROBOT_RADIUS_M > 0 (PH-0018).
bool clearance_radius_cells(long long& radius_cells) {
    const double r = CORE_PH_ROBOT_RADIUS_M;
    if (!std::isfinite(r) || !(r > 0.0)) {
        return false;
    }
    const double half_diag = 0.5 * map::kCellResolution_m * std::sqrt(2.0);
    const double cells = std::ceil((r + half_diag) / map::kCellResolution_m);
    if (!std::isfinite(cells)) {
        return false;
    }
    radius_cells = static_cast<long long>(cells);
    return true;
}

// Maximum smoothed segment length in cells: CORE_PH_PATH_SMOOTH_MAX_SEGMENT_M
// divided by the cell size (PH-0022). Returns false unless the cap is finite,
// > 0, and at least one cell diagonal, so the smoother can always take one grid
// step without breaking the cap.
bool segment_cap_cells(double& cap_cells) {
    const double cap_m = CORE_PH_PATH_SMOOTH_MAX_SEGMENT_M;
    if (!std::isfinite(cap_m) || !(cap_m > 0.0)) {
        return false;
    }
    const double cells = cap_m / map::kCellResolution_m;
    if (!(cells >= std::sqrt(2.0))) {
        return false;
    }
    cap_cells = cells;
    return true;
}

// True when the straight distance between two cells is at most `max_cells`.
bool within_length(const map::GridMap& grid, std::size_t from, std::size_t to,
                   double max_cells) {
    const double dx = static_cast<double>(from % grid.width) -
                      static_cast<double>(to % grid.width);
    const double dy = static_cast<double>(from / grid.width) -
                      static_cast<double>(to / grid.width);
    return dx * dx + dy * dy <= max_cells * max_cells;
}

// True when every cell within `radius` of (cx, cy) is inside the grid and Free.
// Out-of-grid cells count as blocked.
bool disk_is_free(const map::GridMap& grid, long long cx, long long cy,
                  long long radius) {
    const long long w = static_cast<long long>(grid.width);
    const long long h = static_cast<long long>(grid.height);
    for (long long dy = -radius; dy <= radius; ++dy) {
        for (long long dx = -radius; dx <= radius; ++dx) {
            if (dx * dx + dy * dy > radius * radius) {
                continue;
            }
            const long long x = cx + dx;
            const long long y = cy + dy;
            if (x < 0 || y < 0 || x >= w || y >= h) {
                return false;
            }
            if (grid.cells[static_cast<std::size_t>(y * w + x)] != CellState::Free) {
                return false;
            }
        }
    }
    return true;
}

// Bresenham walk from cell `from` to cell `to`. Every visited cell's footprint
// disk must be clear. Both endpoints are included in the walk.
bool line_is_clear(const map::GridMap& grid, std::size_t from, std::size_t to,
                    long long radius) {
    const long long w = static_cast<long long>(grid.width);
    long long x = static_cast<long long>(from) % w;
    long long y = static_cast<long long>(from) / w;
    const long long x1 = static_cast<long long>(to) % w;
    const long long y1 = static_cast<long long>(to) / w;
    const long long dx = std::llabs(x1 - x);
    const long long dy = -std::llabs(y1 - y);
    const long long sx = x < x1 ? 1 : -1;
    const long long sy = y < y1 ? 1 : -1;
    long long err = dx + dy;
    for (;;) {
        if (!disk_is_free(grid, x, y, radius)) {
            return false;
        }
        if (x == x1 && y == y1) {
            return true;
        }
        const long long e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y += sy;
        }
    }
}

// Greedy string pulling over chain[0 .. count), start first. From each kept
// cell, jump to the farthest later cell reached by a clear line no longer than
// `max_cells` (PH-0022). If none is clear, take the next cell, which is already
// a valid grid step. Kept cells are written at an index <= the index last read,
// so no unread entry is overwritten. Returns the new count.
std::size_t smooth_chain(const map::GridMap& grid, std::size_t* chain,
                         std::size_t count, long long radius,
                         double max_cells) {
    if (count <= 2) {
        return count;
    }
    std::size_t kept = 1;
    std::size_t cur = 0;
    while (cur + 1 < count) {
        std::size_t next = cur + 1;
        for (std::size_t j = count - 1; j > cur + 1; --j) {
            if (within_length(grid, chain[cur], chain[j], max_cells) &&
                line_is_clear(grid, chain[cur], chain[j], radius)) {
                next = j;
                break;
            }
        }
        chain[kept++] = chain[next];
        cur = next;
    }
    return kept;
}

// Search state for one plan() call. Holds references to caller storage only.
struct Search {
    const map::GridMap& grid;
    PlannerWorkspace& ws;
    std::size_t goal_cell;
    std::size_t goal_x;
    std::size_t goal_y;
    std::size_t heap_size;
};

// Octile distance to the goal, in cell units. Admissible and consistent for
// 8-connected moves with costs 1 and sqrt(2).
double heuristic(const Search& s, std::size_t cell) {
    const std::size_t w = s.grid.width;
    const std::size_t x = cell % w;
    const std::size_t y = cell / w;
    const double dx = static_cast<double>(x > s.goal_x ? x - s.goal_x : s.goal_x - x);
    const double dy = static_cast<double>(y > s.goal_y ? y - s.goal_y : s.goal_y - y);
    const double major = dx > dy ? dx : dy;
    const double minor = dx > dy ? dy : dx;
    return major + (kDiagonalCost - 1.0) * minor;
}

double f_cost(const Search& s, std::size_t cell) {
    return s.ws.g_cost[cell] + heuristic(s, cell);
}

// Heap order: lower f first; ties broken by lower cell index. Deterministic.
bool before(const Search& s, std::size_t a, std::size_t b) {
    const double fa = f_cost(s, a);
    const double fb = f_cost(s, b);
    if (fa != fb) {
        return fa < fb;
    }
    return a < b;
}

void place(const Search& s, std::size_t pos, std::size_t cell) {
    s.ws.heap[pos] = cell;
    s.ws.heap_slot[cell] = pos;
}

void sift_up(const Search& s, std::size_t pos) {
    const std::size_t cell = s.ws.heap[pos];
    while (pos > 0) {
        const std::size_t parent_pos = (pos - 1) / 2;
        const std::size_t parent = s.ws.heap[parent_pos];
        if (!before(s, cell, parent)) {
            break;
        }
        place(s, pos, parent);
        pos = parent_pos;
    }
    place(s, pos, cell);
}

void sift_down(const Search& s, std::size_t pos) {
    const std::size_t cell = s.ws.heap[pos];
    for (;;) {
        std::size_t child_pos = 2 * pos + 1;
        if (child_pos >= s.heap_size) {
            break;
        }
        if (child_pos + 1 < s.heap_size &&
            before(s, s.ws.heap[child_pos + 1], s.ws.heap[child_pos])) {
            ++child_pos;
        }
        if (!before(s, s.ws.heap[child_pos], cell)) {
            break;
        }
        place(s, pos, s.ws.heap[child_pos]);
        pos = child_pos;
    }
    place(s, pos, cell);
}

void push(Search& s, std::size_t cell) {
    place(s, s.heap_size, cell);
    ++s.heap_size;
    sift_up(s, s.heap_size - 1);
}

std::size_t pop(Search& s) {
    const std::size_t top = s.ws.heap[0];
    s.ws.heap_slot[top] = kNoCell;
    --s.heap_size;
    if (s.heap_size > 0) {
        place(s, 0, s.ws.heap[s.heap_size]);
        sift_down(s, 0);
    }
    return top;
}

class AStarPlanner final : public IGlobalPlanner {
public:
    GlobalGuide plan(Point2D start, Point2D goal, map::GridMap map,
                     const GlobalPlannerConfig& config,
                     PlannerWorkspace& ws, WaypointBuffer& out) const override {
        const PointView none{nullptr, 0};
        const GlobalGuide fault{Status::Fault, none};
        const GlobalGuide no_path{Status::NoPath, none};
        const GlobalGuide timeout{Status::PlannerTimeout, none};

        // Refuse to run without a valid resolution or grid. PH-0008 is
        // unreplaced until initialize() passes.
        if (!std::isfinite(map::kCellResolution_m) ||
            !(map::kCellResolution_m > 0.0) || !grid_is_usable(map)) {
            return fault;
        }
        if (config.max_expansions == 0) {
            return fault;
        }
        // Smoothing needs a usable footprint radius and margin (PH-0018, PH-0021).
        long long footprint_cells = 0;
        if (!footprint_radius_cells(footprint_cells)) {
            return fault;
        }
        // Smoothed segment cap (PH-0022).
        double max_segment_cells = 0.0;
        if (!segment_cap_cells(max_segment_cells)) {
            return fault;
        }
        // Search clearance around each node (PH-0018).
        long long clearance_cells = 0;
        if (!clearance_radius_cells(clearance_cells)) {
            return fault;
        }
        const std::size_t w = map.width;
        const std::size_t h = map.height;
        const std::size_t cell_count = w * h;
        if (!workspace_is_usable(ws, cell_count)) {
            return fault;
        }

        std::size_t sx = 0;
        std::size_t sy = 0;
        std::size_t gx = 0;
        std::size_t gy = 0;
        if (!point_to_cell(map, start, sx, sy) || !point_to_cell(map, goal, gx, gy)) {
            return fault;
        }
        const std::size_t start_cell = sy * w + sx;
        const std::size_t goal_cell = gy * w + gx;
        if (map.cells[start_cell] == CellState::Occupied ||
            map.cells[goal_cell] == CellState::Occupied) {
            return fault;
        }
        // Unknown cells are not traversable [FROM: Phase A decision].
        if (map.cells[start_cell] == CellState::Unknown ||
            map.cells[goal_cell] == CellState::Unknown) {
            return no_path;
        }

        for (std::size_t i = 0; i < cell_count; ++i) {
            ws.g_cost[i] = std::numeric_limits<double>::infinity();
            ws.parent[i] = kNoCell;
            ws.closed[i] = false;
            ws.heap_slot[i] = kNoCell;
        }

        Search s{map, ws, goal_cell, gx, gy, 0};
        ws.g_cost[start_cell] = 0.0;
        push(s, start_cell);

        std::size_t expansions = 0;
        bool found = false;
        while (s.heap_size > 0) {
            const std::size_t cur = pop(s);
            if (cur == goal_cell) {
                found = true;
                break;
            }
            if (expansions == config.max_expansions) {
                return timeout;
            }
            ++expansions;
            ws.closed[cur] = true;

            const std::size_t cx = cur % w;
            const std::size_t cy = cur / w;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) {
                        continue;
                    }
                    const long long nx = static_cast<long long>(cx) + dx;
                    const long long ny = static_cast<long long>(cy) + dy;
                    if (nx < 0 || ny < 0 || nx >= static_cast<long long>(w) ||
                        ny >= static_cast<long long>(h)) {
                        continue;
                    }
                    const std::size_t ux = static_cast<std::size_t>(nx);
                    const std::size_t uy = static_cast<std::size_t>(ny);
                    const std::size_t n = uy * w + ux;
                    if (ws.closed[n] || !disk_is_free(map, static_cast<long long>(ux),
                                                      static_cast<long long>(uy),
                                                      clearance_cells)) {
                        continue;
                    }
                    const bool diagonal = dx != 0 && dy != 0;
                    // No corner cutting: both orthogonal neighbours must be
                    // Free [ASSUMED: conservative rule, see report].
                    if (diagonal && (map.cells[cy * w + ux] != CellState::Free ||
                                     map.cells[uy * w + cx] != CellState::Free)) {
                        continue;
                    }
                    const double step = diagonal ? kDiagonalCost : 1.0;
                    const double tentative = ws.g_cost[cur] + step;
                    if (tentative < ws.g_cost[n]) {
                        ws.parent[n] = cur;
                        ws.g_cost[n] = tentative;
                        if (ws.heap_slot[n] == kNoCell) {
                            push(s, n);
                        } else {
                            sift_up(s, ws.heap_slot[n]);
                        }
                    }
                }
            }
        }
        if (!found) {
            return no_path;
        }

        std::size_t count = 1;
        for (std::size_t c = goal_cell; c != start_cell; c = ws.parent[c]) {
            ++count;
        }
        if (out.data == nullptr || count > out.capacity) {
            return fault;
        }

        // Copy the parent chain into ws.heap, start first. The open list is
        // finished, so the array can be reused for this. [ASSUMED: reuse is safe
        // after the search; see Phase A]
        std::size_t k = count;
        std::size_t c = goal_cell;
        for (;;) {
            --k;
            ws.heap[k] = c;
            if (c == start_cell) {
                break;
            }
            c = ws.parent[c];
        }
        const std::size_t kept =
            smooth_chain(map, ws.heap, count, footprint_cells, max_segment_cells);
        for (std::size_t i = 0; i < kept; ++i) {
            const std::size_t cell = ws.heap[i];
            out.data[i] = cell_centre(map, cell % w, cell / w);
        }
        return GlobalGuide{Status::Valid, PointView{out.data, kept}};
    }
};

// Used for strategies that are not implemented yet. Always Status::Fault, so
// the caller sees the failure rather than a silent switch to another planner.
class UnimplementedPlanner final : public IGlobalPlanner {
public:
    GlobalGuide plan(Point2D, Point2D, map::GridMap, const GlobalPlannerConfig&,
                     PlannerWorkspace&, WaypointBuffer&) const override {
        return GlobalGuide{Status::Fault, PointView{nullptr, 0}};
    }
};

}  // namespace

std::unique_ptr<IGlobalPlanner> make_global_planner(GlobalPlannerKind kind) {
    switch (kind) {
        case GlobalPlannerKind::AStar:
            return std::make_unique<AStarPlanner>();
        case GlobalPlannerKind::DStarLite:
        case GlobalPlannerKind::RRT:
            return std::make_unique<UnimplementedPlanner>();
    }
    return std::make_unique<UnimplementedPlanner>();
}

bool needs_replan(const PlanCache& cache, Point2D goal,
                  std::uint64_t map_generation, double now) {
    if (!cache.valid) {
        return true;
    }
    if (goal.x != cache.goal.x || goal.y != cache.goal.y) {
        return true;
    }
    if (map_generation != cache.map_generation) {
        return true;
    }
    // Also true for NaN and for a clock that went backwards.
    if (!(now >= cache.planned_at)) {
        return true;
    }
    return (now - cache.planned_at) >= CORE_PH_GLOBAL_PLANNER_REPLAN_PERIOD_S;
}

void record_plan(PlanCache& cache, Point2D goal,
                 std::uint64_t map_generation, double now) {
    cache.valid = true;
    cache.goal = goal;
    cache.map_generation = map_generation;
    cache.planned_at = now;
}

}  // namespace core::global_planner
