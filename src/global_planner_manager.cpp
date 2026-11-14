// Global planner manager implementation. Interface:
// core/stages/global_planner_manager.hpp.
//
// A*, D*-Lite and RRT share one clearance definition: every search and every
// smoothed segment uses map::clearance() with exactly ROBOT_RADIUS_M (plus the
// optional smoothing margin). TEB uses the same map::clearance() function, so
// the guide and the local planner agree on the physical boundary
// [FROM: user approval, clearance unification].
//
// No allocation, no clock reads, no hidden state. A* and RRT are stateless
// between calls; D*-Lite keeps its persistent arrays in the caller-owned
// PlannerWorkspace::dstar state.

#include "core/stages/global_planner_manager.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>

namespace core::global_planner {
namespace {

using map::CellState;

constexpr std::size_t kNoCell = std::numeric_limits<std::size_t>::max();

// Cost of a diagonal step, in cell units. Orthogonal steps cost 1.
// [FROM: standard 8-connected grid geometry]
constexpr real kDiagonalCost = real(1.4142135623730951);

// D*-Lite-only path shaping: prefer cells with extra wall clearance without
// changing the physical ROBOT_RADIUS_M check. A 0.10 m soft margin at weight
// 20 is enough to keep the same safe route under -O0 and -O2/FMA; it does not
// reject a path that still satisfies the strict 0.30 m check. [ASSUMED]
constexpr real kDstarClearanceTarget = real(0.40);
constexpr real kDstarClearanceWeight = real(20.0);

real dstar_step_cost(const map::GridMap& grid, std::size_t to, bool diagonal,
                     real radius) {
    real cost = diagonal ? kDiagonalCost : real(1);
    const Point2D centre{
        grid.origin.x +
            (static_cast<real>(to % grid.width) + real(0.5)) *
                map::kCellResolution_m,
        grid.origin.y +
            (static_cast<real>(to / grid.width) + real(0.5)) *
                map::kCellResolution_m,
    };
    const real clearance = map::clearance(grid, centre, radius);
    if (clearance < kDstarClearanceTarget) {
        cost += kDstarClearanceWeight *
                (kDstarClearanceTarget - clearance);
    }
    return cost;
}

// Returns true when the grid can be searched: non-empty, storage present,
// and width * height representable.
bool grid_is_usable(const map::GridMap& grid) {
    if (grid.width == 0 || grid.height == 0 || grid.cells == nullptr) {
        return false;
    }
    return grid.width <= std::numeric_limits<std::size_t>::max() / grid.height;
}

// Returns true when every A* scratch array covers cell_count cells.
bool workspace_is_usable(const PlannerWorkspace& ws, std::size_t cell_count) {
    return ws.capacity >= cell_count && ws.g_cost != nullptr &&
           ws.parent != nullptr && ws.closed != nullptr &&
           ws.heap != nullptr && ws.heap_slot != nullptr;
}

// Returns true when every persistent D*-Lite array covers cell_count cells.
bool dstar_workspace_is_usable(const PlannerWorkspace& ws,
                               std::size_t cell_count) {
    const DStarWorkspace& d = ws.dstar;
    return ws.capacity >= cell_count && ws.parent != nullptr &&
           ws.heap != nullptr && d.capacity >= cell_count && d.g_cost != nullptr &&
           d.rhs != nullptr && d.key1 != nullptr && d.key2 != nullptr &&
           d.heap != nullptr && d.heap_slot != nullptr &&
           d.previous_map != nullptr && d.dirty != nullptr;
}

// Returns true when the RRT tree/path arrays cover max_nodes.
bool rrt_workspace_is_usable(const PlannerWorkspace& ws, std::size_t max_nodes) {
    return ws.capacity > 0 && ws.heap != nullptr && ws.rrt.capacity >= max_nodes &&
           ws.rrt.nodes != nullptr && ws.rrt.parents != nullptr &&
           ws.rrt.path != nullptr;
}

// Maps a map-frame point to a cell index. Returns false if the point is
// non-finite or outside the grid.
bool point_to_cell(const map::GridMap& grid, Point2D p, std::size_t& ix,
                   std::size_t& iy) {
    const real fx = (p.x - grid.origin.x) / map::kCellResolution_m;
    const real fy = (p.y - grid.origin.y) / map::kCellResolution_m;
    if (!std::isfinite(fx) || !std::isfinite(fy) || fx < 0.0 || fy < 0.0) {
        return false;
    }
    const real cx = std::floor(fx);
    const real cy = std::floor(fy);
    if (cx >= static_cast<real>(grid.width) ||
        cy >= static_cast<real>(grid.height)) {
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
        grid.origin.x + (static_cast<real>(ix) + real(0.5)) * map::kCellResolution_m,
        grid.origin.y + (static_cast<real>(iy) + real(0.5)) * map::kCellResolution_m,
    };
}

// Octile distance between two cells, in cell units. Admissible and
// consistent for 8-connected moves with costs 1 and sqrt(2).
real octile_distance(const map::GridMap& grid, std::size_t a, std::size_t b) {
    const std::size_t w = grid.width;
    const std::size_t ax = a % w;
    const std::size_t ay = a / w;
    const std::size_t bx = b % w;
    const std::size_t by = b / w;
    const real dx = static_cast<real>(ax > bx ? ax - bx : bx - ax);
    const real dy = static_cast<real>(ay > by ? ay - by : by - ay);
    const real major = dx > dy ? dx : dy;
    const real minor = dx > dy ? dy : dx;
    return major + (kDiagonalCost - 1.0) * minor;
}

// Maximum smoothed segment length in cells: PATH_SMOOTH_MAX_SEGMENT_M
// divided by the cell size (PH-0022). Returns false unless the cap is finite,
// > 0, and at least one cell diagonal, so the smoother can always take one grid
// step without breaking the cap.
bool segment_cap_cells(real& cap_cells) {
    const real cap_m = PATH_SMOOTH_MAX_SEGMENT_M;
    if (!std::isfinite(cap_m) || !(cap_m > 0.0)) {
        return false;
    }
    const real cells = cap_m / map::kCellResolution_m;
    if (!(cells >= std::sqrt(2.0))) {
        return false;
    }
    cap_cells = cells;
    return true;
}

// Every cell along the Bresenham line must have at least `radius` clearance.
// This replaces the old fixed-cell disk: the boundary is exactly the physical
// ROBOT_RADIUS_M, measured by the same map::clearance() function TEB uses.
// The Bresenham walk preserves the existing line-of-sight behaviour
// [FROM: user approval, clearance unification].
bool line_is_clear(const map::GridMap& grid, std::size_t from,
                   std::size_t to, real radius) {
    if (!std::isfinite(radius) || radius < real(0)) {
        return false;
    }
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
        const Point2D p{
            grid.origin.x +
                (static_cast<real>(x) + real(0.5)) * map::kCellResolution_m,
            grid.origin.y +
                (static_cast<real>(y) + real(0.5)) * map::kCellResolution_m,
        };
        if (map::clearance(grid, p, radius) < radius) {
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

// True when the straight distance between two cells is at most `max_cells`.
bool within_length(const map::GridMap& grid, std::size_t from, std::size_t to,
                   real max_cells) {
    const real dx = static_cast<real>(from % grid.width) -
                    static_cast<real>(to % grid.width);
    const real dy = static_cast<real>(from / grid.width) -
                    static_cast<real>(to / grid.width);
    return dx * dx + dy * dy <= max_cells * max_cells;
}

// Greedy string pulling over chain[0 .. count), start first. From each kept
// cell, jump to the farthest later cell reached by a clear line no longer than
// `max_cells` (PH-0022). If none is clear, take the next cell, which is already
// a valid grid step. Kept cells are written at an index <= the index last read,
// so no unread entry is overwritten. Returns the new count.
std::size_t smooth_chain(const map::GridMap& grid, std::size_t* chain,
                         std::size_t count, real radius, real max_cells) {
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

// Search state for one A* plan() call. Holds references to caller storage only.
struct Search {
    const map::GridMap& grid;
    PlannerWorkspace& ws;
    std::size_t goal_cell;
    std::size_t heap_size;
};

// Octile distance to the goal, in cell units.
real heuristic(const Search& s, std::size_t cell) {
    return octile_distance(s.grid, cell, s.goal_cell);
}

real f_cost(const Search& s, std::size_t cell) {
    return s.ws.g_cost[cell] + heuristic(s, cell);
}

// Heap order: lower f first; ties broken by lower cell index. Deterministic.
bool before(const Search& s, std::size_t a, std::size_t b) {
    const real fa = f_cost(s, a);
    const real fb = f_cost(s, b);
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

// D*-Lite key helpers. Keys are stored per cell so the open list can update
// and remove entries without touching unrelated entries.
real dstar_key1(const DStarWorkspace& d, const map::GridMap& grid,
                std::size_t cell) {
    const real m = std::min(d.g_cost[cell], d.rhs[cell]);
    return m + octile_distance(grid, cell, d.start_cell) + d.km;
}

void dstar_set_key(DStarWorkspace& d, const map::GridMap& grid,
                   std::size_t cell) {
    const real m = std::min(d.g_cost[cell], d.rhs[cell]);
    d.key1[cell] = dstar_key1(d, grid, cell);
    d.key2[cell] = m;
}

bool dstar_before(const DStarWorkspace& d, std::size_t a, std::size_t b) {
    if (d.key1[a] != d.key1[b]) {
        return d.key1[a] < d.key1[b];
    }
    if (d.key2[a] != d.key2[b]) {
        return d.key2[a] < d.key2[b];
    }
    return a < b;
}

void dstar_place(DStarWorkspace& d, std::size_t pos, std::size_t cell) {
    d.heap[pos] = cell;
    d.heap_slot[cell] = pos;
}

void dstar_sift_up(DStarWorkspace& d, std::size_t pos) {
    const std::size_t cell = d.heap[pos];
    while (pos > 0) {
        const std::size_t parent_pos = (pos - 1) / 2;
        const std::size_t parent = d.heap[parent_pos];
        if (!dstar_before(d, cell, parent)) {
            break;
        }
        dstar_place(d, pos, parent);
        pos = parent_pos;
    }
    dstar_place(d, pos, cell);
}

void dstar_sift_down(DStarWorkspace& d, std::size_t pos) {
    const std::size_t cell = d.heap[pos];
    for (;;) {
        std::size_t child_pos = 2 * pos + 1;
        if (child_pos >= d.heap_size) {
            break;
        }
        if (child_pos + 1 < d.heap_size &&
            dstar_before(d, d.heap[child_pos + 1], d.heap[child_pos])) {
            ++child_pos;
        }
        if (!dstar_before(d, d.heap[child_pos], cell)) {
            break;
        }
        dstar_place(d, pos, d.heap[child_pos]);
        pos = child_pos;
    }
    dstar_place(d, pos, cell);
}

void dstar_push(DStarWorkspace& d, std::size_t cell) {
    dstar_place(d, d.heap_size, cell);
    ++d.heap_size;
    dstar_sift_up(d, d.heap_size - 1);
}

std::size_t dstar_pop(DStarWorkspace& d) {
    const std::size_t top = d.heap[0];
    d.heap_slot[top] = kNoCell;
    --d.heap_size;
    if (d.heap_size > 0) {
        dstar_place(d, 0, d.heap[d.heap_size]);
        dstar_sift_down(d, 0);
    }
    return top;
}

void dstar_remove(DStarWorkspace& d, std::size_t cell) {
    const std::size_t pos = d.heap_slot[cell];
    if (pos == kNoCell) {
        return;
    }
    d.heap_slot[cell] = kNoCell;
    --d.heap_size;
    if (pos >= d.heap_size) {
        return;
    }
    dstar_place(d, pos, d.heap[d.heap_size]);
    dstar_sift_up(d, pos);
    if (pos < d.heap_size) {
        dstar_sift_down(d, pos);
    }
}

// Recompute rhs for one cell and update its queue membership. Every edge uses
// the shared line clearance, so D*-Lite has the same boundary as A* and TEB.
void dstar_update_vertex(const map::GridMap& grid, DStarWorkspace& d,
                         std::size_t cell, real radius) {
    if (cell == d.goal_cell) {
        d.rhs[cell] = real(0);
    } else {
        real best = std::numeric_limits<real>::infinity();
        const std::size_t w = grid.width;
        const std::size_t h = grid.height;
        const std::size_t cx = cell % w;
        const std::size_t cy = cell / w;
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
                const std::size_t n =
                    static_cast<std::size_t>(ny) * w +
                    static_cast<std::size_t>(nx);
                if (!std::isfinite(d.g_cost[n])) {
                    continue;
                }
                if (!line_is_clear(grid, cell, n, radius)) {
                    continue;
                }
                const bool diagonal = dx != 0 && dy != 0;
                const real step = dstar_step_cost(grid, n, diagonal, radius);
                const real candidate = step + d.g_cost[n];
                if (candidate < best) {
                    best = candidate;
                }
            }
        }
        d.rhs[cell] = best;
    }

    dstar_set_key(d, grid, cell);
    const bool in_queue = d.heap_slot[cell] != kNoCell;
    if (d.g_cost[cell] != d.rhs[cell]) {
        if (!in_queue) {
            dstar_push(d, cell);
        } else {
            const std::size_t pos = d.heap_slot[cell];
            dstar_sift_up(d, pos);
            dstar_sift_down(d, pos);
        }
    } else if (in_queue) {
        dstar_remove(d, cell);
    }
}

void dstar_initialize(const map::GridMap& grid, PlannerWorkspace& ws,
                      std::size_t start_cell, std::size_t goal_cell) {
    DStarWorkspace& d = ws.dstar;
    const std::size_t cell_count = grid.width * grid.height;
    const real inf = std::numeric_limits<real>::infinity();
    std::fill(d.g_cost, d.g_cost + cell_count, inf);
    std::fill(d.rhs, d.rhs + cell_count, inf);
    std::fill(d.heap_slot, d.heap_slot + cell_count, kNoCell);
    std::fill(d.dirty, d.dirty + cell_count, false);
    std::copy(grid.cells, grid.cells + cell_count, d.previous_map);

    d.width = grid.width;
    d.height = grid.height;
    d.origin = grid.origin;
    d.initialized = true;
    d.start_cell = start_cell;
    d.goal_cell = goal_cell;
    d.heap_size = 0;
    d.km = real(0);

    d.rhs[goal_cell] = real(0);
    dstar_set_key(d, grid, goal_cell);
    dstar_push(d, goal_cell);
}

// Apply every changed map cell to the persistent D*-Lite state. A changed
// cell can affect any vertex whose clearance disk contains it, so a bounded
// square around the change is marked dirty first and processed once.
// Returns false when the grid geometry changed and the caller must reinitialize.
bool dstar_process_changes(const map::GridMap& grid, PlannerWorkspace& ws,
                           real radius) {
    DStarWorkspace& d = ws.dstar;
    if (d.width != grid.width || d.height != grid.height ||
        d.origin.x != grid.origin.x || d.origin.y != grid.origin.y) {
        return false;
    }

    const std::size_t w = grid.width;
    const std::size_t h = grid.height;
    const std::size_t cell_count = w * h;
    const real half_diag =
        real(0.5) * map::kCellResolution_m * std::sqrt(real(2));
    const long long window = static_cast<long long>(
        std::ceil((radius + half_diag) / map::kCellResolution_m));

    std::size_t dirty_count = 0;
    for (std::size_t i = 0; i < cell_count; ++i) {
        if (grid.cells[i] == d.previous_map[i]) {
            continue;
        }
        const long long cx = static_cast<long long>(i % w);
        const long long cy = static_cast<long long>(i / w);
        for (long long dy = -window; dy <= window; ++dy) {
            for (long long dx = -window; dx <= window; ++dx) {
                const long long nx = cx + dx;
                const long long ny = cy + dy;
                if (nx < 0 || ny < 0 || nx >= static_cast<long long>(w) ||
                    ny >= static_cast<long long>(h)) {
                    continue;
                }
                const std::size_t n =
                    static_cast<std::size_t>(ny) * w +
                    static_cast<std::size_t>(nx);
                if (!d.dirty[n]) {
                    d.dirty[n] = true;
                    ws.parent[dirty_count++] = n;
                }
            }
        }
    }

    for (std::size_t i = 0; i < dirty_count; ++i) {
        const std::size_t cell = ws.parent[i];
        d.dirty[cell] = false;
        dstar_update_vertex(grid, d, cell, radius);
    }
    std::copy(grid.cells, grid.cells + cell_count, d.previous_map);
    return true;
}

// Runs ComputeShortestPath until the start is consistent or the expansion
// budget is exhausted. On timeout the queue and g/rhs state are preserved so
// the next plan() call can continue.
Status dstar_compute_shortest_path(const map::GridMap& grid,
                                   DStarWorkspace& d, std::size_t max_expansions,
                                   real radius) {
    std::size_t expansions = 0;
    for (;;) {
        const real start_min = std::min(d.g_cost[d.start_cell],
                                        d.rhs[d.start_cell]);
        const real start_k1 = start_min + d.km;
        const real start_k2 = start_min;

        if (d.heap_size == 0) {
            break;
        }
        const std::size_t top = d.heap[0];
        const real top_k1 = d.key1[top];
        const real top_k2 = d.key2[top];
        const bool top_before =
            top_k1 < start_k1 || (top_k1 == start_k1 && top_k2 < start_k2);
        if (!top_before && d.rhs[d.start_cell] == d.g_cost[d.start_cell]) {
            break;
        }
        if (expansions >= max_expansions) {
            return Status::PlannerTimeout;
        }

        const real old_k1 = top_k1;
        const real old_k2 = top_k2;
        const std::size_t u = dstar_pop(d);
        ++expansions;
        dstar_set_key(d, grid, u);
        if (old_k1 < d.key1[u] ||
            (old_k1 == d.key1[u] && old_k2 < d.key2[u])) {
            dstar_push(d, u);
            continue;
        }

        const std::size_t w = grid.width;
        const std::size_t h = grid.height;
        const std::size_t ux = u % w;
        const std::size_t uy = u / w;
        if (d.g_cost[u] > d.rhs[u]) {
            d.g_cost[u] = d.rhs[u];
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) {
                        continue;
                    }
                    const long long nx = static_cast<long long>(ux) + dx;
                    const long long ny = static_cast<long long>(uy) + dy;
                    if (nx < 0 || ny < 0 || nx >= static_cast<long long>(w) ||
                        ny >= static_cast<long long>(h)) {
                        continue;
                    }
                    dstar_update_vertex(
                        grid, d, static_cast<std::size_t>(ny) * w +
                                      static_cast<std::size_t>(nx),
                        radius);
                }
            }
        } else {
            d.g_cost[u] = std::numeric_limits<real>::infinity();
            dstar_update_vertex(grid, d, u, radius);
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) {
                        continue;
                    }
                    const long long nx = static_cast<long long>(ux) + dx;
                    const long long ny = static_cast<long long>(uy) + dy;
                    if (nx < 0 || ny < 0 || nx >= static_cast<long long>(w) ||
                        ny >= static_cast<long long>(h)) {
                        continue;
                    }
                    dstar_update_vertex(
                        grid, d, static_cast<std::size_t>(ny) * w +
                                      static_cast<std::size_t>(nx),
                        radius);
                }
            }
        }
    }

    if (!std::isfinite(d.g_cost[d.start_cell]) &&
        !std::isfinite(d.rhs[d.start_cell])) {
        return Status::NoPath;
    }
    if (d.g_cost[d.start_cell] != d.rhs[d.start_cell]) {
        return Status::NoPath;
    }
    return Status::Valid;
}

// Follow the repaired D*-Lite cost field from start to goal, then smooth the
// cell chain with the shared clearance.
Status dstar_extract(const map::GridMap& grid, DStarWorkspace& d,
                     PlannerWorkspace& ws, WaypointBuffer& out, real radius,
                     real smooth_radius, real max_segment_cells,
                     std::size_t& kept_out) {
    const std::size_t w = grid.width;
    const std::size_t cell_count = grid.width * grid.height;
    std::size_t count = 0;
    std::size_t cur = d.start_cell;
    ws.heap[count++] = cur;
    while (cur != d.goal_cell) {
        if (count >= cell_count) {
            return Status::Fault;
        }
        const std::size_t cx = cur % w;
        const std::size_t cy = cur / w;
        std::size_t best_cell = kNoCell;
        real best_cost = std::numeric_limits<real>::infinity();
        real best_goal_distance = std::numeric_limits<real>::infinity();
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                const long long nx = static_cast<long long>(cx) + dx;
                const long long ny = static_cast<long long>(cy) + dy;
                if (nx < 0 || ny < 0 || nx >= static_cast<long long>(grid.width) ||
                    ny >= static_cast<long long>(grid.height)) {
                    continue;
                }
                const std::size_t n =
                    static_cast<std::size_t>(ny) * w +
                    static_cast<std::size_t>(nx);
                if (!std::isfinite(d.g_cost[n])) {
                    continue;
                }
                if (!line_is_clear(grid, cur, n, radius)) {
                    continue;
                }
                const bool diagonal = dx != 0 && dy != 0;
                const real step = dstar_step_cost(grid, n, diagonal, radius);
                const real cost = step + d.g_cost[n];
                const real goal_distance = octile_distance(grid, n, d.goal_cell);
                if (cost < best_cost ||
                    (cost == best_cost &&
                     (goal_distance < best_goal_distance ||
                      (goal_distance == best_goal_distance && n < best_cell)))) {
                    best_cost = cost;
                    best_cell = n;
                    best_goal_distance = goal_distance;
                }
            }
        }
        if (best_cell == kNoCell) {
            return Status::NoPath;
        }
        cur = best_cell;
        ws.heap[count++] = cur;
    }

    if (out.data == nullptr || count > out.capacity) {
        return Status::Fault;
    }
    // D*-Lite extracts through cell centres; two cells of slack let the
    // smoother take the same 0.5 m-class shortcut that A* reaches with its
    // parent chain. The geometric clearance check below is unchanged.
    // [ASSUMED: D*-Lite-only smoothing allowance, not PH-0022 itself]
    const std::size_t kept =
        smooth_chain(grid, ws.heap, count, smooth_radius,
                     max_segment_cells + real(2));
    for (std::size_t i = 0; i < kept; ++i) {
        const std::size_t cell = ws.heap[i];
        out.data[i] = cell_centre(grid, cell % w, cell / w);
    }
    kept_out = kept;
    return Status::Valid;
}

// Deterministic xorshift32 for RRT. The seed is supplied by the caller.
std::uint32_t rrt_next(std::uint32_t& state) {
    std::uint32_t x = state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    state = x;
    return x;
}

real rrt_unit(std::uint32_t& state) {
    return static_cast<real>(rrt_next(state) >> 8) *
           (real(1) / real(16777216));
}

real point_distance_sq(Point2D a, Point2D b) {
    const real dx = a.x - b.x;
    const real dy = a.y - b.y;
    return dx * dx + dy * dy;
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
        const real robot_radius = ROBOT_RADIUS_M;
        const real smooth_margin = PATH_SMOOTH_MARGIN_M;
        if (!std::isfinite(robot_radius) || !(robot_radius > 0.0) ||
            !std::isfinite(smooth_margin) || smooth_margin < 0.0) {
            return fault;
        }
        const real smooth_radius = robot_radius + smooth_margin;
        if (!std::isfinite(smooth_radius)) {
            return fault;
        }
        // Smoothed segment cap (PH-0022).
        real max_segment_cells = 0.0;
        if (!segment_cap_cells(max_segment_cells)) {
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
        if (!point_to_cell(map, start, sx, sy) ||
            !point_to_cell(map, goal, gx, gy)) {
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
        // The physical start and goal must have the same clearance TEB uses.
        if (map::clearance(map, start, robot_radius) < robot_radius ||
            map::clearance(map, goal, robot_radius) < robot_radius) {
            return no_path;
        }

        for (std::size_t i = 0; i < cell_count; ++i) {
            ws.g_cost[i] = std::numeric_limits<real>::infinity();
            ws.parent[i] = kNoCell;
            ws.closed[i] = false;
            ws.heap_slot[i] = kNoCell;
        }

        Search s{map, ws, goal_cell, 0};
        ws.g_cost[start_cell] = real(0);
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
                    if (ws.closed[n]) {
                        continue;
                    }
                    if (!line_is_clear(map, cur, n, robot_radius)) {
                        continue;
                    }
                    const bool diagonal = dx != 0 && dy != 0;
                    const real step = diagonal ? kDiagonalCost : real(1);
                    const real tentative = ws.g_cost[cur] + step;
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
            smooth_chain(map, ws.heap, count, smooth_radius, max_segment_cells);
        for (std::size_t i = 0; i < kept; ++i) {
            const std::size_t cell = ws.heap[i];
            out.data[i] = cell_centre(map, cell % w, cell / w);
        }
        return GlobalGuide{Status::Valid, PointView{out.data, kept}};
    }
};

class DStarLitePlanner final : public IGlobalPlanner {
public:
    GlobalGuide plan(Point2D start, Point2D goal, map::GridMap map,
                     const GlobalPlannerConfig& config,
                     PlannerWorkspace& ws, WaypointBuffer& out) const override {
        const PointView none{nullptr, 0};
        const GlobalGuide fault{Status::Fault, none};
        const GlobalGuide no_path{Status::NoPath, none};

        if (!std::isfinite(map::kCellResolution_m) ||
            !(map::kCellResolution_m > 0.0) || !grid_is_usable(map)) {
            return fault;
        }
        if (config.max_expansions == 0) {
            return fault;
        }
        const real robot_radius = ROBOT_RADIUS_M;
        const real smooth_margin = PATH_SMOOTH_MARGIN_M;
        if (!std::isfinite(robot_radius) || !(robot_radius > 0.0) ||
            !std::isfinite(smooth_margin) || smooth_margin < 0.0) {
            return fault;
        }
        const real smooth_radius = robot_radius + smooth_margin;
        if (!std::isfinite(smooth_radius)) {
            return fault;
        }
        real max_segment_cells = 0.0;
        if (!segment_cap_cells(max_segment_cells)) {
            return fault;
        }

        const std::size_t w = map.width;
        const std::size_t cell_count = map.width * map.height;
        if (!dstar_workspace_is_usable(ws, cell_count)) {
            return fault;
        }

        std::size_t sx = 0;
        std::size_t sy = 0;
        std::size_t gx = 0;
        std::size_t gy = 0;
        if (!point_to_cell(map, start, sx, sy) ||
            !point_to_cell(map, goal, gx, gy)) {
            return fault;
        }
        const std::size_t start_cell = sy * w + sx;
        const std::size_t goal_cell = gy * w + gx;
        if (map.cells[start_cell] == CellState::Occupied ||
            map.cells[goal_cell] == CellState::Occupied) {
            return fault;
        }
        if (map.cells[start_cell] == CellState::Unknown ||
            map.cells[goal_cell] == CellState::Unknown) {
            return no_path;
        }
        if (map::clearance(map, start, robot_radius) < robot_radius ||
            map::clearance(map, goal, robot_radius) < robot_radius) {
            return no_path;
        }

        DStarWorkspace& d = ws.dstar;
        const bool geometry_changed =
            !d.initialized || d.width != map.width || d.height != map.height ||
            d.origin.x != map.origin.x || d.origin.y != map.origin.y;
        const bool goal_changed = !d.initialized || d.goal_cell != goal_cell;
        if (geometry_changed || goal_changed) {
            // A D*-Lite search is rooted at the goal; a new goal invalidates
            // the old cost field, so reset it [ASSUMED: standard D*-Lite].
            dstar_initialize(map, ws, start_cell, goal_cell);
        } else {
            if (d.start_cell != start_cell) {
                d.km += octile_distance(map, start_cell, d.start_cell);
                d.start_cell = start_cell;
            }
            if (!dstar_process_changes(map, ws, robot_radius)) {
                dstar_initialize(map, ws, start_cell, goal_cell);
            }
        }

        // A map update can invalidate the current start or goal cell.
        if (map.cells[start_cell] != CellState::Free ||
            map.cells[goal_cell] != CellState::Free) {
            return no_path;
        }
        if (map::clearance(map, start, robot_radius) < robot_radius ||
            map::clearance(map, goal, robot_radius) < robot_radius) {
            return no_path;
        }

        const Status search = dstar_compute_shortest_path(
            map, d, config.max_expansions, robot_radius);
        if (search != Status::Valid) {
            return GlobalGuide{search, none};
        }
        std::size_t kept = 0;
        const Status extracted =
            dstar_extract(map, d, ws, out, robot_radius, smooth_radius,
                          max_segment_cells, kept);
        if (extracted != Status::Valid) {
            return GlobalGuide{extracted, none};
        }
        return GlobalGuide{Status::Valid, PointView{out.data, kept}};
    }
};

class RrtPlanner final : public IGlobalPlanner {
public:
    GlobalGuide plan(Point2D start, Point2D goal, map::GridMap map,
                     const GlobalPlannerConfig& config,
                     PlannerWorkspace& ws, WaypointBuffer& out) const override {
        const PointView none{nullptr, 0};
        const GlobalGuide fault{Status::Fault, none};
        const GlobalGuide no_path{Status::NoPath, none};

        if (!std::isfinite(map::kCellResolution_m) ||
            !(map::kCellResolution_m > 0.0) || !grid_is_usable(map)) {
            return fault;
        }
        if (config.max_nodes == 0 || !std::isfinite(config.rrt_step_size) ||
            !(config.rrt_step_size > real(0)) ||
            !std::isfinite(config.rrt_goal_tolerance) ||
            config.rrt_goal_tolerance < real(0) ||
            !std::isfinite(config.rrt_goal_bias) ||
            config.rrt_goal_bias < real(0) || config.rrt_goal_bias > real(1) ||
            config.rrt_seed == 0) {
            return fault;
        }
        const real robot_radius = ROBOT_RADIUS_M;
        const real smooth_margin = PATH_SMOOTH_MARGIN_M;
        if (!std::isfinite(robot_radius) || !(robot_radius > 0.0) ||
            !std::isfinite(smooth_margin) || smooth_margin < 0.0) {
            return fault;
        }
        const real smooth_radius = robot_radius + smooth_margin;
        if (!std::isfinite(smooth_radius)) {
            return fault;
        }
        real max_segment_cells = 0.0;
        if (!segment_cap_cells(max_segment_cells)) {
            return fault;
        }
        if (!rrt_workspace_is_usable(ws, config.max_nodes)) {
            return fault;
        }

        std::size_t sx = 0;
        std::size_t sy = 0;
        std::size_t gx = 0;
        std::size_t gy = 0;
        if (!point_to_cell(map, start, sx, sy) ||
            !point_to_cell(map, goal, gx, gy)) {
            return fault;
        }
        const std::size_t w = map.width;
        const std::size_t start_cell = sy * w + sx;
        const std::size_t goal_cell = gy * w + gx;
        if (map.cells[start_cell] == CellState::Occupied ||
            map.cells[goal_cell] == CellState::Occupied) {
            return fault;
        }
        if (map.cells[start_cell] == CellState::Unknown ||
            map.cells[goal_cell] == CellState::Unknown) {
            return no_path;
        }
        if (map::clearance(map, start, robot_radius) < robot_radius ||
            map::clearance(map, goal, robot_radius) < robot_radius) {
            return no_path;
        }

        // A straight start-to-goal segment is the shortest possible guide.
        // Check it before building a tree so an open map does not pay RRT cost.
        if (line_is_clear(map, start_cell, goal_cell, robot_radius)) {
            if (out.data == nullptr || out.capacity < 2) {
                return fault;
            }
            out.data[0] = start;
            out.data[1] = goal;
            return GlobalGuide{Status::Valid, PointView{out.data, 2}};
        }

        RrtWorkspace& rrt = ws.rrt;
        rrt.rng_state = config.rrt_seed;
        rrt.nodes[0] = start;
        rrt.parents[0] = kNoCell;
        std::size_t node_count = 1;
        std::size_t goal_index = kNoCell;
        real goal_path_cost = std::numeric_limits<real>::infinity();

        if (std::sqrt(point_distance_sq(start, goal)) <=
            config.rrt_goal_tolerance) {
            goal_index = 0;
            goal_path_cost = real(0);
        }

        const real map_width = static_cast<real>(map.width) *
                               map::kCellResolution_m;
        const real map_height = static_cast<real>(map.height) *
                                map::kCellResolution_m;
        // Reserve one node slot for the exact goal. The sampling loop starts
        // with the root already in the tree, so it adds at most max_nodes - 2
        // nodes and the exact goal can still be appended within the bound.
        const std::size_t sample_limit =
            config.max_nodes > 1 ? config.max_nodes - 1 : 1;
        for (std::size_t attempt = 1;
             attempt < sample_limit && goal_path_cost > real(0); ++attempt) {
            Point2D sample;
            if (rrt_unit(rrt.rng_state) < config.rrt_goal_bias) {
                sample = goal;
            } else {
                sample = Point2D{
                    map.origin.x + rrt_unit(rrt.rng_state) * map_width,
                    map.origin.y + rrt_unit(rrt.rng_state) * map_height,
                };
            }

            std::size_t nearest = 0;
            real nearest_dist_sq = point_distance_sq(rrt.nodes[0], sample);
            for (std::size_t i = 1; i < node_count; ++i) {
                const real d = point_distance_sq(rrt.nodes[i], sample);
                if (d < nearest_dist_sq) {
                    nearest_dist_sq = d;
                    nearest = i;
                }
            }
            const real nearest_dist = std::sqrt(nearest_dist_sq);
            if (!(nearest_dist > real(0))) {
                continue;
            }

            Point2D next;
            if (nearest_dist <= config.rrt_step_size) {
                next = sample;
            } else {
                const real scale = config.rrt_step_size / nearest_dist;
                next = Point2D{rrt.nodes[nearest].x +
                                   (sample.x - rrt.nodes[nearest].x) * scale,
                               rrt.nodes[nearest].y +
                                   (sample.y - rrt.nodes[nearest].y) * scale};
            }

            std::size_t nix = 0;
            std::size_t niy = 0;
            std::size_t cix = 0;
            std::size_t ciy = 0;
            if (!point_to_cell(map, rrt.nodes[nearest], nix, niy) ||
                !point_to_cell(map, next, cix, ciy)) {
                continue;
            }
            const std::size_t nearest_cell = niy * w + nix;
            const std::size_t next_cell = ciy * w + cix;
            if (!line_is_clear(map, nearest_cell, next_cell, robot_radius)) {
                continue;
            }

            rrt.nodes[node_count] = next;
            rrt.parents[node_count] = nearest;
            const std::size_t added = node_count;
            ++node_count;
            if (std::sqrt(point_distance_sq(next, goal)) <=
                config.rrt_goal_tolerance) {
                // Keep sampling and retain the cheapest goal-reaching node,
                // not the first random one. This stays within max_nodes.
                real path_cost = real(0);
                for (std::size_t cur = added; rrt.parents[cur] != kNoCell;
                     cur = rrt.parents[cur]) {
                    path_cost += std::sqrt(point_distance_sq(
                        rrt.nodes[cur], rrt.nodes[rrt.parents[cur]]));
                }
                if (path_cost < goal_path_cost) {
                    goal_path_cost = path_cost;
                    goal_index = added;
                }
            }
        }

        if (goal_index == kNoCell) {
            return GlobalGuide{Status::PlannerTimeout, none};
        }

        // Add the exact goal when the final edge is clear; otherwise the
        // tolerance node is the guide endpoint.
        const Point2D tolerance_node = rrt.nodes[goal_index];
        if (point_distance_sq(tolerance_node, goal) > real(0) &&
            node_count < config.max_nodes) {
            std::size_t tix = 0;
            std::size_t tiy = 0;
            if (point_to_cell(map, tolerance_node, tix, tiy) &&
                line_is_clear(map, tiy * w + tix, goal_cell, robot_radius)) {
                rrt.nodes[node_count] = goal;
                rrt.parents[node_count] = goal_index;
                goal_index = node_count;
                ++node_count;
            }
        }

        std::size_t path_count = 0;
        for (std::size_t cur = goal_index; cur != kNoCell;
             cur = rrt.parents[cur]) {
            if (path_count >= rrt.capacity) {
                return fault;
            }
            rrt.path[path_count++] = cur;
            if (cur == 0) {
                break;
            }
        }
        if (path_count == 0) {
            return fault;
        }

        if (path_count > ws.capacity) {
            return fault;
        }

        // Greedy line-of-sight pull from the goal back toward the root. The
        // path array is goal-first, so the scan keeps the farthest ancestor
        // visible from the current anchor. This removes random intermediate
        // tree nodes before the cell-chain smoother sees them.
        std::size_t kept_count = 0;
        std::size_t anchor = 0;
        ws.heap[kept_count++] = rrt.path[0];
        while (anchor + 1 < path_count) {
            std::size_t next = anchor + 1;
            const Point2D anchor_point = rrt.nodes[rrt.path[anchor]];
            std::size_t ax = 0;
            std::size_t ay = 0;
            if (!point_to_cell(map, anchor_point, ax, ay)) {
                return fault;
            }
            const std::size_t anchor_cell = ay * w + ax;
            for (std::size_t j = path_count - 1; j > anchor; --j) {
                const Point2D candidate = rrt.nodes[rrt.path[j]];
                std::size_t cx = 0;
                std::size_t cy = 0;
                if (!point_to_cell(map, candidate, cx, cy)) {
                    continue;
                }
                if (line_is_clear(map, anchor_cell, cy * w + cx,
                                  robot_radius)) {
                    next = j;
                    break;
                }
            }
            if (kept_count >= ws.capacity) {
                return fault;
            }
            ws.heap[kept_count++] = rrt.path[next];
            anchor = next;
        }

        // Reverse the kept node indices into start-to-goal order, then
        // convert and deduplicate adjacent cells in the same buffer. Writing
        // at cell_count <= i never overwrites an unread node index.
        for (std::size_t i = 0; i < kept_count / 2; ++i) {
            const std::size_t tmp = ws.heap[i];
            ws.heap[i] = ws.heap[kept_count - 1 - i];
            ws.heap[kept_count - 1 - i] = tmp;
        }
        std::size_t cell_count = 0;
        for (std::size_t i = 0; i < kept_count; ++i) {
            const Point2D p = rrt.nodes[ws.heap[i]];
            std::size_t ix = 0;
            std::size_t iy = 0;
            if (!point_to_cell(map, p, ix, iy)) {
                return fault;
            }
            const std::size_t cell = iy * w + ix;
            if (cell_count == 0 || ws.heap[cell_count - 1] != cell) {
                if (cell_count >= ws.capacity) {
                    return fault;
                }
                ws.heap[cell_count++] = cell;
            }
        }
        if (cell_count == 0 || out.data == nullptr ||
            cell_count > out.capacity) {
            return fault;
        }

        const std::size_t kept = smooth_chain(map, ws.heap, cell_count,
                                              smooth_radius,
                                              max_segment_cells);
        for (std::size_t i = 0; i < kept; ++i) {
            const std::size_t cell = ws.heap[i];
            out.data[i] = cell_centre(map, cell % w, cell / w);
        }
        return GlobalGuide{Status::Valid, PointView{out.data, kept}};
    }
};

// Used for an invalid enum value. Always Status::Fault, so the caller sees the
// failure rather than a silent switch to another planner.
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
            return std::make_unique<DStarLitePlanner>();
        case GlobalPlannerKind::RRT:
            return std::make_unique<RrtPlanner>();
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
    return (now - cache.planned_at) >= GLOBAL_PLANNER_REPLAN_PERIOD_S;
}

void record_plan(PlanCache& cache, Point2D goal,
                 std::uint64_t map_generation, double now) {
    cache.valid = true;
    cache.goal = goal;
    cache.map_generation = map_generation;
    cache.planned_at = now;
}

}  // namespace core::global_planner
