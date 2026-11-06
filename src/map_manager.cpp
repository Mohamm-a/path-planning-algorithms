// Map manager implementation. The per-cycle contract is documented on
// GridMapManager::update() in include/core/stages/map_manager.hpp.
//
// Geometry: the cell of map point p is floor((p - origin) / resolution).
// Beam origin and direction from the LiDAR mount placeholders
// [UNKNOWN: PH-0010, PH-0011, PH-0012]:
//   origin = pose_xy + R(pose_theta) * mount_xy
//   angle  = pose_theta + mount_theta + alpha
// alpha is CCW from the LiDAR +x axis [FROM: user answer, Phase A].
// Ray traversal is an exact-count DDA over cells (Amanatides and Woo)
// [ASSUMED: standard algorithm; no physical value].
// Inflation uses cell centers: an offset (dx, dy) is inflated when
// (dx^2 + dy^2) * res^2 <= R^2 [ASSUMED: metric inflation radius].
// Inflation radius is PH-0007, cell size is PH-0008, max range is PH-0016
// [UNKNOWN]. Guards: any unreplaced placeholder returns Status::Fault.

#include "core/stages/map_manager.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace core::map {
namespace {

constexpr real kInf = std::numeric_limits<real>::infinity();

// Placeholder guards. Zero is never valid for resolution or max range; the
// inflation radius sentinel is -1.0, so 0.0 (no inflation) is valid.
bool placeholders_valid() {
    return INFLATION_RADIUS_M >= 0.0 &&
           kCellResolution_m > 0.0 &&
           LIDAR_MAX_RANGE_M > 0.0 &&
           std::isfinite(LIDAR_MOUNT_X_M) &&
           std::isfinite(LIDAR_MOUNT_Y_M) &&
           std::isfinite(LIDAR_MOUNT_THETA_RAD);
}

// Checks extents and storage. A grid with zero cells needs no storage.
bool storage_valid(const Point2D& origin, std::size_t width, std::size_t height,
                   const void* cells) {
    if (!std::isfinite(origin.x) || !std::isfinite(origin.y)) {
        return false;
    }
    if (width != 0 && height > std::numeric_limits<std::size_t>::max() / width) {
        return false;
    }
    return width * height == 0 || cells != nullptr;
}

bool same_geometry(const GridMap& a, const GridMap& b) {
    return a.origin.x == b.origin.x && a.origin.y == b.origin.y &&
           a.width == b.width && a.height == b.height;
}

// Every beam needs finite alpha and finite, non-negative r. A null data
// pointer with a nonzero count is invalid.
bool lidar_valid(const LidarView& lidar) {
    if (lidar.count == 0) {
        return true;
    }
    if (lidar.data == nullptr) {
        return false;
    }
    for (std::size_t i = 0; i < lidar.count; ++i) {
        const LidarPoint& beam = lidar.data[i];
        if (!std::isfinite(beam.r) || beam.r < 0.0 || !std::isfinite(beam.alpha)) {
            return false;
        }
    }
    return true;
}

// Liang-Barsky clip of segment a->b to the closed box [0, w] x [0, h] in
// grid coordinates. On success, [t0, t1] is the visible part of the segment.
bool clip_to_grid(real ax, real ay, real bx, real by, real w, real h,
                  real& t0, real& t1) {
    const real dx = bx - ax;
    const real dy = by - ay;
    t0 = 0.0;
    t1 = 1.0;
    const auto clip = [&t0, &t1](real p, real q) {
        if (p == 0.0) {
            return q >= 0.0;
        }
        const real r = q / p;
        if (p < 0.0) {
            if (r > t1) {
                return false;
            }
            if (r > t0) {
                t0 = r;
            }
        } else {
            if (r < t0) {
                return false;
            }
            if (r < t1) {
                t1 = r;
            }
        }
        return true;
    };
    return clip(-dx, ax) && clip(dx, w - ax) && clip(-dy, ay) && clip(dy, h - ay) &&
           t0 <= t1;
}

// Maps a grid coordinate in [0, n] to a valid cell index in [0, n - 1].
long long clamp_cell(real g, long long n) {
    const real f = std::floor(g);
    if (f < 0.0) {
        return 0;
    }
    if (f >= static_cast<real>(n)) {
        return n - 1;
    }
    return static_cast<long long>(f);
}

// Walks the cells crossed by the clipped segment a->b (grid coordinates) and
// sets them. Every cell before the last is Free. The last cell is set to
// end_state. The number of steps is fixed up front, so the walk is bounded.
void trace_segment(const MutableGridMap& grid, real ax, real ay, real bx,
                   real by, CellState end_state) {
    const auto w = static_cast<long long>(grid.width);
    const auto h = static_cast<long long>(grid.height);

    long long cx = clamp_cell(ax, w);
    long long cy = clamp_cell(ay, h);
    const long long ex = clamp_cell(bx, w);
    const long long ey = clamp_cell(by, h);

    long long steps_x = std::llabs(ex - cx);
    long long steps_y = std::llabs(ey - cy);
    const long long sx = ex >= cx ? 1 : -1;
    const long long sy = ey >= cy ? 1 : -1;

    const real dx = bx - ax;
    const real dy = by - ay;
    real t_max_x = kInf;
    real t_max_y = kInf;
    real t_delta_x = kInf;
    real t_delta_y = kInf;
    if (dx != 0.0) {
        t_delta_x = 1.0 / std::fabs(dx);
        t_max_x = (dx > 0.0 ? (static_cast<real>(cx + 1) - ax) : (ax - static_cast<real>(cx))) /
                  std::fabs(dx);
    }
    if (dy != 0.0) {
        t_delta_y = 1.0 / std::fabs(dy);
        t_max_y = (dy > 0.0 ? (static_cast<real>(cy + 1) - ay) : (ay - static_cast<real>(cy))) /
                  std::fabs(dy);
    }

    long long remaining = steps_x + steps_y;
    for (;;) {
        const auto idx = static_cast<std::size_t>(cy) * grid.width + static_cast<std::size_t>(cx);
        grid.cells[idx] = (remaining == 0) ? end_state : CellState::Free;
        if (remaining == 0) {
            break;
        }
        if (steps_x > 0 && (steps_y == 0 || t_max_x < t_max_y)) {
            cx += sx;
            t_max_x += t_delta_x;
            --steps_x;
        } else {
            cy += sy;
            t_max_y += t_delta_y;
            --steps_y;
        }
        --remaining;
    }
}

// Casts one beam into the SLAM overlay. The beam is truncated at the max
// range, and clipped silently at the grid boundary. The endpoint is marked
// Occupied only if the return is within range and inside the grid.
void cast_beam(const MutableGridMap& slam, const Pose2D& pose, const LidarPoint& beam) {
    const real res = kCellResolution_m;
    const real max_range = LIDAR_MAX_RANGE_M;
    const real c = std::cos(pose.theta);
    const real s = std::sin(pose.theta);
    const real mx = LIDAR_MOUNT_X_M;
    const real my = LIDAR_MOUNT_Y_M;
    const real mth = LIDAR_MOUNT_THETA_RAD;

    // Sensor origin and beam endpoint in the map frame.
    const real ox = pose.x + c * mx - s * my;
    const real oy = pose.y + s * mx + c * my;
    const real angle = pose.theta + mth + beam.alpha;
    const bool in_range = beam.r <= max_range;
    const real len = std::min(beam.r, max_range);
    const real ex = ox + len * std::cos(angle);
    const real ey = oy + len * std::sin(angle);

    // Grid coordinates of the origin and endpoint.
    const real w = static_cast<real>(slam.width);
    const real h = static_cast<real>(slam.height);
    const real gox = (ox - slam.origin.x) / res;
    const real goy = (oy - slam.origin.y) / res;
    const real gex = (ex - slam.origin.x) / res;
    const real gey = (ey - slam.origin.y) / res;

    real t0 = 0.0;
    real t1 = 0.0;
    if (!clip_to_grid(gox, goy, gex, gey, w, h, t0, t1)) {
        return;  // Beam lies entirely outside the grid: clipped silently.
    }
    const real dx = gex - gox;
    const real dy = gey - goy;
    const bool endpoint_clipped = t1 < 1.0;
    const bool endpoint_inside = gex >= 0.0 && gex < w && gey >= 0.0 && gey < h;
    const bool mark_hit = in_range && !endpoint_clipped && endpoint_inside;

    trace_segment(slam, gox + t0 * dx, goy + t0 * dy, gox + t1 * dx, goy + t1 * dy,
                  mark_hit ? CellState::Occupied : CellState::Free);
}

// Rebuilds the inflation layer. Cells default to Unknown. Each cell within
// the inflation radius of an occupied cell (fixed or SLAM) becomes Occupied.
void build_inflation(const GridMap& fixed, const GridMap& slam,
                     const MutableGridMap& inflation) {
    const real res = kCellResolution_m;
    const real radius = INFLATION_RADIUS_M;
    const real radius_sq = radius * radius;
    const auto window = static_cast<long long>(std::ceil(radius / res));
    const auto w = static_cast<long long>(inflation.width);
    const auto h = static_cast<long long>(inflation.height);

    std::fill(inflation.cells, inflation.cells + inflation.width * inflation.height,
              CellState::Unknown);

    for (long long y = 0; y < h; ++y) {
        for (long long x = 0; x < w; ++x) {
            const auto i = static_cast<std::size_t>(y * w + x);
            if (fixed.cells[i] != CellState::Occupied && slam.cells[i] != CellState::Occupied) {
                continue;
            }
            for (long long dy = -window; dy <= window; ++dy) {
                for (long long dx = -window; dx <= window; ++dx) {
                    const long long nx = x + dx;
                    const long long ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
                        continue;
                    }
                    const real ox = static_cast<real>(dx) * res;
                    const real oy = static_cast<real>(dy) * res;
                    if (ox * ox + oy * oy > radius_sq) {
                        continue;
                    }
                    inflation.cells[static_cast<std::size_t>(ny * w + nx)] = CellState::Occupied;
                }
            }
        }
    }
}

// Composes the layers. Priority: Occupied > Free > Unknown. Returns true when
// any output cell changed value. The comparison happens before each write, so
// the change flag is exact and needs no extra storage.
bool compose(const GridMap& fixed, const GridMap& slam, const GridMap& inflation,
             const MutableGridMap& out) {
    const std::size_t count = out.width * out.height;
    bool changed = false;
    for (std::size_t i = 0; i < count; ++i) {
        const CellState layers[3] = {fixed.cells[i], slam.cells[i], inflation.cells[i]};
        bool any_occupied = false;
        bool any_free = false;
        for (const CellState state : layers) {
            any_occupied = any_occupied || state == CellState::Occupied;
            any_free = any_free || state == CellState::Free;
        }
        const CellState next = any_occupied ? CellState::Occupied
                               : any_free   ? CellState::Free
                                            : CellState::Unknown;
        if (out.cells[i] != next) {
            changed = true;
            out.cells[i] = next;
        }
    }
    return changed;
}

}  // namespace

GridMapManager::GridMapManager(GridMap fixed_borders, MutableGridMap slam_overlay,
                               MutableGridMap inflation, MutableGridMap composed)
    : fixed_layer_(LayerKind::FixedBorders, fixed_borders),
      slam_layer_(LayerKind::SlamOverlay, as_const(slam_overlay)),
      inflation_layer_(LayerKind::Inflation, as_const(inflation)),
      slam_(slam_overlay),
      inflation_(inflation),
      composed_(composed) {}

const IMapLayer* GridMapManager::layer(LayerKind kind) const {
    switch (kind) {
        case LayerKind::FixedBorders:
            return &fixed_layer_;
        case LayerKind::SlamOverlay:
            return &slam_layer_;
        case LayerKind::Inflation:
            return &inflation_layer_;
    }
    return nullptr;
}

Status GridMapManager::update(const Pose2D& pose, const LidarView& lidar) {
    // No silent fallback: validate everything before any grid is changed.
    if (!placeholders_valid()) {
        return Status::Fault;
    }
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta)) {
        return Status::Fault;
    }
    if (!lidar_valid(lidar)) {
        return Status::Fault;
    }

    const GridMap fixed = fixed_layer_.grid();
    const GridMap slam = as_const(slam_);
    const GridMap inflation = as_const(inflation_);
    const GridMap composed = as_const(composed_);

    if (!storage_valid(fixed.origin, fixed.width, fixed.height, fixed.cells) ||
        !storage_valid(slam.origin, slam.width, slam.height, slam.cells) ||
        !storage_valid(inflation.origin, inflation.width, inflation.height, inflation.cells) ||
        !storage_valid(composed.origin, composed.width, composed.height, composed.cells)) {
        return Status::Fault;
    }
    if (!same_geometry(fixed, slam) || !same_geometry(fixed, inflation) ||
        !same_geometry(fixed, composed)) {
        return Status::Fault;
    }

    // Writable grids must not share storage: the output would overwrite an input.
    const std::size_t count = fixed.width * fixed.height;
    if (count != 0 && (slam.cells == inflation.cells || slam.cells == composed.cells ||
                       inflation.cells == composed.cells)) {
        return Status::Fault;
    }
    // The first successful update always bumps: no plan can yet reflect the
    // composed grid. After that, bump only when the composed content changed,
    // since the global planner reads only the composed grid.
    const bool first_success = map_generation_ == 0;
    if (count == 0) {
        if (first_success) {
            ++map_generation_;
        }
        return Status::Valid;  // Nothing to map.
    }

    for (std::size_t i = 0; i < lidar.count; ++i) {
        cast_beam(slam_, pose, lidar.data[i]);
    }
    build_inflation(fixed, as_const(slam_), inflation_);
    const bool composed_changed =
        compose(fixed, as_const(slam_), as_const(inflation_), composed_);
    if (first_success || composed_changed) {
        ++map_generation_;
    }
    return Status::Valid;
}

}  // namespace core::map
