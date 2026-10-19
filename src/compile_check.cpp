// Compile-only check for the skeleton headers. Not a test: it includes every
// header so that compilation proves they are self-consistent. It defines
// nothing and calls nothing.

#include "core/core.hpp"
#include "core/placeholders.hpp"
#include "core/stages/global_planner_manager.hpp"
#include "core/stages/map_manager.hpp"
#include "core/stages/odometry.hpp"
#include "core/stages/output_converter.hpp"
#include "core/stages/teb_local_planner.hpp"
