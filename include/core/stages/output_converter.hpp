// Output converter stage interface. Declarations only; no implementation.
// Produces the next pose and status at the defined control period, with
// dt, timestamp, frame_id and status [FROM: AGENTS.md section 2].

#pragma once

#include "core/core.hpp"
#include "core/placeholders.hpp"
#include "core/stages/teb_local_planner.hpp"

namespace core::output {

// Output converter interface.
class IOutputConverter {
public:
    virtual ~IOutputConverter() = default;

    // Build the cycle Output from the local plan and the snapshot.
    virtual Output convert(const teb::LocalPlan& plan,
                           const InputSnapshot& snapshot) const = 0;
};

// Converter that takes the next pose from the first horizon step of the local
// plan. Stateless: all inputs come from the arguments.
class OutputConverter final : public IOutputConverter {
public:
    Output convert(const teb::LocalPlan& plan,
                   const InputSnapshot& snapshot) const override;
};

}  // namespace core::output
