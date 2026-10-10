#pragma once

#include "interactive_observation.h"

#include <functional>

namespace interactive_b2z1 {

// Pure inference adapter: NO DDS, SDK2, Z1 SDK, FSM, or actuator commands.
// Evaluator takes 799 floats and returns the raw 18-dimensional policy output.
using PolicyEvaluator = std::function<Joint18(const Observation799&)>;

struct PipelineResult {
    Observation799 observation{};
    Joint18 raw_action{};
    Joint18 clipped_action{};
};

class InteractivePipeline final {
public:
    explicit InteractivePipeline(PolicyEvaluator policy,
                                 ObservationConfig cfg = ObservationConfig::baseline());

    // On any observation/inference failure the adapter latches a fault.
    // The caller must not use stale actions and must explicitly reset after addressing it.
    PipelineResult step(const RobotSample& state, const Command3& cmd);
    void reset();
    [[nodiscard]] bool faulted() const noexcept { return faulted_; }
    [[nodiscard]] const ObservationBuilder& observationBuilder() const noexcept { return builder_; }

private:
    ObservationBuilder builder_;
    PolicyEvaluator policy_;
    bool faulted_{false};
};

} // namespace interactive_b2z1
