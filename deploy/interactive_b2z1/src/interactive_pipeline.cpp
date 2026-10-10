#include "interactive_pipeline.h"

#include <stdexcept>
#include <utility>

namespace interactive_b2z1 {

InteractivePipeline::InteractivePipeline(PolicyEvaluator policy, ObservationConfig cfg)
    : builder_(std::move(cfg)), policy_(std::move(policy)) {
    if (!policy_) throw std::invalid_argument("InteractivePipeline requires a policy evaluator");
}

PipelineResult InteractivePipeline::step(const RobotSample& state, const Command3& cmd) {
    if (faulted_) {
        throw std::runtime_error("InteractivePipeline is fault-latched; explicit reset required");
    }
    try {
        PipelineResult result;
        result.observation = builder_.build(state, cmd);
        result.raw_action = policy_(result.observation);
        // acceptPolicyAction rejects NaN/Inf and stores the clipped action for NEXT observation.
        builder_.acceptPolicyAction(result.raw_action);
        result.clipped_action = builder_.lastAction();
        return result;
    } catch (...) {
        faulted_ = true;
        throw;
    }
}

void InteractivePipeline::reset() {
    builder_.reset();
    faulted_ = false;
}

} // namespace interactive_b2z1
