// No ONNX / robot I/O dependencies: check fault latch, reset, history.
#include "interactive_pipeline.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace interactive_b2z1;
namespace {
void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
}
int main() {
    try {
        int counter = 0;
        InteractivePipeline pipeline([&counter](const Observation799&) {
            Joint18 raw{};
            if (++counter == 2) {
                raw[4] = std::numeric_limits<float>::quiet_NaN();
            } else {
                raw[0] = 125.f;
            }
            return raw;
        });
        RobotSample sample;
        const auto cfg = ObservationConfig::baseline();
        sample.joint_q = cfg.default_joint_pos;
        sample.imu_quat_wxyz = {1.f, 0.f, 0.f, 0.f};
        const Command3 cmd{0.2f, 0.f, 0.f};
        const auto first = pipeline.step(sample, cmd);
        check(first.clipped_action[0] == 100.f, "actor clipping incorrect");
        check(first.observation[41] == 0.f, "initial last_action must be zero");
        bool rejected = false;
        try { (void)pipeline.step(sample, cmd); }
        catch (const std::invalid_argument&) { rejected = true; }
        check(rejected && pipeline.faulted(), "NaN action did not latch fault");
        try { (void)pipeline.step(sample, cmd); }
        catch (const std::runtime_error&) { rejected = true; }
        check(rejected && counter == 2, "faulted pipeline incorrectly continued inference");
        pipeline.reset();
        check(!pipeline.faulted(), "reset did not clear fault");
        const auto after_reset = pipeline.step(sample, cmd);
        check(after_reset.observation[41] == 0.f, "reset must zero last actor action");
        check(after_reset.observation[89] == 0.f, "reset must zero history");
        check(pipeline.observationBuilder().stepCount() == 1, "reset step count incorrect");
        std::cout << "M1.4a PIPELINE SAFETY SELFTEST: PASS\n";
    } catch (const std::exception& e) {
        std::cerr << "M1.4a SELFTEST FAIL: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
