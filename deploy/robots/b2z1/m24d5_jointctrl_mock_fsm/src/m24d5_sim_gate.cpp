#include "m24d5_sim_gate.h"
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace b2z1::m24d5 {
namespace {
b2z1::z1_jointctrl::Config makeSafety(const Joint6& lo, const Joint6& hi) {
    b2z1::z1_jointctrl::Config out{};
    out.q_min=lo; out.q_max=hi;
    out.max_velocity_rad_s.fill(0.8);  // SIM only: <=0.004 rad / 5ms
    out.max_tracking_error_rad=0.65;  // SIM only, M2.3 existing bound
    out.max_feedback_age=std::chrono::milliseconds(80);
    out.max_command_interval=std::chrono::milliseconds(80);
    out.min_consecutive_new_packets=3;
    return out;
}
b2z1::m24d1::Limits makeRx(const Joint6& lo, const Joint6& hi) {
    b2z1::m24d1::Limits out{};
    out.q_min=lo; out.q_max=hi;
    out.max_motor_temperature_c.fill(65); // synthetic test numbers ONLY
    out.max_receive_age=std::chrono::milliseconds(80);
    out.max_decode_lag=std::chrono::milliseconds(10);
    return out;
}
void valid(const Joint6& q, const char* label) {
    for (double v:q) if(!std::isfinite(v))
        throw std::runtime_error(std::string("M2.4d5 nonfinite ")+label);
}
}
SimGate::SimGate(const Joint6& lo, const Joint6& hi, const std::string& lock_file) {
    runtime_=std::make_unique<b2z1::m24d4::MockOnlyRuntime>(
        mock_,makeSafety(lo,hi),makeRx(lo,hi),lock_file);
}
SimGate::~SimGate() {disarm();}
void SimGate::disarm() noexcept {
    faulted_=true;
    if(runtime_) runtime_->disarm();
}
std::optional<Joint6> SimGate::step(std::uint32_t tick,const Joint6& q,
                                    const Joint6& dq,const Joint6& goal,Time now) {
    if(faulted_) throw std::runtime_error("M2.4d5 mock gate is disarmed/faulted");
    try {
        valid(q,"measured q");valid(dq,"measured dq");valid(goal,"IK goal");
        if(have_tick_ && tick < last_tick_)
            throw std::runtime_error("M2.4d5 MuJoCo tick moved backwards");
        if(!have_tick_ || tick > last_tick_) {
            // SIMULATED evidence only: this is *not* proof of Z1 UDP reception.
            b2z1::m24d1::Snapshot s{};
            s.q=q;s.dq=dq;
            s.temperature_c.fill(25); // no real motor temperature used
            s.sdk_connected=true; s.jointctrl_reported=true; // SIM FIXTURE ONLY
            mock_.injectSynthetic(s,now);
            if(!runtime_->pumpOne(now))
                throw std::runtime_error("M2.4d5 internal synthetic frame not consumed");
            last_tick_=tick;have_tick_=true;++new_ticks_;
        }
        if(!runtime_->authorized()) {
            if(new_ticks_ < 3) return std::nullopt;
            runtime_->authorizeForOfflineTest(now); // IMMUTABLY MOCK-ONLY method
        }
        runtime_->setIkGoal(goal,now);
        if(!runtime_->tick(now)) return std::nullopt;
        // Read only the most recent synthetic sink command in O(1). The mock
        // history is bounded to 64 samples; the controller's independent
        // sentCount remains monotonic for fault/missed-output detection.
        ++commands_;
        if(runtime_->sentCount()!=commands_)
            throw std::runtime_error("M2.4d5 mock sink/command count mismatch");
        const auto latest=mock_.latestCommand();
        if(!latest)
            throw std::runtime_error("M2.4d5 expected latest mock command");
        return latest->q;
    } catch(...) {disarm();throw;}
}
} // namespace b2z1::m24d5
