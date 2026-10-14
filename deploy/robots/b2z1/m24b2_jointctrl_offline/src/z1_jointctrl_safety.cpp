#include "z1_jointctrl_safety.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace b2z1::z1_jointctrl {
namespace {
void finite6(const Joint6& x, const char* name) {
    for (double v : x) if (!std::isfinite(v))
        throw std::invalid_argument(std::string(name) + " contains NaN or Inf");
}
}

SafetyCore::SafetyCore(Config config) : config_(std::move(config)) {
    finite6(config_.q_min, "q_min");
    finite6(config_.q_max, "q_max");
    finite6(config_.max_velocity_rad_s, "max_velocity_rad_s");
    for (std::size_t i=0; i<6; ++i) {
        if (config_.q_min[i] >= config_.q_max[i] ||
            config_.max_velocity_rad_s[i] <= 0.0)
            throw std::invalid_argument("missing or invalid Z1 joint limits");
    }
    if (!std::isfinite(config_.max_tracking_error_rad) ||
        config_.max_tracking_error_rad <= 0 ||
        config_.max_feedback_age.count() <= 0 ||
        config_.max_command_interval.count() <= 0 ||
        config_.min_consecutive_new_packets == 0)
        throw std::invalid_argument("invalid Z1 safety configuration");
}

[[noreturn]] void SafetyCore::fail(const std::string& why) {
    faulted_ = true;
    authorized_ = false;
    reason_ = why;
    throw std::runtime_error("Z1 JOINTCTRL fault latched: " + why);
}

void SafetyCore::requireFresh(Time now) {
    if (faulted_) fail(reason_);
    if (!has_feedback_ || feedback_.received_at == Time{} || now < feedback_.received_at ||
        now - feedback_.received_at > config_.max_feedback_age)
        fail("missing or stale confirmed feedback");
}

void SafetyCore::receive(const Feedback& fb, Time now) {
    if (faulted_) fail(reason_);
    if (!fb.confirmed_new_packet || fb.received_at == Time{} ||
        fb.received_at > now || now - fb.received_at > config_.max_feedback_age ||
        (has_feedback_ && fb.received_at <= feedback_.received_at))
        fail("packet not independently confirmed/new/fresh");

    for (std::size_t i=0; i<6; ++i) {
        if (!std::isfinite(fb.q[i]) || !std::isfinite(fb.dq[i]) ||
            !std::isfinite(fb.tau[i]) ||
            fb.motor_error[i] != 0 || fb.motor_connection[i] != 0 ||
            fb.q[i] < config_.q_min[i] || fb.q[i] > config_.q_max[i])
            fail("joint nonfinite/out of soft limits/motor health fault");
        if (authorized_ && emitted_once_ &&
            std::abs(commanded_q_[i] - fb.q[i]) > config_.max_tracking_error_rad)
            fail("joint tracking deviation exceeds limit");
    }

    feedback_ = fb;
    has_feedback_ = true;
    ++packet_count_;
}

void SafetyCore::authorize(bool electrical_ok, bool acknowledged,
                           bool jointctrl_ready, Time now) {
    requireFresh(now);
    if (!electrical_ok || !acknowledged || !jointctrl_ready)
        throw std::logic_error("Z1 JOINTCTRL authorization requires 3 independent approvals");
    if (packet_count_ < config_.min_consecutive_new_packets)
        throw std::logic_error("not enough confirmed new packets");
    commanded_q_ = feedback_.q;  // start from measured position, not zero
    desired_q_ = commanded_q_;
    authorized_ = true;
    goal_set_ = false;
    emitted_once_ = false;
    last_command_at_ = now;
}

void SafetyCore::setGoal(const Joint6& desired, Time now) {
    if (!authorized_) throw std::logic_error("Z1 JOINTCTRL not authorized");
    requireFresh(now);
    for (std::size_t i=0; i<6; ++i) {
        if (!std::isfinite(desired[i]) || desired[i] < config_.q_min[i] ||
            desired[i] > config_.q_max[i])
            fail("invalid goal/outside joint soft limits");
    }
    desired_q_ = desired;
    goal_set_ = true;
}

std::optional<JointCommand> SafetyCore::next(Time now) {
    if (!authorized_) return std::nullopt;
    requireFresh(now);
    if (now < last_command_at_ ||
        now-last_command_at_ > config_.max_command_interval)
        fail("JOINTCTRL periodic control missed deadline");

    const auto dt = std::chrono::duration<double>(now-last_command_at_).count();
    JointCommand out{};
    out.q = commanded_q_;
    for (std::size_t i=0; i<6; ++i) {
        if (goal_set_ && dt > 0.0) {
            const double max_step = config_.max_velocity_rad_s[i] * dt;
            out.q[i] += std::clamp(desired_q_[i] - out.q[i], -max_step, max_step);
            out.dq[i] = (out.q[i]-commanded_q_[i])/dt;
        }
    }
    commanded_q_ = out.q;
    last_command_at_ = now;
    emitted_once_ = true;
    return out;
}

void SafetyCore::stop() {
    authorized_ = false;
    goal_set_ = false;
    // Does NOT send a stop, PASSIVE or hold command to any actual actuator.
    // Hardware-level controlled stop/E-stop remains a separate requirement.
}

} // namespace b2z1::z1_jointctrl
