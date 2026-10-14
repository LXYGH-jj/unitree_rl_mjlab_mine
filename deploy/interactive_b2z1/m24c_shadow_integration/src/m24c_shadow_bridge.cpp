#include "m24c_shadow_bridge.h"
#include <cmath>
#include <utility>

namespace b2z1::m24c {
namespace {
constexpr float kStandGoalDisplacement = 0.10f; // OFFLINE EXPLORATION only, NOT hardware approval
constexpr float kWalkGoalDisplacement = 0.03f;
constexpr float kWalkMaxVx = 0.12f;
void check3(const Command3& x) {
    for (float v : x) if (!std::isfinite(v)) throw std::invalid_argument("nonfinite vector3");
}
}
ShadowBridge::ShadowBridge(PolicyEvaluator actor) : pipeline_(std::move(actor)) {}
void ShadowBridge::onB2(const Joint12& q, const Joint12& dq,
    const std::array<float,4>& quat, const Command3& gyro, MonoTime stamp) {
    if (faulted_) fault(reason_);
    try { store_.updateB2(q,dq,quat,gyro,stamp); }
    catch (const std::exception& e) { fault(e.what()); }
}
void ShadowBridge::onZ1(const Arm6& q, const Arm6& dq, MonoTime stamp,
                        bool independently_confirmed_new_packet) {
    if (faulted_) fault(reason_);
    if (!independently_confirmed_new_packet)
        fault("Z1 feedback lacks independent new-datagram proof");
    try { store_.updateZ1(q,dq,stamp); confirmed_z1_=true; }
    catch (const std::exception& e) { fault(e.what()); }
}
Preview ShadowBridge::preview(Mode mode, const Command3& cmd,
     const Command3& entry_goal, const Command3& goal, const Arm6& ik_q, MonoTime now) {
    if (faulted_) fault(reason_);
    try {
        if (!confirmed_z1_) throw std::runtime_error("missing verified Z1 feedback");
        check3(cmd); check3(entry_goal); check3(goal);
        for(float q : ik_q) if(!std::isfinite(q))
            throw std::invalid_argument("nonfinite IK proposal");
        // Command mode is explicit, and moving operation is limited to the
        // exact forward-only vx=0.12m/s research baseline. No live actuation.
        if (mode == Mode::Stand) {
            for (float v : cmd) if (std::abs(v) > 1.e-6f)
                throw std::invalid_argument("stand mode requires zero velocity command");
        } else {
            if (cmd[0] < -1.e-6f || cmd[0] > kWalkMaxVx+1.e-6f ||
                std::abs(cmd[1])>1.e-6f || std::abs(cmd[2])>1.e-6f)
                throw std::invalid_argument("walk mode exceeds baseline forward command");
        }
        const float dx=goal[0]-entry_goal[0],dy=goal[1]-entry_goal[1],dz=goal[2]-entry_goal[2];
        const float displacement=std::sqrt(dx*dx+dy*dy+dz*dz);
        const float max_displacement = mode==Mode::Stand ? kStandGoalDisplacement : kWalkGoalDisplacement;
        if (displacement > max_displacement + 1.e-6f)
            throw std::invalid_argument("EE target exceeds offline mode workspace sphere");
        const auto adapted=adaptFeedback(store_.copy(cmd,goal),now);
        const auto result=pipeline_.step(adapted.robot,adapted.command);
        Preview out;
        out.observation=result.observation;
        out.policy_action=result.clipped_action;
        for(std::size_t i=0;i<kLegDof;++i) out.leg_policy_action[i]=result.clipped_action[i];
        out.z1_ik_goal_unqualified=ik_q;
        out.ee_obs_xyz=adapted.robot.ee_local_xyz;
        return out;
    } catch(const std::exception& e) { fault(e.what()); }
}
void ShadowBridge::resetOffline() {
    store_.reset(); pipeline_.reset(); confirmed_z1_=false; faulted_=false; reason_.clear();
}
[[noreturn]] void ShadowBridge::fault(const std::string& why) {
    if (!faulted_) { faulted_=true; reason_=why; }
    throw std::runtime_error("M24c SHADOW FAULT (no motor IO): "+reason_);
}
} // namespace b2z1::m24c
