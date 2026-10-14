#include "joint_target_guard.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace interactive_b2z1::sim_control {
namespace {
constexpr Joint12 kScale{0.4f, 0.45f, 0.45f, 0.4f, 0.45f, 0.45f,
                         0.4f, 0.45f, 0.45f, 0.4f, 0.45f, 0.45f};
// Policy ordering is FR FL RR RL. Use the same nominal B2 posture as Python Interactive baseline.
constexpr Joint12 kDefault{-0.1f,0.8f,-1.5f, 0.1f,0.8f,-1.5f,
                            -0.1f,0.8f,-1.5f, 0.1f,0.8f,-1.5f};
constexpr Joint12 kMin{-0.8f,0.15f,-2.7f, -0.8f,0.15f,-2.7f,
                        -0.8f,0.15f,-2.7f, -0.8f,0.15f,-2.7f};
constexpr Joint12 kMax{ 0.8f,2.4f,-0.35f, 0.8f,2.4f,-0.35f,
                         0.8f,2.4f,-0.35f, 0.8f,2.4f,-0.35f};
}

TargetGuard makeLegTargets(const Joint18& action) {
    TargetGuard out;
    for (std::size_t i=0;i<kDof;++i)
        if(!std::isfinite(action[i])) throw std::invalid_argument("policy action NaN/Inf");
    for (std::size_t i=0;i<kLegDof;++i) {
        const float raw=kDefault[i] + kScale[i]*action[i];
        if (!std::isfinite(raw)) throw std::invalid_argument("policy joint target overflow");
        const float guarded=std::clamp(raw,kMin[i],kMax[i]);
        if (raw != guarded) ++out.saturated;
        out.q_target_policy[i]=guarded;
        out.q_target_motor[kPolicyToB2Motor[i]]=guarded;
    }
    return out;
}

Joint12 slewMotorTargets(const Joint12& desired, const Joint12& previous, float step) {
    if (!std::isfinite(step) || step<=0.0f || step>0.1f)
        throw std::invalid_argument("bad max target step");
    Joint12 result{};
    for (std::size_t i=0;i<kLegDof;++i) {
        if (!std::isfinite(desired[i]) || !std::isfinite(previous[i]))
            throw std::invalid_argument("nonfinite joint target");
        result[i]=previous[i]+std::clamp(desired[i]-previous[i],-step,step);
    }
    return result;
}

void checkSimFeedback(const Joint12& q, const Joint12& dq,
                      const std::array<float,4>& quat) {
    for (std::size_t i=0;i<kLegDof;++i) {
        if(!std::isfinite(q[i]) || !std::isfinite(dq[i]) || std::abs(dq[i])>20.f)
            throw std::runtime_error("invalid B2 simulation feedback");
        const std::size_t p = (i < 3 ? 1 : i < 6 ? 0 : i < 9 ? 3 : 2)*3+i%3;
        // Extra tolerance on measured feedback; joint target clamps are tighter.
        if(q[i]<kMin[p]-0.4f || q[i]>kMax[p]+0.4f)
            throw std::runtime_error("B2 simulation joint outside plausible range");
    }
    double norm=0;
    for (float v: quat) {
        if(!std::isfinite(v)) throw std::runtime_error("invalid IMU quaternion");
        norm+=static_cast<double>(v)*v;
    }
    if (norm<0.81 || norm>1.21) throw std::runtime_error("bad IMU quaternion norm");
    const auto w=quat[0]/std::sqrt(norm), x=quat[1]/std::sqrt(norm);
    const auto y=quat[2]/std::sqrt(norm), z=quat[3]/std::sqrt(norm);
    const double roll=std::atan2(2*(w*x+y*z),1-2*(x*x+y*y));
    const double pitch=std::asin(std::clamp(2*(w*y-z*x),-1.,1.));
    if(std::abs(roll)>0.60 || std::abs(pitch)>0.60)
        throw std::runtime_error("simulation torso tilt >0.60 rad; stop test/reset simulator");
}
} // namespace interactive_b2z1::sim_control
