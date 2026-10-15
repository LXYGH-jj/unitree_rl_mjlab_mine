#pragma once
// M2.4e2h: SDK-independent numerical sanity gate for Z1 model-based feedforward.
// NOT a motor torque safety specification or substitute for mechanical support/E-stop.
#include <array>
#include <cmath>
#include <stdexcept>
namespace m24e2h {
using Vec6=std::array<double,6>;
// Coarse fail-closed magnitude sanity only. This is NOT an approved motor rating.
inline constexpr double kModelTorqueSanityBoundNm=20.0;
inline Vec6 combineGravityTorque(const Vec6& gravity,const Vec6& extra){
    Vec6 out{};
    for(int i=0;i<6;++i){
        if(!std::isfinite(gravity[i])||!std::isfinite(extra[i]))
            throw std::runtime_error("gravity compensation nonfinite");
        out[i]=gravity[i]+extra[i];
        if(!std::isfinite(out[i])||std::abs(out[i])>kModelTorqueSanityBoundNm)
            throw std::runtime_error("gravity compensation exceeds unqualified sanity bound");
    }
    return out;
}
} // namespace m24e2h
