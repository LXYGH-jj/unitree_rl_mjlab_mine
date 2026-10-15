#include "z1_lowcmd_plan.h"
#include <algorithm>
#include <cmath>
namespace m24e2b {
Planner::Planner(Limits l):limits_(l){
    if(l.target_ttl.count()<10 || l.target_ttl.count()>1000) throw std::invalid_argument("invalid TTL");
    for(int i=0;i<6;i++){
        if(!std::isfinite(l.lo[i]) || !std::isfinite(l.hi[i]) || l.lo[i]>=l.hi[i] ||
           !std::isfinite(l.velocity_max[i]) || l.velocity_max[i]<=0 ||
           !std::isfinite(l.acceleration_max[i]) || l.acceleration_max[i]<=0 ||
           l.temperature_max[i]<=0) throw std::invalid_argument("invalid per-joint limits");
    }
}
[[noreturn]] void Planner::reject(const char* why){ armed_=false;fault_=true;reason_=why;throw std::runtime_error(why); }
void Planner::validateFeedback(const Feedback& f,Time now){
    if(fault_) throw std::runtime_error("fault latched: "+reason_);
    if(!f.sdk_connected||!f.lowcmd_reported) reject("sdk link or LOWCMD mode lost");
    if(f.read_at==Time{}||f.read_at>now||now-f.read_at>std::chrono::milliseconds(80))
        reject("sample time is stale/invalid (not UDP receipt proof)");
    for(int i=0;i<6;i++){
        if(!std::isfinite(f.q[i])||!std::isfinite(f.dq[i])||!std::isfinite(f.tau[i])||
           f.motor_error[i]!=0 || f.motor_connection[i]!=0 ||
           f.temperature[i]>limits_.temperature_max[i] ||
           f.q[i]<limits_.lo[i]-0.02 || f.q[i]>limits_.hi[i]+0.02)
            reject("joint feedback/temperature/connection fault");
    }
}
void Planner::reset(const Feedback& f,Time now){
    if(fault_) throw std::runtime_error("fault latched: construct new planner");
    validateFeedback(f,now);
    q_command_=f.q;last_velocity_.fill(0);last_sequence_=0;last_issue_={};
    target_={}; initialized_=true; armed_=false;
}
void Planner::acceptTarget(const Target& t,Time now){
    if(fault_||!initialized_) throw std::runtime_error("planner unavailable");
    if(!t.seq || t.seq<=last_sequence_ || t.issued_at==Time{} ||
       t.issued_at>now+std::chrono::milliseconds(10) ||
       now-t.issued_at>std::chrono::milliseconds(100) ||
       (last_issue_!=Time{} && t.issued_at<=last_issue_)) reject("invalid/replayed/stale target");
    for(int i=0;i<6;i++) if(!std::isfinite(t.q[i]) ||
          t.q[i]<limits_.lo[i] || t.q[i]>limits_.hi[i]) reject("out-of-limit target");
    last_sequence_=t.seq; last_issue_=t.issued_at;target_=t;armed_=t.enable;
}
Command Planner::tick(const Feedback& f,Time now,double dt){
    if(!initialized_) throw std::runtime_error("planner not initialized");
    validateFeedback(f,now);
    if(!std::isfinite(dt)||dt<=0 ||dt>0.02) reject("invalid/missed control period");
    Command out{};
    if(armed_&&now-target_.issued_at>limits_.target_ttl){armed_=false;reason_="target watchdog expired";}
    if(!armed_){
        // On timeout, follow measured configuration rather than a prior moving target.
        // This is a *software target hold*, NOT proof of physical safe stop.
        q_command_=f.q;last_velocity_.fill(0);
        out.q=f.q;out.dq.fill(0);out.track=false;return out;
    }
    // Rate + acceleration limited planned position. No PD gains or real force limits here.
    for(int i=0;i<6;i++){
        const double desired_v=std::clamp((target_.q[i]-q_command_[i])/dt,-limits_.velocity_max[i],limits_.velocity_max[i]);
        const double step_a=limits_.acceleration_max[i]*dt;
        const double next_v=std::clamp(desired_v,last_velocity_[i]-step_a,last_velocity_[i]+step_a);
        const double proposed=std::clamp(q_command_[i]+next_v*dt,limits_.lo[i],limits_.hi[i]);
        if(std::abs(proposed-f.q[i])>0.30) reject("tracking error exceeds offline guard");
        // Reconstruction via position delta / dt can exceed velocity_max by
        // floating-point roundoff even though desired_v was clamped.
        // Keep the emitted velocity within the same original bound.
        out.dq[i]=std::clamp((proposed-q_command_[i])/dt,
                             -limits_.velocity_max[i],limits_.velocity_max[i]);
        q_command_[i]=proposed;last_velocity_[i]=out.dq[i];out.q[i]=proposed;
    }
    out.track=true;return out;
}
void Planner::disarm() noexcept {armed_=false;}
}
