#include "m24c_shadow_bridge.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace b2z1::m24c;

static void require(bool v, const char* message) { if(!v) throw std::runtime_error(message); }
template<class F> static void rejects(F&& f, const char* message) {
    bool thrown=false;try{f();}catch(const std::exception&){thrown=true;}
    require(thrown,message);
}
static ShadowBridge bridge() {
    return ShadowBridge([](const Observation799& obs){
       Joint18 a{};
       // Actor output is a function of the real feedback and desired EE obs.
       for(int i=0;i<12;++i) a[i]=0.1f*i;
       a[0] = obs[60] + obs[5];
       for(int i=12;i<18;++i) a[i]=5.f; // MUST NOT be used as Z1 goal
       return a;
    });
}
static void feed(ShadowBridge& b, MonoTime now) {
    Joint12 motor_q{}, motor_dq{};
    for(int i=0;i<12;++i) motor_q[i]=static_cast<float>(i);
    Arm6 arm_q{0.f,1.48f,-.63f,-.84f,0.f,1.57f}; Arm6 arm_dq{};
    b.onB2(motor_q,motor_dq,{1.f,0.f,0.f,0.f},{0.f,0.f,0.f},now-std::chrono::milliseconds(9));
    b.onZ1(arm_q,arm_dq,now-std::chrono::milliseconds(8),true);
}
int main() {
    const auto now = MonoClock::now();
    const Command3 entry{.45f,0.f,.35f};
    const Arm6 ik{0.f,1.48f,-.63f,-.84f,0.f,1.57f};
    {
      auto b=bridge();feed(b,now);
      auto result=b.preview(Mode::Stand,{0.f,0.f,0.f},entry,{.46f,0.f,.35f},ik,now);
      require(result.observation.size()==799,"799D expected");
      // motor FR first maps from motor input [3], default_q[0]=-0.1
      require(std::abs(result.observation[5]-3.1f)<1.e-5f,"B2 motor to policy order");
      require(std::abs(result.observation[60]-.26f)<1.e-5f,"EE training nominal mount offset");
      require(std::abs(result.observation[62]-.26f)<1.e-5f,"EE training nominal Z offset");
      require(std::abs(result.observation[5+12]-ik[0])<1.e-5f,"Z1 joint feedback");
      require(std::abs(result.leg_policy_action[0]-(3.1f+.26f))<1.e-5f,"policy actor input routed");
      require(result.z1_ik_goal_unqualified[1]==ik[1],"IK separate from actor");
      require(result.policy_action[12]==5.f,"actor arm output exists but is NOT used as Z1 goal");
      require(!b.faulted(),"healthy shadow");
    }
    {
      auto b=bridge();feed(b,now);
      auto out=b.preview(Mode::Walk,{.12f,0.f,0.f},entry,{.47f,0.f,.35f},ik,now);
      require(out.observation[57]>.2f,"walking cmd scaling");
      rejects([&]{b.preview(Mode::Walk,{.12f,0.f,0.f},entry,{.55f,0.f,.35f},ik,now);},"walk large EE must reject");
      require(b.faulted(),"workspace rejection latches fault");
      rejects([&]{b.preview(Mode::Stand,{0,0,0},entry,entry,ik,now);},"fault sticky");
    }
    {
      auto b=bridge();feed(b,now);
      rejects([&]{b.preview(Mode::Stand,{.1f,0,0},entry,entry,ik,now);},"stand motion reject");
      require(b.faulted(),"mode rejection latched");
    }
    {
      auto b=bridge();feed(b,now);
      rejects([&]{b.preview(Mode::Stand,{0,0,0},entry,entry,ik,now+std::chrono::milliseconds(120));},"stale feedback reject");
    }
    {
      auto b=bridge();Joint12 motor{};Arm6 arm{};
      b.onB2(motor,motor,{1,0,0,0},{0,0,0},now-std::chrono::milliseconds(2));
      rejects([&]{b.onZ1(arm,arm,now-std::chrono::milliseconds(1),false);},"unverified Z1 reject");
    }
    {
      auto b=bridge();Joint12 motor{};Arm6 arm{};
      b.onB2(motor,motor,{1,0,0,0},{0,0,0},now-std::chrono::milliseconds(95));
      b.onZ1(arm,arm,now-std::chrono::milliseconds(1),true);
      rejects([&]{b.preview(Mode::Stand,{0,0,0},entry,entry,ik,now);},"B2 Z1 desync reject");
    }
    std::cout << "M2.4c SHADOW INTEGRATION: PASS (799D + B2 action12 + independent IK6, NO robot IO)\n";
}
