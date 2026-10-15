#include "b2z1_live_input.h"
#include <cassert>
#include <iostream>
#include <stdexcept>
using namespace m24e2e;
int main(){
    const auto now=Clock::now();
    B2State b{};b.valid=true;b.packet_received_at=now-std::chrono::milliseconds(10);
    for(unsigned i=0;i<12;i++){b.motor_q[i]=float(i);b.motor_dq[i]=float(i)*.5f;}
    ArmState z{};z.mode=Mode::Mock;z.ipc_received_at=now-std::chrono::milliseconds(5);
    z.q={.1,1.48,-.63,-.84,0,1.57};z.dq={0,0,0,0,0,0};
    auto out=makeLiveInput(b,z,{0,0,0},{.45f,0,.35f},now,Mode::Mock);
    assert(out.robot.joint_q[0]==3.f);assert(out.robot.joint_q[3]==0.f);
    assert(out.robot.joint_q[12]==.1f);assert(out.robot.joint_q[13]==1.48f);
    assert(out.robot.ee_local_xyz[0]>.2f&&out.robot.ee_local_xyz[0]<.3f);
    interactive_b2z1::ObservationBuilder obs_builder;
    const auto obs=obs_builder.build(out.robot, {0.f,0.f,0.f});
    assert(obs.size()==799);
    assert(obs[5]==(out.robot.joint_q[0]-.0f+.1f)); // default first q = -0.1
    assert(obs[5+12]==out.robot.joint_q[12]-0.f); // Z1 default joint1 = 0
    auto fails=[&](auto invoke){bool threw=false;try{invoke();}catch(const std::exception&){threw=true;}assert(threw);};
    fails([&]{makeLiveInput(b,z,{0,0,0},{.45f,0,.35f},now,Mode::SdkUnqualified);});
    b.packet_received_at=now-std::chrono::milliseconds(140);
    fails([&]{makeLiveInput(b,z,{0,0,0},{.45f,0,.35f},now,Mode::Mock);});
    b.packet_received_at=now-std::chrono::milliseconds(80);
    fails([&]{makeLiveInput(b,z,{0,0,0},{.45f,0,.35f},now,Mode::Mock);}); // >50ms skew
    b.packet_received_at=now-std::chrono::milliseconds(10);
    z.ipc_received_at=now-std::chrono::milliseconds(110);
    fails([&]{makeLiveInput(b,z,{0,0,0},{.45f,0,.35f},now,Mode::Mock);});
    std::cout<<"M2.4e2e live input checks PASS\n";
}
