// SIM-only end-to-end test: C++ FSM-style command source -> one IPC
// ReadOnlyPoller thread -> mock Worker TARGET/GET. NO DDS, SDK, motors.
#include "z1_ipc_client.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace m24e2e;
namespace {
void require(bool ok,const char* text){if(!ok)throw std::runtime_error(text);}
template<class P>
ArmState waitFor(ReadOnlyPoller& poll,P pred,const char* what){
    auto deadline=Clock::now()+std::chrono::seconds(4);
    ArmState state{};std::string err;
    while(Clock::now()<deadline){
        if(poll.latest(state,err) && pred(state))return state;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::runtime_error(std::string("timeout waiting for ")+what+
                             "; last IPC status "+err);
}
}
int main(int argc,char** argv){
    if(argc!=2) return 2;
    ReadOnlyPoller read_only;
    bool refused=false;
    try{read_only.start(argv[1],Mode::SdkUnqualified,40,true);}
    catch(const std::invalid_argument&){refused=true;}
    require(refused,"SDK must reject the mock command opt-in");
    read_only.start(argv[1],Mode::Mock,40,true);
    auto baseline=waitFor(read_only,[](const ArmState& f){
        return f.tracking&&f.last_target_seq>=1;
    },"mock initial TRACK");
    const double origin=baseline.q[4];
    refused=false;
    try{read_only.setMockJ5Delta(0.031);}
    catch(const std::invalid_argument&){refused=true;}
    require(refused,"out-of-envelope J5 target accepted");
    read_only.setMockJ5Delta(0.005);
    auto forward=waitFor(read_only,[origin](const ArmState& f){
        return f.tracking && std::abs(f.q[4]-(origin+0.005))<0.0005;
    },"mock J5 +0.005 feedback");
    require(forward.last_target_seq>baseline.last_target_seq,
            "mock J5 command sequence did not advance");
    for(int i=0;i<6;++i) if(i!=4)
        require(std::abs(forward.q[i]-baseline.q[i])<1e-7,
                "mock command moved an unrelated joint");
    read_only.setMockJ5Delta(0.0);
    auto returned=waitFor(read_only,[origin](const ArmState& f){
        return f.tracking && std::abs(f.q[4]-origin)<0.0005;
    },"mock J5 return");
    require(returned.last_target_seq>forward.last_target_seq,
            "return did not advance command sequence");
    read_only.stop();
    std::string error;ArmState stale;
    require(!read_only.latest(stale,error),"stopped IPC still advertises valid state");
    std::cout<<"M2.4e2e MOCK J5 COMMAND PASS: one IPC owner, +0.005, return, "
                "six-joint invariance, SDK command gate remains locked.\n"
                "NO vendor SDK, DDS, or motors used.\n";
}
