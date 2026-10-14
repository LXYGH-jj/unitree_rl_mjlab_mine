#include "m24d5_sim_gate.h"
#include <cassert>
#include <chrono>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>
using namespace b2z1;
using Time=z1_jointctrl::Time;
using Joint6=z1_jointctrl::Joint6;
int main(){
    Joint6 lo{},hi{},q{},dq{},goal{};
    lo.fill(-3);hi.fill(3); q={0.,1.4,-.6,-.8,0.,1.5};goal=q;goal[0]+=.1;
    const auto p="/tmp/m24d5_test_"+std::to_string(getpid())+".lock";
    auto now=z1_jointctrl::Clock::now();
    {
        m24d5::SimGate g(lo,hi,p);
        assert(!g.step(101,q,dq,goal,now).has_value());
        assert(!g.step(102,q,dq,goal,now+std::chrono::milliseconds(5)).has_value());
        const auto out=g.step(103,q,dq,goal,now+std::chrono::milliseconds(10));
        assert(out.has_value());assert((*out)[0]>=q[0]);
        assert(g.commandsEmitted()==1);
        // second owner must be refused while first exists
        bool conflict=false;
        try{m24d5::SimGate second(lo,hi,p);}catch(const std::exception&){conflict=true;}
        assert(conflict);
        const auto nxt=g.step(104,q,dq,goal,now+std::chrono::milliseconds(15));
        assert(nxt.has_value() && (*nxt)[0]>(*out)[0]);
        g.disarm();
        bool refused=false;
        try{g.step(105,q,dq,goal,now+std::chrono::milliseconds(20));}
        catch(const std::exception&){refused=true;}
        assert(refused);
    }
    {
        m24d5::SimGate g(lo,hi,p);
        assert(!g.step(10,q,dq,goal,now).has_value());
        bool bad=false;
        try {g.step(9,q,dq,goal,now+std::chrono::milliseconds(5));}
        catch(const std::exception&){bad=true;}
        assert(bad&&g.faulted());
    }
    // 10,500 > legacy 10,000-command (50-second) cutoff.
    // Simulated 5ms ticks only: no real time sleep, IO, network or motors.
    {
        m24d5::SimGate g(lo,hi,p);
        auto t=now+std::chrono::seconds(1);
        for(int i=0;i<10510;++i){
            auto output=g.step(static_cast<std::uint32_t>(300+i),q,dq,goal,
                               t+std::chrono::milliseconds(5*i));
            if(i>=2) assert(output.has_value());
            else assert(!output.has_value());
        }
        assert(g.commandsEmitted()==10508);
        assert(!g.faulted());
    }
    // Bound the synthetic history; the original d4 test of the first command
    // remains valid, while long runs only retain the latest 64 commands.
    {
        m24d4::MockZ1Transport mock;
        assert(!mock.latestCommand().has_value());
        for(int i=0;i<1000;++i){
            z1_jointctrl::JointCommand c{};c.q[0]=double(i);
            mock.emit(c);
        }
        const auto history=mock.recordedCommands();
        assert(history.size()==64);
        assert(history.front().q[0]==936.);
        assert(history.back().q[0]==999.);
        assert(mock.latestCommand().has_value());
        assert(mock.latestCommand()->q[0]==999.);
    }
    ::unlink(p.c_str());
    std::cout<<"M2.4d5p1: 10508 ticks PASS; mock history bounded at 64; no SDK IO\n";
}
