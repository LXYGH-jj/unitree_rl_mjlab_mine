#include "z1_lowcmd_plan.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace m24e2b;
static Limits config(){
    Limits l;
    l.lo={-2.6,0.,-2.87,-1.50,-1.32,-2.78};
    l.hi={2.6,2.95,-.01,1.50,1.32,2.78};
    l.velocity_max.fill(.4);l.acceleration_max.fill(.8);l.temperature_max.fill(75);
    return l;
}
static Feedback state(Time t){
    Feedback f;f.q={0,1.48,-.63,-.84,0,1.57};f.sdk_connected=true;
    f.lowcmd_reported=true;f.read_at=t;f.temperature.fill(30);return f;
}
static void speed_accel(){
    const auto t=Clock::now();Planner p(config());auto f=state(t);p.reset(f,t);
    auto g=f.q;g[1]+=0.15;p.acceptTarget({1,t,g,true},t);
    Vec6 prev_vel{};
    for(int n=0;n<30;n++){
        auto now=t+std::chrono::milliseconds(2*(n+1));f.read_at=now;
        const auto c=p.tick(f,now,.002);
        assert(c.track);
        assert(c.q[1]>=f.q[1]);
        assert(std::abs(c.dq[1])<=.4+1e-10);
        assert(std::abs(c.dq[1]-prev_vel[1])<=.0016+1e-10);
        prev_vel=c.dq;
        f.q=c.q;
    }
}
static void watchdog(){
    const auto t=Clock::now();Planner p(config());auto f=state(t);p.reset(f,t);
    auto g=f.q;g[0]+=.04;p.acceptTarget({1,t,g,true},t);
    f.read_at=t+std::chrono::milliseconds(151);
    auto c=p.tick(f,f.read_at,.002);
    assert(!c.track&&!p.armed());
    assert(c.q==f.q);
}
static void replay_and_limits(){
    auto t=Clock::now();Planner p(config());auto f=state(t);p.reset(f,t);
    p.acceptTarget({1,t,f.q,true},t);
    bool bad=false;
    try{p.acceptTarget({1,t,f.q,true},t);}catch(const std::exception&){bad=true;}
    assert(bad&&p.faulted());
    Planner p2(config());p2.reset(f,t);auto g=f.q;g[1]=99;
    bad=false;try{p2.acceptTarget({1,t,g,true},t);}catch(const std::exception&){bad=true;}
    assert(bad&&p2.faulted());
}
static void feedback_fault(){
    const auto t=Clock::now();Planner p(config());auto f=state(t);p.reset(f,t);
    f.motor_error[2]=1;bool bad=false;
    try{p.tick(f,t+std::chrono::milliseconds(2),.002);}catch(const std::exception&){bad=true;}
    assert(bad&&p.faulted());
}
static void missing_owner(){
    auto t=Clock::now();Planner p(config());auto f=state(t);p.reset(f,t);
    f.lowcmd_reported=false;bool bad=false;
    try{p.tick(f,t+std::chrono::milliseconds(2),.002);}catch(const std::exception&){bad=true;}
    assert(bad&&p.faulted());
}
int main(){speed_accel();watchdog();replay_and_limits();feedback_fault();missing_owner();
    std::cout<<"M2.4e2b pure C++ planner tests PASS (5 cases); NO SDK/no hardware IO\n";
}
