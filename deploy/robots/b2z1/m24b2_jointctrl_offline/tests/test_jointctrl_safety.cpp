#include "z1_jointctrl_safety.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace b2z1::z1_jointctrl;
using namespace std::chrono_literals;

void check(bool ok, const std::string& msg) {
    if (!ok) throw std::runtime_error(msg);
}
template <typename F>
void rejects(F&& f, const std::string& msg) {
    bool threw = false;
    try { f(); } catch (const std::exception&) { threw = true; }
    check(threw, msg);
}
Config mockConfig() {
    Config c;
    // Deliberately fictitious mock limits. NOT real Unitree Z1 specifications.
    c.q_min.fill(-2.0);
    c.q_max.fill(2.0);
    c.max_velocity_rad_s.fill(0.50);
    return c;
}
Feedback mockFeedback(Time t, double q0 = 0) {
    Feedback f;
    f.q[0]=q0;
    f.received_at=t;
    f.confirmed_new_packet=true;
    return f;
}
int main() {
    const Time t = Clock::now();
    rejects([]{ SafetyCore unsafe(Config{}); }, "zero limits must reject");
    SafetyCore gate(mockConfig());
    check(!gate.authorized() && !gate.next(t).has_value(), "default fail closed");
    rejects([&]{gate.authorize(true,true,true,t);},"no feedback must reject");
    // That failure latches: create a fresh core for next tests.
    SafetyCore s(mockConfig());
    for (int i=1;i<=3;++i) s.receive(mockFeedback(t+std::chrono::milliseconds(i*5)), t+std::chrono::milliseconds(i*5));
    rejects([&]{s.authorize(false,true,true,t+15ms);},"electrical inspection required");
    rejects([&]{s.authorize(true,false,true,t+15ms);},"operator acknowledgement required");
    rejects([&]{s.authorize(true,true,false,t+15ms);},"SDK mode confirmation required");
    s.authorize(true,true,true,t+15ms);
    check(s.authorized(),"armed");
    Joint6 goal{};
    goal[0]=1.0;
    s.setGoal(goal,t+15ms);
    auto out = s.next(t+20ms);
    check(out.has_value(),"trajectory output available");
    check(std::abs(out->q[0]-0.0025)<1e-9,"rate bound 0.50rad/s x5ms");
    check(std::abs(out->dq[0]-0.50)<1e-9,"command derivative");
    auto f=mockFeedback(t+24ms,0.0025);
    s.receive(f,t+24ms);
    out = s.next(t+25ms);
    check(out && std::abs(out->q[0]-0.005)<1e-9,"second bounded step");
    rejects([&]{s.next(t+300ms);},"watchdog must fail closed");
    check(s.faulted()&&!s.authorized(),"deadline latches fault");

    SafetyCore stale(mockConfig());
    f=mockFeedback(t+1ms);
    f.confirmed_new_packet=false;
    rejects([&]{stale.receive(f,t+1ms);}, "must reject unproven packet freshness");
    check(stale.faulted(),"packet failure latches");

    SafetyCore motors(mockConfig());
    f=mockFeedback(t+1ms);
    f.motor_error[2]=0x04;
    rejects([&]{motors.receive(f,t+1ms);}, "motor error reject");

    SafetyCore duplicate(mockConfig());
    duplicate.receive(mockFeedback(t+1ms),t+1ms);
    rejects([&]{duplicate.receive(mockFeedback(t+1ms),t+2ms);},"duplicate sample reject");

    SafetyCore limit(mockConfig());
    for(int i=1;i<=3;++i) limit.receive(mockFeedback(t+std::chrono::milliseconds(i)),t+std::chrono::milliseconds(i));
    limit.authorize(true,true,true,t+3ms);
    goal[0]=3.0;
    rejects([&]{limit.setGoal(goal,t+3ms);},"goal outside bounds reject");

    SafetyCore track(mockConfig());
    for(int i=1;i<=3;++i) track.receive(mockFeedback(t+std::chrono::milliseconds(i)),t+std::chrono::milliseconds(i));
    track.authorize(true,true,true,t+3ms);
    track.next(t+5ms);
    rejects([&]{track.receive(mockFeedback(t+6ms,0.5),t+6ms);},"excessive tracking deviation reject");

    std::cout << "M2.4b2 JOINTCTRL safety core: PASS (mock feedback; NO robot IO)\n";
}
