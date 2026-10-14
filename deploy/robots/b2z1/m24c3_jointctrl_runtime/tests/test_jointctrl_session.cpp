#include "z1_jointctrl_session.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace b2z1::m24c3;
using namespace std::chrono_literals;

struct FakeSender final : CommandSink {
    std::vector<JointCommand> emitted;
    bool refuse{false};
    void emit(const JointCommand& cmd) override {
        if (refuse) throw std::runtime_error("fake vendor send failure");
        emitted.push_back(cmd);
    }
};

void check(bool condition, const char* msg) {
    if (!condition) throw std::runtime_error(msg);
}
template<class F> void rejects(F&& f, const char* msg) {
    bool did_throw=false;
    try { f(); } catch (const std::exception&) { did_throw=true; }
    check(did_throw,msg);
}
Config fakeConfig() {
    Config c{};
    // Not vendor Z1 limits. Fake mathematical test range only.
    c.q_min.fill(-2.0); c.q_max.fill(2.0);
    c.max_velocity_rad_s.fill(0.5);
    c.max_feedback_age=100ms; c.max_command_interval=100ms;
    c.max_tracking_error_rad=.2;
    c.min_consecutive_new_packets=3;
    return c;
}
Feedback sample(Time t, double q0=0) {
    Feedback fb{}; fb.received_at=t; fb.confirmed_new_packet=true; fb.q[0]=q0;
    return fb;
}
void prime(JointCtrlSession& s, Time t) {
    for(int i=1;i<=3;++i) s.acceptFeedback(sample(t+i*5ms),t+i*5ms);
}
int main() {
    const Time t=b2z1::z1_jointctrl::Clock::now();
    FakeSender fake;
    JointCtrlSession session(fakeConfig(),fake);
    check(!session.authorized() && !session.controlTick(t), "never send before auth");
    rejects([&]{session.updateIkGoal(Joint6{},t);}, "IK not allowed before auth");
    prime(session,t);
    rejects([&]{session.authorize(false,true,true,t+15ms);}, "requires electrical approval");
    rejects([&]{session.authorize(true,false,true,t+15ms);}, "requires operator approval");
    rejects([&]{session.authorize(true,true,false,t+15ms);}, "requires mode confirmation");
    check(fake.emitted.empty(),"commands before authorization");
    session.authorize(true,true,true,t+15ms); // FAKE APPROVALS ONLY, TEST BENCH
    Joint6 ik{}; ik[0]=.06;
    session.updateIkGoal(ik,t+15ms);
    check(session.controlTick(t+20ms), "first command must be emitted");
    check(fake.emitted.size()==1,"one packet");
    check(std::abs(fake.emitted.back().q[0]-.0025)<1e-9,"velocity-limited IK0");
    check(std::abs(fake.emitted.back().dq[0]-.5)<1e-9,"velocity-limited dq0");
    session.acceptFeedback(sample(t+23ms,.0025),t+23ms);
    check(session.controlTick(t+25ms),"next command");
    check(std::abs(fake.emitted.back().q[0]-.005)<1e-9,"second bounded step");
    check(session.sentCount()==2,"sent count");
    session.disarm();
    check(!session.controlTick(t+30ms),"disarm stops sends");
    check(fake.emitted.size()==2,"no output after disarm");

    // A valid-looking single SDK snapshot is insufficient: three actual receive
    // events must be independently demonstrated before allowing any issue().
    FakeSender insufficient_sink;
    JointCtrlSession insufficient(fakeConfig(),insufficient_sink);
    insufficient.acceptFeedback(sample(t+5ms),t+5ms);
    insufficient.acceptFeedback(sample(t+10ms),t+10ms);
    rejects([&]{insufficient.authorize(true,true,true,t+10ms);},
            "two frames are insufficient");
    check(insufficient_sink.emitted.empty(), "no output with insufficient packets");

    FakeSender bad;
    JointCtrlSession rejected(fakeConfig(),bad);
    Feedback stale=sample(t+1ms);stale.confirmed_new_packet=false;
    rejects([&]{rejected.acceptFeedback(stale,t+1ms);},"cannot fabricate Z1 packet proof");
    check(rejected.faulted()&&!rejected.controlTick(t+2ms),"stale rejected forever");
    check(bad.emitted.empty(),"never emit after stale feedback");

    FakeSender broken;
    JointCtrlSession failed(fakeConfig(),broken);prime(failed,t);
    failed.authorize(true,true,true,t+15ms);
    failed.updateIkGoal(ik,t+15ms);
    broken.refuse=true;
    rejects([&]{failed.controlTick(t+20ms);},"vendor send fail");
    check(failed.faulted()&&!failed.authorized()&&broken.emitted.empty(),"sink failure disables session");
    broken.refuse=false;
    rejects([&]{failed.controlTick(t+25ms);},"cannot bypass fault after transport recovers");

    FakeSender deadline_sink;
    JointCtrlSession missed(fakeConfig(),deadline_sink);prime(missed,t);
    missed.authorize(true,true,true,t+15ms);
    missed.updateIkGoal(ik,t+15ms);
    rejects([&]{missed.controlTick(t+250ms);}, "watchdog missed");
    check(deadline_sink.emitted.empty(),"watchdog cannot emit");

    FakeSender motor_sink;
    JointCtrlSession motor(fakeConfig(),motor_sink);
    auto fb=sample(t+1ms);fb.motor_error[0]=1;
    rejects([&]{motor.acceptFeedback(fb,t+1ms);},"reject motor error");
    check(motor_sink.emitted.empty(),"motor error cannot emit");
    std::cout << "M2.4c3 JOINTCTRL EXECUTION GATE: PASS (mock sender + bounded q/dq, NO hardware IO)\n";
}
