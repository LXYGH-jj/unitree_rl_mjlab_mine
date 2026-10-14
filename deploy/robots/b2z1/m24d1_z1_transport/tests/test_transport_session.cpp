#include "z1_transport_session.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace std::chrono_literals;
using namespace b2z1::m24d1;
using namespace b2z1::z1_jointctrl;
struct FakeSink final: b2z1::m24c3::CommandSink {
    std::vector<JointCommand> writes;
    void emit(const JointCommand& x) override {writes.push_back(x);}
};
void check(bool v,const char* msg) {if(!v)throw std::runtime_error(msg);}
template<class F> void rejects(F f,const char* msg){bool thrown=false;try{f();}catch(const std::exception&){thrown=true;}check(thrown,msg);}
Config cfg(){Config c;c.q_min.fill(-2);c.q_max.fill(2);c.max_velocity_rad_s.fill(0.5);c.max_feedback_age=100ms;c.max_command_interval=100ms;c.max_tracking_error_rad=.2;return c;}
Limits lim(){Limits l;l.q_min.fill(-2);l.q_max.fill(2);l.max_motor_temperature_c.fill(65);return l;}
Snapshot snap(Time t){Snapshot s;s.sdk_connected=true;s.jointctrl_reported=true;s.temperature_c.fill(25);s.sampled_at=t;return s;}
RxEvidence event(std::uint64_t n,Time t){RxEvidence e;e.completed_receive_index=n;e.receive_completed_at=t;e.callback_saw_completed_datagram=true;e.from_expected_peer=true;e.snapshot_decoded_from_this_datagram=true;return e;}
int main(){
 const Time t=Clock::now();
 FakeSink sink;TransportSession s(cfg(),lim(),sink);
 check(!s.tick(t),"unauthorized tick emits");
 FakeSink getter_sink;TransportSession getter_only(cfg(),lim(),getter_sink);
 check(!getter_only.onSdkGetterOnly(snap(t)),"SDK getter cannot authorize");
 check(getter_only.faulted()&&!getter_only.authorized()&&getter_sink.writes.empty(),"getter did not latch ineligible session");
 for(int i=1;i<=3;++i)s.onReceiverCallback(snap(t+i*5ms),event(i,t+i*5ms),t+i*5ms);
 s.authorize(true,true,true,t+15ms);
 Joint6 goal{};goal[0]=.06;s.setIkGoal(goal,t+15ms);
 check(s.tick(t+20ms)&&sink.writes.size()==1,"verified packet route to mock sink");
 check(std::abs(sink.writes[0].q[0]-.0025)<1e-10,"bounded command wrong");
 check(!s.onSdkGetterOnly(snap(t+21ms)),"polling must never create proof");
 check(s.faulted()&&!s.authorized()&&!s.tick(t+25ms),"getter must latch already authorized session");
 rejects([&]{s.authorize(true,true,true,t+25ms);},"cannot authorize old RX after getter poll");
 check(sink.writes.size()==1,"extra output after disarm");
 FakeSink bad_sink;TransportSession bad(cfg(),lim(),bad_sink);
 auto e=event(1,t+5ms);e.from_expected_peer=false;
 rejects([&]{bad.onReceiverCallback(snap(t+5ms),e,t+5ms);},"reject unexpected peer");
 check(bad.faulted()&&!bad.tick(t+10ms),"fault must latch");
 FakeSink replay_sink;TransportSession replay(cfg(),lim(),replay_sink);
 replay.onReceiverCallback(snap(t+5ms),event(3,t+5ms),t+5ms);
 rejects([&]{replay.onReceiverCallback(snap(t+10ms),event(3,t+10ms),t+10ms);},"replayed receive index");
 FakeSink temp_sink;TransportSession temp(cfg(),lim(),temp_sink);
 auto hot=snap(t+5ms);hot.temperature_c[2]=88;
 rejects([&]{temp.onReceiverCallback(hot,event(1,t+5ms),t+5ms);},"high motor temp");
 FakeSink slow_sink;TransportSession slow(cfg(),lim(),slow_sink);
 auto old=snap(t+5ms);rejects([&]{slow.onReceiverCallback(old,event(1,t+5ms),t+200ms);},"stale packet");
 FakeSink decoder_sink;TransportSession decoder(cfg(),lim(),decoder_sink);
 auto bogus=event(1,t+5ms);bogus.snapshot_decoded_from_this_datagram=false;
 rejects([&]{decoder.onReceiverCallback(snap(t+5ms),bogus,t+5ms);},"unattributed SDK getter sample");
 FakeSink fault_sink;TransportSession motor(cfg(),lim(),fault_sink);
 auto faulty=snap(t+5ms);faulty.motor_error[0]=0x40;
 rejects([&]{motor.onReceiverCallback(faulty,event(1,t+5ms),t+5ms);},"motor nonzero code");
 check(motor.faulted(),"motor code did not fault");
 FakeSink time_sink;TransportSession skew(cfg(),lim(),time_sink);
 auto skewed=snap(t+40ms);
 rejects([&]{skew.onReceiverCallback(skewed,event(1,t+5ms),t+40ms);},"large decode skew");
 std::cout<<"M2.4d1 RX PROVENANCE + M2.4c3 MOCK COMMAND ROUTE: PASS (NO SDK IO)\n";
}
