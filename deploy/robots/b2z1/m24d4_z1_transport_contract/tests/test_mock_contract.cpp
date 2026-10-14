#include "z1_transport_contract.h"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
using namespace std::chrono_literals;
using namespace b2z1::m24d4;
using namespace b2z1::m24d1;
using namespace b2z1::z1_jointctrl;
void check(bool yes,const char* reason){if(!yes)throw std::runtime_error(reason);}
template <typename F> void rejects(F f,const char* msg){bool caught=false;try{f();}catch(const std::exception&){caught=true;}check(caught,msg);}
Config config(){Config c;c.q_min.fill(-2);c.q_max.fill(2);c.max_velocity_rad_s.fill(0.5);c.max_feedback_age=100ms;c.max_command_interval=100ms;return c;}
Limits limits(){Limits l;l.q_min.fill(-2);l.q_max.fill(2);l.max_motor_temperature_c.fill(65);return l;}
Snapshot sample(){Snapshot s;s.sdk_connected=true;s.jointctrl_reported=true;s.temperature_c.fill(25);return s;}
struct Impostor final : IZ1Transport {
 TransportKind kind() const noexcept override { return TransportKind::SyntheticMock; }
 std::optional<ReceivedFrame> popFrame() override { return std::nullopt; }
 void emit(const JointCommand&) override { throw std::logic_error("impersonator"); }
 void disarm() noexcept override {}
};
std::string newPath(){char dir[]="/tmp/m24d4_lease_XXXXXX";auto* p=::mkdtemp(dir);if(!p)throw std::runtime_error("mkdtemp failed");return std::string(p)+"/exclusive.lock";}
void cleanPath(const std::string& p){::unlink(p.c_str());::rmdir(p.substr(0,p.find_last_of('/')).c_str());}
void feed3(MockZ1Transport& t, MockOnlyRuntime& r,Time at){
  for(int i=1;i<=3;i++){const Time now=at+5ms*i;t.injectSynthetic(sample(),now);check(r.pumpOne(now),"synthetic feedback missing");}
}
int main(){
 const Time t=Clock::now();
 const std::string lock=newPath();
 {
  MockZ1Transport mock;
  MockOnlyRuntime rt(mock,config(),limits(),lock);
  check(!rt.tick(t),"unauthorized tick wrote");
  check(!rt.pumpOne(t),"empty queue unexpectedly contained RX");
  check(rt.sentCount()==0,"sent count before approval");
  check(!mock.softwareDisarmed(),"initial synthetic sink disarmed");
  rejects([&]{CooperativeOwnerLease duplicate(lock);},"same-process cooperative lock not exclusive");
  const pid_t pid=::fork();
  check(pid>=0,"fork failed");
  if(pid==0){
    try { CooperativeOwnerLease duplicate(lock); _exit(4); }
    catch(const std::exception&) { _exit(0); }
  }
  int st=0;check(::waitpid(pid,&st,0)==pid && WIFEXITED(st)&&WEXITSTATUS(st)==0,"another process acquired cooperative lease");
  feed3(mock,rt,t);
  rt.authorizeForOfflineTest(t+15ms);
  check(rt.authorized(),"mock authorization failed");
  Joint6 goal{};goal[0]=0.06;
  rt.setIkGoal(goal,t+15ms);
  check(rt.tick(t+20ms),"expected a mock command");
  const auto commands=mock.recordedCommands();
  check(commands.size()==1,"mock issued extra commands");
  check(std::abs(commands[0].q[0]-.0025)<1e-10,"joint speed limit mismatch");
  rejects([&]{rt.tick(t+300ms);},"stale feedback must fail");
  check(rt.faulted()&&!rt.authorized()&&mock.softwareDisarmed(),"timeout did not stop software output");
  check(mock.recordedCommands().size()==1,"command after fault");
 }
 {
  CooperativeOwnerLease renewed(lock);
  check(renewed.held(),"cooperative owner not released");
 }
 cleanPath(lock);
 {
  VendorDisabledTransport vendor;
  check(vendor.kind()==TransportKind::VendorDisabled,"vendor type incorrect");
  check(!vendor.popFrame().has_value(),"locked vendor made up feedback");
  rejects([&]{vendor.emit(JointCommand{});},"locked vendor emitted motor command");
  const auto path=newPath();
  rejects([&]{MockOnlyRuntime never(vendor,config(),limits(),path);},"vendor accepted by mock runtime");
  cleanPath(path);
 }
 {
  Impostor forged;auto path=newPath();
  rejects([&]{MockOnlyRuntime never(forged,config(),limits(),path);},"transport kind impersonation accepted");
  cleanPath(path);
 }
 {
  auto path=newPath(); MockZ1Transport mock;MockOnlyRuntime rt(mock,config(),limits(),path);
  auto overheat=sample();overheat.temperature_c[3]=100;
  mock.injectSynthetic(overheat,t+5ms);
  rejects([&]{rt.pumpOne(t+5ms);},"overtemp packet not rejected");
  check(rt.faulted()&&mock.softwareDisarmed(),"overtemp did not latch");
  cleanPath(path);
 }
 {
  auto path=newPath(); MockZ1Transport mock;MockOnlyRuntime rt(mock,config(),limits(),path);
  feed3(mock,rt,t);
  rt.authorizeForOfflineTest(t+15ms);
  mock.failNextEmit();
  rejects([&]{rt.tick(t+20ms);},"injected command sink failure not rejected");
  check(rt.faulted()&&mock.softwareDisarmed()&&mock.recordedCommands().empty(),"sink error not latched");
  cleanPath(path);
 }
 {
  auto path=newPath(); MockZ1Transport mock;MockOnlyRuntime rt(mock,config(),limits(),path);
  mock.injectSynthetic(sample(),t+10ms);
  mock.injectSynthetic(sample(),t+5ms);
  check(rt.pumpOne(t+10ms),"first frame missing");
  rejects([&]{rt.pumpOne(t+15ms);},"non-monotonic frame was accepted");
  check(rt.faulted()&&mock.softwareDisarmed(),"replay did not latch");
  cleanPath(path);
 }
 std::cout<<"M2.4d4 MOCK CONTRACT + M2.4d1/c3 ROUTE + ADVISORY OWNER: PASS (NO SDK IO)\n";
}
