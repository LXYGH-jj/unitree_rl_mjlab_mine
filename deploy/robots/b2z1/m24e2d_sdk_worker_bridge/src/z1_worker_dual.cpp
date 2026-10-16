// M2.4e2d: V1 IPC + planner + selectable mock / explicitly gated vendor SDK.
// Default build is MOCK ONLY. Physical SDK is opt-in at compile AND runtime, NOT bench qualified.
// Linux AF_UNIX/SOCK_SEQPACKET messages are delimited, text V1 messages.
// One client session at a time; local UID verified with SO_PEERCRED.
#include "z1_lowcmd_plan.h"
#ifdef M24E2D_ENABLE_HARDWARE_SDK
#include "z1_sdk_bridge.h"
#endif
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/types.h>
#include <poll.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>
#include <vector>
#include <utility>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>

namespace {
using Clock = std::chrono::steady_clock;
using Vec6 = std::array<double, 6>;
constexpr std::size_t kPacketMax = 512;
constexpr auto kGoalTtl = std::chrono::milliseconds(150); // UNQUALIFIED development value
constexpr const char* kPhysicalAck = "Z1_BENCH_ATTENDED_LOWCMDSDK_RISK_ACCEPTED";
// Existing Worker command limits, shared by IPC validation and the Planner.
// These are NOT certified actuator safety limits; the operator must ensure
// adequate clearance and support before enabling physical joint motion.
constexpr Vec6 kWorkerJointLo{-2.618, 0., -2.8798, -1.5184, -1.3439, -2.7925};
constexpr Vec6 kWorkerJointHi{ 2.618, 2.9671, 0., 1.5184, 1.3439, 2.7925};
// Mock uses the planner itself as a perfect position servo, NOT a physical actuator model.
volatile sig_atomic_t stopping = 0;
void on_signal(int) { stopping = 1; }
std::uint64_t mono_ns(Clock::time_point t = Clock::now()) {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count());
}
struct ScopedFd {
    int fd{-1};
    explicit ScopedFd(int x=-1) : fd(x) {}
    ~ScopedFd(){ if(fd>=0) ::close(fd); }
    ScopedFd(const ScopedFd&)=delete;
    ScopedFd& operator=(const ScopedFd&)=delete;
};
void send_packet(int fd, const std::string& str) {
    if(str.size() >= kPacketMax) throw std::runtime_error("internal oversized reply");
    ssize_t n = ::send(fd, str.data(), str.size(), MSG_NOSIGNAL);
    if(n != static_cast<ssize_t>(str.size())) throw std::runtime_error("IPC send failed");
}
void require_end(std::istringstream& in) {
    std::string extra;
    if(in >> extra) throw std::invalid_argument("extra_fields");
}
m24e2b::Limits development_limits(bool sdk_mode){
    m24e2b::Limits l;
    // The previous Planner J3 upper limit was -0.01 rad, while the IPC
    // accepts 0.0 rad. Measured startup J3 can be at 0.0; the mismatch
    // caused an immediate fault even on an unarmed keyboard connection.
    l.lo=kWorkerJointLo;
    l.hi=kWorkerJointHi;
    // Keep synthetic/non-hardware semantics unchanged. Real SDK bench
    // gets slow, acceleration-limited trajectories as an extra defense.
    l.velocity_max.fill(sdk_mode ? .10 : .4);
    l.acceleration_max.fill(sdk_mode ? .20 : .8);
    l.temperature_max.fill(sdk_mode ? 60 : 75);
    l.target_ttl=kGoalTtl;
    return l; // OFFLINE DEVELOPMENT FIXTURE, NOT APPROVED Z1 MOTOR LIMITS
}

struct TraceRow {
    std::uint64_t time_ns{0}, target_seq{0};
    double dt_ms{0}, io_ms{0}, target_age_ms{-1};
    bool track{false};
    Vec6 goal{}, q_cmd{}, dq_cmd{}, q_meas{}, dq_meas{}, sent_ff{}, tau_meas{};
};
// Optional fixed-size rolling buffer. No disk I/O in motor-control loop.
// Values are SDK getter samples, NOT verified individual fresh UDP packets.
class TraceRing {
public:
    static constexpr std::size_t kCapacity=16384;
    TraceRing():data_(kCapacity){}
    void add(const TraceRow& r) noexcept {
        data_[next_]=r;
        next_=(next_+1)%kCapacity;
        if(count_<kCapacity) ++count_;
    }
    void save(const std::string& path) const noexcept {
        if(path.empty()) return;
        const int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0600);
        if(fd<0){
            std::cerr<<"[Z1 TRACE] Cannot create new CSV "<<path
                     <<" errno="<<errno<<'\n';
            return;
        }
        std::FILE* fp=::fdopen(fd,"w");
        if(!fp){::close(fd); std::cerr<<"[Z1 TRACE] fdopen failed\n";return;}
        std::fputs("mono_ns,cycle_dt_ms,sdk_exchange_ms,target_age_ms,track,target_seq",fp);
        for(int i=1;i<=6;i++)
            std::fprintf(fp,",goal_J%d,qcmd_J%d,dqcmd_J%d,qmeas_J%d,dqmeas_J%d,ff_sent_J%d,tau_meas_J%d",
                         i,i,i,i,i,i,i);
        std::fputc('\n',fp);
        const auto first=(next_+kCapacity-count_)%kCapacity;
        for(std::size_t n=0;n<count_;++n){
            const auto& r=data_[(first+n)%kCapacity];
            std::fprintf(fp,"%llu,%.9g,%.9g,%.9g,%d,%llu",
                         static_cast<unsigned long long>(r.time_ns),
                         r.dt_ms,r.io_ms,r.target_age_ms,r.track?1:0,
                         static_cast<unsigned long long>(r.target_seq));
            for(int i=0;i<6;++i)
                std::fprintf(fp,",%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g",
                             r.goal[i],r.q_cmd[i],r.dq_cmd[i],r.q_meas[i],
                             r.dq_meas[i],r.sent_ff[i],r.tau_meas[i]);
            std::fputc('\n',fp);
        }
        const bool io_error=std::ferror(fp)!=0;
        const int close_rc=std::fclose(fp);
        std::cerr<<"[Z1 TRACE] "<<(io_error||close_rc?"CSV write error ":"CSV saved ")
                 <<path<<" samples="<<count_<<" (last "<<kCapacity<<")\n";
    }
private:
    std::vector<TraceRow> data_;
    std::size_t next_{0},count_{0};
};
struct State {
    Vec6 q{0., 1.48, -.63, -.84, 0., 1.57};
    Vec6 dq{};
    bool enabled{false};
    bool fault_latched{false};
    std::string fault_reason{};
    std::uint64_t last_seq{0};
    std::uint64_t last_stamp{0};
    Clock::time_point last_target{};
    Clock::time_point last_step{Clock::now()};
    const Vec6 lo{kWorkerJointLo};
    const Vec6 hi{kWorkerJointHi};
    m24e2b::Planner planner;
    bool sdk_mode{false};
    Vec6 goal_q{}; // Last accepted keyboard target; diagnostics only.
    std::string trace_csv_path{};
    std::unique_ptr<TraceRing> trace{};
#ifdef M24E2D_ENABLE_HARDWARE_SDK
    std::unique_ptr<m24e2b::SdkBridge> sdk;
    m24e2b::Feedback latest{};
#endif
    explicit State(bool use_sdk,bool has_gripper,bool alore_gain,std::string csv_path):
        planner(development_limits(use_sdk)),sdk_mode(use_sdk),
        trace_csv_path(std::move(csv_path)){
        if(!trace_csv_path.empty()) trace=std::make_unique<TraceRing>();

#ifndef M24E2D_ENABLE_HARDWARE_SDK
        (void)has_gripper; // Mock build keeps SDK hardware options inert.
        (void)alore_gain;
#endif
        if(!sdk_mode){
            goal_q=q;
            auto f=mock_feedback(Clock::now());
            planner.reset(f, f.read_at);
            return;
        }
#ifdef M24E2D_ENABLE_HARDWARE_SDK
        sdk=std::make_unique<m24e2b::SdkBridge>(
            has_gripper, alore_gain ? m24e2b::GainProfile::AloreV2
                                    : m24e2b::GainProfile::VendorDefault);
        sdk->start(); // PHYSICAL I/O AND MODE SWITCH: runtime operator gate checked first.
        latest=sdk->initialFeedback();
        const auto now=Clock::now();
        planner.reset(latest,now); // Reject missing/error/nonfinite/invalid real feedback.
        // Planner HOLD and TRACK share one fixed command reference.
        // LOWCMD still drives motors even with no keyboard TARGET.
        std::cerr<<"[Z1] SDK HOLD anchored at initial commanded joints; "
                    "HOLD->TRACK position setpoint cannot jump.\n";
        q=latest.q; dq=latest.dq;
        goal_q=latest.q;
        last_step=Clock::now(); // SDK mode transitions may take seconds.
#else
        throw std::runtime_error("SDK backend not compiled");
#endif
    }
    m24e2b::Feedback mock_feedback(Clock::time_point now) const {
        m24e2b::Feedback f{};
        f.q=q; f.dq=dq;
        f.sdk_connected=true; f.lowcmd_reported=true;
        f.read_at=now;
        f.temperature.fill(30);
        // These booleans describe ONLY a synthetic mock. Never use as hardware proof.
        return f;
    }
    void record_trace(const m24e2b::Command& cmd,const m24e2b::Feedback& fb,
                      Clock::time_point now,double dt,double sdk_io_ms) noexcept {
        if(!trace) return;
        TraceRow r{};
        r.time_ns=mono_ns(now);
        r.target_seq=last_seq;
        r.dt_ms=dt*1000.;
        r.io_ms=sdk_io_ms;
        r.target_age_ms=last_target==Clock::time_point{} ? -1. :
            std::chrono::duration<double,std::milli>(now-last_target).count();
        r.track=cmd.track;
        r.goal=goal_q;
        r.q_cmd=cmd.q;
        r.dq_cmd=cmd.dq;
        r.q_meas=fb.q;
        r.dq_meas=fb.dq;
        r.sent_ff=fb.sent_feedforward_tau;
        r.tau_meas=fb.tau;
        trace->add(r);
    }
    bool stop_sdk() noexcept {
#ifdef M24E2D_ENABLE_HARDWARE_SDK
        if(sdk_mode && sdk) {
            const bool confirmed=sdk->stop();
            std::cerr << "[M2.4e2j] SDK PASSIVE "
                      << (confirmed?"confirmed":"NOT CONFIRMED; operator must use physical E-stop/support")
                      << '\n';
            return confirmed;
        }
#endif
        return !sdk_mode;
    }
    void step() {
        const auto now=Clock::now();
        double dt=std::chrono::duration<double>(now-last_step).count();
        last_step=now;
        if(fault_latched){enabled=false;dq.fill(0.);return;}
        try {
            if(sdk_mode){
#ifdef M24E2D_ENABLE_HARDWARE_SDK
                // NEVER use mock state to arm planner or initialize physical joint targets.
                // The SDK is TX+RX. This path does not imply validated UDP RX provenance.
                const auto command=planner.tick(latest,now,dt);
                const auto sdk_start=Clock::now();
                latest=sdk->exchange(command); // one caller/thread; no concurrent vendor RX thread
                const auto sdk_end=Clock::now();
                record_trace(command,latest,now,dt,
                             std::chrono::duration<double,std::milli>(sdk_end-sdk_start).count());
                q=latest.q;dq=latest.dq;enabled=command.track;
#else
                throw std::runtime_error("SDK backend not compiled");
#endif
            }else{
                dt=std::clamp(dt,.0001,.019); // Existing mock semantics unchanged.
                const auto command=planner.tick(mock_feedback(now),now,dt);
                q=command.q; dq=command.dq; enabled=command.track;
                record_trace(command,mock_feedback(Clock::now()),now,dt,0.);
            }
        } catch(const std::exception& e) {
            fault_latched=true;enabled=false;dq.fill(0.);planner.disarm();
            fault_reason=e.what();
            std::cerr<<"M2.4e2d "<<(sdk_mode?"SDK":"MOCK")<<" FAULT "<<mono_ns(now)<<' '<<fault_reason<<'\n';
            // In SDK mode, software fault alone is NOT a proven physical safe stop.
            // Real motion requires independent operator E-stop and bench validation.
        }
    }
    std::string handle(const std::string& packet){
        std::istringstream in(packet);
        std::string version, verb;
        if(!(in>>version>>verb) || version!="V1") return "V1 ERR 0 BAD_VERSION";
        if(verb=="HELLO"){
            require_end(in);
            return sdk_mode ? "V1 HELLO_ACK SDK_MODE_UNQUALIFIED" :
                "V1 HELLO_ACK MOCK_ONLY NO_SDK_NO_DDS_NO_MOTOR_IO";
        }
        if(verb=="REF"){
            // Additive read-only IPC: report the exact Planner output reference
            // (not the drifting encoder measurement). No SDK I/O or motor effect.
            std::uint64_t req=0;
            if(!(in>>req)||req==0)return "V1 ERR 0 INVALID_REQUEST";
            try{require_end(in);}catch(...){return "V1 ERR 0 EXTRA_FIELDS";}
            if(sdk_mode && fault_latched)return "V1 ERR 0 SDK_FAULT_LATCHED";
            std::ostringstream out;
            out<<std::setprecision(17)<<"V1 REF "<<req<<' '<<mono_ns()
               <<' '<<(planner.armed()?"TRACK":"HOLD");
            for(const double v:planner.commandPosition())out<<' '<<v;
            return out.str();
        }
        if(verb=="TARGET"){
            std::uint64_t seq=0, stamp=0;
            int enable_int=-1;
            Vec6 next{};
            if(!(in>>seq>>stamp>>enable_int))return "V1 ERR 0 INVALID_TARGET_HEADER";
            for(auto& v:next)if(!(in>>v))return "V1 ERR 0 INVALID_TARGET_VALUES";
            try{require_end(in);}catch(...){return "V1 ERR 0 EXTRA_FIELDS";}
            if(!fault_latched && (seq==0||seq<=last_seq||stamp==0||stamp<last_stamp)){
                fault_latched=true;enabled=false;planner.disarm();
                fault_reason="NONMONOTONIC_SEQUENCE";
                return "V1 ERR 0 NONMONOTONIC_SEQUENCE";
            }
            if(enable_int!=0 && enable_int!=1) return "V1 ERR 0 INVALID_ENABLE";
            if(enable_int==1 && !planner.armed()){
                // Fail closed if HOLD started just outside a strict target
                // boundary, e.g. J3=+0.00064 with J3 upper limit 0.
                // Never silently move/clamp an arm toward its hard stop on ARM.
                const auto hold_ref=planner.commandPosition();
                for(int i=0;i<6;++i)
                    if(!planner.targetAllowed(i,hold_ref[i]))
                        return "V1 ERR 0 HOLD_REFERENCE_OUT_OF_LIMIT_J"+std::to_string(i+1);
            }
            // Match Planner validation exactly; the only J3 exception is
            // reproducing the measured startup command within +0.0015 rad.
            // No newly requested travel outside the original bounds.
            for(int i=0;i<6;++i)if(!planner.targetAllowed(i,next[i])){
                enabled=false;fault_latched=true;planner.disarm();
                fault_reason="TARGET_OUT_OF_LIMIT";
                return "V1 ERR 0 TARGET_OUT_OF_LIMIT";
            }
            const auto now = Clock::now();
            const auto now_ns = mono_ns(now);
            // Sender must use CLOCK_MONOTONIC on this same PC; reject old/future goals.
            if(stamp>now_ns+100000000ULL||now_ns>stamp+500000000ULL){
                enabled=false;fault_latched=true;planner.disarm();
                fault_reason="TARGET_TIMESTAMP_STALE";
                return "V1 ERR 0 TARGET_TIMESTAMP_STALE";
            }
            if(fault_latched)return "V1 ERR 0 FAULT_LATCHED";
            const m24e2b::Target t{
                seq, m24e2b::Time(std::chrono::nanoseconds(stamp)), next, bool(enable_int)};
            try { planner.acceptTarget(t,now); }
            catch(const std::exception& e){
                enabled=false;fault_latched=true;fault_reason=e.what();
                std::cerr<<"M2.4e2d TARGET REJECT "<<mono_ns(now)<<' '<<fault_reason<<'\n';
                return "V1 ERR 0 PLANNER_REJECTED";
            }
            last_seq=seq;
            goal_q=next; // accepted target, for trace only.
            last_stamp=stamp;
            last_target=now;
            enabled=bool(enable_int);
            return "V1 ACK "+std::to_string(seq)+" "+(enabled?"ACCEPTED":"DISARMED");
        }
        if(verb=="DIAG"){
            require_end(in);
            std::string code=fault_reason;
            for(char& c:code) if(c==' ')c='_';
            return "V1 DIAG " + std::string(sdk_mode?"SDK_UNQUALIFIED":"MOCK_ONLY") +
                " " + std::string(fault_latched?"FAULT":"OK") +
                " " + (code.empty()?"NONE":code);
        }
        if(verb=="SENSE"){
            // Additive bench-only telemetry; V1 GET layout remains unchanged.
            if(!sdk_mode || fault_latched) return "V1 ERR 0 SDK_SENSE_UNAVAILABLE";
            std::uint64_t req=0;
            if(!(in>>req)||req==0) return "V1 ERR 0 INVALID_REQUEST";
            try{require_end(in);}catch(...){return "V1 ERR 0 EXTRA_FIELDS";}
#ifdef M24E2D_ENABLE_HARDWARE_SDK
            std::ostringstream out;
            out<<std::setprecision(8)<<"V1 SENSE "<<req<<' '<<mono_ns();
            for(double v:latest.tau) out<<' '<<v;
            for(int v:latest.temperature) out<<' '<<v;
            return out.str();
#else
            return "V1 ERR 0 SDK_NOT_COMPILED";
#endif
        }
        if(verb=="GET"){
            if(sdk_mode && fault_latched) return "V1 ERR 0 SDK_FAULT_LATCHED";
            std::uint64_t req=0;
            if(!(in>>req)||req==0)return "V1 ERR 0 INVALID_REQUEST";
            try{require_end(in);}catch(...){return "V1 ERR 0 EXTRA_FIELDS";}
            const auto now=Clock::now();
            auto age=last_target==Clock::time_point{}?-1LL:
                static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(now-last_target).count());
            std::ostringstream out;
            out<<std::setprecision(10)<<"V1 STATE "<<(sdk_mode?"SDK":"MOCK")<<" "<<req<<' '<<mono_ns(now)<<' '
               <<last_seq<<' '<<age<<' '<<(enabled?"TRACK":"HOLD");
            for(double v:q)out<<' '<<v;
            for(double v:dq)out<<' '<<v;
            return out.str();
        }
        return "V1 ERR 0 UNKNOWN_VERB";
    }
};
void session(int fd,bool sdk_mode,bool has_gripper,bool alore_gain,const std::string& trace_csv){
    struct ucred cred{}; socklen_t size=sizeof(cred);
    if(::getsockopt(fd,SOL_SOCKET,SO_PEERCRED,&cred,&size)!=0 || cred.uid!=::geteuid()){
        send_packet(fd,"V1 ERR 0 FOREIGN_UID");return;
    }
    State state{sdk_mode,has_gripper,alore_gain,trace_csv};
    char buf[kPacketMax];
    while(!stopping){
        state.step();
        // Do not continue the communication loop after a latched SDK fault.
        // A software latch is not a physical stop: attempt transition once.
        if(sdk_mode && state.fault_latched) break;
        pollfd p{fd,POLLIN,0};
        int ready=::poll(&p,1,sdk_mode?2:5);
        if(ready<0){if(errno==EINTR)continue;break;}
        if(ready==0)continue;
        if(p.revents&(POLLERR|POLLHUP|POLLNVAL))break;
        if(!(p.revents&POLLIN))continue;
        const ssize_t n=::recv(fd,buf,sizeof(buf),0);
        if(n<=0)break;
        if(n>=static_cast<ssize_t>(sizeof(buf))){send_packet(fd,"V1 ERR 0 OVERSIZED_PACKET");break;}
        std::string reply;
        try { reply=state.handle(std::string(buf,static_cast<std::size_t>(n))); }
        catch (...) { reply="V1 ERR 0 MALFORMED_PACKET"; }
        try{send_packet(fd,reply);}catch(...){break;}
    }
    // Covers Ctrl+C, peer close, IPC send failure, TARGET watchdog errors and
    // normal end-of-session. Actual hardware safety still needs operator oversight.
    const bool safe_to_write=state.stop_sdk();
    if(state.trace && safe_to_write)
        state.trace->save(state.trace_csv_path); // no disk I/O while motors may be active
    else if(state.trace && !safe_to_write)
        std::cerr<<"[Z1 TRACE] CSV skipped: SDK PASSIVE NOT CONFIRMED\n";
}
} // namespace
int main(int argc,char** argv){
    if(argc==2 && std::string(argv[1])=="--help"){
        std::cout<<"M2.4e2d: --socket /absolute/path [--backend mock]\n"
                 <<"SDK requires build opt-in, --backend sdk, --has-gripper yes|no, bench-ack.\n"
                 <<"Gain profiles: --gain-profile vendor (default) or alore-v2 (EXPERIMENTAL).\n"
                 <<"Optional: --trace-csv /absolute/new.csv (dump AFTER control session).\n";
        return 0;
    }
    bool sdk_mode=false;
    bool ack=false;
    bool has_gripper=false,gripper_specified=false;
    bool alore_gain=false,profile_specified=false;
    std::string path;
    std::string trace_csv;
    for(int i=1;i<argc;++i){
        const std::string arg=argv[i];
        if(arg=="--socket" && i+1<argc){path=argv[++i];}
        else if(arg=="--backend" && i+1<argc){
            const std::string backend=argv[++i];
            if(backend=="sdk")sdk_mode=true;
            else if(backend!="mock"){std::cerr<<"Invalid backend\n";return 2;}
        }else if(arg=="--has-gripper" && i+1<argc){
            if(gripper_specified){std::cerr<<"Duplicate --has-gripper\n";return 2;}
            const std::string choice=argv[++i];
            if(choice=="yes")has_gripper=true;
            else if(choice=="no")has_gripper=false;
            else {std::cerr<<"--has-gripper must be yes or no\n";return 2;}
            gripper_specified=true;
        }else if(arg=="--gain-profile" && i+1<argc){
            if(profile_specified){std::cerr<<"Duplicate --gain-profile\n";return 2;}
            const std::string p=argv[++i];
            if(p=="vendor")alore_gain=false;
            else if(p=="alore-v2")alore_gain=true;
            else {std::cerr<<"--gain-profile must be vendor or alore-v2\n";return 2;}
            profile_specified=true;
        }else if(arg=="--trace-csv" && i+1<argc){trace_csv=argv[++i];}
        else if(arg=="--bench-ack" && i+1<argc){ack=std::string(argv[++i])==kPhysicalAck;}
        else {std::cerr<<"Unknown/missing argument: "<<arg<<'\n';return 2;}
    }
    if(sdk_mode){
#ifndef M24E2D_ENABLE_HARDWARE_SDK
        std::cerr<<"M2.4e2d sdk backend NOT wired/authorized. No SDK load or physical I/O.\n";
        return 3;
#else
        if(!ack){std::cerr<<"SDK bench-ack required BEFORE any SDK load/I/O\n";return 3;}
        if(!gripper_specified){std::cerr<<"SDK needs explicit --has-gripper yes|no BEFORE any I/O\n";return 3;}
        std::cerr<<"WARNING: SDK mode has real motor effects; stop semantics NOT bench qualified.\n";
        std::cerr<<"Gain profile: "<<(alore_gain?"ALORE V2 EXPERIMENTAL":"vendor default")
                 <<"; load support and physical E-stop required.\n";
#endif
    }else if(ack || gripper_specified || profile_specified){
        std::cerr<<"Invalid mock options\n";return 2;
    }
    if(path.empty()) {std::cerr<<"usage: --socket /absolute/path [--backend mock]\n";return 2;}
    if(path.empty()||path[0]!='/'||path.size()>=sizeof(sockaddr_un::sun_path)){
        std::cerr<<"Socket path must be short and absolute\n";return 2;
    }
    if(!trace_csv.empty() && (trace_csv[0]!='/' || trace_csv==path)){
        std::cerr<<"trace-csv must be an absolute path different from socket\n";return 2;
    }
    ::signal(SIGINT,on_signal);::signal(SIGTERM,on_signal);
    ::umask(0077);
    ScopedFd listener(::socket(AF_UNIX,SOCK_SEQPACKET|SOCK_CLOEXEC,0));
    if(listener.fd<0){std::perror("socket");return 2;}
    sockaddr_un addr{};addr.sun_family=AF_UNIX;
    std::strncpy(addr.sun_path,path.c_str(),sizeof(addr.sun_path)-1);
    // Never unlink an existing path (socket, file or symlink).
    struct stat existing{};
    if(::lstat(path.c_str(),&existing)==0 || errno!=ENOENT){
        std::cerr<<"Socket path exists or cannot be checked; refusing: "<<path<<'\n';return 2;
    }
    if(::bind(listener.fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))!=0){std::perror("bind");return 2;}
    if(::listen(listener.fd,1)!=0){std::perror("listen");::unlink(path.c_str());return 2;}
    std::cout<<"M2.4e2d worker listening: "<<path<<std::endl;
    while(!stopping){
        pollfd p{listener.fd,POLLIN,0};
        int ready=::poll(&p,1,100);
        if(ready<0){if(errno==EINTR)continue;break;}
        if(ready==0)continue;
        if(!(p.revents&POLLIN))continue;
        ScopedFd client(::accept4(listener.fd,nullptr,nullptr,SOCK_CLOEXEC));
        if(client.fd<0){if(errno==EINTR)continue;break;}
        try {session(client.fd,sdk_mode,has_gripper,alore_gain,trace_csv);}
        catch(const std::exception& e){
            std::cerr<<"Worker session failed: "<<e.what()<<'\n';
            if(sdk_mode){stopping=1;break;} // NEVER restart live SDK after startup failure
        }
        if(sdk_mode){
            // One attended hardware session per Worker process.
            // Prevent another IPC client from silently reinitializing LOWCMD.
            stopping=1;
            break;
        }
    }
    // Only unlink our socket if it still points to the socket inode we created.
    struct stat st{};
    if(::lstat(path.c_str(),&st)==0 && S_ISSOCK(st.st_mode) && st.st_uid==::geteuid()){
        ::unlink(path.c_str());
    }
    return 0;
}
