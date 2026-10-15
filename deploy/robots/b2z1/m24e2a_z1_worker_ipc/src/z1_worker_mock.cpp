// M2.4e2a: strictly offline standalone Z1 mock. No SDK, DDS or motor IO.
// Linux AF_UNIX/SOCK_SEQPACKET messages are delimited, text V1 messages.
// One client session at a time; local UID verified with SO_PEERCRED.
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/types.h>
#include <poll.h>
#include <signal.h>
#include <unistd.h>

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
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>

namespace {
using Clock = std::chrono::steady_clock;
using Vec6 = std::array<double, 6>;
constexpr std::size_t kPacketMax = 512;
constexpr auto kGoalTtl = std::chrono::milliseconds(150); // MOCK demo only
constexpr double kMockSpeed = 0.50; // mock radians/second, NOT real Z1 limit
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
struct State {
    Vec6 q{0., 1.48, -.63, -.84, 0., 1.57};
    Vec6 dq{};
    Vec6 goal{q};
    bool enabled{false};
    std::uint64_t last_seq{0};
    std::uint64_t last_stamp{0};
    Clock::time_point last_target{};
    Clock::time_point last_step{Clock::now()};
    const Vec6 lo{-2.618, 0., -2.8798, -1.5184, -1.3439, -2.7925};
    const Vec6 hi{2.618, 2.9671, 0., 1.5184, 1.3439, 2.7925};
    void step() {
        auto now = Clock::now();
        double dt = std::clamp(std::chrono::duration<double>(now - last_step).count(), 0., .05);
        last_step = now;
        if(enabled && now-last_target>kGoalTtl){ enabled=false; goal=q; }
        for(int i=0;i<6;++i){
            double diff = (enabled ? goal[i] : q[i])-q[i];
            double move=std::clamp(diff,-kMockSpeed*dt,kMockSpeed*dt);
            q[i]+=move;
            dq[i]=dt>1.e-9?move/dt:0.;
        }
    }
    std::string handle(const std::string& packet){
        std::istringstream in(packet);
        std::string version, verb;
        if(!(in>>version>>verb) || version!="V1") return "V1 ERR 0 BAD_VERSION";
        if(verb=="HELLO"){
            require_end(in);
            return "V1 HELLO_ACK MOCK_ONLY NO_SDK_NO_DDS_NO_MOTOR_IO";
        }
        if(verb=="TARGET"){
            std::uint64_t seq=0, stamp=0;
            int enable_int=-1;
            Vec6 next{};
            if(!(in>>seq>>stamp>>enable_int))return "V1 ERR 0 INVALID_TARGET_HEADER";
            for(auto& v:next)if(!(in>>v))return "V1 ERR 0 INVALID_TARGET_VALUES";
            try{require_end(in);}catch(...){return "V1 ERR 0 EXTRA_FIELDS";}
            if(seq==0||seq<=last_seq||stamp==0||stamp<last_stamp) return "V1 ERR 0 NONMONOTONIC_SEQUENCE";
            if(enable_int!=0 && enable_int!=1) return "V1 ERR 0 INVALID_ENABLE";
            for(int i=0;i<6;++i)if(!std::isfinite(next[i])||next[i]<lo[i]||next[i]>hi[i]){
                enabled=false;goal=q;
                return "V1 ERR 0 TARGET_OUT_OF_LIMIT";
            }
            const auto now = Clock::now();
            const auto now_ns = mono_ns(now);
            // Sender must use CLOCK_MONOTONIC on this same PC; reject old/future goals.
            if(stamp>now_ns+100000000ULL||now_ns>stamp+500000000ULL){
                enabled=false;goal=q;
                return "V1 ERR 0 TARGET_TIMESTAMP_STALE";
            }
            last_seq=seq;
            last_stamp=stamp;
            last_target=now;
            enabled=bool(enable_int);
            goal=enabled?next:q;
            return "V1 ACK "+std::to_string(seq)+" "+(enabled?"ACCEPTED":"DISARMED");
        }
        if(verb=="GET"){
            std::uint64_t req=0;
            if(!(in>>req)||req==0)return "V1 ERR 0 INVALID_REQUEST";
            try{require_end(in);}catch(...){return "V1 ERR 0 EXTRA_FIELDS";}
            const auto now=Clock::now();
            auto age=last_target==Clock::time_point{}?-1LL:
                static_cast<long long>(std::chrono::duration_cast<std::chrono::milliseconds>(now-last_target).count());
            std::ostringstream out;
            out<<std::setprecision(10)<<"V1 STATE MOCK "<<req<<' '<<mono_ns(now)<<' '
               <<last_seq<<' '<<age<<' '<<(enabled?"TRACK":"HOLD");
            for(double v:q)out<<' '<<v;
            for(double v:dq)out<<' '<<v;
            return out.str();
        }
        return "V1 ERR 0 UNKNOWN_VERB";
    }
};
void session(int fd){
    struct ucred cred{}; socklen_t size=sizeof(cred);
    if(::getsockopt(fd,SOL_SOCKET,SO_PEERCRED,&cred,&size)!=0 || cred.uid!=::geteuid()){
        send_packet(fd,"V1 ERR 0 FOREIGN_UID");return;
    }
    State state{};
    char buf[kPacketMax];
    while(!stopping){
        state.step();
        pollfd p{fd,POLLIN,0};
        int ready=::poll(&p,1,20);
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
    // End of session: Mock only, no physical stop behavior is implied.
}
} // namespace
int main(int argc,char** argv){
    if(argc==2 && std::string(argv[1])=="--help"){
        std::cout<<"MOCK ONLY: m24e2a_z1_mock_worker --socket /absolute/path/to/socket\n";
        return 0;
    }
    if(argc!=3||std::string(argv[1])!="--socket"){
        std::cerr<<"usage: m24e2a_z1_mock_worker --socket /absolute/path\n";return 2;
    }
    std::string path=argv[2];
    if(path.empty()||path[0]!='/'||path.size()>=sizeof(sockaddr_un::sun_path)){
        std::cerr<<"Socket path must be short and absolute\n";return 2;
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
    std::cout<<"M2.4e2a MOCK_ONLY listening: "<<path<<std::endl;
    while(!stopping){
        pollfd p{listener.fd,POLLIN,0};
        int ready=::poll(&p,1,100);
        if(ready<0){if(errno==EINTR)continue;break;}
        if(ready==0)continue;
        if(!(p.revents&POLLIN))continue;
        ScopedFd client(::accept4(listener.fd,nullptr,nullptr,SOCK_CLOEXEC));
        if(client.fd<0){if(errno==EINTR)continue;break;}
        session(client.fd);
    }
    // Only unlink our socket if it still points to the socket inode we created.
    struct stat st{};
    if(::lstat(path.c_str(),&st)==0 && S_ISSOCK(st.st_mode) && st.st_uid==::geteuid()){
        ::unlink(path.c_str());
    }
    return 0;
}
