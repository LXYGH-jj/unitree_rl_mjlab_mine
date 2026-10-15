#include "z1_ipc_client.h"
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace m24e2e {
namespace {
std::uint64_t steadyNs(Clock::time_point t=Clock::now()) {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count());
}
std::runtime_error err(const std::string& what){return std::runtime_error(what+": "+std::strerror(errno));}
void onlyEnd(std::istringstream& is) { std::string more; if(is>>more)throw std::runtime_error("IPC trailing fields"); }
std::string realNumber(std::istringstream& in) {
    std::string s;
    if(!(in>>s)) throw std::runtime_error("IPC missing number");
    std::size_t end=0;
    double d=0;
    try {d=std::stod(s,&end);}catch(...){throw std::runtime_error("IPC invalid number");}
    if(end!=s.size()||!std::isfinite(d))throw std::runtime_error("IPC nonfinite/malformed number");
    return s;
}
double number(std::istringstream& in){return std::stod(realNumber(in));}
}
Session::~Session(){close();}
void Session::close() noexcept {if(fd_>=0){::close(fd_);fd_=-1;}}
void Session::connectTo(const std::string& path, Mode expected, int timeout_ms){
    close();
    if(path.empty()||path[0]!='/'||path.size()>=sizeof(sockaddr_un::sun_path)||timeout_ms<5||timeout_ms>200)
        throw std::invalid_argument("invalid socket path/timeout");
    fd_=::socket(AF_UNIX,SOCK_SEQPACKET|SOCK_CLOEXEC,0);
    if(fd_<0)throw err("socket");
    try {
        timeval tv{timeout_ms/1000,(timeout_ms%1000)*1000};
        if(::setsockopt(fd_,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv))!=0 ||
           ::setsockopt(fd_,SOL_SOCKET,SO_SNDTIMEO,&tv,sizeof(tv))!=0) throw err("setsockopt");
        sockaddr_un address{};address.sun_family=AF_UNIX;
        std::memcpy(address.sun_path,path.c_str(),path.size()+1);
        if(::connect(fd_,reinterpret_cast<sockaddr*>(&address),sizeof(address))!=0)throw err("connect");
        std::string reply=transact("V1 HELLO");
        if(expected==Mode::Mock){
            if(reply!="V1 HELLO_ACK MOCK_ONLY NO_SDK_NO_DDS_NO_MOTOR_IO")
                throw std::runtime_error("unexpected worker mode (expected Mock)");
        }else{
            if(reply!="V1 HELLO_ACK SDK_MODE_UNQUALIFIED")
                throw std::runtime_error("unexpected worker mode (expected SDK)");
        }
        mode_=expected;
    }catch(...){close();throw;}
}
std::string Session::transact(const std::string& msg){
    if(fd_<0)throw std::runtime_error("IPC not connected");
    if(msg.size()>=512)throw std::runtime_error("IPC outgoing packet too large");
    ssize_t sent=::send(fd_,msg.data(),msg.size(),MSG_NOSIGNAL);
    if(sent!=static_cast<ssize_t>(msg.size()))throw err("IPC send");
    char buf[512];
    ssize_t r=::recv(fd_,buf,sizeof(buf),0);
    if(r<=0)throw err("IPC recv/disconnected");
    if(r>=static_cast<ssize_t>(sizeof(buf)))throw std::runtime_error("IPC oversized reply");
    return std::string(buf,static_cast<std::size_t>(r));
}
ArmState Session::get(std::uint64_t req){
    if(req==0)throw std::invalid_argument("request id must be nonzero");
    std::string reply=transact("V1 GET "+std::to_string(req));
    const auto received_at=Clock::now();
    std::istringstream in(reply);
    std::string ver,verb,mode,flag;
    ArmState s{};
    std::uint64_t actual_req=0;
    if(!(in>>ver>>verb>>mode>>actual_req>>s.response_ns>>s.last_target_seq>>s.target_age_ms>>flag))
        throw std::runtime_error("invalid STATE reply");
    if(ver!="V1"||verb!="STATE"||actual_req!=req)
        throw std::runtime_error("IPC response/request mismatch");
    if(mode!=(mode_==Mode::Mock?"MOCK":"SDK"))throw std::runtime_error("IPC mode mismatch");
    if(flag!="TRACK" && flag!="HOLD")throw std::runtime_error("IPC track field invalid");
    s.mode=mode_;
    s.tracking=(flag=="TRACK");
    for(auto& x:s.q)x=number(in);
    for(auto& x:s.dq)x=number(in);
    onlyEnd(in);
    const auto now_ns=steadyNs(received_at);
    // This only limits IPC response age, NOT vendor packet age.
    if(s.response_ns==0 || s.response_ns>now_ns+10000000ULL ||
       now_ns>s.response_ns+100000000ULL)throw std::runtime_error("IPC response timestamp invalid/stale");
    s.ipc_received_at=received_at;
    return s;
}
void Session::target(std::uint64_t seq, const Vec6& q, bool enable){
    if(mode_!=Mode::Mock)throw std::runtime_error("SDK TARGET disabled pending supervised bench qualification");
    if(seq==0)throw std::invalid_argument("sequence id invalid");
    std::ostringstream out;
    out<<std::setprecision(17)<<"V1 TARGET "<<seq<<' '<<steadyNs()<<' '<<(enable?1:0);
    for(double x:q){if(!std::isfinite(x))throw std::invalid_argument("nonfinite target");out<<' '<<x;}
    const auto reply=transact(out.str());
    if(reply!="V1 ACK "+std::to_string(seq)+(enable?" ACCEPTED":" DISARMED"))
        throw std::runtime_error("IPC target rejected: "+reply);
}
ReadOnlyPoller::~ReadOnlyPoller(){stop();}
void ReadOnlyPoller::start(const std::string& path,Mode expected,int timeout_ms){
    stop();
    {std::lock_guard<std::mutex> lk(mu_);available_=false;error_="CONNECTING";}
    stop_=false;
    thread_=std::thread(&ReadOnlyPoller::loop,this,path,expected,timeout_ms);
}
void ReadOnlyPoller::stop() noexcept{
    stop_=true;
    if(thread_.joinable())thread_.join();
    std::lock_guard<std::mutex> lk(mu_);
    available_=false;
    error_="STOPPED";
}
bool ReadOnlyPoller::latest(ArmState& out,std::string& error)const{
    std::lock_guard<std::mutex> lk(mu_);
    out=latest_;error=error_;return available_;
}
void ReadOnlyPoller::loop(std::string path,Mode expected,int timeout_ms)noexcept{
    try{
        Session s; s.connectTo(path,expected,timeout_ms);
        std::uint64_t req=1;
        while(!stop_){
            const auto next=Clock::now()+std::chrono::milliseconds(20);
            const auto frame=s.get(req++);
            {std::lock_guard<std::mutex> lk(mu_);latest_=frame;available_=true;error_="NONE";}
            std::this_thread::sleep_until(next);
        }
    }catch(const std::exception& ex){
        std::lock_guard<std::mutex> lk(mu_);
        available_=false;error_=ex.what();
    }
}
}
