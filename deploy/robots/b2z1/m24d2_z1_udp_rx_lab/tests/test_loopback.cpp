#include "z1_loopback_rx_lab.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace b2z1::m24d2;
using namespace b2z1::m24d1;
using namespace b2z1::z1_jointctrl;
using namespace std::chrono_literals;

static void check(bool condition, const char* description) {
    if(!condition) throw std::runtime_error(description);
}
template <class F> static void rejects(F&& f,const char* description) {
    bool refused=false;
    try { f(); } catch(const std::exception&) { refused=true; }
    check(refused,description);
}
class Sender final {
public:
    Sender() {
        fd_=::socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC,0);
        if(fd_<0) throw std::runtime_error("test sender socket");
        sockaddr_in addr{};
        addr.sin_family=AF_INET;addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        addr.sin_port=htons(0);
        if(::bind(fd_,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))!=0)
            throw std::runtime_error("test sender bind");
        socklen_t n=sizeof(addr);
        if(::getsockname(fd_,reinterpret_cast<sockaddr*>(&addr),&n)!=0)
            throw std::runtime_error("test sender port");
        port_=ntohs(addr.sin_port);
    }
    ~Sender() { if(fd_>=0) ::close(fd_); }
    Sender(const Sender&)=delete;
    Sender& operator=(const Sender&)=delete;
    std::uint16_t port() const {return port_;}
    template<std::size_t N> void send(std::uint16_t destination,const std::array<std::uint8_t,N>& packet) const {
        sendRaw(destination,packet.data(),packet.size());
    }
    void sendRaw(std::uint16_t destination,const std::uint8_t* data,std::size_t n) const {
        sockaddr_in addr{};addr.sin_family=AF_INET;
        addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        addr.sin_port=htons(destination);
        const auto got=::sendto(fd_,data,n,0,reinterpret_cast<sockaddr*>(&addr),sizeof(addr));
        if(got!=static_cast<ssize_t>(n)) throw std::runtime_error("test sendto failed");
    }
private:
    int fd_=-1;
    std::uint16_t port_=0;
};

static void put16(std::uint8_t*& out,std::uint16_t x) {
    *out++=static_cast<std::uint8_t>(x>>8);*out++=static_cast<std::uint8_t>(x);
}
static void put32(std::uint8_t*& out,std::uint32_t x) {
    for(int i=3;i>=0;--i) *out++=static_cast<std::uint8_t>(x>>(i*8));
}
static void put64(std::uint8_t*& out,std::uint64_t x) {
    for(int i=7;i>=0;--i) *out++=static_cast<std::uint8_t>(x>>(i*8));
}
static void putDouble(std::uint8_t*& out,double value) {
    std::uint64_t bits=0;std::memcpy(&bits,&value,sizeof(bits));put64(out,bits);
}
struct FrameOptions {
    std::uint16_t flags=kMockConnected|kMockJointCtrl;
    int temperature=25;
    std::uint8_t motor_error=0;
    std::uint8_t motor_connection=0;
    double q0=0.0;
    bool corrupt_crc=false;
};
static std::array<std::uint8_t,kMockWireBytes> makeFrame(std::uint64_t sequence,FrameOptions x={}) {
    std::array<std::uint8_t,kMockWireBytes> buf{};
    auto* w=buf.data();
    put32(w,kMockMagic);put16(w,kMockVersion);put16(w,x.flags);put64(w,sequence);
    for(int j=0;j<6;++j) {
        putDouble(w,j==0?x.q0:0.0);putDouble(w,0.0);putDouble(w,0.0);
        put16(w,static_cast<std::uint16_t>(x.temperature));
        *w++=(j==0?x.motor_error:0);*w++=(j==0?x.motor_connection:0);
    }
    check(w==buf.data()+kMockWireBytes-4,"bad test wire length");
    put32(w,crc32(buf.data(),kMockWireBytes-4));
    if(x.corrupt_crc) buf[kMockWireBytes-1]^=0x80;
    return buf;
}

static Config safety() {
    Config c{};c.q_min.fill(-2.0);c.q_max.fill(2.0);
    c.max_velocity_rad_s.fill(.5);c.max_tracking_error_rad=.2;
    c.max_feedback_age=100ms;c.max_command_interval=100ms;
    c.min_consecutive_new_packets=3;
    return c;
}
static Limits limits() {
    Limits l{};l.q_min.fill(-2.0);l.q_max.fill(2.0);
    l.max_motor_temperature_c.fill(65);
    l.max_receive_age=100ms;l.max_decode_lag=20ms;
    return l;
}
static void testSuccess() {
    Sender sender;
    LoopbackLab lab(safety(),limits(),sender.port());
    check(lab.mockCommands().empty(),"unexpected mock output before auth");
    for(std::uint64_t seq=1;seq<=3;++seq) {
        const auto pkt=makeFrame(seq);
        sender.send(lab.listenerPort(),pkt);
        lab.receiveOne(100);
    }
    auto now=Clock::now();
    lab.authorizeForMockTest(now); // ALL flags simulated; NOT production authorization
    Joint6 ik{};ik[0]=.06;
    lab.setMockGoal(ik,now);
    check(lab.mockTick(now+5ms),"expected mock command emission");
    check(lab.mockCommands().size()==1,"exactly one mock output expected");
    check(std::fabs(lab.mockCommands().at(0).q[0]-.0025)<1e-8,"mock bounded step incorrect");
    check(std::fabs(lab.mockCommands().at(0).dq[0]-.5)<1e-8,"mock bounded velocity incorrect");
    check(!lab.faulted(),"success case unexpectedly faulted");
}
static void testInsufficientPackets() {
    Sender sender;LoopbackLab lab(safety(),limits(),sender.port());
    for(std::uint64_t seq=1;seq<=2;++seq) {
        sender.send(lab.listenerPort(),makeFrame(seq));lab.receiveOne(100);
    }
    rejects([&]{lab.authorizeForMockTest(Clock::now());},"must refuse <3 valid frames");
    check(lab.mockCommands().empty(),"insufficient RX unexpectedly sent command");
}
static void testReplay() {
    Sender sender;LoopbackLab lab(safety(),limits(),sender.port());
    auto pkt=makeFrame(7);
    sender.send(lab.listenerPort(),pkt);lab.receiveOne(100);
    sender.send(lab.listenerPort(),pkt);
    rejects([&]{lab.receiveOne(100);},"duplicate seq must fail");
    check(lab.faulted(),"replay must latch");
    check(!lab.mockTick(Clock::now()),"faulted replay must never send");
}
static void testUnexpectedPeer() {
    Sender expected;Sender attacker;
    LoopbackLab lab(safety(),limits(),expected.port());
    attacker.send(lab.listenerPort(),makeFrame(1));
    rejects([&]{lab.receiveOne(100);},"wrong source port must fault");
    check(lab.faulted()&&lab.mockCommands().empty(),"source mismatch did not latch");
}
static void testCRC() {
    Sender sender;LoopbackLab lab(safety(),limits(),sender.port());
    FrameOptions opt{};opt.corrupt_crc=true;
    sender.send(lab.listenerPort(),makeFrame(1,opt));
    rejects([&]{lab.receiveOne(100);},"bad mock CRC must fault");
    check(lab.faulted(),"CRC fault must latch");
}
static void testShortAndLong() {
    Sender sender;
    {
        LoopbackLab lab(safety(),limits(),sender.port());
        auto pkt=makeFrame(1);
        sender.sendRaw(lab.listenerPort(),pkt.data(),pkt.size()-1);
        rejects([&]{lab.receiveOne(100);},"short mock datagram must fault");
    }
    {
        LoopbackLab lab(safety(),limits(),sender.port());
        auto pkt=makeFrame(1);
        std::array<std::uint8_t,kMockWireBytes+1> too_long{};
        std::memcpy(too_long.data(),pkt.data(),pkt.size());
        sender.send(lab.listenerPort(),too_long);
        rejects([&]{lab.receiveOne(100);},"truncated oversized mock datagram must fault");
    }
}
static void testHealth() {
    Sender sender;
    {
        LoopbackLab lab(safety(),limits(),sender.port());
        FrameOptions hot{};hot.temperature=90;
        sender.send(lab.listenerPort(),makeFrame(1,hot));
        rejects([&]{lab.receiveOne(100);},"hot joint must fault");
    }
    {
        LoopbackLab lab(safety(),limits(),sender.port());
        FrameOptions err{};err.motor_error=1;
        sender.send(lab.listenerPort(),makeFrame(1,err));
        rejects([&]{lab.receiveOne(100);},"motor error must fault");
    }
    {
        LoopbackLab lab(safety(),limits(),sender.port());
        FrameOptions inactive{};inactive.flags=kMockConnected;
        sender.send(lab.listenerPort(),makeFrame(1,inactive));
        rejects([&]{lab.receiveOne(100);},"jointctrl absent must fault");
    }
}
static void testTimeout() {
    Sender sender;LoopbackLab lab(safety(),limits(),sender.port());
    rejects([&]{lab.receiveOne(15);},"no packets must time out and fault");
    check(lab.faulted()&&lab.mockCommands().empty(),"timeout must block commands");
    sender.send(lab.listenerPort(),makeFrame(1));
    rejects([&]{lab.receiveOne(20);},"cannot recover latched timeout by new packet");
}
static void testInvalidFlags() {
    Sender sender;LoopbackLab lab(safety(),limits(),sender.port());
    FrameOptions opt{};opt.flags=0x100;
    sender.send(lab.listenerPort(),makeFrame(1,opt));
    rejects([&]{lab.receiveOne(100);},"unknown protocol flags must fault");
}
int main() {
    testSuccess();testInsufficientPackets();testReplay();testUnexpectedPeer();testCRC();
    testShortAndLong();testHealth();testTimeout();testInvalidFlags();
    std::cout<<"M2.4d2 UDP LOOPBACK -> M2.4d1 RX -> M2.4c3 MOCK OUTPUT: PASS"
             <<" (synthetic protocol, NO vendor SDK / NO hardware IO)\n";
}
