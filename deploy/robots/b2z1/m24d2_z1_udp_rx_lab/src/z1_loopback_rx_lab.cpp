#include "z1_loopback_rx_lab.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>
#include <array>

namespace b2z1::m24d2 {
namespace {
using b2z1::m24d1::Snapshot;
using b2z1::z1_jointctrl::Clock;

std::uint16_t u16(const std::uint8_t* b) { return (std::uint16_t(b[0])<<8)|b[1]; }
std::uint32_t u32(const std::uint8_t* b) {
    return (std::uint32_t(b[0])<<24)|(std::uint32_t(b[1])<<16)|
           (std::uint32_t(b[2])<<8)|b[3];
}
std::uint64_t u64(const std::uint8_t* b) {
    std::uint64_t x=0;
    for(int i=0;i<8;++i) x=(x<<8)|b[i];
    return x;
}
double f64(const std::uint8_t* p) {
    const auto bits=u64(p);
    double result=0.0;
    static_assert(sizeof(result)==sizeof(bits),"requires binary64 double");
    std::memcpy(&result,&bits,sizeof(result));
    return result;
}
struct Parsed { Snapshot snapshot; std::uint64_t sequence=0; };
Parsed parseMock(const std::uint8_t* b, std::size_t n) {
    if(n!=kMockWireBytes) throw std::runtime_error("truncated/extra mock datagram");
    if(u32(b)!=kMockMagic || u16(b+4)!=kMockVersion)
        throw std::runtime_error("invalid mock magic or protocol version");
    const auto flags=u16(b+6);
    if(flags & ~std::uint16_t(kMockConnected|kMockJointCtrl))
        throw std::runtime_error("unsupported mock flags");
    if(crc32(b,n-4)!=u32(b+n-4)) throw std::runtime_error("mock datagram CRC mismatch");
    Parsed parsed{};
    parsed.sequence=u64(b+8);
    if(parsed.sequence==0) throw std::runtime_error("zero mock sequence");
    parsed.snapshot.sdk_connected=(flags&kMockConnected)!=0;
    parsed.snapshot.jointctrl_reported=(flags&kMockJointCtrl)!=0;
    const std::uint8_t* p=b+16;
    for(int j=0;j<6;++j) {
        parsed.snapshot.q[j]=f64(p);p+=8;
        parsed.snapshot.dq[j]=f64(p);p+=8;
        parsed.snapshot.tau[j]=f64(p);p+=8;
        parsed.snapshot.temperature_c[j]=static_cast<std::int16_t>(u16(p));p+=2;
        parsed.snapshot.motor_error[j]=*p++;
        parsed.snapshot.motor_connection[j]=*p++;
    }
    return parsed;
}
}
std::uint32_t crc32(const std::uint8_t* bytes,std::size_t n) {
    std::uint32_t c=0xFFFFFFFFu;
    for(std::size_t i=0;i<n;++i) {
        c^=bytes[i];
        for(int bit=0;bit<8;++bit) c=(c>>1) ^ ((c&1u)?0xEDB88320u:0u);
    }
    return c^0xFFFFFFFFu;
}

LoopbackLab::LoopbackLab(b2z1::z1_jointctrl::Config safety,
                         b2z1::m24d1::Limits limits,
                         std::uint16_t expected_port)
    : session_(std::move(safety),std::move(limits),sink_),
      expected_peer_port_(expected_port) {
    if(expected_port==0) throw std::invalid_argument("mock peer UDP port required");
    fd_=::socket(AF_INET,SOCK_DGRAM|SOCK_CLOEXEC,0);
    if(fd_<0) throw std::runtime_error("mock loopback socket creation failed");
    sockaddr_in bind_addr{};
    bind_addr.sin_family=AF_INET;
    bind_addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    bind_addr.sin_port=htons(0);
    if(::bind(fd_,reinterpret_cast<sockaddr*>(&bind_addr),sizeof(bind_addr))!=0) {
        ::close(fd_); fd_=-1;
        throw std::runtime_error("mock loopback bind failed");
    }
    socklen_t len=sizeof(bind_addr);
    if(::getsockname(fd_,reinterpret_cast<sockaddr*>(&bind_addr),&len)!=0) {
        ::close(fd_); fd_=-1;
        throw std::runtime_error("mock getsockname failed");
    }
    listener_port_=ntohs(bind_addr.sin_port);
}
LoopbackLab::~LoopbackLab() { if(fd_>=0) ::close(fd_); }
[[noreturn]] void LoopbackLab::abortRx(const std::string& reason) {
    faulted_=true; fault_reason_=reason;session_.disarm();
    throw std::runtime_error("M2.4d2 TEST-ONLY RX fault (latched): "+reason);
}
void LoopbackLab::receiveOne(int timeout_ms) {
    if(faulted_) abortRx(fault_reason_);
    if(timeout_ms<=0 || timeout_ms>1000) abortRx("invalid test receive deadline");
    pollfd pfd{fd_,POLLIN,0};
    int pr=::poll(&pfd,1,timeout_ms);
    if(pr<0) abortRx("poll failed");
    if(pr==0) abortRx("mock RX deadline expired");
    if(pfd.revents!=POLLIN) abortRx("unexpected poll state");

    std::array<std::uint8_t,kMockWireBytes> bytes{};
    sockaddr_in from{};
    iovec iov{bytes.data(),bytes.size()};
    msghdr msg{};
    msg.msg_name=&from;msg.msg_namelen=sizeof(from);
    msg.msg_iov=&iov;msg.msg_iovlen=1;
    // MSG_TRUNC flags detect oversized datagrams despite a fixed buffer.
    const ssize_t n=::recvmsg(fd_,&msg,MSG_DONTWAIT);
    const auto completed=Clock::now(); // receiver-side post-recv timestamp
    if(n<0 || (msg.msg_flags & MSG_TRUNC)!=0 ||
       static_cast<std::size_t>(n)!=kMockWireBytes)
        abortRx("invalid/truncated/oversized mock UDP frame");
    if(from.sin_family!=AF_INET || from.sin_addr.s_addr!=htonl(INADDR_LOOPBACK) ||
       ntohs(from.sin_port)!=expected_peer_port_)
        abortRx("unexpected mock UDP source IP or port");

    try {
        auto p=parseMock(bytes.data(),bytes.size());
        if(p.sequence<=last_sequence_) abortRx("duplicate/replayed mock sequence");
        p.snapshot.sampled_at=Clock::now(); // same just-decoded UDP buffer
        b2z1::m24d1::RxEvidence evidence{};
        evidence.completed_receive_index=++event_index_;
        evidence.receive_completed_at=completed;
        evidence.callback_saw_completed_datagram=true;
        evidence.from_expected_peer=true;
        evidence.snapshot_decoded_from_this_datagram=true;
        session_.onReceiverCallback(p.snapshot,evidence,Clock::now());
        last_sequence_=p.sequence;
    } catch (const std::exception& ex) {
        if(faulted_) throw;
        abortRx(ex.what());
    }
}
void LoopbackLab::authorizeForMockTest(b2z1::z1_jointctrl::Time now) {
    if(faulted_) abortRx(fault_reason_);
    session_.authorize(true,true,true,now); // FAKE test authorizations: NO real capability
}
void LoopbackLab::setMockGoal(const b2z1::z1_jointctrl::Joint6& goal,
                              b2z1::z1_jointctrl::Time now) {
    if(faulted_) abortRx(fault_reason_);
    session_.setIkGoal(goal,now);
}
bool LoopbackLab::mockTick(b2z1::z1_jointctrl::Time now) {
    if(faulted_) return false;
    return session_.tick(now);
}
} // namespace b2z1::m24d2
