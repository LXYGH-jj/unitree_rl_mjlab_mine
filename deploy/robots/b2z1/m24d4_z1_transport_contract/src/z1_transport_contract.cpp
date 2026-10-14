#include "z1_transport_contract.h"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>

namespace b2z1::m24d4 {
void MockZ1Transport::injectSynthetic(Snapshot s, Time when) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(queue_.size() >= kMaxFrames) throw std::runtime_error("synthetic feedback queue overflow");
    if(when == Time{}) throw std::invalid_argument("synthetic RX timestamp is missing");
    s.sampled_at = when;
    RxEvidence e{};
    e.completed_receive_index = next_index_++;
    e.receive_completed_at = when;
    e.callback_saw_completed_datagram = true; // synthetic, never authentic
    e.from_expected_peer = true;               // synthetic, never authentic
    e.snapshot_decoded_from_this_datagram = true; // synthetic, never authentic
    queue_.push_back(ReceivedFrame{std::move(s), e});
}
std::optional<ReceivedFrame> MockZ1Transport::popFrame() {
    std::lock_guard<std::mutex> lock(mutex_);
    if(queue_.empty()) return std::nullopt;
    auto frame = queue_.front();
    queue_.pop_front();
    return frame;
}
void MockZ1Transport::emit(const JointCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(fail_next_emit_) { fail_next_emit_ = false; throw std::runtime_error("injected mock sink failure"); }
    if(disarmed_) throw std::runtime_error("mock sink logically disarmed");
    commands_.push_back(command);
}
void MockZ1Transport::disarm() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    disarmed_ = true;
}
std::vector<JointCommand> MockZ1Transport::recordedCommands() const {
    std::lock_guard<std::mutex> lock(mutex_); return commands_;
}
std::size_t MockZ1Transport::pendingFrames() const {
    std::lock_guard<std::mutex> lock(mutex_); return queue_.size();
}
bool MockZ1Transport::softwareDisarmed() const {
    std::lock_guard<std::mutex> lock(mutex_); return disarmed_;
}
void MockZ1Transport::failNextEmit(){std::lock_guard<std::mutex> lock(mutex_);fail_next_emit_=true;}

CooperativeOwnerLease::CooperativeOwnerLease(const std::string& path) {
    if(path.empty() || path[0] != '/') throw std::invalid_argument("lease path must be absolute");
    const int fd = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if(fd < 0) throw std::runtime_error("cannot open cooperative lease file: " + std::string(std::strerror(errno)));
    struct stat st{};
    if(::fstat(fd, &st) != 0 || !S_ISREG(st.st_mode) || st.st_uid != ::geteuid() || (st.st_mode & 0077) != 0) {
        ::close(fd);
        throw std::runtime_error("unsafe cooperative lease file");
    }
    if(::flock(fd, LOCK_EX | LOCK_NB) != 0) {
        const int saved = errno;
        ::close(fd);
        throw std::runtime_error("Z1 cooperative owner already exists / lease denied: " + std::string(std::strerror(saved)));
    }
    fd_ = fd;
}
CooperativeOwnerLease::~CooperativeOwnerLease() {
    if(fd_ >= 0) { ::flock(fd_, LOCK_UN); ::close(fd_); }
}
MockOnlyRuntime::MockOnlyRuntime(IZ1Transport& t,b2z1::z1_jointctrl::Config c,
                                 b2z1::m24d1::Limits l,const std::string& path)
    : owner_(path),transport_(t),sink_(t),gate_(std::move(c),std::move(l),sink_) {
    if(t.kind() != TransportKind::SyntheticMock || dynamic_cast<MockZ1Transport*>(&t) == nullptr)
        throw std::logic_error("M2.4d4 runtime refuses all vendor transports");
}
MockOnlyRuntime::~MockOnlyRuntime() { disarm(); }
bool MockOnlyRuntime::pumpOne(Time now) {
    try {
        auto frame = transport_.popFrame();
        if(!frame) return false;
        gate_.onReceiverCallback(frame->snapshot, frame->evidence, now);
        return true;
    } catch(...) { disarm(); throw; }
}
void MockOnlyRuntime::authorizeForOfflineTest(Time now) {
    // These booleans mean ONLY test fixtures. This class cannot be used with Vendor transport.
    gate_.authorize(true,true,true,now);
}
void MockOnlyRuntime::setIkGoal(const b2z1::z1_jointctrl::Joint6& q,Time now) {
    try { gate_.setIkGoal(q,now); }
    catch(...) { disarm(); throw; }
}
bool MockOnlyRuntime::tick(Time now) {
    try { return gate_.tick(now); }
    catch(...) { disarm(); throw; }
}
void MockOnlyRuntime::disarm() noexcept {
    gate_.disarm(); // existing gate is nonthrowing in current audited implementation
    transport_.disarm();
}
bool MockOnlyRuntime::authorized() const {return gate_.authorized();}
bool MockOnlyRuntime::faulted() const {return gate_.faulted();}
std::size_t MockOnlyRuntime::sentCount() const {return gate_.sentCount();}
std::string MockOnlyRuntime::faultReason() const {return gate_.faultReason();}
} // namespace b2z1::m24d4
